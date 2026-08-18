use crate::consts::{GLS_WEIGHT_DECAY, GLS_WEIGHT_MAX, GLS_WEIGHT_MAX_INC_RATIO, GLS_WEIGHT_MIN_INC_RATIO};
use crate::quantify::pair_matrix::PairMatrix;
use crate::quantify::circles_soa::CirclesSoA;
use crate::quantify::{quantify_collision_poly_container, quantify_collision_poly_hole, quantify_collision_poly_poly_soa};
use crate::util::assertions::tracker_matches_layout;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Layout, PItemKey};
use jagua_rs::geometry::primitives::SPolygon;
use itertools::Itertools;
use ordered_float::Float;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use slotmap::SecondaryMap;

/// Collision losses of a single item: against the container, against other items (by tracker index)
/// and against the holes of the container (by hole index).
struct ItemLosses {
    container: f32,
    pairs: Vec<(usize, f32)>,
    /// `(hole_idx, loss)` for every [`HazardEntity::Hole`] the item currently collides with.
    /// Always empty for containers without holes (the plain strip-packing case).
    holes: Vec<(usize, f32)>,
}

/// Tracker of both collisions between pair of items and collisions with the container.
/// It also stores the weights for every pair of hazards and is used as a cache for collisions.
#[derive(Debug, Clone)]
pub struct CollisionTracker {
    pub size: usize,
    pub pk_idx_map: SecondaryMap<PItemKey, usize>,
    pub pair_collisions: PairMatrix,
    pub container_collisions: Vec<CTEntry>,
    /// Number of [`HazardEntity::Hole`] hazards of the layout's container (0 for a plain strip).
    pub n_holes: usize,
    /// Losses/weights between every item and every hole, row-major (`size` rows x `n_holes` columns).
    /// Empty when the container has no holes, so the plain strip-packing path is unaffected.
    pub hole_collisions: Vec<CTEntry>,
}

pub type CTSnapshot = CollisionTracker;

impl CollisionTracker {
    pub fn new(l: &Layout) -> Self {
        let size = l.placed_items.len();
        let n_holes = n_holes_of(l);

        // Create the tracker
        let mut ot = Self {
            size,
            pk_idx_map: l.placed_items.keys().enumerate()
                .map(|(i, pk)| (pk, i))
                .collect(),
            pair_collisions: PairMatrix::new(size),
            container_collisions: vec![CTEntry { weight: 1.0, loss: 0.0 }; size],
            n_holes,
            hole_collisions: vec![CTEntry { weight: 1.0, loss: 0.0 }; size * n_holes],
        };

        // Compute the losses for all items (in parallel, the computation is read-only w.r.t. the layout and tracker),
        // and subsequently write them into the (fresh, all zero) tracker.
        let pks = l.placed_items.keys().collect_vec();
        let item_losses: Vec<ItemLosses> = pks.par_iter()
            .map(|&pk| ot.compute_losses_for_item(pk, l))
            .collect();

        for (pk, losses) in pks.into_iter().zip(item_losses) {
            ot.store_losses_for_item(pk, losses);
        }

        debug_assert!(tracker_matches_layout(&ot, l));

        ot
    }

    /// Computes the collision losses of a single item against all other hazards in the layout (without modifying the tracker).
    fn compute_losses_for_item(&self, pk: PItemKey, l: &Layout) -> ItemLosses {
        let pi = &l.placed_items[pk];
        let shape = &pi.shape;

        // Compute which hazards are currently colliding with the item
        let mut collector = BasicHazardCollector::with_capacity(l.placed_items.len() + 1);
        l.cde().collect_poly_collisions(shape, &mut collector);
        // Remove the item itself from the detector
        collector.remove_by_entity(&HazardEntity::from((pk, pi)));

        let mut losses = ItemLosses { container: 0.0, pairs: Vec::with_capacity(collector.len()), holes: vec![] };
        if collector.is_empty() {
            return losses;
        }

        // Poles of the item in SoA layout (vectorized quantification against all colliding items)
        let poles_soa = CirclesSoA::from_circles(&shape.surrogate().poles);

        // For each colliding hazard, quantify the collision
        for (_, haz) in collector.iter() {
            match haz {
                HazardEntity::PlacedItem { pk: other_pk, .. } => {
                    let shape_other = &l.placed_items[*other_pk].shape;
                    let idx_other = self.pk_idx_map[*other_pk];

                    let loss = quantify_collision_poly_poly_soa(shape_other, shape, &poles_soa);
                    assert!(loss > 0.0, "loss for a collision should be > 0.0");
                    losses.pairs.push((idx_other, loss));
                }
                HazardEntity::Exterior => {
                    let loss = quantify_collision_poly_container(shape, l.container.outer_cd.bbox);
                    assert!(loss > 0.0, "loss for a collision should be > 0.0");
                    losses.container = loss;
                }
                HazardEntity::Hole { idx } => {
                    // Holes (sheet walls) are quantified by bbox overlap, see
                    // [`quantify_collision_poly_hole`]: they have no pole surrogate, and for a long
                    // thin axis-aligned wall the exact bbox overlap is the better gradient anyway.
                    let loss = quantify_collision_poly_hole(shape, hole_shape(l, *idx).bbox);
                    assert!(loss > 0.0, "loss for a collision should be > 0.0");
                    losses.holes.push((*idx, loss));
                }
                HazardEntity::InferiorQualityZone { .. } =>
                    unimplemented!("inferior quality zones are not supported by the separator; only holes (quality 0) are"),
            }
        }
        losses
    }

    /// Stores previously computed losses of an item in the tracker (does not reset any existing entries).
    fn store_losses_for_item(&mut self, pk: PItemKey, losses: ItemLosses) {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].loss = losses.container;
        for (idx_other, loss) in losses.pairs {
            self.pair_collisions[(idx, idx_other)].loss = loss;
        }
        for (hole_idx, loss) in losses.holes {
            self.hole_collisions[idx * self.n_holes + hole_idx].loss = loss;
        }
    }

    fn recompute_loss_for_item(&mut self, pk: PItemKey, l: &Layout) {
        let idx = self.pk_idx_map[pk];

        // Reset all current loss values for the item
        for i in 0..self.size {
            self.pair_collisions[(idx, i)].loss = 0.0;
        }
        self.container_collisions[idx].loss = 0.0;
        for h in 0..self.n_holes {
            self.hole_collisions[idx * self.n_holes + h].loss = 0.0;
        }

        // Recompute and store
        let losses = self.compute_losses_for_item(pk, l);
        self.store_losses_for_item(pk, losses);
    }

    pub fn restore_but_keep_weights(&mut self, cts: &CTSnapshot, layout: &Layout) {
        //Copy the loss and keys, but keep the weights
        self.pk_idx_map = cts.pk_idx_map.clone();
        self.pair_collisions.data.iter_mut()
            .zip(cts.pair_collisions.data.iter())
            .for_each(|(a, b)| a.loss = b.loss);
        self.container_collisions.iter_mut()
            .zip(cts.container_collisions.iter())
            .for_each(|(a, b)| a.loss = b.loss);
        self.hole_collisions.iter_mut()
            .zip(cts.hole_collisions.iter())
            .for_each(|(a, b)| a.loss = b.loss);
        debug_assert!(tracker_matches_layout(self, layout));
    }

    pub fn save(&self) -> CTSnapshot {
        self.clone()
    }

    pub fn register_item_move(&mut self, l: &Layout, old_pk: PItemKey, new_pk: PItemKey) {
        //swap the keys in the pk_idx_map
        let idx = self.pk_idx_map.remove(old_pk).unwrap();
        self.pk_idx_map.insert(new_pk, idx);

        self.recompute_loss_for_item(new_pk, l);

        debug_assert!(tracker_matches_layout(self, l));
    }


    /// Algorithm 8 from https://doi.org/10.48550/arXiv.2509.13329
    pub fn update_weights(&mut self) {
        // Find the maximum loss across all entries
        let max_loss = self.pair_collisions.data.iter()
            .chain(self.container_collisions.iter())
            .chain(self.hole_collisions.iter())
            .map(|e| e.loss)
            .fold(0.0, |a, b| a.max(b));

        // Go over all entries (pairs, container, holes) and modify their weights.
        for e in self.pair_collisions.data.iter_mut()
            .chain(self.container_collisions.iter_mut())
            .chain(self.hole_collisions.iter_mut()) {
            let multiplier = match e.loss == 0.0 {
                true => {
                    // No collision at the moment, slowly decay the weight back to 1.0
                    GLS_WEIGHT_DECAY
                },
                false => {
                    // Collision detected, increase the weight based on 'how bad' the collision is relative to the worst collision
                    GLS_WEIGHT_MIN_INC_RATIO + (GLS_WEIGHT_MAX_INC_RATIO - GLS_WEIGHT_MIN_INC_RATIO) * (e.loss / max_loss)
                },
            };
            e.weight = (e.weight * multiplier).clamp(1.0, GLS_WEIGHT_MAX);
        }
    }

    pub fn get_pair_weight(&self, pk1: PItemKey, pk2: PItemKey) -> f32 {
        let (idx1, idx2) = (self.pk_idx_map[pk1], self.pk_idx_map[pk2]);
        self.pair_collisions[(idx1, idx2)].weight
    }

    pub fn get_container_weight(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].weight
    }

    /// GLS weight between item `pk` and the hole with index `idx_hole`
    /// (the index of the shape within `container.quality_zones[0]`).
    pub fn get_hole_weight(&self, pk: PItemKey, idx_hole: usize) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.hole_collisions[idx * self.n_holes + idx_hole].weight
    }

    /// Collision loss between item `pk` and the hole with index `idx_hole`.
    pub fn get_hole_loss(&self, pk: PItemKey, idx_hole: usize) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.hole_collisions[idx * self.n_holes + idx_hole].loss
    }

    /// Algorithm 1 from https://doi.org/10.48550/arXiv.2509.13329
    /// Evaluations between item pairs are stored in this data-structure for quick and easy retrieval.
    pub fn get_pair_loss(&self, pk1: PItemKey, pk2: PItemKey) -> f32 {
        let (idx1, idx2) = (self.pk_idx_map[pk1], self.pk_idx_map[pk2]);
        self.pair_collisions[(idx1, idx2)].loss
    }

    pub fn get_container_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].loss
    }

    pub fn get_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];

        let pair_loss = (0..self.size)
            .map(|i| self.pair_collisions[(idx, i)].loss)
            .sum::<f32>();

        let hole_loss = self.hole_collisions[idx * self.n_holes..(idx + 1) * self.n_holes].iter()
            .map(|e| e.loss)
            .sum::<f32>();

        self.container_collisions[idx].loss + pair_loss + hole_loss
    }

    pub fn get_weighted_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];

        let w_pair_loss = (0..self.size)
            .map(|i| self.pair_collisions[(idx, i)].weighted_loss())
            .sum::<f32>();

        let w_hole_loss = self.hole_collisions[idx * self.n_holes..(idx + 1) * self.n_holes].iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        self.container_collisions[idx].weighted_loss() + w_pair_loss + w_hole_loss
    }

    pub fn get_total_loss(&self) -> f32 {
        let cont_o = self.container_collisions.iter().map(|e| e.loss).sum::<f32>();

        let pair_o = self.pair_collisions.data.iter()
            .map(|e| e.loss)
            .sum::<f32>();

        let hole_o = self.hole_collisions.iter()
            .map(|e| e.loss)
            .sum::<f32>();

        cont_o + pair_o + hole_o
    }

    pub fn get_total_weighted_loss(&self) -> f32 {
        let cont_w_o = self.container_collisions.iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        let pair_w_o = self.pair_collisions.data.iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        let hole_w_o = self.hole_collisions.iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        cont_w_o + pair_w_o + hole_w_o
    }
}

/// Number of [`HazardEntity::Hole`] hazards induced by the layout's container.
/// Holes are exactly the shapes of the quality-0 zone (see [`jagua_rs::entities::InferiorQualityZone`]).
pub fn n_holes_of(l: &Layout) -> usize {
    l.container.quality_zones[0].as_ref().map_or(0, |z| z.shapes_cd.len())
}

/// The collision-detection shape of the hole with index `idx` of the layout's container.
pub fn hole_shape(l: &Layout, idx: usize) -> &SPolygon {
    l.container.quality_zones[0].as_ref()
        .expect("layout has a hole hazard, so it must have a quality-0 zone")
        .shapes_cd[idx].as_ref()
}

#[derive(Debug, Clone, Copy)]
pub struct CTEntry {
    pub loss: f32,
    pub weight: f32,
}

impl CTEntry {
    pub fn weighted_loss(&self) -> f32 {
        self.weight * self.loss
    }
}