use crate::consts::{GLS_WEIGHT_DECAY, GLS_WEIGHT_MAX_INC_RATIO, GLS_WEIGHT_MIN_INC_RATIO};
use crate::quantify::pair_matrix::PairMatrix;
use crate::quantify::circles_soa::CirclesSoA;
use crate::quantify::{quantify_collision_poly_container, quantify_collision_poly_poly_soa};
use crate::util::assertions::tracker_matches_layout;
use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Layout, PItemKey};
use itertools::Itertools;
use ordered_float::Float;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use slotmap::SecondaryMap;

/// Collision losses of a single item: against the container and against other items (by tracker index)
struct ItemLosses {
    container: f32,
    pairs: Vec<(usize, f32)>,
}

/// Tracker of both collisions between pair of items and collisions with the container.
/// It also stores the weights for every pair of hazards and is used as a cache for collisions.
#[derive(Debug, Clone)]
pub struct CollisionTracker {
    pub size: usize,
    pub pk_idx_map: SecondaryMap<PItemKey, usize>,
    pub pair_collisions: PairMatrix,
    pub container_collisions: Vec<CTEntry>,
}

pub type CTSnapshot = CollisionTracker;

impl CollisionTracker {
    pub fn new(l: &Layout) -> Self {
        let size = l.placed_items.len();

        // Create the tracker
        let mut ot = Self {
            size,
            pk_idx_map: l.placed_items.keys().enumerate()
                .map(|(i, pk)| (pk, i))
                .collect(),
            pair_collisions: PairMatrix::new(size),
            container_collisions: vec![CTEntry { weight: 1.0, loss: 0.0 }; size],
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

        let mut losses = ItemLosses { container: 0.0, pairs: Vec::with_capacity(collector.len()) };
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
                _ => unimplemented!("unsupported hazard entity"),
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
    }

    fn recompute_loss_for_item(&mut self, pk: PItemKey, l: &Layout) {
        let idx = self.pk_idx_map[pk];

        // Reset all current loss values for the item
        for i in 0..self.size {
            self.pair_collisions[(idx, i)].loss = 0.0;
        }
        self.container_collisions[idx].loss = 0.0;

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
            .map(|e| e.loss)
            .fold(0.0, |a, b| a.max(b));

        // Go over all entries (pairs) and modify their weights.
        for e in self.pair_collisions.data.iter_mut()
            .chain(self.container_collisions.iter_mut()) {
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
            e.weight = (e.weight * multiplier).max(1.0);
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

        self.container_collisions[idx].loss + pair_loss
    }

    pub fn get_weighted_loss(&self, pk: PItemKey) -> f32 {
        let idx = self.pk_idx_map[pk];

        let w_pair_loss = (0..self.size)
            .map(|i| self.pair_collisions[(idx, i)].weighted_loss())
            .sum::<f32>();

        self.container_collisions[idx].weighted_loss() + w_pair_loss
    }

    pub fn get_total_loss(&self) -> f32 {
        let cont_o = self.container_collisions.iter().map(|e| e.loss).sum::<f32>();

        let pair_o = self.pair_collisions.data.iter()
            .map(|e| e.loss)
            .sum::<f32>();

        cont_o + pair_o
    }

    pub fn get_total_weighted_loss(&self) -> f32 {
        let cont_w_o = self.container_collisions.iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        let pair_w_o = self.pair_collisions.data.iter()
            .map(|e| e.weighted_loss())
            .sum::<f32>();

        cont_w_o + pair_w_o
    }
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