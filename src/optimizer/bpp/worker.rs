use crate::eval::sep_evaluator::SeparationEvaluator;
use crate::optimizer::worker::SepStats;
use crate::quantify::tracker::CollisionTracker;
use crate::sample::search;
use crate::sample::search::SampleConfig;
use crate::util::assertions::tracker_matches_layout;
use crate::FMT;
use itertools::Itertools;
use jagua_rs::entities::{Instance, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPProblem, BPSolution, LayKey};
use log::debug;
use rand::prelude::SliceRandom;
use rand::rngs::Xoshiro256PlusPlus;
use slotmap::SecondaryMap;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::geometry::Transformation;
use tap::Tap;

/// BPP counterpart of [`crate::optimizer::worker::SeparatorWorker`].
///
/// A worker holds its own private copy of the problem and of *all* per-layout collision trackers.
/// It is loaded with the master's state, performs one round of item moves (SPP Algorithm 5, applied
/// per layout) with its own RNG, and the master afterwards picks the worker that ended up with the
/// lowest total weighted loss.
pub struct BPSeparatorWorker {
    pub instance: BPInstance,
    pub prob: BPProblem,
    /// One collision tracker per open layout, keyed identically to `prob.layouts`.
    pub trackers: SecondaryMap<LayKey, CollisionTracker>,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
}

impl BPSeparatorWorker {
    /// Restores the state of the worker to the given solution and accompanying trackers.
    ///
    /// [`BPProblem::restore`] reports whether any layout keys changed. If they did, the worker's
    /// trackers can no longer be matched to the master's by key alone, so they are rebuilt from
    /// the layouts. Otherwise the (cheap) `clone_from` path is taken, which reuses the worker's
    /// existing allocations (pair matrices etc.) instead of reallocating every iteration.
    pub fn load(&mut self, sol: &BPSolution, trackers: &SecondaryMap<LayKey, CollisionTracker>) {
        let layout_keys_changed = self.prob.restore(sol);

        if layout_keys_changed {
            // Keys are not comparable anymore, rebuild everything from scratch.
            self.rebuild_trackers();
        } else {
            // Keys are stable: sync tracker-by-tracker, reusing allocations where possible.
            // Drop trackers of layouts that no longer exist (should not happen when keys are
            // stable, but keeps the map strictly in sync with the problem).
            let stale = self.trackers.keys()
                .filter(|lkey| !self.prob.layouts.contains_key(*lkey))
                .collect_vec();
            for lkey in stale {
                self.trackers.remove(lkey);
            }

            for (lkey, master_ct) in trackers.iter() {
                debug_assert!(self.prob.layouts.contains_key(lkey));
                match self.trackers.get_mut(lkey) {
                    Some(ct) => ct.clone_from(master_ct),
                    None => {
                        self.trackers.insert(lkey, master_ct.clone());
                    }
                }
            }
        }

        debug_assert!(self.trackers.len() == self.prob.layouts.len());
        debug_assert!(self.prob.layouts.iter().all(|(lkey, l)| tracker_matches_layout(&self.trackers[lkey], l)));
    }

    /// Rebuilds all collision trackers from the current layouts (GLS weights are reset to 1.0).
    fn rebuild_trackers(&mut self) {
        self.trackers = self.prob.layouts.iter()
            .map(|(lkey, l)| (lkey, CollisionTracker::new(l)))
            .collect();
    }

    /// Algorithm 5 from <https://doi.org/10.48550/arXiv.2509.13329>, applied over all layouts.
    ///
    /// Candidates are all `(lkey, pk)` pairs whose item is currently colliding. They are collected
    /// in deterministic (SlotMap/SecondaryMap) order and subsequently shuffled with the worker's RNG,
    /// so every worker tries a different ordering.
    ///
    /// v1 performs **intra-layout** moves only: an item is always re-placed in the layout it
    /// currently sits in. Cross-layout moves (i.e. migrating an item to another bin during
    /// separation) are a natural extension and would hook in right here, by letting the destination
    /// layout be chosen before constructing the evaluator.
    pub fn move_items(&mut self) -> SepStats {
        // Collect all colliding items over all layouts, in a random order
        let candidates = self.prob.layouts.iter()
            .flat_map(|(lkey, l)| {
                let ct = &self.trackers[lkey];
                l.placed_items.keys()
                    .filter(move |pk| ct.get_loss(*pk) > 0.0)
                    .map(move |pk| (lkey, pk))
            })
            .collect_vec()
            .tap_mut(|v| v.shuffle(&mut self.rng));

        let mut total_moves = 0;
        let mut total_evals = 0;

        // Give each colliding item the opportunity to move to a better (eval) position
        for &(lkey, pk) in candidates.iter() {
            // First check if the item is still colliding
            if self.trackers[lkey].get_loss(pk) > 0.0 {
                let layout = &self.prob.layouts[lkey];
                let item_id = layout.placed_items[pk].item_id;
                let item = self.instance.item(item_id);

                // Create an 'evaluator' to perform collision detection and collision quantification of the samples during the search
                let evaluator = SeparationEvaluator::new(layout, item, pk, &self.trackers[lkey]);

                // Perform the search for a better position for the item, within the same layout
                let (best_sample, n_evals) =
                    search::search_placement(layout, item, Some(pk), evaluator, self.sample_config, &mut self.rng);

                let (new_dt, _eval) = best_sample.expect("search_placement should always return a sample");

                // Keep the item inside the bin. The coordinate descent inside `search_placement` is
                // unbounded, and in a large, sparsely filled bin it can walk an item far outside the
                // container: the quadtree cannot index such a placement, which makes the specialized
                // collision pipeline miss collisions against it (and trips its debug assertion).
                // In the SPP the strip is always fitted tightly around the items, so this cannot
                // happen there — hence the clamp lives here and SPP semantics stay untouched.
                let (new_dt, clamped) = clamp_to_container(new_dt, item, layout.container.outer_cd.bbox);

                // Move the item to the new position
                self.move_item(lkey, pk, new_dt, clamped);
                total_moves += 1;
                total_evals += n_evals;
            }
        }
        SepStats { total_moves, total_evals }
    }

    /// Removes the item and places it again in the **same** layout with a new transformation.
    ///
    /// Since the item is re-placed in a non-empty layout ([`BPLayoutType::Open`]), the layout is
    /// guaranteed to stay open across the remove/place pair, so the layout key stays valid.
    ///
    /// `clamped` indicates that `d_transf` is *not* the transformation the evaluator scored, but a
    /// clamped version of it (see [`clamp_to_container`]). In that case the "weighted loss never
    /// increases" invariant does not hold and the corresponding debug assertion is skipped.
    pub fn move_item(&mut self, lkey: LayKey, pk: PItemKey, d_transf: DTransformation, clamped: bool) -> PItemKey {
        debug_assert!(tracker_matches_layout(&self.trackers[lkey], &self.prob.layouts[lkey]));

        let item_id = self.prob.layouts[lkey].placed_items[pk].item_id;

        let ct = &self.trackers[lkey];
        let (old_l, old_w_l) = (ct.get_loss(pk), ct.get_weighted_loss(pk));

        debug_assert!(old_l > 0.0, "Item with key {:?} should be colliding, but has no loss: {}", pk, FMT().fmt2(old_l));
        debug_assert!(old_w_l > 0.0, "Item with key {:?} should be colliding, but has no weighted loss: {}", pk, FMT().fmt2(old_w_l));

        // First remove the item and subsequently place it in its new position.
        // The layout cannot become empty here (it contains at least one colliding *other* item is
        // not guaranteed, but a single-item layout can never collide with another item; it could
        // however collide with the container). To be safe we re-open by key only if the layout survived.
        let old_placement = self.prob.remove_item(lkey, pk);
        let layout_id = match old_placement.layout_id {
            // Layout survived the removal: place the item back into it.
            BPLayoutType::Open(lkey) => BPLayoutType::Open(lkey),
            // Layout was auto-closed (it held only this item): re-open the same bin type.
            BPLayoutType::Closed { bin_id } => BPLayoutType::Closed { bin_id },
        };
        let new_placement = BPPlacement { layout_id, item_id, d_transf };
        let (new_lkey, new_pk) = self.prob.place_item(new_placement);

        if new_lkey != lkey {
            // The layout was auto-closed and re-opened under a fresh key: migrate the tracker.
            // Weights are lost here, but this only happens for single-item layouts.
            self.trackers.remove(lkey);
            self.trackers.insert(new_lkey, CollisionTracker::new(&self.prob.layouts[new_lkey]));
        } else {
            // Update the collision tracker to reflect the changes
            let layout = &self.prob.layouts[new_lkey];
            self.trackers[new_lkey].register_item_move(layout, pk, new_pk);
        }

        let ct = &self.trackers[new_lkey];
        let (new_l, new_w_l) = (ct.get_loss(new_pk), ct.get_weighted_loss(new_pk));

        debug!("Moved {:?} (l: {}, wl: {}) to {:?} (l+1: {}, wl+1: {})", old_placement, FMT().fmt2(old_l), FMT().fmt2(old_w_l), new_placement, FMT().fmt2(new_l), FMT().fmt2(new_w_l));
        debug_assert!(clamped || new_lkey != lkey || new_w_l <= old_w_l * 1.001, "weighted loss should never increase: {} > {}", FMT().fmt2(old_w_l), FMT().fmt2(new_w_l));
        debug_assert!(tracker_matches_layout(&self.trackers[new_lkey], &self.prob.layouts[new_lkey]));

        new_pk
    }
}

/// Clamps a placement so that the item's (rotated) bounding box stays inside `container_bbox`.
///
/// Only the translation is adjusted; the rotation is left untouched (it is always one of the item's
/// allowed rotations). If the item is wider or taller than the container in this rotation, the
/// translation is left as-is: there is nothing sensible to clamp to, and the resulting `Exterior`
/// collision is quantified normally.
///
/// Returns the (possibly adjusted) transformation and whether an adjustment was actually made.
pub fn clamp_to_container(dt: DTransformation, item: &jagua_rs::entities::Item, container_bbox: Rect) -> (DTransformation, bool) {
    // bbox of the item rotated by `dt.rotation()`, still centred on the item's own origin
    let mut shape_buffer = item.shape_cd.as_ref().clone();
    let r_bbox = shape_buffer
        .transform_from(item.shape_cd.as_ref(), &Transformation::from_rotation(dt.rotation()))
        .bbox;

    // Valid translation range: the rotated bbox, shifted by the translation, must stay inside.
    let (x_min, x_max) = (container_bbox.x_min - r_bbox.x_min, container_bbox.x_max - r_bbox.x_max);
    let (y_min, y_max) = (container_bbox.y_min - r_bbox.y_min, container_bbox.y_max - r_bbox.y_max);

    let (tx, ty) = dt.translation();
    let c_tx = if x_min <= x_max { tx.clamp(x_min, x_max) } else { tx };
    let c_ty = if y_min <= y_max { ty.clamp(y_min, y_max) } else { ty };

    (DTransformation::new(dt.rotation(), (c_tx, c_ty)), c_tx != tx || c_ty != ty)
}
