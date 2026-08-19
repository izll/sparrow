use crate::optimizer::bpp::worker::BPSeparatorWorker;
use crate::optimizer::separator::SeparatorConfig;
use crate::optimizer::worker::SepStats;
use crate::quantify::tracker::{CTSnapshot, CollisionTracker};
use crate::sample::uniform_sampler::UniformBBoxSampler;
use crate::util::assertions::tracker_matches_layout;
use crate::util::terminator::Terminator;
use crate::FMT;
use itertools::Itertools;
use jagua_rs::entities::{Instance, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPProblem, BPSolution, LayKey};
use jagua_rs::Instant;
use log::{debug, log, warn};
use ordered_float::OrderedFloat;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, RngExt, SeedableRng};
use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
use rayon::ThreadPool;
use slotmap::SecondaryMap;
use std::cmp::Reverse;

/// A snapshot of a [`BPSeparator`]: the solution together with a snapshot of every layout's tracker.
pub type BPSnapshot = (BPSolution, SecondaryMap<LayKey, CTSnapshot>);

/// BPP counterpart of [`crate::optimizer::separator::Separator`].
///
/// Where the SPP separator works on a single layout with a single [`CollisionTracker`], the BPP
/// separator works on a *set* of layouts and keeps one tracker per layout (the GLS weights are
/// therefore also per layout). "Total loss" is the sum over all trackers; the solution is feasible
/// exactly when that sum is zero.
pub struct BPSeparator {
    pub instance: BPInstance,
    pub prob: BPProblem,
    /// One collision tracker per open layout (GLS weights live here, per layout)
    pub trackers: SecondaryMap<LayKey, CollisionTracker>,
    pub rng: Xoshiro256PlusPlus,
    pub workers: Vec<BPSeparatorWorker>,
    /// Reuses the SPP [`SeparatorConfig`] as-is
    pub config: SeparatorConfig,
    pub thread_pool: Option<ThreadPool>,
}

impl BPSeparator {
    pub fn new(instance: BPInstance, prob: BPProblem, mut rng: Xoshiro256PlusPlus, mut config: SeparatorConfig) -> Self {
        // Same escape hatch as the SPP separator: allow overriding the worker count from the
        // environment so the same input can be compared across 1 / 3 / 8 / 16 workers.
        if let Ok(v) = std::env::var("SPARROW_N_WORKERS")
            && let Ok(n) = v.parse::<usize>()
            && n > 0
        {
            config.n_workers = n;
        }

        let trackers: SecondaryMap<LayKey, CollisionTracker> = prob.layouts.iter()
            .map(|(lkey, l)| (lkey, CollisionTracker::new(l)))
            .collect();

        let workers = (0..config.n_workers).map(|_|
            BPSeparatorWorker {
                instance: instance.clone(),
                prob: prob.clone(),
                trackers: trackers.clone(),
                rng: Xoshiro256PlusPlus::seed_from_u64(rng.random()),
                sample_config: config.sample_config,
            }).collect();

        let pool = if cfg!(target_arch = "wasm32") {
            // On wasm32, only the global thread pool is available
            None
        } else {
            // Create a local thread pool to keep using the same threads for the same optimization (helps the OS scheduler)
            Some(rayon::ThreadPoolBuilder::new().num_threads(config.n_workers).build().unwrap())
        };

        Self { instance, prob, trackers, rng, workers, config, thread_pool: pool }
    }

    /// Algorithm 9 from <https://doi.org/10.48550/arXiv.2509.13329>, over the sum of all layouts' losses.
    ///
    /// Identical strike / no-improvement / GLS weight-update logic as the SPP
    /// [`Separator::separate`](crate::optimizer::separator::Separator::separate); the only difference
    /// is that the loss is aggregated over all open layouts and that the GLS weights of *every*
    /// tracker are updated each iteration.
    ///
    /// Returns the best solution found: a feasible one if separation succeeded, otherwise the
    /// 'least infeasible' one, together with the matching tracker snapshots.
    pub fn separate(&mut self, term: &impl Terminator) -> BPSnapshot {
        let mut min_loss_sol = self.save();
        let mut min_loss = self.total_loss();
        log!(self.config.log_level, "[BPSEP] separating {} layout(s) at loss: {}", self.prob.layouts.len(), FMT().fmt2(min_loss));

        let mut n_strikes = 0;
        let mut n_iter = 0;
        let mut sep_stats = SepStats { total_moves: 0, total_evals: 0 };
        let start = Instant::now();

        // As long as the strike limit is not reached, and the solution is not yet separated.
        'outer: while n_strikes < self.config.strike_limit && !term.kill() {
            let mut n_iter_no_improvement = 0;

            let initial_strike_loss = self.total_loss();
            debug!("[BPSEP] [s:{n_strikes},i:{n_iter}]     init_l: {}", FMT().fmt2(initial_strike_loss));

            while n_iter_no_improvement < self.config.iter_no_imprv_limit && !term.kill() {
                let (loss_before, w_loss_before) = (self.total_loss(), self.total_weighted_loss());
                sep_stats += self.move_items_multi();
                let (loss, w_loss) = (self.total_loss(), self.total_weighted_loss());

                debug!("[BPSEP] [s:{n_strikes},i:{n_iter}] ( ) l: {} -> {}, wl: {} -> {}, (min l: {})", FMT().fmt2(loss_before), FMT().fmt2(loss), FMT().fmt2(w_loss_before), FMT().fmt2(w_loss), FMT().fmt2(min_loss));

                if loss == 0.0 {
                    //All collisions are resolved
                    log!(self.config.log_level, "[BPSEP] [s:{n_strikes},i:{n_iter}] (S)  min_l: {}", FMT().fmt2(loss));
                    min_loss_sol = self.save();
                    break 'outer;
                } else if loss < min_loss {
                    //Not all collisions are resolved, but we found a new 'best' solution
                    log!(self.config.log_level, "[BPSEP] [s:{n_strikes},i:{n_iter}] (*) min_l: {}", FMT().fmt2(loss));
                    if loss < min_loss * 0.98 {
                        //Reset the `iter_no_improvement` counter if the best solution is a substantial improvement
                        n_iter_no_improvement = 0;
                    }
                    min_loss_sol = self.save();
                    min_loss = loss;
                } else {
                    // No improvement this iteration
                    n_iter_no_improvement += 1;
                }

                // Update the GLS weights of every layout
                for ct in self.trackers.values_mut() {
                    ct.update_weights();
                }
                n_iter += 1;
            }

            if initial_strike_loss * 0.98 <= min_loss {
                // No substantial improvement during this attempt, add a strike
                n_strikes += 1;
            } else {
                // Substantial improvement, reset strike counter
                n_strikes = 0;
            }
            self.rollback(&min_loss_sol.0, Some(&min_loss_sol.1));
        }
        let secs = start.elapsed().as_secs_f32();
        log!(self.config.log_level, "[BPSEP] finished, evals/s: {} K, evals/move: {}, moves/s: {}, iter/s: {}, #workers: {}, total {:.3}s",
            (sep_stats.total_evals as f32 / (1000.0 * secs)) as usize,
            FMT().fmt2(sep_stats.total_evals as f32 / sep_stats.total_moves.max(1) as f32),
            FMT().fmt2(sep_stats.total_moves as f32 / secs),
            FMT().fmt2(n_iter as f32 / secs),
            self.workers.len(),
            FMT().fmt2(secs),
        );

        min_loss_sol
    }

    /// Algorithm 10 from <https://doi.org/10.48550/arXiv.2509.13329>.
    ///
    /// Every worker loads the master state, runs [`BPSeparatorWorker::move_items`] with its own
    /// random ordering, and the master adopts the result of the worker with the lowest total
    /// weighted loss. `min_by_key` returns the *first* minimum, so with a fixed seed and worker
    /// count the outcome is deterministic.
    fn move_items_multi(&mut self) -> SepStats {
        let master_sol = self.prob.save();

        // Define the parallel execution closure
        let mut separate_multi = || -> SepStats {
            self.workers.par_iter_mut().map(|worker| {
                // Sync the workers with the master
                worker.load(&master_sol, &self.trackers);
                // Let all of them run `move_items` with unique random orderings in which the items are moved
                worker.move_items()
            }).sum()
        };

        // Execute the parallel separation either using the local thread pool or the global one
        let sep_report = match self.thread_pool.as_mut() {
            Some(pool) => pool.install(&mut separate_multi),
            None => separate_multi(),
        };

        debug!("[BPSEP] workers' weighted losses: {:?}", self.workers.iter().map(worker_total_weighted_loss).collect_vec());

        // Check what run yielded the best solution (lowest collision quantification)
        let best_idx = (0..self.workers.len())
            .min_by_key(|&i| OrderedFloat(worker_total_weighted_loss(&self.workers[i])))
            .expect("there should be at least one worker");

        let best_sol = self.workers[best_idx].prob.save();

        // Load this 'best' solution into the master, effectively throwing away all other work.
        let layout_keys_changed = self.prob.restore(&best_sol);
        if layout_keys_changed {
            // Keys shifted, the worker's trackers can't be matched by key: rebuild from the layouts.
            self.rebuild_trackers();
        } else {
            self.trackers.clone_from(&self.workers[best_idx].trackers);
        }

        debug_assert!(self.prob.layouts.iter().all(|(lkey, l)| tracker_matches_layout(&self.trackers[lkey], l)));

        sep_report
    }

    /// Takes a snapshot of the current state (solution + all tracker snapshots).
    pub fn save(&self) -> BPSnapshot {
        let ct_snapshots = self.trackers.iter()
            .map(|(lkey, ct)| (lkey, ct.save()))
            .collect();
        (self.prob.save(), ct_snapshots)
    }

    /// Restores the separator to a previously saved solution.
    ///
    /// If [`BPProblem::restore`] reports that layout keys changed (layouts were added or removed),
    /// all trackers are rebuilt from the layouts — the snapshots can no longer be trusted to line
    /// up by key. Otherwise, and only if tracker snapshots were provided, the losses are restored
    /// per layout while **keeping** the GLS weights (they are the search's long-term memory).
    ///
    /// A tracker's GLS weights can only be kept when its *shape* still matches: `restore_but_keep_weights`
    /// copies the snapshot's losses into the live tracker's (pre-sized) pair matrix, so the live
    /// tracker must hold exactly as many items as the snapshot does. Cross-layout moves
    /// ([`Self::transfer_item`]) change item counts **without** changing layout keys, so the size is
    /// checked per layout and mismatching trackers are rebuilt from the snapshot instead.
    pub fn rollback(&mut self, sol: &BPSolution, cts: Option<&SecondaryMap<LayKey, CTSnapshot>>) {
        let layout_keys_changed = self.prob.restore(sol);

        match cts {
            Some(cts) if !layout_keys_changed => {
                // Drop trackers of layouts that vanished
                let stale = self.trackers.keys()
                    .filter(|lkey| !self.prob.layouts.contains_key(*lkey))
                    .collect_vec();
                for lkey in stale {
                    self.trackers.remove(lkey);
                }
                for (lkey, cts) in cts.iter() {
                    let layout = &self.prob.layouts[lkey];
                    match self.trackers.get_mut(lkey) {
                        // A snapshot of the tracker was provided and it has the same number of
                        // items: restore the losses but keep the (long-term memory) GLS weights.
                        Some(ct) if ct.size == cts.size && cts.size == layout.placed_items.len() => {
                            ct.restore_but_keep_weights(cts, layout)
                        }
                        // Either there is no tracker for this layout yet, or the item count changed
                        // (a cross-layout move). Adopt the snapshot as-is: it is by construction the
                        // tracker of exactly this layout state, weights included.
                        _ => {
                            debug_assert!(cts.size == layout.placed_items.len(),
                                "the tracker snapshot must match the restored layout");
                            self.trackers.insert(lkey, cts.clone());
                        }
                    }
                }
            }
            //otherwise, rebuild them
            _ => self.rebuild_trackers(),
        }

        debug_assert!(self.trackers.len() == self.prob.layouts.len());
        debug_assert!(self.prob.layouts.iter().all(|(lkey, l)| tracker_matches_layout(&self.trackers[lkey], l)));
    }

    /// Removes the item and places it again in the **same** layout with a new transformation.
    pub fn move_item(&mut self, lkey: LayKey, pk: PItemKey, d_transf: DTransformation) -> PItemKey {
        debug_assert!(tracker_matches_layout(&self.trackers[lkey], &self.prob.layouts[lkey]));

        let item_id = self.prob.layouts[lkey].placed_items[pk].item_id;

        let ct = &self.trackers[lkey];
        let (old_loss, old_weighted_loss) = (ct.get_loss(pk), ct.get_weighted_loss(pk));

        //Remove the item from the problem (this may auto-close a single-item layout)
        let old_placement = self.prob.remove_item(lkey, pk);

        //Place the item again but with a new transformation
        let new_placement = BPPlacement { layout_id: old_placement.layout_id, item_id, d_transf };
        let (new_lkey, new_pk) = self.prob.place_item(new_placement);

        if new_lkey != lkey {
            // The layout was auto-closed and re-opened under a fresh key: migrate the tracker.
            self.trackers.remove(lkey);
            self.trackers.insert(new_lkey, CollisionTracker::new(&self.prob.layouts[new_lkey]));
        } else {
            self.trackers[new_lkey].register_item_move(&self.prob.layouts[new_lkey], pk, new_pk);
        }

        let ct = &self.trackers[new_lkey];
        let (new_loss, new_weighted_loss) = (ct.get_loss(new_pk), ct.get_weighted_loss(new_pk));

        debug!("[BPMV] moved item {} from l: {}, wl: {} to l+1: {}, wl+1: {}",
            item_id, FMT().fmt2(old_loss), FMT().fmt2(old_weighted_loss), FMT().fmt2(new_loss), FMT().fmt2(new_weighted_loss));

        debug_assert!(tracker_matches_layout(&self.trackers[new_lkey], &self.prob.layouts[new_lkey]));

        new_pk
    }

    /// Moves an item **out of one layout and into another**, at a random feasible position inside
    /// the destination container.
    ///
    /// This is the transactional cross-layout primitive the pack-down step
    /// ([`crate::optimizer::bpp::compress::pack_down`]) is built on. It embraces jagua-rs'
    /// auto-close semantics rather than fighting them:
    ///
    /// * removing the last item of `src` closes that layout and returns its bin to stock — which
    ///   is precisely the outcome the pack-down step is hoping for (one bin fewer);
    /// * the destination is always non-empty (it is a different, open layout), so `dst` keeps its
    ///   key across the placement.
    ///
    /// The trackers of both touched layouts are rebuilt (GLS weights reset for those two only, the
    /// weights of untouched layouts survive), and the workers are reseeded from the new state.
    ///
    /// Returns `(new_pk, src_closed)`; `None` if the item does not fit inside the destination
    /// container in any allowed rotation (in which case **nothing is changed**).
    pub fn transfer_item(&mut self, src: LayKey, pk: PItemKey, dst: LayKey) -> Option<(PItemKey, bool)> {
        debug_assert!(src != dst, "transfer_item is for cross-layout moves only");
        debug_assert!(self.prob.layouts.contains_key(src) && self.prob.layouts.contains_key(dst));

        let item_id = self.prob.layouts[src].placed_items[pk].item_id;
        let item = self.instance.item(item_id);
        let dst_bbox = self.prob.layouts[dst].container.outer_cd.bbox;

        // Sample a random (feasible-rotation) position anywhere inside the destination container.
        // Bail out *before* touching the problem if the item cannot fit there at all.
        let sampler = UniformBBoxSampler::new(dst_bbox, item, dst_bbox)?;
        let d_transf = sampler.sample(&mut self.rng);

        // 1. Remove from the source. This may auto-close the (now empty) source layout.
        self.prob.remove_item(src, pk);
        let src_closed = !self.prob.layouts.contains_key(src);
        if src_closed {
            self.trackers.remove(src);
        }

        // 2. Place into the destination, which is guaranteed to stay open (it was non-empty).
        let (new_lkey, new_pk) = self.prob.place_item(BPPlacement {
            layout_id: BPLayoutType::Open(dst),
            item_id,
            d_transf,
        });
        debug_assert!(new_lkey == dst, "the destination layout must keep its key");

        // 3. Rebuild the trackers of the (at most two) touched layouts and resync the workers.
        if !src_closed {
            let ct = CollisionTracker::new(&self.prob.layouts[src]);
            debug_assert!(tracker_matches_layout(&ct, &self.prob.layouts[src]));
            self.trackers.insert(src, ct);
        }
        let ct = CollisionTracker::new(&self.prob.layouts[dst]);
        debug_assert!(tracker_matches_layout(&ct, &self.prob.layouts[dst]));
        self.trackers.insert(dst, ct);
        self.reseed_workers();

        debug_assert!(self.trackers.len() == self.prob.layouts.len());
        debug_assert!(self.prob.layouts.iter().all(|(lkey, l)| tracker_matches_layout(&self.trackers[lkey], l)));

        Some((new_pk, src_closed))
    }

    /// Rebuilds all collision trackers from the current layouts. GLS weights are reset to 1.0.
    pub fn rebuild_trackers(&mut self) {
        self.trackers = self.prob.layouts.iter()
            .map(|(lkey, l)| (lkey, CollisionTracker::new(l)))
            .collect();
    }

    /// Swaps in a different [`SeparatorConfig`] (returning the previous one) and reseeds the workers
    /// so they pick up the new `sample_config`.
    ///
    /// Used by the pack-down step in [`crate::optimizer::bpp::compress`], which runs *many* short
    /// separations and therefore wants a much cheaper separator than the surrounding phase.
    /// Note that `n_workers` is **not** applied retroactively: the worker vector (and the thread
    /// pool) are sized at construction time, so only the iteration limits and the sample config
    /// take effect.
    pub fn swap_config(&mut self, config: SeparatorConfig) -> SeparatorConfig {
        let old = self.config;
        self.config = config;
        self.reseed_workers();
        old
    }

    /// Rebuilds the workers from the master's current state (fresh RNG seeds from the master RNG).
    fn reseed_workers(&mut self) {
        let (instance, prob, trackers, sample_config) =
            (&self.instance, &self.prob, &self.trackers, self.config.sample_config);
        let mut seeds = Vec::with_capacity(self.workers.len());
        for _ in 0..self.workers.len() {
            seeds.push(self.rng.random::<u64>());
        }
        for (worker, seed) in self.workers.iter_mut().zip(seeds) {
            *worker = BPSeparatorWorker {
                instance: instance.clone(),
                prob: prob.clone(),
                trackers: trackers.clone(),
                rng: Xoshiro256PlusPlus::seed_from_u64(seed),
                sample_config,
            };
        }
    }

    /// The total collision loss over all layouts. Zero ⟺ the solution is feasible.
    pub fn total_loss(&self) -> f32 {
        self.trackers.values().map(|ct| ct.get_total_loss()).sum()
    }

    /// The total GLS-weighted collision loss over all layouts (the value the workers minimize).
    pub fn total_weighted_loss(&self) -> f32 {
        self.trackers.values().map(|ct| ct.get_total_weighted_loss()).sum()
    }

    /// Closes one bin and scatters its content over the remaining open bins.
    ///
    /// This is the BPP analogue of [`Separator::change_strip_width`](crate::optimizer::separator::Separator::change_strip_width):
    /// it makes the solution *smaller* (one bin fewer) at the cost of introducing overlap, which
    /// [`Self::separate`] then has to resolve.
    ///
    /// Steps:
    /// 1. Remove **all** items of `lkey`. The layout auto-closes once the last item is removed and
    ///    the bin's stock is returned.
    /// 2. Re-insert the items **largest first** into the remaining open layouts, round-robin
    ///    starting from the least dense one, each at a random position drawn from a
    ///    [`UniformBBoxSampler`] over the full destination container bbox (feasible rotation only).
    ///    Overlaps are explicitly allowed here.
    /// 3. Rebuild the trackers of the affected layouts (weights reset) and reseed the workers.
    ///
    /// Returns `true` if a bin was actually closed. If `lkey` is the only open layout there is
    /// nowhere to scatter into, so this is a no-op returning `false`.
    pub fn close_bin_and_scatter(&mut self, lkey: LayKey) -> bool {
        if self.prob.layouts.len() < 2 {
            warn!("[BPSEP] cannot close the only open bin, nothing to scatter into");
            return false;
        }

        // Taken before anything is removed, so the scatter can bail out cleanly if an item turns
        // out not to fit any surviving bin (see the sampler below): by that point the closed bin's
        // items are already gone, and without this the separator would be left in a half-scattered
        // state with items missing from the problem entirely.
        let snapshot_before = self.save();

        // --- 1. Collect and remove the content of the target layout --------------------------
        // Deterministic order (SlotMap iteration), sorted largest-first by original item area.
        let mut items_to_scatter = self.prob.layouts[lkey].placed_items.keys().collect_vec();
        items_to_scatter.sort_by_key(|pk| {
            let item_id = self.prob.layouts[lkey].placed_items[*pk].item_id;
            Reverse(OrderedFloat(self.instance.item(item_id).area()))
        });
        let item_ids = items_to_scatter.iter()
            .map(|pk| self.prob.layouts[lkey].placed_items[*pk].item_id)
            .collect_vec();

        for pk in items_to_scatter {
            // Removing the last item auto-closes the layout and returns the bin to stock
            self.prob.remove_item(lkey, pk);
        }
        debug_assert!(!self.prob.layouts.contains_key(lkey), "layout should have been auto-closed");
        self.trackers.remove(lkey);

        // --- 2. Scatter the items over the remaining layouts ---------------------------------
        // Destination order: least dense layout first (ties broken by LayKey order => deterministic)
        let destinations = self.prob.layouts.iter()
            .map(|(dst_lkey, l)| (dst_lkey, OrderedFloat(l.density(&self.instance))))
            .sorted_by_key(|(_, dens)| *dens)
            .map(|(dst_lkey, _)| dst_lkey)
            .collect_vec();

        let mut affected: Vec<LayKey> = vec![];
        for (i, item_id) in item_ids.iter().enumerate() {
            // Round-robin over the destinations, starting from the least dense
            let dst_lkey = destinations[i % destinations.len()];
            let item = self.instance.item(*item_id);
            let dst_bbox = self.prob.layouts[dst_lkey].container.outer_cd.bbox;

            // Sample a random (feasible-rotation) position anywhere inside the destination
            // container. The item may genuinely not fit this particular bin in any rotation (bins
            // of different sizes are allowed), which is a routine outcome rather than a bug — so
            // skip that destination instead of panicking mid-scatter and leaving the problem with
            // the closed bin's items already removed.
            let Some(sampler) = UniformBBoxSampler::new(dst_bbox, item, dst_bbox) else {
                warn!("[BPSEP] item {item_id} does not fit into layout {dst_lkey:?} in any rotation; \
                       aborting the scatter and restoring the previous solution");
                let (sol, cts) = &snapshot_before;
                self.rollback(sol, Some(cts));
                self.reseed_workers();
                return false;
            };
            let d_transf = sampler.sample(&mut self.rng);

            self.prob.place_item(BPPlacement {
                layout_id: BPLayoutType::Open(dst_lkey),
                item_id: *item_id,
                d_transf,
            });
            if !affected.contains(&dst_lkey) {
                affected.push(dst_lkey);
            }
        }

        // --- 3. Rebuild the trackers of the affected layouts and reseed the workers -----------
        for dst_lkey in affected {
            let ct = CollisionTracker::new(&self.prob.layouts[dst_lkey]);
            debug_assert!(tracker_matches_layout(&ct, &self.prob.layouts[dst_lkey]));
            self.trackers.insert(dst_lkey, ct);
        }
        self.reseed_workers();

        log!(self.config.log_level, "[BPSEP] closed a bin and scattered {} item(s) over {} remaining layout(s), loss: {}",
            item_ids.len(), self.prob.layouts.len(), FMT().fmt2(self.total_loss()));

        debug_assert!(self.trackers.len() == self.prob.layouts.len());
        true
    }

    /// The open layout with the lowest density (ties broken by LayKey order → deterministic).
    pub fn least_dense_layout(&self) -> Option<LayKey> {
        self.nth_least_dense_layout(0)
    }

    /// The `n`-th least dense open layout (0 = least dense), or `None` if there are fewer layouts.
    ///
    /// Used by the exploration phase to vary the bin it tries to eliminate across consecutive
    /// attempts: always attacking the same (least dense) bin makes retries very similar to each
    /// other, whereas the second/third least dense bin gives a genuinely different subproblem.
    /// Ties are broken by `LayKey` order, so the ordering is deterministic.
    pub fn nth_least_dense_layout(&self, n: usize) -> Option<LayKey> {
        self.prob.layouts.iter()
            .map(|(lkey, l)| (lkey, OrderedFloat(l.density(&self.instance))))
            .sorted_by_key(|(_, dens)| *dens)
            .map(|(lkey, _)| lkey)
            .nth(n)
    }
}

/// Helper to keep the `min_by_key` closure in [`BPSeparator::move_items_multi`] readable.
fn worker_total_weighted_loss(worker: &BPSeparatorWorker) -> f32 {
    worker.trackers.values().map(|ct| ct.get_total_weighted_loss()).sum()
}
