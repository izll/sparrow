//! Remainder consolidation: the BPP counterpart of [`crate::optimizer::compress`].
//!
//! Once the exploration phase can no longer eliminate a bin, the primary objective (bin count) is
//! settled and the *secondary* objective takes over: make the leftover material as reusable as
//! possible by concentrating it in a single bin, so that what remains is a single rectangular
//! offcut instead of a scattered set of gaps spread over every bin.
//!
//! This happens in two steps:
//!
//! 1. **Pack-down** ([`pack_down`]) — the *cross-layout* step. Items are moved out of the least
//!    dense bin into the other (denser) ones, one at a time, each accepted only when the global
//!    separation loop can make room for it without any collision left anywhere. This is what
//!    empties the sparsest bin; if it empties completely, jagua-rs auto-closes it and the bin count
//!    drops as a bonus.
//! 2. **Strip consolidation** ([`consolidate_layout`]) — the *intra-layout* step, on the (now much
//!    sparser) least dense bin: push its remaining content against one edge.
//!
//! The second step is done by treating the least dense bin as a **strip packing** subproblem: its items are
//! lifted into a fresh [`SPInstance`] whose strip has the bin's height and width, seeded with the
//! current placements, and the existing SPP machinery ([`crate::optimizer::explore::exploration_phase`])
//! is asked to shrink that strip. If it succeeds, the resulting (narrower) placements are written
//! back into the BPP layout.
//!
//! Everything is guarded: the write-back only happens when the rebuilt layout is verified feasible
//! (`Layout::is_feasible`), otherwise the phase rolls back to the input solution. So in the worst
//! case this phase is a no-op that only *reports* the per-bin statistics.

use crate::config::BPCompressionConfig;
use crate::eval::sep_evaluator::SeparationEvaluator;
use crate::optimizer::bpp::separator::BPSeparator;
use crate::optimizer::bpp::worker::clamp_to_container;
use crate::optimizer::explore::exploration_phase as sp_exploration_phase;
use crate::optimizer::separator::Separator;
use crate::sample::search::search_placement;
use crate::util::bpp_io::BPSolutionListener;
use crate::util::listener::{DummySolListener, ReportType};
use crate::util::terminator::{BasicTerminator, Terminator};
use itertools::Itertools;
use jagua_rs::Instant;
use jagua_rs::entities::{Instance, PItemKey};
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPSolution, LayKey};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, Strip};
use log::{debug, info, warn};
use ordered_float::OrderedFloat;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, RngExt, SeedableRng};
use std::cmp::Reverse;
use std::time::Duration;

/// BPP counterpart of [`crate::optimizer::compress::compression_phase`].
///
/// Takes the best solution of the exploration phase and alternates the two consolidation steps —
/// [`pack_down`] (cross-layout, `config.pack_down`) and [`consolidate_layout`] (intra-layout,
/// `config.consolidate_remainder`) — until the budget runs out or a full round changes nothing.
///
/// Returns the improved solution, or `init_sol` unchanged when no (verified feasible) improvement
/// could be made.
pub fn compression_phase(
    instance: &BPInstance,
    sep: &mut BPSeparator,
    init_sol: &BPSolution,
    sol_listener: &mut impl BPSolutionListener,
    term: &impl Terminator,
    config: &BPCompressionConfig,
) -> BPSolution {
    sep.rollback(init_sol, None);
    let mut best_sol = init_sol.clone();

    if report_stats(instance, sep, "start").is_empty() {
        warn!("[BPCMPR] no layouts to compress");
        return best_sol;
    }

    // The two steps alternate until the budget runs out or a full round changes nothing: the strip
    // consolidation frees a contiguous block in the sparsest bin, which occasionally lets the next
    // pack-down round move another item out of it (and vice versa).
    let mut round = 0usize;
    while !term.kill() {
        round += 1;
        let mut progress = false;

        // --- 1. Pack-down: empty the least dense bin into the other ones ---------------------
        // It gets a *share* of the remaining budget (`pack_down_time_ratio`); the rest is reserved
        // for the strip consolidation that turns the emptied bin's remainder into a single offcut.
        if config.pack_down && !term.kill() {
            let pack_down_term = share_of(term, config.pack_down_time_ratio, config.time_limit);
            let (new_sol, n_moved) = pack_down(sep, &pack_down_term, config, sol_listener, instance);
            if n_moved > 0 {
                best_sol = new_sol;
                progress = true;
            }
            sep.rollback(&best_sol, None);
        }

        if !config.consolidate_remainder {
            info!("[BPCMPR] remainder consolidation disabled");
            break;
        }
        if term.kill() {
            info!("[BPCMPR] no time left for remainder consolidation");
            break;
        }

        // --- 2. Strip consolidation of the (now sparser) least dense bin ---------------------
        // Recompute the statistics: the pack-down step may have changed which bin is the least
        // dense one (and may have removed a bin entirely).
        let stats = report_stats(instance, sep, &format!("round {round}, before consolidation"));
        let Some((target, used_width, _)) = stats.first().copied() else {
            warn!("[BPCMPR] no layouts left to consolidate");
            break;
        };
        let pre_consolidation_sol = best_sol.clone();

        match consolidate_layout(sep, target, used_width, term, config) {
            Some(new_sol) => {
                best_sol = new_sol;
                sep.rollback(&best_sol, None);
                report_stats(instance, sep, &format!("round {round}, after consolidation"));
                sol_listener.report(ReportType::CmprFeas, &best_sol, instance);
                progress = true;
            }
            None => {
                // Any failed attempt leaves the separator in an undefined state: restore the last
                // known-good solution (the pack-down result, or the exploration input if pack-down
                // was a no-op).
                best_sol = pre_consolidation_sol;
                sep.rollback(&best_sol, None);
                info!("[BPCMPR] no consolidation improvement, keeping the pre-consolidation solution");
            }
        }

        if !progress {
            info!("[BPCMPR] round {round} changed nothing, compression converged");
            break;
        }
    }

    best_sol
}

/// **Pack-down**: repeatedly move items out of the least dense bin into the other (denser) ones.
///
/// This is the cross-layout counterpart of the (intra-layout) separation loop, and the step that
/// actually makes the leftover material *one* large piece:
///
/// ```text
/// loop while time remains:
///   L = least dense open layout
///   for every item of L, largest area first:
///     for every other layout M, most free area first:
///       snapshot; move the item L -> M at a random position; search its best position in M;
///       separate() with a short budget (this is what "makes room" for the newcomer);
///       total loss == 0 ?  accept  :  roll back and try the next M
///   stop when a full pass over L moved nothing
/// ```
///
/// Every accepted state is verified feasible (zero total loss over *all* layouts, `is_feasible()`
/// per layout and the full demand still placed), so the returned solution is always feasible.
/// If a bin empties out completely it auto-closes and the bin count — the primary objective —
/// drops as a side effect.
///
/// Returns the best (feasible) solution found **and how many items were moved across bins**; the
/// separator is left on that solution.
pub fn pack_down(
    sep: &mut BPSeparator,
    term: &impl Terminator,
    config: &BPCompressionConfig,
    sol_listener: &mut impl BPSolutionListener,
    instance: &BPInstance,
) -> (BPSolution, usize) {
    let mut best_sol = sep.prob.save();
    debug_assert!(sep.total_loss() == 0.0, "pack_down must start from a feasible solution");

    if sep.prob.layouts.len() < 2 {
        info!("[BPCMPR] only one bin, nothing to pack down");
        return (best_sol, 0);
    }

    // Short, cheap separations: a pack-down attempt is a *local* repair (one extra item in one
    // bin) and hundreds of them are made.
    let outer_config = sep.swap_config(config.pack_down_separator_config);

    let mut n_moved = 0usize;
    let start = Instant::now();

    'outer: loop {
        if term.kill() {
            break;
        }
        let Some(src) = sep.least_dense_layout() else { break };
        if sep.prob.layouts.len() < 2 {
            break;
        }

        // Items of the source, largest original area first (ties by PItemKey order → deterministic).
        let mut src_items = sep.prob.layouts[src].placed_items.keys().collect_vec();
        src_items.sort_by_key(|pk| {
            let item_id = sep.prob.layouts[src].placed_items[*pk].item_id;
            (Reverse(OrderedFloat(sep.instance.item(item_id).area())), *pk)
        });

        let mut moved_this_pass = false;
        for pk in src_items {
            if term.kill() {
                break 'outer;
            }
            // The source may have been closed by a successful move, or `pk` may be stale after an
            // accepted move (accepted moves never touch the *other* items of the source, but a
            // rollback restores keys, so guard anyway).
            if !sep.prob.layouts.contains_key(src) {
                break;
            }
            if !sep.prob.layouts[src].placed_items.contains_key(pk) {
                continue;
            }

            // Destinations: every other open layout, most free area first.
            let destinations = sep.prob.layouts.iter()
                .filter(|(lkey, _)| *lkey != src)
                .map(|(lkey, l)| (lkey, OrderedFloat(l.container.area() - l.placed_item_area(&sep.instance))))
                .sorted_by_key(|(lkey, free)| (Reverse(*free), *lkey))
                .map(|(lkey, _)| lkey)
                .collect_vec();

            for dst in destinations {
                if term.kill() {
                    break 'outer;
                }
                // A `destinations` lista a BELSO ciklus ELOTT keszult, de egy
                // sikeres athelyezes bezarhat egy tablat, a rollback pedig uj
                // kulcsokkal allitja vissza az allapotot. Ezert MINDHAROM
                // kulcsot ujra ellenorizni kell, kulonben a SlotMap
                // indexeles "invalid SlotMap key used" panickal all le.
                if !sep.prob.layouts.contains_key(src) {
                    break;
                }
                if !sep.prob.layouts.contains_key(dst) {
                    continue;
                }
                if !sep.prob.layouts[src].placed_items.contains_key(pk) {
                    break;
                }
                let snapshot = sep.save();
                let item_id = sep.prob.layouts[src].placed_items[pk].item_id;

                // 1. Move the item across at a random feasible position in `dst`.
                let Some((new_pk, src_closed)) = sep.transfer_item(src, pk, dst) else {
                    // The item does not fit in `dst` in any rotation; nothing was changed.
                    continue;
                };

                // 2. Give it its *best* position in `dst` (lowest collision loss).
                search_best_position(sep, dst, new_pk);

                // 3. Let the global separation loop make room for the newcomer. It may move items
                //    inside `dst` — and, since the loss is summed over all layouts, anywhere else.
                let sub_term = short_term(term, config.pack_down_move_time_limit);
                let (candidate, cts) = sep.separate(&sub_term);
                let total_loss: f32 = cts.values().map(|ct| ct.get_total_loss()).sum();

                if total_loss == 0.0 && {
                    sep.rollback(&candidate, Some(&cts));
                    solution_is_valid(sep)
                } {
                    let n_left = sep.prob.layouts.get(src).map_or(0, |l| l.placed_items.len());
                    info!("[BPCMPR] moved item {item_id} from bin {src:?} to bin {dst:?}: {n_left} items left");
                    if src_closed {
                        info!("[BPCMPR] bin {src:?} is now empty and was closed: cost is now {}", sep.prob.bin_cost());
                    }
                    best_sol = candidate;
                    n_moved += 1;
                    moved_this_pass = true;
                    sol_listener.report(ReportType::CmprFeas, &best_sol, instance);

                    if src_closed {
                        // The source is gone: restart from the (new) least dense bin.
                        continue 'outer;
                    }
                    break;
                }

                // Failed: restore the state from before this attempt and try the next destination.
                let (sol, cts) = snapshot;
                sep.rollback(&sol, Some(&cts));
                debug!("[BPCMPR] item {item_id} does not fit into bin {dst:?} (loss {}), rolling back",
                    crate::FMT().fmt2(total_loss));
            }
        }

        if !moved_this_pass {
            debug!("[BPCMPR] a full pass over bin {src:?} moved nothing, pack-down converged");
            break;
        }
    }

    sep.swap_config(outer_config);
    sep.rollback(&best_sol, None);
    info!("[BPCMPR] pack-down finished: {n_moved} item(s) moved across bins in {:.1}s, {} bin(s), cost {}",
        start.elapsed().as_secs_f32(), sep.prob.layouts.len(), sep.prob.bin_cost());

    (best_sol, n_moved)
}

/// Searches for the lowest-loss position of `pk` inside layout `lkey` and moves it there.
///
/// Uses exactly the same machinery as the separation workers — a [`SeparationEvaluator`] over the
/// destination layout and [`search_placement`] with the item's current placement as the reference —
/// so a freshly transferred item is not left at the random position [`BPSeparator::transfer_item`]
/// dropped it at. Returns the item's (possibly new) key.
fn search_best_position(sep: &mut BPSeparator, lkey: LayKey, pk: PItemKey) -> PItemKey {
    // The search borrows the layout and the tracker immutably while it needs an RNG mutably, so it
    // gets its own RNG seeded from the master stream (which keeps the whole step deterministic).
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(sep.rng.next_u64());

    let best_sample = {
        let layout = &sep.prob.layouts[lkey];
        let item = sep.instance.item(layout.placed_items[pk].item_id);
        let evaluator = SeparationEvaluator::new(layout, item, pk, &sep.trackers[lkey]);
        let (best_sample, _) = search_placement(
            layout, item, Some(pk), evaluator, sep.config.sample_config, &mut rng,
        );
        // Keep the item inside the bin (see `worker::clamp_to_container`).
        best_sample.map(|(dt, _)| clamp_to_container(dt, item, layout.container.outer_cd.bbox).0)
    };

    match best_sample {
        Some(dt) => sep.move_item(lkey, pk, dt),
        None => pk,
    }
}

/// A private terminator granting `ratio` of `term`'s remaining budget (falling back to `ratio` of
/// `fallback` when `term` has no deadline at all).
fn share_of(term: &impl Terminator, ratio: f32, fallback: Duration) -> BasicTerminator {
    let remaining = match term.timeout_at() {
        Some(deadline) => deadline.saturating_duration_since(Instant::now()),
        None => fallback,
    };
    let mut t = BasicTerminator::new();
    t.new_timeout(remaining.mul_f32(ratio.clamp(0.0, 1.0)));
    t
}

/// A private terminator granting `min(remaining budget, limit)`.
fn short_term(term: &impl Terminator, limit: Duration) -> BasicTerminator {
    let budget = match term.timeout_at() {
        Some(deadline) => deadline.saturating_duration_since(Instant::now()).min(limit),
        None => limit,
    };
    let mut t = BasicTerminator::new();
    t.new_timeout(budget);
    t
}

/// Full feasibility guard, matching the one the write-back uses: zero total loss, every layout
/// verified by jagua-rs' own CDE, and the complete demand still placed.
fn solution_is_valid(sep: &BPSeparator) -> bool {
    sep.total_loss() == 0.0
        && sep.prob.layouts.values().all(|l| l.is_feasible())
        && sep.prob.item_demand_qtys.iter().all(|&d| d == 0)
}

/// Logs the per-bin density and used width and returns them, sorted by density (least dense first).
///
/// The 'used width' is the largest `bbox.x_max` of the layout's placed items, expressed relative to
/// the container's bbox origin: the width of the sub-strip that actually holds material. The
/// difference to the container width is the width of the rectangular offcut that remains.
fn report_stats(instance: &BPInstance, sep: &BPSeparator, tag: &str) -> Vec<(LayKey, f32, f32)> {
    let mut stats = sep.prob.layouts.iter()
        .map(|(lkey, l)| {
            let bbox = l.container.outer_cd.bbox;
            let used_width = l.placed_items.values()
                .map(|pi| pi.shape.bbox.x_max)
                .fold(bbox.x_min, f32::max) - bbox.x_min;
            (lkey, used_width, l.density(instance))
        })
        .collect_vec();
    stats.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap().then(a.0.cmp(&b.0)));

    info!("[BPCMPR] ({tag}) {} bin(s), cost: {}, total dens: {:.3}%",
        sep.prob.layouts.len(), sep.prob.bin_cost(), sep.prob.density() * 100.0);
    for (lkey, used_width, density) in stats.iter() {
        let bbox = sep.prob.layouts[*lkey].container.outer_cd.bbox;
        info!("[BPCMPR] ({tag}) bin {:?}: dens {:.3}%, used width {:.3} / {:.3} ({} items)",
            lkey, density * 100.0, used_width, bbox.width(),
            sep.prob.layouts[*lkey].placed_items.len());
    }
    stats
}

/// Consolidates one layout by solving it as a strip packing problem.
///
/// Returns `Some(solution)` if the layout's content could be squeezed into a strictly narrower
/// strip *and* the resulting BPP layout is feasible; `None` otherwise (the caller must roll back).
fn consolidate_layout(
    sep: &mut BPSeparator,
    lkey: LayKey,
    used_width: f32,
    term: &impl Terminator,
    config: &BPCompressionConfig,
) -> Option<BPSolution> {
    let bbox = sep.prob.layouts[lkey].container.outer_cd.bbox;
    let n_items = sep.prob.layouts[lkey].placed_items.len();
    if n_items < 2 {
        debug!("[BPCMPR] layout {lkey:?} holds {n_items} item(s), nothing to consolidate");
        return None;
    }
    // Nothing to gain when the content already spans (practically) the whole bin.
    if used_width >= bbox.width() * 0.999 {
        info!("[BPCMPR] bin {lkey:?} is already full-width ({used_width:.3}), skipping consolidation");
        return None;
    }

    // --- 1. Lift the layout into a strip packing (sub)problem -------------------------------
    let (sp_instance, placements) = build_sp_subproblem(sep, lkey, bbox, used_width)?;

    let mut sp_prob = SPProblem::new(sp_instance.clone());
    for placement in placements.iter() {
        sp_prob.place_item(*placement);
    }
    debug_assert!(sp_prob.layout.is_feasible(), "the lifted strip packing problem must start feasible");

    // --- 2. Let the SPP exploration phase shrink the strip -----------------------------------
    // A private terminator so the sub-optimization cannot outlive the compression budget: it gets
    // whatever is left of it, capped by the configured per-attempt limit.
    let mut sub_term = BasicTerminator::new();
    let budget = match term.timeout_at() {
        Some(deadline) => deadline.saturating_duration_since(Instant::now())
            .min(config.consolidation_expl_cfg.time_limit),
        None => config.consolidation_expl_cfg.time_limit,
    };
    if budget.is_zero() {
        info!("[BPCMPR] no time budget left for consolidation");
        return None;
    }
    sub_term.new_timeout(budget);

    let sub_rng = Xoshiro256PlusPlus::seed_from_u64(sep.rng.next_u64());
    let mut sp_sep = Separator::new(
        sp_instance.clone(),
        sp_prob,
        sub_rng,
        config.consolidation_expl_cfg.separator_config,
    );
    info!("[BPCMPR] consolidating bin {lkey:?}: {n_items} items, strip {used_width:.3} x {:.3}, budget {:?}",
        bbox.height(), budget);

    let sp_sols = sp_exploration_phase(
        &sp_instance,
        &mut sp_sep,
        &mut DummySolListener,
        &sub_term,
        &config.consolidation_expl_cfg,
    );
    let sp_best = sp_sols.last().expect("the SPP exploration phase always returns a solution");
    let new_width = sp_best.strip_width();

    if new_width >= used_width * 0.999 {
        info!("[BPCMPR] consolidation of bin {lkey:?} did not narrow the content ({used_width:.3} -> {new_width:.3})");
        return None;
    }
    info!("[BPCMPR] consolidation of bin {lkey:?} narrowed the content: {used_width:.3} -> {new_width:.3}");

    // --- 3. Write the placements back into the BPP layout ------------------------------------
    write_back(sep, lkey, bbox, sp_best)
}

/// Builds the strip packing instance for `lkey`'s content plus the matching (translated) placements.
///
/// The item ids of an [`SPInstance`] must be `0..n` and consecutive, while the items in a bin are an
/// arbitrary subset of the BPP instance's items, so a *local* id space is used. `local_to_global`
/// (returned implicitly through `SPInstance::items`' order) maps them back: the `i`-th SPP item is
/// the BPP item whose original `Item::id` is preserved in [`write_back`] via the same ordering.
///
/// Returns `None` when the strip cannot be constructed (e.g. degenerate dimensions).
#[allow(clippy::type_complexity)]
fn build_sp_subproblem(
    sep: &BPSeparator,
    lkey: LayKey,
    bbox: Rect,
    used_width: f32,
) -> Option<(SPInstance, Vec<SPPlacement>)> {
    let layout = &sep.prob.layouts[lkey];

    // Global item id -> (local id, demand). Deterministic: driven by the SlotMap iteration order.
    let mut global_ids: Vec<usize> = vec![];
    let mut demands: Vec<usize> = vec![];
    // Item placements, translated so that the container bbox origin becomes (0, 0).
    let mut placements: Vec<SPPlacement> = vec![];

    for pi in layout.placed_items.values() {
        let local_id = match global_ids.iter().position(|id| *id == pi.item_id) {
            Some(idx) => {
                demands[idx] += 1;
                idx
            }
            None => {
                global_ids.push(pi.item_id);
                demands.push(1);
                global_ids.len() - 1
            }
        };
        let t = pi.d_transf;
        let d_transf = DTransformation::new(t.rotation(), (t.translation().0 - bbox.x_min, t.translation().1 - bbox.y_min));
        placements.push(SPPlacement { item_id: local_id, d_transf });
    }

    // Clone the items into the local id space (the shapes are behind `Arc`s, so this is cheap).
    let items = global_ids.iter().enumerate()
        .map(|(local_id, global_id)| {
            let mut item = sep.instance.item(*global_id).clone();
            item.id = local_id;
            (item, demands[local_id])
        })
        .collect_vec();

    // The strip mirrors the bin's *collision detection* outer shape bbox, which already has the
    // `min_item_separation` deflation baked in, hence `ShapeModifyConfig::default()` here: applying
    // the deflation a second time would shrink the usable area.
    let strip = Strip::new(
        bbox.height(),
        sep.prob.layouts[lkey].container.base_cde.config,
        Default::default(),
        // Start from the width the content currently occupies (plus a hair of slack so the initial
        // solution is strictly inside the strip).
        used_width * 1.0001,
    ).ok()?;

    Some((SPInstance::new(items, strip), placements))
}

/// Writes a strip packing solution back into BPP layout `lkey`.
///
/// The subtlety here is that [`BPProblem::remove_item`](jagua_rs::probs::bpp::entities::BPProblem::remove_item)
/// **auto-closes** a layout once its last item is removed: the layout key becomes invalid and the
/// bin is returned to stock. Rather than fighting that, this function embraces it:
///
/// 1. remember the layout's `bin_id` and remove *all* its items — the layout closes, the bin goes
///    back to stock, so the stock bookkeeping stays balanced;
/// 2. place the **first** new placement with `BPLayoutType::Closed { bin_id }`, which re-opens a
///    layout of the very same bin type under a **fresh** `LayKey` (this is fine: the function
///    returns a whole solution, and the caller reloads the separator from it);
/// 3. place the remaining ones with `BPLayoutType::Open(new_lkey)`.
///
/// Afterwards the trackers are rebuilt and the new layout is verified with `Layout::is_feasible()`.
/// If it is not feasible (float round-off during the coordinate translation could in principle push
/// an item over the container edge), `None` is returned and the caller rolls back to the input.
fn write_back(
    sep: &mut BPSeparator,
    lkey: LayKey,
    bbox: Rect,
    sp_sol: &jagua_rs::probs::spp::entities::SPSolution,
) -> Option<BPSolution> {
    // The SPP solution's item ids are local; map them back through the *global* ids, using exactly
    // the same (SlotMap-driven, deterministic) ordering `build_sp_subproblem` used. This is valid
    // because the layout has not been modified since.
    let mut global_ids: Vec<usize> = vec![];
    for pi in sep.prob.layouts[lkey].placed_items.values() {
        if !global_ids.contains(&pi.item_id) {
            global_ids.push(pi.item_id);
        }
    }

    // Target placements, translated back into the bin's coordinate system.
    let new_placements = sp_sol.layout_snapshot.placed_items.values()
        .map(|pi| {
            let t = pi.d_transf;
            let d_transf = DTransformation::new(
                t.rotation(),
                (t.translation().0 + bbox.x_min, t.translation().1 + bbox.y_min),
            );
            (global_ids[pi.item_id], d_transf)
        })
        .collect_vec();

    let old_pks = sep.prob.layouts[lkey].placed_items.keys().collect_vec();
    if new_placements.len() != old_pks.len() {
        warn!("[BPCMPR] item count mismatch ({} vs {}), aborting write-back", new_placements.len(), old_pks.len());
        return None;
    }
    let bin_id = sep.prob.layouts[lkey].container.id;

    // 1. Empty the layout. The removal of the last item closes it and returns the bin to stock.
    for pk in old_pks {
        sep.prob.remove_item(lkey, pk);
    }
    debug_assert!(!sep.prob.layouts.contains_key(lkey), "the layout should have been auto-closed");
    sep.trackers.remove(lkey);
    debug_assert!(sep.prob.bin_stock_qtys[bin_id] > 0, "the bin must have returned to stock");

    // 2./3. Re-open a layout of the same bin type and fill it with the new placements.
    let mut new_lkey = None;
    for (item_id, d_transf) in new_placements {
        let layout_id = match new_lkey {
            None => BPLayoutType::Closed { bin_id },
            Some(l) => BPLayoutType::Open(l),
        };
        let (l, _) = sep.prob.place_item(BPPlacement { layout_id, item_id, d_transf });
        new_lkey = Some(l);
    }
    let new_lkey = new_lkey.expect("at least one item was written back");

    // 4. Verify. A single infeasible layout invalidates the whole write-back.
    sep.rebuild_trackers();
    if !sep.prob.layouts[new_lkey].is_feasible() || sep.total_loss() > 0.0 {
        warn!("[BPCMPR] write-back produced an infeasible layout (loss: {}), rolling back", sep.total_loss());
        return None;
    }
    debug_assert!(sep.prob.layouts.values().all(|l| l.is_feasible()));
    debug_assert!(sep.prob.item_demand_qtys.iter().all(|&d| d == 0), "all demand must still be placed");

    Some(sep.prob.save())
}
