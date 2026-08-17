//! Remainder consolidation: the BPP counterpart of [`crate::optimizer::compress`].
//!
//! Once the exploration phase can no longer eliminate a bin, the primary objective (bin count) is
//! settled and the *secondary* objective takes over: make the leftover material as reusable as
//! possible by pushing the content of the least dense bin into one corner, so that what remains is
//! a single rectangular offcut instead of a scattered set of gaps.
//!
//! This is done by treating the least dense bin as a **strip packing** subproblem: its items are
//! lifted into a fresh [`SPInstance`] whose strip has the bin's height and width, seeded with the
//! current placements, and the existing SPP machinery ([`crate::optimizer::explore::exploration_phase`])
//! is asked to shrink that strip. If it succeeds, the resulting (narrower) placements are written
//! back into the BPP layout.
//!
//! Everything is guarded: the write-back only happens when the rebuilt layout is verified feasible
//! (`Layout::is_feasible`), otherwise the phase rolls back to the input solution. So in the worst
//! case this phase is a no-op that only *reports* the per-bin statistics.

use crate::config::BPCompressionConfig;
use crate::optimizer::bpp::separator::BPSeparator;
use crate::optimizer::explore::exploration_phase as sp_exploration_phase;
use crate::optimizer::separator::Separator;
use crate::util::bpp_io::BPSolutionListener;
use crate::util::listener::{DummySolListener, ReportType};
use crate::util::terminator::{BasicTerminator, Terminator};
use itertools::Itertools;
use jagua_rs::Instant;
use jagua_rs::entities::Instance;
use jagua_rs::geometry::DTransformation;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPSolution, LayKey};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, Strip};
use log::{debug, info, warn};
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, RngExt, SeedableRng};

/// BPP counterpart of [`crate::optimizer::compress::compression_phase`].
///
/// Takes the best solution of the exploration phase, reports the per-bin statistics and — when
/// `config.consolidate_remainder` is enabled — attempts to consolidate the content of the least
/// dense bin towards the left edge of that bin. Returns the improved solution, or `init_sol`
/// unchanged when no (verified feasible) improvement could be made.
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

    let stats = report_stats(instance, sep, "start");

    let Some((target, used_width, _)) = stats.first().copied() else {
        warn!("[BPCMPR] no layouts to compress");
        return best_sol;
    };

    if !config.consolidate_remainder {
        info!("[BPCMPR] remainder consolidation disabled, returning the exploration solution unchanged");
        return best_sol;
    }
    if term.kill() {
        info!("[BPCMPR] no time left for remainder consolidation");
        return best_sol;
    }

    match consolidate_layout(sep, target, used_width, term, config) {
        Some(new_sol) => {
            best_sol = new_sol;
            sep.rollback(&best_sol, None);
            report_stats(instance, sep, "after consolidation");
            sol_listener.report(ReportType::CmprFeas, &best_sol, instance);
        }
        None => {
            // Any failed attempt leaves the separator in an undefined state: restore the input.
            sep.rollback(init_sol, None);
            info!("[BPCMPR] no improvement, keeping the exploration solution");
        }
    }

    best_sol
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
