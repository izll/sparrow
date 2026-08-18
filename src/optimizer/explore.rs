use crate::config::{ExplorationConfig, SheetConfig};
use crate::optimizer::separator::{Separator, SeparatorConfig};
use crate::optimizer::sheets::{n_sheets, required_density_for, rollback_to_width, try_drop_sheet};
use crate::sample::uniform_sampler::convert_sample_to_closest_feasible;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use crate::FMT;
use float_cmp::approx_eq;
use itertools::Itertools;
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Instance, Layout, PItemKey};
use jagua_rs::geometry::geo_traits::CollidesWith;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::{debug, info, warn};
use ordered_float::OrderedFloat;
use rand::prelude::{Distribution, IteratorRandom};
use rand_distr::Normal;
use slotmap::SecondaryMap;
use std::cmp::Reverse;

/// Algorithm 12 from https://doi.org/10.48550/arXiv.2509.13329
pub fn exploration_phase(instance: &SPInstance, sep: &mut Separator, sol_listener: &mut impl SolutionListener, term: &impl Terminator, config: &ExplorationConfig) -> Vec<SPSolution> {
    let mut current_width = sep.prob.strip_width();
    let mut best_width = current_width;

    // The exploration phase assumes it starts from a feasible (collision-free) layout: it is recorded as the first
    // feasible solution without being separated. Callers that build the start themselves (warm starts, sheet-wall
    // installation) must guarantee this; in debug builds we verify it.
    debug_assert!(sep.ct.get_total_loss() == 0.0, "[EXPL] exploration must start from a feasible layout (loss: {})", sep.ct.get_total_loss());
    let mut feasible_sols = vec![sep.prob.save()];

    sol_listener.report(ReportType::ExplFeas, &feasible_sols[0], instance);
    info!("[EXPL] starting optimization with initial width: {:.3} ({:.3}%)",current_width,sep.prob.density() * 100.0);

    let mut infeas_sol_pool: Vec<(SPSolution, f32)> = vec![];

    // --- Phase 8: sheet-drop bookkeeping (walled mode only) -----------------------------------
    // How many drop attempts have failed *at the current sheet count*. Once it reaches
    // `sheet_drop_strikes` the phase stops attempting drops and spends the rest of its budget on
    // the plain fine shrink, which is what minimises the last sheet's band. A successful drop (or
    // any change of the sheet count) resets it: a new sheet count is a genuinely new subproblem.
    let mut drop_strikes = 0usize;
    // The sheet count the strikes above were collected at.
    let mut drop_strikes_at: Option<usize> = None;
    // The tightest feasible solution seen so far: what a drop attempt starts from and rolls back to.
    let mut last_feasible: Option<SPSolution> = Some(feasible_sols[0].clone());
    // Consecutive failures of the *fine* shrink since the last feasible solution. The drop is only
    // attempted once this reaches `SHEET_DROP_AFTER_SHRINK_FAILURES`; see there.
    let mut shrink_failures = 0usize;
    // Fallback rounds since the strike budget ran out; refills it, see the use site.
    let mut drop_cooldown = 0usize;

    while !term.kill() {
        // Attempt to separate the current layout
        let local_best = sep.separate(term, sol_listener);
        let total_loss = local_best.1.get_total_loss();

        if total_loss == 0.0 {
            // If successfully separated
            if current_width < best_width {
                info!("[EXPL] feasible solution found! (width: {:.3}, dens: {:.3}%)",current_width,sep.prob.density() * 100.0);
                best_width = current_width;
                feasible_sols.push(local_best.0.clone());
                sol_listener.report(ReportType::ExplFeas, &local_best.0, instance);
            }

            // Phase 8: this is the tightest feasible layout seen so far, so it is the solution a
            // sheet-drop attempt must start from and roll back to. The attempt itself is *not* made
            // here — see the infeasible branch below for why.
            last_feasible = Some(local_best.0.clone());
            shrink_failures = 0;

            // Shrink the strip width and clear the infeasible solution pool
            let next_width = current_width * (1.0 - config.shrink_step);
            info!("[EXPL] shrinking strip by {}%: {:.3} -> {:.3}", config.shrink_step * 100.0, current_width, next_width);
            sep.change_strip_width(next_width, None);
            current_width = next_width;
            infeas_sol_pool.clear();
        } else {
            info!("[EXPL] unable to reach feasibility (width: {:.3}, dens: {:.3}%, min loss: {:.3})", current_width, sep.prob.density() * 100.0, FMT().fmt2(total_loss));
            sol_listener.report(ReportType::ExplInfeas, &local_best.0, instance);

            // Separation was not successful add it to the pool of infeasible solutions
            match infeas_sol_pool.binary_search_by(|(_, o)| o.partial_cmp(&total_loss).unwrap()) {
                Ok(idx) | Err(idx) => infeas_sol_pool.insert(idx, (local_best.0.clone(), total_loss)),
            }

            if infeas_sol_pool.len() >= config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
                info!("[EXPL] max consecutive failed attempts ({}), terminating", infeas_sol_pool.len());
                break;
            }

            // --- Phase 8: the sheet-drop move ---------------------------------------------
            //
            // The drop is attempted **here**, from the last feasible solution, once the fine
            // shrink has failed `SHEET_DROP_AFTER_SHRINK_FAILURES` times in a row — and not in
            // the feasible branch above. The reason is measured: a drop from a *loose* n-sheet
            // layout has to relocate the whole content of a nearly-full last sheet at once and
            // essentially never succeeds, whereas after the fine shrink has stalled the last
            // sheet holds as little as it ever will, which is the smallest possible relocation
            // and the only one with a real chance. Waiting also stops the (expensive) attempts
            // from eating the budget the fine shrink needs to get there.
            //
            // Policy (deterministic, documented in `docs/sheets.md`):
            //   * at most `sheet_drop_strikes` failed attempts per sheet count; afterwards the
            //     phase falls back to the fine shrink alone for the rest of its budget at that
            //     sheet count, so the last band still gets minimised;
            //   * consecutive attempts differ in the RNG state (a different random scatter),
            //     which is what makes a retry worth anything;
            //   * the area bound skips the attempt outright when `n-1` sheets could not hold the
            //     items even at `max_reduction_density`.
            shrink_failures += 1;
            if let Some(sheet) = config.sheet.as_ref()
                && shrink_failures >= SHEET_DROP_AFTER_SHRINK_FAILURES
                && let Some(feasible_sol) = last_feasible.clone()
            {
                shrink_failures = 0;
                let n_now = n_sheets(feasible_sol.strip_width(), sheet);
                if drop_strikes_at != Some(n_now) {
                    // Sheet count changed since the strikes were collected: fresh budget.
                    drop_strikes = 0;
                    drop_strikes_at = Some(n_now);
                }
                if drop_strikes >= sheet.sheet_drop_strikes {
                    // The strike budget is spent, so the phase has fallen back to the fine shrink
                    // for a while — but arriving *here* means that shrink has failed again, i.e. it
                    // is stuck too. Spending the rest of the budget repeating a move that provably
                    // cannot progress is worse than trying the drop once more from a disrupted
                    // layout, so the strikes are refilled after `SHEET_DROP_COOLDOWN` fallback
                    // rounds. The counter is what keeps the drops from crowding the shrink out.
                    drop_cooldown += 1;
                    if drop_cooldown >= SHEET_DROP_COOLDOWN {
                        debug!("[EXPL] [SHEET] refilling the sheet-drop strike budget: the fine \
                                shrink is stuck as well");
                        drop_strikes = 0;
                        drop_cooldown = 0;
                    }
                }
                // The attempt starts from the last feasible solution, not from the current
                // (infeasible) one.
                rollback_to_width(sep, &feasible_sol);
                if let Some(dropped_width) = attempt_sheet_drop(
                    instance, sep, sheet, &feasible_sol, &mut drop_strikes,
                    &mut infeas_sol_pool, sol_listener, term,
                ) {
                    // Success: the separator holds a feasible solution one sheet narrower. Record
                    // it and let the loop resume the fine shrink on the *new* last sheet.
                    best_width = dropped_width;
                    current_width = dropped_width;
                    let sol = sep.prob.save();
                    last_feasible = Some(sol.clone());
                    feasible_sols.push(sol.clone());
                    sol_listener.report(ReportType::ExplFeas, &sol, instance);
                    drop_strikes = 0;
                    drop_strikes_at = Some(n_sheets(current_width, sheet));
                    infeas_sol_pool.clear();
                    continue;
                }
                // Failed: `attempt_sheet_drop` restored `feasible_sol` at its original width, and
                // the pool rollback below re-establishes the width again, so nothing more is
                // needed here — the phase simply carries on with the normal disruption.
            }

            // Restore to a random solution from the pool, with better solutions having more chance to be selected
            let selected_sol = {
                // Sample a value in range [0.0, 1.0[ from a normal distribution
                let distribution = Normal::new(0.0, config.solution_pool_distribution_stddev).unwrap();
                let sample = distribution.sample(&mut sep.rng).abs().min(0.999);
                // Map it to an index in the infeasible solution pool (better solutions are at the start of the pool)
                let selected_idx = (sample * infeas_sol_pool.len() as f32) as usize;

                let (selected_sol, loss) = &infeas_sol_pool[selected_idx];
                info!("[EXPL] starting solution {}/{} selected from solution pool (l: {}) to disrupt", selected_idx, infeas_sol_pool.len(), FMT().fmt2(*loss));
                selected_sol
            };

            // Rollback to this solution and disrupt it.
            //
            // In the walled mode the pool may also hold the (narrower) result of a failed
            // sheet-drop attempt, so the rollback has to be width-aware; `rollback_to_width` is a
            // plain `Separator::rollback` whenever the widths already match, which is always the
            // case in plain strip-packing mode.
            rollback_to_width(sep, selected_sol);
            current_width = sep.prob.strip_width();
            disrupt_solution(sep, config);
        }
    }

    info!("[EXPL] finished, best feasible solution: width: {:.3} ({:.3}%)",best_width,feasible_sols.last().unwrap().density(instance) * 100.0);

    feasible_sols
}

/// After how many consecutive failures of the *fine* (0.1 %) shrink a sheet-drop is attempted.
///
/// The value is small on purpose: a couple of failures already mean the fine shrink has run out of
/// easy room at the current sheet count, which is exactly the state a drop wants to start from (the
/// last sheet holds as little as it is going to). Making it larger only delays the attempt into a
/// part of the budget where a failure can no longer be recovered from.
const SHEET_DROP_AFTER_SHRINK_FAILURES: usize = 2;

/// How many fallback rounds (fine shrink attempts that also failed) refill the sheet-drop strike
/// budget. Keeps a long budget from being spent entirely on a fine shrink that has demonstrably
/// stopped making progress, while still leaving the shrink the majority of the iterations.
const SHEET_DROP_COOLDOWN: usize = 6;

/// One **sheet-drop attempt** (phase 8), driven by the exploration phase from a feasible solution.
///
/// Returns `Some(new_width)` when the solution now fits into one sheet fewer (the separator is left
/// holding that feasible, narrower solution), and `None` otherwise — in which case `feasible_sol`
/// has been restored at its original width, so the caller can carry on with the fine shrink as if
/// nothing had happened.
///
/// The attempt is skipped (returning `None` immediately, without consuming a strike) when
/// * there is only one sheet — nothing to drop;
/// * the strike budget for this sheet count is exhausted;
/// * the **area bound** says `n-1` sheets could not hold the items even at
///   [`SheetConfig::max_reduction_density`].
///
/// A failed attempt costs one strike, and its (infeasible) result is added to the caller's pool
/// exactly like a failed fine shrink — the pool is what the disruption logic samples from.
#[allow(clippy::too_many_arguments)]
fn attempt_sheet_drop(
    instance: &SPInstance,
    sep: &mut Separator,
    sheet: &SheetConfig,
    feasible_sol: &SPSolution,
    drop_strikes: &mut usize,
    infeas_sol_pool: &mut Vec<(SPSolution, f32)>,
    sol_listener: &mut impl SolutionListener,
    term: &impl Terminator,
) -> Option<f32> {
    let width = sep.prob.strip_width();
    let n = n_sheets(width, sheet);
    if n < 2 {
        return None;
    }
    if *drop_strikes >= sheet.sheet_drop_strikes {
        return None;
    }
    // Area bound: can `n-1` sheets hold all the items at a density that is reachable at all?
    let required = required_density_for(sep, n - 1, sheet);
    if required > sheet.max_reduction_density {
        debug!("[EXPL] sheet drop to {} sheet(s) would need {:.1}% density > cap {:.1}%, skipping",
            n - 1, required * 100.0, sheet.max_reduction_density * 100.0);
        return None;
    }

    let (succeeded, attempt, loss) = try_drop_sheet(sep, sheet, term, sol_listener);
    if succeeded {
        let new_width = sep.prob.strip_width();
        // Leave the separator holding the *feasible* result of the attempt.
        sep.rollback(&attempt, None);
        info!("[EXPL] [SHEET] sheet drop succeeded: {} -> {} sheet(s) (width {:.3} -> {:.3}, dens {:.3}%)",
            n, n_sheets(new_width, sheet), width, new_width, sep.prob.density() * 100.0);
        return Some(new_width);
    }

    *drop_strikes += 1;
    info!("[EXPL] [SHEET] sheet drop to {} sheet(s) failed (min loss: {}), strike {}/{}",
        n - 1, FMT().fmt2(loss), drop_strikes, sheet.sheet_drop_strikes);
    sol_listener.report(ReportType::ExplInfeas, &attempt, instance);

    // Keep the failed attempt in the pool, like any other infeasible result. It is at a *different*
    // (narrower) width than the pool's other entries, which the pool itself does not care about —
    // but the caller's rollback does, so `rollback_to_width` is used there. Pooling it is still
    // worthwhile: the pool is only consulted when the *fine* shrink fails, and at that point a
    // narrow-but-nearly-separated layout is a legitimate (if aggressive) restart point.
    match infeas_sol_pool.binary_search_by(|(_, o)| o.partial_cmp(&loss).unwrap()) {
        Ok(idx) | Err(idx) => infeas_sol_pool.insert(idx, (attempt, loss)),
    }

    // Roll back to the feasible solution at its original width and let the fine shrink resume.
    rollback_to_width(sep, feasible_sol);
    None
}

fn disrupt_solution(sep: &mut Separator, config: &ExplorationConfig) {
    if sep.prob.layout.placed_items.len() < 2 {
        warn!("[DSRP] cannot disrupt solution with less than 2 items");
        return;
    }

    // The general idea is to disrupt a solution by swapping two 'large' items in the layout.
    // 'Large' items are those whose convex hull area falls within a certain top percentile
    // of the total convex hull area of all items in the layout.

    // Step 1: Define what constitutes a 'large' item.

    // Calculate the total convex hull area of all items, considering quantities.
    let total_convex_hull_area: f32 = sep
        .prob
        .instance
        .items
        .iter()
        .map(|(item, quantity)| item.shape_cd.surrogate().convex_hull_area * (*quantity as f32))
        .sum();

    let cutoff_threshold_area = total_convex_hull_area * config.large_item_ch_area_cutoff_percentile;

    // Sort items by convex hull area in descending order.
    let sorted_items_by_ch_area = sep
        .prob
        .instance
        .items
        .iter()
        .sorted_by_key(|(item, _)| Reverse(OrderedFloat(item.shape_cd.surrogate().convex_hull_area)))
        .peekable();

    let mut cumulative_ch_area = 0.0;
    let mut ch_area_cutoff = 0.0;

    // Iterate through items, accumulating their convex hull areas until the cumulative sum
    // exceeds the cutoff_threshold_area. The convex hull area of the item that causes
    // this excess becomes the ch_area_cutoff.
    for (item, quantity) in sorted_items_by_ch_area {
        let item_ch_area = item.shape_cd.surrogate().convex_hull_area;
        cumulative_ch_area += item_ch_area * (*quantity as f32);
        if cumulative_ch_area > cutoff_threshold_area {
            ch_area_cutoff = item_ch_area;
            debug!("[DSRP] cutoff ch area: {}, for item id: {}, bbox: {:?}",ch_area_cutoff, item.id, item.shape_cd.bbox);
            break;
        }
    }

    // Step 2: Select two 'large' items and 'swap' them.

    let large_items = sep.prob.layout.placed_items.iter()
        .filter(|(_, pi)| pi.shape.surrogate().convex_hull_area >= ch_area_cutoff);

    //Choose a first item with a large enough convex hull
    let (pk1, pi1) = large_items.clone().choose(&mut sep.rng).expect("[DSRP] failed to choose first item");

    //Choose a second item with a large enough convex hull and different enough from the first.
    //If no such item is found, choose a random one.
    let (pk2, pi2) = large_items.clone()
        .filter(|(_, pi)|
            // Ensure the second item is different from the first
            !approx_eq!(f32, pi.shape.area,pi1.shape.area, epsilon = pi1.shape.area * 0.01) &&
                !approx_eq!(f32, pi.shape.diameter, pi1.shape.diameter, epsilon = pi1.shape.diameter * 0.01)
        )
        .choose(&mut sep.rng)
        .or_else(|| {
            sep.prob.layout.placed_items.iter()
                .filter(|(pk, _)| *pk != pk1) // Ensure the second item is not the same as the first
                .choose(&mut sep.rng)
        }) // As a fallback, choose any item
        .expect("[EXPL] failed to choose second item for disruption");

    // Step 3: Swap the two items' positions in the layout.

    let dt1_old = pi1.d_transf;
    let dt2_old = pi2.d_transf;

    // Make sure the swaps do not violate feasibility (rotation).
    let dt1_new = convert_sample_to_closest_feasible(dt2_old, sep.prob.instance.item(pi1.item_id));
    let dt2_new = convert_sample_to_closest_feasible(dt1_old, sep.prob.instance.item(pi2.item_id));

    info!("[EXPL] disrupting by swapping two large items (id: {} <-> {})", pi1.item_id, pi2.item_id);

    let pk1 = sep.move_item(pk1, dt1_new);
    let pk2 = sep.move_item(pk2, dt2_new);


    // Step 4: Move all items that are practically contained by one of the swapped items to the "empty space" created by the moved item.
    //         This is particularly important when huge items are swapped with smaller items. 
    //         The huge item will create a large empty space and many of the items which previously 
    //         surrounded the smaller one will be contained by the huge one.
    {
        // transformation to convert the contained items' position (relative to the old and new positions of the swapped items)
        let converting_transformation = dt1_new.compose().inverse()
            .transform(&dt1_old.compose());

        for c1_pk in practically_contained_items(&sep.prob.layout, pk1).into_iter().filter(|c1_pk| *c1_pk != pk2) {
            let c1_pi = &sep.prob.layout.placed_items[c1_pk];

            let new_dt = c1_pi.d_transf
                .compose()
                .transform(&converting_transformation)
                .decompose();

            //Ensure the sure the new position is feasible
            let new_feasible_dt = convert_sample_to_closest_feasible(new_dt, sep.prob.instance.item(c1_pi.item_id));
            sep.move_item(c1_pk, new_feasible_dt);
        }
    }

    // Do the same for the second item, but using the second transformation
    {
        let converting_transformation = dt2_new.compose().inverse()
            .transform(&dt2_old.compose());

        for c2_pk in practically_contained_items(&sep.prob.layout, pk2).into_iter().filter(|c2_pk| *c2_pk != pk1) {
            let c2_pi = &sep.prob.layout.placed_items[c2_pk];
            let new_dt = c2_pi.d_transf
                .compose()
                .transform(&converting_transformation)
                .decompose();

            //make sure the new position is feasible
            let new_feasible_dt = convert_sample_to_closest_feasible(new_dt, sep.prob.instance.item(c2_pi.item_id));
            sep.move_item(c2_pk, new_feasible_dt);
        }
    }
}

/// Collects all items which point of inaccessibility (POI) is contained by pk_c's shape.
fn practically_contained_items(layout: &Layout, pk_c: PItemKey) -> Vec<PItemKey> {
    let pi_c = &layout.placed_items[pk_c];
    // Detect all collisions with the item pk_c's shape.
    let mut collector = SecondaryMap::new();
    layout.cde().collect_poly_collisions(&pi_c.shape, &mut collector);

    // Filter out the items that have their POI contained by pk_c's shape.
    collector.iter()
        .filter_map(|(_,he)| {
            match he {
                HazardEntity::PlacedItem { pk, .. } => Some(*pk),
                _ => None
            }
        })
        .filter(|pk| *pk != pk_c) // Ensure we don't include the item itself
        .filter(|pk| {
            // Check if the POI of the item is contained by pk_c's shape
            let poi = layout.placed_items[*pk].shape.poi;
            pi_c.shape.collides_with(&poi.center)
        })
        .collect_vec()
}