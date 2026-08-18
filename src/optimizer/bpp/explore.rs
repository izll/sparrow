//! Bin-count reduction loop: the BPP counterpart of [`crate::optimizer::explore`].
//!
//! Where the strip packing exploration phase shrinks a *continuous* objective (the strip width) in
//! small steps, the bin packing exploration phase attacks a *discrete* one: it repeatedly tries to
//! eliminate one whole bin. Every attempt consists of
//!
//! 1. picking a target bin (the least dense open layout, varied across retries),
//! 2. [`BPSeparator::close_bin_and_scatter`] — removing the bin and scattering its items over the
//!    remaining ones, which introduces overlap,
//! 3. [`BPSeparator::separate`] — trying to resolve that overlap.
//!
//! If the overlap is fully resolved, a strictly better (one bin fewer) feasible solution has been
//! found and it becomes the new best. If not, the attempt is recorded in a pool of infeasible
//! solutions and the search rolls back to a pooled solution (or to the best one) and *disrupts* it
//! before retrying, exactly like the SPP version does.

use crate::FMT;
use crate::config::{BPExplorationConfig, STAGNATION_MIN_IMPROVEMENT};
use crate::optimizer::bpp::separator::BPSeparator;
use crate::optimizer::bpp::worker::clamp_to_container;
use crate::util::bpp_io::BPSolutionListener;
use crate::util::listener::ReportType;
use crate::sample::uniform_sampler::convert_sample_to_closest_feasible;
use crate::util::terminator::Terminator;
use float_cmp::approx_eq;
use itertools::Itertools;
use jagua_rs::Instant;
use jagua_rs::collision_detection::hazards::HazardEntity;
use jagua_rs::entities::{Instance, Layout, PItemKey};
use jagua_rs::geometry::geo_traits::CollidesWith;
use jagua_rs::probs::bpp::entities::{BPInstance, BPSolution, LayKey};
use log::{debug, info, warn};
use ordered_float::OrderedFloat;
use rand::prelude::{Distribution, IndexedRandom, IteratorRandom};
use rand_distr::Normal;
use slotmap::SecondaryMap;
use std::cmp::Reverse;

/// BPP counterpart of [`crate::optimizer::explore::exploration_phase`] (Algorithm 12 from
/// <https://doi.org/10.48550/arXiv.2509.13329>), minimising the **number (cost) of bins**.
///
/// Returns every feasible solution found, in improving order; the **last one is the best**
/// (fewest bins). The first element is always the solution the separator started from, so the
/// returned vector is never empty.
///
/// The loop terminates when the terminator fires, when `max_conseq_failed_attempts` consecutive
/// attempts have failed, or when only a single bin is left (there is nothing left to eliminate).
pub fn exploration_phase(
    instance: &BPInstance,
    sep: &mut BPSeparator,
    sol_listener: &mut impl BPSolutionListener,
    term: &impl Terminator,
    config: &BPExplorationConfig,
) -> Vec<BPSolution> {
    let mut best = sep.prob.save();
    debug_assert!(sep.total_loss() == 0.0, "the exploration phase must start from a feasible solution");

    let mut feasible_sols = vec![best.clone()];
    sol_listener.report(ReportType::ExplFeas, &best, instance);

    let mut best_cost = best.cost(instance);
    info!("[BPEXPL] starting optimization with {} bin(s), cost: {}, dens: {:.3}%",
        sep.prob.layouts.len(), best_cost, sep.prob.density() * 100.0);

    // Pool of infeasible solutions of the *current* attempt series, sorted by total loss (ascending).
    let mut infeas_sol_pool: Vec<(BPSolution, f32)> = vec![];
    // How many attempts in a row have failed; also selects which bin is targeted next.
    let mut n_failed_attempts = 0usize;
    // --- Stagnation tracking, per bin-count level ---------------------------------------------
    // `best_level_loss` is the lowest total loss any attempt at the *current* bin count reached;
    // `n_stagnant` counts the consecutive failures since it last improved meaningfully. Both reset
    // whenever the bin count changes (a new level is a new subproblem).
    let mut best_level_loss = f32::INFINITY;
    let mut n_stagnant = 0usize;
    // Why the loop stopped, for the one-line summary at the end.
    let mut stop_reason = "time limit";

    while !term.kill() {
        if sep.prob.layouts.len() < 2 {
            // A single bin cannot be eliminated (there would be nowhere to scatter its items to),
            // and it is trivially the optimum for the bin-count objective.
            info!("[BPEXPL] only one bin left, nothing to reduce");
            stop_reason = "single bin";
            break;
        }

        // --- Area bound ---------------------------------------------------------------------
        // A reduction to `n - 1` bins can only exist if the placed item area fits into the `n - 1`
        // *largest* remaining containers at a density the nesting can realistically reach. If it
        // cannot, every attempt below is doomed: return instead of spinning until the timeout.
        let required_density = required_density_for_reduction(sep);
        if required_density > config.max_reduction_density {
            info!("[BPEXPL] reduction to {} bins needs {:.1}% density > cap, skipping exploration",
                sep.prob.layouts.len() - 1, required_density * 100.0);
            stop_reason = "area bound";
            break;
        }

        // Vary the target across retries: the 1st, 2nd, ... least dense bin. This makes consecutive
        // attempts genuinely different subproblems instead of re-runs of the same one.
        let target_rank = if config.n_scatter_retries == 0 {
            0
        } else {
            n_failed_attempts % config.n_scatter_retries
        };
        let target = match sep.nth_least_dense_layout(target_rank).or_else(|| sep.least_dense_layout()) {
            Some(target) => target,
            None => break,
        };
        debug!("[BPEXPL] targeting the #{} least dense bin ({:?})", target_rank, target);

        if !sep.close_bin_and_scatter(target) {
            warn!("[BPEXPL] could not close a bin, stopping");
            stop_reason = "could not close a bin";
            break;
        }

        // Try to resolve the overlap the scattering introduced.
        let attempt_start = Instant::now();
        let (local_best, cts) = sep.separate(term);
        let total_loss: f32 = cts.values().map(|ct| ct.get_total_loss()).sum();
        let attempt_secs = attempt_start.elapsed().as_secs_f32();

        if total_loss == 0.0 {
            // Feasibility with one bin fewer!
            sep.rollback(&local_best, Some(&cts));
            debug_assert!(sep.prob.layouts.values().all(|l| l.is_feasible()),
                "a zero-loss solution must be feasible in every layout");

            let cost = local_best.cost(instance);
            info!("[BPEXPL] feasible solution found! ({} bins, cost: {} -> {}, dens: {:.3}%)",
                sep.prob.layouts.len(), best_cost, cost, sep.prob.density() * 100.0);

            best = local_best.clone();
            best_cost = cost;
            feasible_sols.push(local_best.clone());
            sol_listener.report(ReportType::ExplFeas, &local_best, instance);

            // Fresh start for the next bin: the pooled infeasible solutions belong to the previous
            // (now obsolete) bin count.
            infeas_sol_pool.clear();
            n_failed_attempts = 0;
            // A new bin count is a new level: the loss history of the old one says nothing here.
            best_level_loss = f32::INFINITY;
            n_stagnant = 0;
        } else {
            // Did this attempt improve the best loss seen at this bin-count level meaningfully?
            let improved = total_loss < best_level_loss * (1.0 - STAGNATION_MIN_IMPROVEMENT);
            if total_loss < best_level_loss {
                best_level_loss = total_loss;
            }
            match improved {
                true => n_stagnant = 0,
                false => n_stagnant += 1,
            }
            let strikes_left = config.max_conseq_failed_attempts
                .map(|max| max.saturating_sub(n_failed_attempts + 1));
            info!("[BPEXPL] unable to reach feasibility with {} bin(s) (dens: {:.3}%, min loss: {}, \
                   best at this level: {}, {:.1}s, strikes left: {}, stagnant: {}/{})",
                sep.prob.layouts.len(), sep.prob.density() * 100.0, FMT().fmt2(total_loss),
                FMT().fmt2(best_level_loss), attempt_secs,
                strikes_left.map_or("inf".to_string(), |n| n.to_string()),
                n_stagnant, config.stagnation_limit.map_or("inf".to_string(), |n| n.to_string()));
            sol_listener.report(ReportType::ExplInfeas, &local_best, instance);

            // Keep the attempt in the pool, sorted by loss (best = lowest loss = first).
            match infeas_sol_pool.binary_search_by(|(_, o)| o.partial_cmp(&total_loss).unwrap()) {
                Ok(idx) | Err(idx) => infeas_sol_pool.insert(idx, (local_best.clone(), total_loss)),
            }
            n_failed_attempts += 1;

            if n_failed_attempts >= config.max_conseq_failed_attempts.unwrap_or(usize::MAX) {
                info!("[BPEXPL] max consecutive failed attempts ({n_failed_attempts}), terminating");
                stop_reason = "strikes exhausted";
                break;
            }

            // Phase-wise stagnation stop: many failures in a row *and* none of them got the min
            // loss meaningfully lower. More time at this bin count would only repeat them, so the
            // remaining budget is worth more to the compression phase.
            if let Some(limit) = config.stagnation_limit
                && n_stagnant >= limit
            {
                info!("[BPEXPL] stagnated: {n_stagnant} attempt(s) without a >{:.0}% improvement of the \
                       min loss ({}) at {} bin(s), handing the rest of the budget to compression",
                    STAGNATION_MIN_IMPROVEMENT * 100.0, FMT().fmt2(best_level_loss), sep.prob.layouts.len());
                stop_reason = "stagnation";
                break;
            }

            // Roll back to a solution with the *original* bin count so the next attempt starts from
            // a feasible configuration again. Pooled solutions have one bin fewer and are infeasible,
            // so restarting from them would compound the infeasibility; instead we always return to
            // the best (feasible) solution and disrupt it.
            //
            // The pool is still used to decide *how strongly* to disrupt: a normally distributed
            // sample selects one of the pooled attempts, and its rank determines how many swaps are
            // applied (better pooled attempts => gentler disruption), mirroring the SPP heuristic of
            // preferring good solutions but occasionally exploring worse ones.
            let n_disruptions = {
                let distribution = Normal::new(0.0, config.solution_pool_distribution_stddev)
                    .expect("stddev must be finite and non-negative");
                let sample = distribution.sample(&mut sep.rng).abs().min(0.999);
                let selected_idx = (sample * infeas_sol_pool.len() as f32) as usize;
                debug!("[BPEXPL] pool pick {}/{} (l: {})", selected_idx, infeas_sol_pool.len(),
                    FMT().fmt2(infeas_sol_pool[selected_idx].1));
                1 + selected_idx
            };

            sep.rollback(&best, None);
            for _ in 0..n_disruptions {
                disrupt_solution(sep, config);
            }
        }
    }

    // Always leave the separator on the best solution found.
    sep.rollback(&best, None);
    info!("[BPEXPL] finished ({stop_reason}), best feasible solution: {} bin(s), cost: {}, dens: {:.3}%",
        sep.prob.layouts.len(), best_cost, best.density(instance) * 100.0);

    feasible_sols
}

/// The density the solution would have to reach to fit into **one bin fewer**.
///
/// Computed as `Σ placed item area / Σ container area of the (n-1) largest containers`. Using the
/// *largest* containers is the optimistic choice: it is the best case for the reduction, so a value
/// above 1.0 proves the reduction impossible, and a value above a (configured) realistic packing
/// density makes it hopeless in practice.
///
/// Returns `f32::INFINITY` when there is nothing to reduce (fewer than 2 layouts), which makes the
/// caller skip the exploration.
pub fn required_density_for_reduction(sep: &BPSeparator) -> f32 {
    if sep.prob.layouts.len() < 2 {
        return f32::INFINITY;
    }
    let total_item_area: f32 = sep.prob.layouts.values()
        .map(|l| l.placed_item_area(&sep.instance))
        .sum();

    // The (n-1) largest containers: sort descending by area (ties by LayKey order → deterministic)
    let remaining_area: f32 = sep.prob.layouts.iter()
        .map(|(lkey, l)| (lkey, OrderedFloat(l.container.area())))
        .sorted_by_key(|(lkey, area)| (Reverse(*area), *lkey))
        .take(sep.prob.layouts.len() - 1)
        .map(|(_, area)| area.0)
        .sum();

    match remaining_area > 0.0 {
        true => total_item_area / remaining_area,
        false => f32::INFINITY,
    }
}

/// Disrupts the current (feasible) solution by swapping two 'large' items.
///
/// Ported from [`crate::optimizer::explore`]'s `disrupt_solution`, generalised to multiple layouts:
/// a layout is picked at random (weighted implicitly by being sampled from the open layouts), and
/// two large items *inside that layout* are swapped, together with all items whose point of
/// inaccessibility is contained by them.
///
/// **Deviation from the task description**: only the intra-layout variant is implemented. A
/// cross-layout swap would need `remove_item` + `place_item(Open(other))`, which invalidates
/// `PItemKey`s in both layouts and can auto-close a layout when it holds a single item (silently
/// changing the bin count mid-disruption, which is exactly the objective being optimised). Doing
/// that safely requires a transactional helper that is out of scope for v1; the hook is
/// [`BPSeparator::move_item`], which would gain a destination-layout parameter.
fn disrupt_solution(sep: &mut BPSeparator, config: &BPExplorationConfig) {
    // Only layouts with at least 2 items can be disrupted by a swap.
    let candidate_layouts = sep.prob.layouts.iter()
        .filter(|(_, l)| l.placed_items.len() >= 2)
        .map(|(lkey, _)| lkey)
        .collect_vec();

    let lkey = match candidate_layouts.choose(&mut sep.rng) {
        Some(lkey) => *lkey,
        None => {
            warn!("[BPEXPL] cannot disrupt: no layout with 2 or more items");
            return;
        }
    };

    // Step 1: Determine what counts as a 'large' item (identical to the SPP version).
    let ch_area_cutoff = large_item_ch_area_cutoff(sep, config.large_item_ch_area_cutoff_percentile);

    // Step 2: Select two 'large' items in the chosen layout and swap them.
    let layout = &sep.prob.layouts[lkey];
    let large_items = layout.placed_items.iter()
        .filter(|(_, pi)| pi.shape.surrogate().convex_hull_area >= ch_area_cutoff);

    let (pk1, pi1) = match large_items.clone().choose(&mut sep.rng) {
        Some(v) => v,
        None => {
            warn!("[BPEXPL] cannot disrupt: no large item in the chosen layout");
            return;
        }
    };

    // Second item: large enough and sufficiently different from the first; any other item otherwise.
    let (pk2, pi2) = match large_items.clone()
        .filter(|(_, pi)|
            !approx_eq!(f32, pi.shape.area, pi1.shape.area, epsilon = pi1.shape.area * 0.01)
                && !approx_eq!(f32, pi.shape.diameter, pi1.shape.diameter, epsilon = pi1.shape.diameter * 0.01)
        )
        .choose(&mut sep.rng)
        .or_else(|| {
            layout.placed_items.iter()
                .filter(|(pk, _)| *pk != pk1)
                .choose(&mut sep.rng)
        }) {
        Some(v) => v,
        None => {
            warn!("[BPEXPL] cannot disrupt: no second item found");
            return;
        }
    };

    // Step 3: Swap the two items' positions (respecting the items' allowed rotations).
    let (dt1_old, dt2_old) = (pi1.d_transf, pi2.d_transf);
    let (item1_id, item2_id) = (pi1.item_id, pi2.item_id);

    let bbox = sep.prob.layouts[lkey].container.outer_cd.bbox;
    let (dt1_new, _) = clamp_to_container(
        convert_sample_to_closest_feasible(dt2_old, sep.instance.item(item1_id)),
        sep.instance.item(item1_id), bbox);
    let (dt2_new, _) = clamp_to_container(
        convert_sample_to_closest_feasible(dt1_old, sep.instance.item(item2_id)),
        sep.instance.item(item2_id), bbox);

    debug!("[BPEXPL] disrupting layout {lkey:?} by swapping two large items (id: {item1_id} <-> {item2_id})");

    let pk1 = sep.move_item(lkey, pk1, dt1_new);
    let pk2 = sep.move_item(lkey, pk2, dt2_new);

    // Step 4: Drag the items 'practically contained' by the swapped ones along, so that the empty
    //         space created by a huge item does not stay populated by the neighbours of the small one.
    move_contained_items(sep, lkey, pk1, pk2, dt1_old, dt1_new);
    move_contained_items(sep, lkey, pk2, pk1, dt2_old, dt2_new);
}

/// Moves every item whose POI is contained by `pk`'s (new) shape along with `pk`'s displacement.
/// `other_pk` is excluded (it is the other half of the swap and has already been moved).
fn move_contained_items(
    sep: &mut BPSeparator,
    lkey: LayKey,
    pk: PItemKey,
    other_pk: PItemKey,
    dt_old: jagua_rs::geometry::DTransformation,
    dt_new: jagua_rs::geometry::DTransformation,
) {
    // Transformation mapping a position relative to the item's old placement onto the new one.
    let converting_transformation = dt_new.compose().inverse().transform(&dt_old.compose());

    let contained = practically_contained_items(&sep.prob.layouts[lkey], pk).into_iter()
        .filter(|c_pk| *c_pk != other_pk)
        .collect_vec();

    for c_pk in contained {
        let c_pi = &sep.prob.layouts[lkey].placed_items[c_pk];
        let item_id = c_pi.item_id;
        let new_dt = c_pi.d_transf.compose().transform(&converting_transformation).decompose();
        // Ensure the new position respects the item's allowed rotations and stays inside the bin
        // (the composed transformation can easily land outside the container, which the collision
        // detection engine's quadtree cannot index).
        let new_feasible_dt = convert_sample_to_closest_feasible(new_dt, sep.instance.item(item_id));
        let bbox = sep.prob.layouts[lkey].container.outer_cd.bbox;
        let (new_feasible_dt, _) = clamp_to_container(new_feasible_dt, sep.instance.item(item_id), bbox);
        sep.move_item(lkey, c_pk, new_feasible_dt);
    }
}

/// The convex hull area above which an item counts as 'large'.
///
/// Identical to the SPP definition: sort the instance's items by convex hull area (descending) and
/// accumulate (area × demand) until `percentile` of the total is exceeded; the area of the item
/// that tips the balance is the cutoff.
fn large_item_ch_area_cutoff(sep: &BPSeparator, percentile: f32) -> f32 {
    let total_convex_hull_area: f32 = sep.instance.items.iter()
        .map(|(item, qty)| item.shape_cd.surrogate().convex_hull_area * (*qty as f32))
        .sum();
    let cutoff_threshold_area = total_convex_hull_area * percentile;

    let mut cumulative_ch_area = 0.0;
    let mut ch_area_cutoff = 0.0;
    for (item, qty) in sep.instance.items.iter()
        .sorted_by_key(|(item, _)| Reverse(OrderedFloat(item.shape_cd.surrogate().convex_hull_area)))
    {
        let item_ch_area = item.shape_cd.surrogate().convex_hull_area;
        cumulative_ch_area += item_ch_area * (*qty as f32);
        if cumulative_ch_area > cutoff_threshold_area {
            ch_area_cutoff = item_ch_area;
            debug!("[BPEXPL] cutoff ch area: {ch_area_cutoff}, for item id: {}", item.id);
            break;
        }
    }
    ch_area_cutoff
}

/// Collects all items whose point of inaccessibility (POI) is contained by `pk_c`'s shape.
/// Identical to the SPP helper of the same name, but takes the layout explicitly.
fn practically_contained_items(layout: &Layout, pk_c: PItemKey) -> Vec<PItemKey> {
    let pi_c = &layout.placed_items[pk_c];
    // Detect all collisions with item pk_c's shape.
    let mut collector = SecondaryMap::new();
    layout.cde().collect_poly_collisions(&pi_c.shape, &mut collector);

    collector.iter()
        .filter_map(|(_, he)| match he {
            HazardEntity::PlacedItem { pk, .. } => Some(*pk),
            _ => None,
        })
        .filter(|pk| *pk != pk_c)
        .filter(|pk| {
            let poi = layout.placed_items[*pk].shape.poi;
            pi_c.shape.collides_with(&poi.center)
        })
        .collect_vec()
}
