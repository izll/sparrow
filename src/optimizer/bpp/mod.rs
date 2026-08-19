//! Bin Packing Problem (BPP) pipeline.
//!
//! Mirrors the strip packing (SPP) pipeline in [`crate::optimizer`], but packs the items into
//! copies of fixed-size bins ([`jagua_rs::probs::bpp`]) instead of a single strip of variable width.
//!
//! The container-agnostic core (sampling, evaluation, collision quantification) is shared with the
//! SPP path: [`crate::sample::search::search_placement`], [`crate::eval::sep_evaluator::SeparationEvaluator`]
//! and [`crate::quantify::tracker::CollisionTracker`] are reused as-is. The only structural difference
//! is that a BPP solution consists of *multiple* layouts, so the separator keeps **one collision
//! tracker per open layout** and the "total loss" is the sum over all of them.
//!
//! The pipeline consists of:
//! * [`lbf::BPLBFBuilder`] — constructive first solution (sampling-based, good for irregular parts),
//! * [`shelf::BPShelfBuilder`] — deterministic bbox column/shelf constructor (good for rectangles),
//! * [`separator::BPSeparator`] — the separation loop (SPP Algorithm 9 over all layouts),
//! * [`worker::BPSeparatorWorker`] — the parallel move workers (SPP Algorithm 5 per layout),
//! * [`explore::exploration_phase`] — the bin-count reduction loop (primary objective),
//! * [`compress::compression_phase`] — remainder consolidation (secondary objective),
//! * [`optimize_bpp`] — the orchestration tying them together.

pub mod compress;
pub mod explore;
pub mod lbf;
pub mod separator;
pub mod shelf;
pub mod worker;

#[doc(inline)]
pub use lbf::BPLBFBuilder;
#[doc(inline)]
pub use separator::{BPSeparator, BPSnapshot};
#[doc(inline)]
pub use shelf::{BPShelfBuilder, Constructive};
#[doc(inline)]
pub use worker::BPSeparatorWorker;

use crate::config::BPConfig;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::bpp::compress::compression_phase;
use crate::optimizer::bpp::explore::exploration_phase;
use crate::util::bpp_io::BPSolutionListener;
use crate::util::listener::ReportType;
use crate::util::terminator::Terminator;
use jagua_rs::Instant;
use jagua_rs::entities::Instance;
use jagua_rs::probs::bpp::entities::{BPInstance, BPProblem, BPSolution};
use log::info;
use ordered_float::OrderedFloat;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, RngExt, SeedableRng};

/// The BPP counterpart of [`crate::optimizer::optimize`] (Algorithm 11 from
/// <https://doi.org/10.48550/arXiv.2509.13329>).
///
/// 1. Build an initial feasible solution with [`BPLBFBuilder`] (or warm-start from
///    `initial_solution`).
/// 2. Run the [`exploration_phase`] within `config.expl_cfg.time_limit`: minimise the number/cost of
///    bins by repeatedly eliminating the least dense one.
/// 3. Run the [`compression_phase`] within `config.cmpr_cfg.time_limit`: consolidate the remainder
///    of the least dense bin so the leftover material forms a single offcut.
///
/// The returned solution is guaranteed to be feasible (all layouts collision-free, all demand
/// placed), as both phases only ever accept verified feasible solutions.
///
/// # Panics
/// Panics if no initial solution can be constructed (e.g. an item fits in no bin, or the bin stock
/// is insufficient). The CLI is expected to validate the instance beforehand.
pub fn optimize_bpp(
    instance: BPInstance,
    mut rng: Xoshiro256PlusPlus,
    sol_listener: &mut impl BPSolutionListener,
    terminator: &mut impl Terminator,
    config: &BPConfig,
    initial_solution: Option<&BPSolution>,
) -> BPSolution {
    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.next_u64());

    // --- 1. Initial solution -----------------------------------------------------------------
    let start_prob = match initial_solution {
        None => build_initial_problem(&instance, next_rng(), config.constructive)
            .expect("[BPOPT] failed to construct an initial solution"),
        Some(init_sol) => {
            info!("[BPOPT] warm starting from provided initial solution");

            // `BPProblem::restore` trusts its input completely: it neither checks the snapshots for
            // collisions nor invents placements for missing demand. The exploration phase then
            // records the restored layout as its first feasible solution without testing it, so a
            // bad warm start is optimized and returned as if it were valid. `bpp_main` validates
            // this for the CLI, but `optimize_bpp` is a library entry point that anyone can call —
            // and in release the debug assertions that would have caught it are gone. Check here
            // too, at the point where the guarantee is actually needed.
            let n_placed: usize = init_sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
            assert_eq!(n_placed, instance.total_item_qty(),
                "[BPOPT] the warm start places {n_placed} item(s) but the instance demands {}; \
                 `restore` cannot invent the missing placements", instance.total_item_qty());
            for (lkey, ls) in init_sol.layout_snapshots.iter() {
                assert!(jagua_rs::entities::Layout::from_snapshot(ls).is_feasible(),
                    "[BPOPT] layout {lkey:?} of the warm start solution is not collision-free");
            }

            let mut prob = BPProblem::new(instance.clone());
            prob.restore(init_sol);
            prob
        }
    };

    // --- 2. Exploration: minimise the bin count ----------------------------------------------
    terminator.new_timeout(config.expl_cfg.time_limit);
    let mut expl_separator = BPSeparator::new(
        instance.clone(),
        start_prob,
        next_rng(),
        config.expl_cfg.separator_config,
    );
    let solutions = exploration_phase(
        &instance,
        &mut expl_separator,
        sol_listener,
        terminator,
        &config.expl_cfg,
    );
    let final_explore_sol = solutions.last()
        .expect("the exploration phase always returns at least one solution")
        .clone();
    info!("[BPOPT] exploration finished: cost {}, dens {:.3}%",
        final_explore_sol.cost(&instance), final_explore_sol.density(&instance) * 100.0);

    // --- 3. Compression: consolidate the remainder -------------------------------------------
    // The exploration phase can return long before its deadline (e.g. the area bound proves that no
    // further bin can be eliminated). Hand that unused time to the compression phase rather than
    // dropping it: the pack-down / consolidation steps always have more work to do.
    let remaining_expl_time = terminator.timeout_at()
        .map(|deadline| deadline.saturating_duration_since(Instant::now()))
        .unwrap_or_default();
    let cmpr_time = config.cmpr_cfg.time_limit + remaining_expl_time;
    if !remaining_expl_time.is_zero() {
        info!("[BPOPT] exploration returned {:.1}s before its deadline, handing that time to compression ({:.1}s -> {:.1}s)",
            remaining_expl_time.as_secs_f32(), config.cmpr_cfg.time_limit.as_secs_f32(), cmpr_time.as_secs_f32());
    }
    terminator.new_timeout(cmpr_time);
    let mut cmpr_separator = BPSeparator::new(
        expl_separator.instance,
        expl_separator.prob,
        next_rng(),
        config.cmpr_cfg.separator_config,
    );
    let cmpr_sol = compression_phase(
        &instance,
        &mut cmpr_separator,
        &final_explore_sol,
        sol_listener,
        terminator,
        &config.cmpr_cfg,
    );

    info!("[BPOPT] final solution: cost {}, dens {:.3}%",
        cmpr_sol.cost(&instance), cmpr_sol.density(&instance) * 100.0);
    sol_listener.report(ReportType::Final, &cmpr_sol, &instance);

    cmpr_sol
}

/// Builds the starting [`BPProblem`] with the configured constructive heuristic.
///
/// With [`Constructive::Best`] (the default) **both** constructors are run and the better result is
/// kept. "Better" is: fewer bins first, then the lower density of the least dense bin (the slack is
/// more concentrated, which is what the compression phase can exploit), and finally LBF — it is the
/// historical default, so a genuine tie must not change existing behaviour.
///
/// The two heuristics are complementary rather than redundant: [`BPLBFBuilder`] samples real
/// contours (so it nests irregular parts), [`BPShelfBuilder`] reasons about bounding boxes (so it
/// finds the guillotine structure a rectangular part set wants). Running both costs milliseconds
/// and removes the need to guess which family an instance belongs to.
///
/// Both results are logged, so the choice is always visible in the log.
pub fn build_initial_problem(
    instance: &BPInstance,
    rng: Xoshiro256PlusPlus,
    constructive: Constructive,
) -> anyhow::Result<BPProblem> {
    /// Bins, and the density of the least dense bin, of a candidate problem.
    fn stats(prob: &BPProblem, instance: &BPInstance) -> (usize, f32) {
        let min_dens = prob.layouts.values()
            .map(|l| l.density(instance))
            .fold(f32::INFINITY, f32::min);
        (prob.layouts.len(), min_dens)
    }

    let lbf = match constructive {
        Constructive::Shelf => None,
        _ => Some(BPLBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG).construct()?.prob),
    };
    let shelf = match constructive {
        Constructive::Lbf => None,
        // A failing shelf constructor must not sink a run that the LBF could have handled (e.g. an
        // item whose bbox fits in no bin although its contour does), so its error is only fatal
        // when it is the only candidate.
        _ => match BPShelfBuilder::new(instance.clone()).construct() {
            Ok(b) => Some(b.prob),
            Err(e) if constructive == Constructive::Shelf => return Err(e),
            Err(e) => {
                info!("[BPOPT] shelf constructor failed ({e}), falling back to LBF");
                None
            }
        },
    };

    match (lbf, shelf) {
        (Some(lbf), Some(shelf)) => {
            let (lbf_bins, lbf_dens) = stats(&lbf, instance);
            let (shelf_bins, shelf_dens) = stats(&shelf, instance);
            info!("[BPOPT] LBF start: {lbf_bins} bin(s), min-bin dens {:.3}% | shelf start: {shelf_bins} bin(s), min-bin dens {:.3}%",
                lbf_dens * 100.0, shelf_dens * 100.0);
            // Strictly better = fewer bins, or the same count with a sparser emptiest bin.
            let shelf_wins = (shelf_bins, OrderedFloat(shelf_dens)) < (lbf_bins, OrderedFloat(lbf_dens));
            match shelf_wins {
                true => {
                    info!("[BPOPT] starting from the shelf solution ({shelf_bins} bin(s))");
                    Ok(shelf)
                }
                false => {
                    info!("[BPOPT] starting from the LBF solution ({lbf_bins} bin(s))");
                    Ok(lbf)
                }
            }
        }
        (Some(lbf), None) => {
            let (bins, dens) = stats(&lbf, instance);
            info!("[BPOPT] LBF start: {bins} bin(s), min-bin dens {:.3}%", dens * 100.0);
            Ok(lbf)
        }
        (None, Some(shelf)) => {
            let (bins, dens) = stats(&shelf, instance);
            info!("[BPOPT] shelf start: {bins} bin(s), min-bin dens {:.3}%", dens * 100.0);
            Ok(shelf)
        }
        (None, None) => unreachable!("at least one constructor runs for every `Constructive` value"),
    }
}
