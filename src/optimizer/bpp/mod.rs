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
//! * [`lbf::BPLBFBuilder`] — constructive first solution,
//! * [`separator::BPSeparator`] — the separation loop (SPP Algorithm 9 over all layouts),
//! * [`worker::BPSeparatorWorker`] — the parallel move workers (SPP Algorithm 5 per layout),
//! * [`explore::exploration_phase`] — the bin-count reduction loop (primary objective),
//! * [`compress::compression_phase`] — remainder consolidation (secondary objective),
//! * [`optimize_bpp`] — the orchestration tying them together.

pub mod compress;
pub mod explore;
pub mod lbf;
pub mod separator;
pub mod worker;

#[doc(inline)]
pub use lbf::BPLBFBuilder;
#[doc(inline)]
pub use separator::{BPSeparator, BPSnapshot};
#[doc(inline)]
pub use worker::BPSeparatorWorker;

use crate::config::BPConfig;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::bpp::compress::compression_phase;
use crate::optimizer::bpp::explore::exploration_phase;
use crate::util::bpp_io::BPSolutionListener;
use crate::util::listener::ReportType;
use crate::util::terminator::Terminator;
use jagua_rs::probs::bpp::entities::{BPInstance, BPProblem, BPSolution};
use log::info;
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
        None => {
            let builder = BPLBFBuilder::new(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG)
                .construct()
                .expect("[BPOPT] failed to construct an initial solution");
            info!("[BPOPT] LBF start: {} bin(s), cost: {}, dens: {:.3}%",
                builder.prob.layouts.len(), builder.prob.bin_cost(), builder.prob.density() * 100.0);
            builder.prob
        }
        Some(init_sol) => {
            info!("[BPOPT] warm starting from provided initial solution");
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
    terminator.new_timeout(config.cmpr_cfg.time_limit);
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
