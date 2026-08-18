use crate::config::*;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::optimizer::sheets::{apply_sheet_walls_opt, log_sheet_report, widen_for_walls};
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::info;
use rand::{Rng, SeedableRng};
use std::time::Duration;
use rand::rngs::Xoshiro256PlusPlus;

pub mod lbf;
pub mod separator;
pub mod worker;
pub mod explore;
pub mod compress;
pub mod bpp;
pub mod sheets;

///Algorithm 11 from https://doi.org/10.48550/arXiv.2509.13329
pub fn optimize(
    instance: SPInstance,
    mut rng: Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    initial_solution: Option<&SPSolution>
) -> SPSolution {
    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.next_u64());
    
    // First build an initial solution if none is provided
    let start_prob = match initial_solution {
        None => {
            let builder = LBFBuilder::new_with_sheet(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG, expl_config.sheet)
                .construct();
            builder.prob
        }
        Some(init_sol) => {
            info!("[OPT] warm starting from provided initial solution");
            let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone());
            apply_sheet_walls_opt(&mut prob, expl_config.sheet.as_ref());
            prob.restore(init_sol);
            // `restore` may have rebuilt the layout from a (wall-less) snapshot, so re-apply.
            apply_sheet_walls_opt(&mut prob, expl_config.sheet.as_ref());
            if let Some(sheet) = expl_config.sheet.as_ref() {
                // The imported solution knows nothing about the sheet walls, so items may straddle
                // them. Widening the strip until the *same* layout has a whole extra sheet's worth
                // of slack gives the separator room to push those items off the walls; the
                // exploration phase then shrinks the width back down as usual.
                widen_for_walls(&mut prob, sheet);
            }
            prob
        }
    };

    // Begin by executing the exploration phase
    terminator.new_timeout(expl_config.time_limit);
    let mut expl_separator = Separator::new_with_sheet(instance.clone(), start_prob, next_rng(), expl_config.separator_config, expl_config.sheet);
    let solutions = exploration_phase(
        &instance,
        &mut expl_separator,
        sol_listener,
        terminator,
        expl_config,
    );
    let final_explore_sol = solutions.last().unwrap().clone();
    if let Some(sheet) = expl_config.sheet.as_ref() {
        log_sheet_report("EXPL", &final_explore_sol, &instance, sheet);
    }

    // Start the compression phase from the final solution from the exploration phase
    terminator.new_timeout(cmpr_config.time_limit);
    let mut cmpr_separator = Separator::new_with_sheet(expl_separator.instance, expl_separator.prob, next_rng(), cmpr_config.separator_config, cmpr_config.sheet);
    let cmpr_sol = compression_phase(
        &instance,
        &mut cmpr_separator,
        &final_explore_sol,
        sol_listener,
        terminator,
        cmpr_config,
    );

    if let Some(sheet) = cmpr_config.sheet.as_ref() {
        log_sheet_report("CMPR", &cmpr_sol, &instance, sheet);
    }

    sol_listener.report(ReportType::Final, &cmpr_sol, &instance);

    // Return the final compressed solution
    cmpr_sol
}