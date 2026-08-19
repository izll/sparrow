use crate::config::*;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::optimizer::sheets::{apply_sheet_walls_opt, install_walls_into_plain, log_sheet_report, n_sheets, rollback_to_width, widen_for_walls};
use crate::util::terminator::{BasicTerminator, Terminator};
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::{info, warn};
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
    let total_elapsed = jagua_rs::Instant::now();
    
    // Whether the walled run starts from a wall-less pre-pass instead of a walled LBF.
    let plain_first = matches!(expl_config.sheet, Some(sheet) if sheet.pipeline == SheetPipeline::PlainFirst)
        && initial_solution.is_none();

    // First build an initial solution if none is provided. Skipped entirely under the plain-first
    // pipeline, which builds its own (wall-less) start inside the pre-pass.
    let start_prob = match initial_solution {
        None if plain_first => None,
        None => Some({
            let builder = LBFBuilder::new_with_sheet(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG, expl_config.sheet)
                .construct();
            builder.prob
        }),
        Some(init_sol) => Some({
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
        }),
    };

    // --- Phase 8: the "plain strip first, walls after" pipeline -------------------------------
    //
    // A walled exploration fights the walls from its first iteration and reaches a far lower
    // density than the plain strip engine does with the same budget (iso7: 61.5 % vs 83.2 %). This
    // pipeline gives the plain engine the first `plain_first_ratio` of the exploration budget with
    // the walls switched *off*, then installs the walls into the result (see
    // `sheets::install_walls_into_plain`) and lets the separator repair the few items that end up
    // on a wall — a local repair instead of a global re-nest. Only then does the walled exploration
    // (fine shrink + sheet-drop) take over for the rest of the budget.
    //
    // Only reachable when a sheet config is present, so plain strip packing is untouched.
    let mut expl_separator = match (plain_first, start_prob) {
        (true, _) => {
            let sheet = expl_config.sheet.expect("plain_first implies a sheet config");
            let plain_budget = expl_config.time_limit.mul_f32(sheet.plain_first_ratio.clamp(0.0, 1.0));
            plain_first_prepass(&instance, &mut next_rng, sol_listener, terminator, expl_config, sheet, plain_budget)
        }
        (false, Some(start_prob)) => {
            let sep = Separator::new_with_sheet(instance.clone(), start_prob, next_rng(), expl_config.separator_config, expl_config.sheet);
            match (initial_solution, expl_config.sheet) {
                // A **walled warm start**: the imported solution knows nothing about the sheet
                // walls, so items straddling a boundary now overlap one. `widen_for_walls` above
                // only gives the separator *room* to repair that — it does not do the repair, and
                // nothing downstream verified it either. In release (no debug assertions) the
                // still-overlapping layout was then seeded straight into `exploration_phase` as a
                // feasible solution and exported: measured on swim -> 700 mm sheets, 85 wall
                // crossings in the output JSON.
                //
                // So repair it here, exactly like the plain-first pipeline does, and fall back to a
                // walled LBF (feasible by construction) when the repair does not reach zero loss.
                (Some(_), Some(sheet)) => repair_walled_warm_start(&instance, sep, &mut next_rng, sol_listener, expl_config, sheet),
                _ => sep,
            }
        }
        (false, None) => unreachable!("a non plain-first run always builds a starting problem"),
    };

    // Begin by executing the (walled) exploration phase with whatever budget is left
    terminator.new_timeout(expl_config.time_limit.saturating_sub(total_elapsed.elapsed()));
    let solutions = exploration_phase(
        &instance,
        &mut expl_separator,
        sol_listener,
        terminator,
        expl_config,
    );
    // `exploration_phase` returns an empty list only when it started from an infeasible layout and
    // never separated its way to a feasible one. Every start built here is repaired first (walled
    // warm start, plain-first pre-pass) or feasible by construction (LBF), so this should not
    // happen — but when it did, the old code **continued from the separator's current, overlapping
    // layout** with a warning, and that layout was then compressed, returned and exported at exit
    // `0`. Measured from an overlapping warm start with a zero-second budget: 1 185 179.5 mm² of
    // overlap in the output JSON.
    //
    // An infeasible layout is not a worse answer, it is a wrong one, and the caller has no way to
    // tell it apart from a good one. Repair it here with a full separation instead, and let the
    // caller's export gate (`util::verify`) refuse the run outright if even that fails.
    let final_explore_sol = match solutions.last() {
        Some(sol) => sol.clone(),
        None => {
            warn!("[OPT] the exploration phase never reached a feasible solution; \
                   attempting a final repair separation before handing over to the compression phase");
            let mut repair_term = BasicTerminator::new();
            repair_term.new_timeout(expl_config.time_limit.mul_f32(WALL_REPAIR_BUDGET_RATIO));
            let outer_cfg = expl_separator.config;
            expl_separator.config.strike_limit = outer_cfg.strike_limit.max(WALL_REPAIR_STRIKE_LIMIT);
            expl_separator.config.iter_no_imprv_limit = outer_cfg.iter_no_imprv_limit.max(WALL_REPAIR_ITER_NO_IMPRV_LIMIT);
            let (repaired, ct) = expl_separator.separate(&repair_term, sol_listener);
            expl_separator.rollback(&repaired, Some(&ct));
            expl_separator.config = outer_cfg;
            if ct.get_total_loss() != 0.0 {
                warn!("[OPT] the repair separation did not reach zero loss (min loss {}); the run \
                       cannot produce a feasible solution and will be rejected by the export gate",
                    crate::FMT().fmt2(ct.get_total_loss()));
            }
            repaired
        }
    };
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

    // **No `ReportType::Final` here.** The listener is what writes `output/final_<name>.svg`, and
    // this point is *before* the caller's export gate (`util::verify`). Reporting the final
    // solution from inside `optimize` therefore published the answer to disk before anything had
    // checked it: an overlapping warm start exited 1 and wrote no JSON, exactly as intended, and
    // still left a `final_<name>.svg` of the rejected layout sitting in `output/` — a file that
    // looks like the run's result and is not. The caller emits `Final` after its gate passes; see
    // `main.rs`.
    //
    // Return the final compressed solution
    cmpr_sol
}

/// Repairs a **walled warm start** before it reaches the exploration phase.
///
/// A solution imported with `-i some_solution.json --sheet-width W` was produced without any notion
/// of the sheet walls, so every part that happened to straddle a boundary now overlaps a wall.
/// [`widen_for_walls`] has already granted the strip a whole spare sheet's worth of slack for those
/// parts to move into, but the move itself has to be made by the separator.
///
/// Runs [`Separator::separate`] with the patient wall-repair settings; on success the separator is
/// returned holding the repaired (feasible) layout. If the repair cannot reach zero loss the warm
/// start is abandoned in favour of a **walled LBF** construction — feasible by construction, and a
/// correct answer from a worse start beats an infeasible one that gets exported as if it were fine.
pub fn repair_walled_warm_start(
    instance: &SPInstance,
    mut sep: Separator,
    next_rng: &mut impl FnMut() -> Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    expl_config: &ExplorationConfig,
    sheet: SheetConfig,
) -> Separator {
    if sep.ct.get_total_loss() == 0.0 {
        // Nothing straddled a wall (or the walls happen to fall into the gaps): keep it as is.
        info!("[OPT] [SHEET] the warm start is already clear of the walls, no repair needed");
        return sep;
    }

    info!("[OPT] [SHEET] the warm start overlaps the sheet walls (loss: {}); repairing before exploration",
        crate::FMT().fmt2(sep.ct.get_total_loss()));

    let outer_cfg = sep.config;
    sep.config.strike_limit = outer_cfg.strike_limit.max(WALL_REPAIR_STRIKE_LIMIT);
    sep.config.iter_no_imprv_limit = outer_cfg.iter_no_imprv_limit.max(WALL_REPAIR_ITER_NO_IMPRV_LIMIT);

    // The repair gets a bounded slice of the exploration budget, so a hopeless one cannot eat the
    // whole run; the LBF fallback below it is cheap.
    let mut repair_term = BasicTerminator::new();
    repair_term.new_timeout(expl_config.time_limit.mul_f32(WALL_REPAIR_BUDGET_RATIO));
    let (repaired, ct) = sep.separate(&repair_term, sol_listener);
    let loss = ct.get_total_loss();
    sep.rollback(&repaired, Some(&ct));
    sep.config = outer_cfg;

    if loss == 0.0 {
        info!("[OPT] [SHEET] warm-start wall repair succeeded: {} sheet(s) at {:.3}%, all items clear of the walls",
            n_sheets(sep.prob.strip_width(), &sheet), sep.prob.density() * 100.0);
        return sep;
    }

    warn!("[OPT] [SHEET] warm-start wall repair failed (min loss {}); discarding the warm start and \
           falling back to a walled LBF construction", crate::FMT().fmt2(loss));
    let builder = LBFBuilder::new_with_sheet(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG, Some(sheet)).construct();
    Separator::new_with_sheet(builder.instance, builder.prob, next_rng(), expl_config.separator_config, Some(sheet))
}

/// The wall-less pre-pass of [`SheetPipeline::PlainFirst`].
///
/// Runs LBF + [`exploration_phase`] with **no** sheet configuration at all for `plain_budget`, so
/// the strip engine works at full strength, then installs the walls into the best solution it found
/// ([`install_walls_into_plain`]) and hands back a walled separator for the main exploration phase.
///
/// The wall installation itself is not verified here: the layout it produces is expected to be
/// infeasible (the items that straddled a boundary now overlap a wall), and repairing it is exactly
/// what the walled `exploration_phase` does on its first `separate()` call. Should that repair
/// fail, the phase behaves as it always does — it never reports an infeasible solution as feasible,
/// it simply pools it and disrupts — and the width it starts from already has a whole sheet's worth
/// of slack in the last sheet, which is the same safety margin `widen_for_walls` provides for a
/// warm start.
fn plain_first_prepass(
    instance: &SPInstance,
    next_rng: &mut impl FnMut() -> Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    sheet: SheetConfig,
    plain_budget: Duration,
) -> Separator {
    info!("[OPT] [SHEET] plain-first pipeline: exploring for {:.0}s WITHOUT walls, then installing them",
        plain_budget.as_secs_f32());

    // 1. A wall-less exploration, with the sheet config stripped from every level.
    let plain_expl_config = ExplorationConfig { sheet: None, time_limit: plain_budget, ..*expl_config };
    let builder = LBFBuilder::new(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG).construct();
    let mut plain_sep = Separator::new(instance.clone(), builder.prob, next_rng(), plain_expl_config.separator_config);

    terminator.new_timeout(plain_budget);
    let plain_sols = exploration_phase(instance, &mut plain_sep, sol_listener, terminator, &plain_expl_config);
    // The pre-pass starts from a wall-less LBF construction, which is feasible by construction, so
    // the exploration phase always records at least that one solution.
    let plain_best = plain_sols.last()
        .expect("the wall-less pre-pass starts from a feasible LBF, so it always yields a solution")
        .clone();
    info!("[OPT] [SHEET] plain pre-pass finished: width {:.1} ({:.3}%) = {} sheet(s) of {}",
        plain_best.strip_width(), plain_best.density(instance) * 100.0,
        (plain_best.strip_width() / sheet.width).ceil().max(1.0) as usize, sheet.width);

    // 2. Turn that separator into a walled one and install the walls into the best solution.
    plain_sep.rollback(&plain_best, None);
    let mut sep = Separator::new_with_sheet(
        plain_sep.instance, plain_sep.prob, next_rng(), expl_config.separator_config, Some(sheet),
    );
    let plain_best_for_retry = plain_best.clone();
    let n = install_walls_into_plain(&mut sep, &sheet, 0.0);
    info!("[OPT] [SHEET] walls installed: {} sheet(s), strip width {:.1}", n, sep.prob.strip_width());
    debug_assert_eq!(n_sheets(sep.prob.strip_width(), &sheet), n);

    // 3. Repair the items that now overlap a wall, and *verify* the repair worked.
    //
    // This check is not optional. `exploration_phase` seeds its list of feasible solutions with
    // whatever layout it is handed, without testing it — so if the repair fails and the phase never
    // reaches feasibility on its own, it would report the unseparated, wall-straddling start as the
    // answer. Measured on iso7: the plain pre-pass hands over a beautiful 83 % layout that needs
    // 2 sheets, the repair cannot resolve it (min loss ~170), and the run would happily print
    // "2 sheets" for a layout whose first sheet is 2088 mm wide on a 1995 mm sheet.
    //
    // The repair therefore gets a bounded budget of its own, and on failure the run falls back to
    // one spare sheet's worth of slack — the same margin `widen_for_walls` gives a warm start —
    // which the walled exploration then shrinks away as usual.
    // The repair is a global job (every wall-crosser has to find a new home, and its neighbours have
    // to make room), so it gets the same patient separator the scatter repair uses: with the
    // exploration's default 3 strikes it gives up after ~3 s regardless of how much budget it has.
    let outer_cfg = sep.config;
    sep.config.strike_limit = outer_cfg.strike_limit.max(WALL_REPAIR_STRIKE_LIMIT);
    sep.config.iter_no_imprv_limit = outer_cfg.iter_no_imprv_limit.max(WALL_REPAIR_ITER_NO_IMPRV_LIMIT);

    // Try `n` sheets, then `n+1`, then `n+2`, ... until the repair actually reaches zero loss.
    // Handing an *infeasible* layout to `exploration_phase` is not an option: the phase seeds its
    // list of feasible solutions with whatever it is given, without testing it, so an unrepaired
    // start is reported as the final answer. Measured on iso6, that produced a "7 sheet" result
    // whose layout still had a total loss of 163 — i.e. overlapping parts. Every extra sheet adds a
    // whole sheet's worth of slack, so this terminates quickly in practice (and the exploration
    // shrinks the extra width straight back).
    let mut n_sheets_used = n;
    for attempt in 0..=WALL_REPAIR_MAX_EXTRA_SHEETS {
        if attempt > 0 {
            // Retry from the *plain* layout again, but cut into shorter chunks: every sheet then
            // keeps `slack` mm free at its right edge for the wall-crossers to slide into. This is
            // a genuinely different (easier) subproblem, unlike merely widening the strip, which
            // leaves every sheet exactly as tight as it already was.
            let slack = sheet.width * WALL_REPAIR_SLACK_STEP * attempt as f32;
            rollback_to_width(&mut sep, &plain_best_for_retry);
            n_sheets_used = install_walls_into_plain(&mut sep, &sheet, slack);
            info!("[OPT] [SHEET] retrying the wall repair with {:.0} mm slack per sheet -> {} sheet(s) (width {:.1})",
                slack, n_sheets_used, sep.prob.strip_width());
        }
        // Every retry gets an equal share of one bounded repair budget, so a run whose repair never
        // succeeds cannot eat the walled exploration's half of the time.
        let mut repair_term = BasicTerminator::new();
        repair_term.new_timeout(
            plain_budget.mul_f32(WALL_REPAIR_BUDGET_RATIO)
                .div_f32((WALL_REPAIR_MAX_EXTRA_SHEETS + 1) as f32),
        );
        let (repaired, ct) = sep.separate(&repair_term, sol_listener);
        if ct.get_total_loss() == 0.0 {
            sep.rollback(&repaired, Some(&ct));
            sep.config = outer_cfg;
            info!("[OPT] [SHEET] wall repair succeeded: {} sheet(s) at {:.3}%, layout feasible and \
                   all items clear of the walls", n_sheets_used, sep.prob.density() * 100.0);
            return sep;
        }
        sep.rollback(&repaired, Some(&ct));
        warn!("[OPT] [SHEET] wall repair at {} sheet(s) failed (min loss {})",
            n_sheets_used, crate::FMT().fmt2(ct.get_total_loss()));
    }

    // Still nothing: fall back to a walled LBF, which is feasible by construction. The plain
    // pre-pass's work is lost, but a correct answer from a worse start beats an infeasible one.
    sep.config = outer_cfg;
    warn!("[OPT] [SHEET] wall repair gave up after {} extra sheet(s); falling back to a walled LBF start",
        WALL_REPAIR_MAX_EXTRA_SHEETS);
    let builder = LBFBuilder::new_with_sheet(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG, Some(sheet)).construct();
    Separator::new_with_sheet(builder.instance, builder.prob, next_rng(), expl_config.separator_config, Some(sheet))
}

/// How many extra sheets the wall-installation repair may ask for before giving up on the plain
/// pre-pass's layout altogether. Each one is a whole sheet's worth of extra slack, so a repair that
/// needs more than a couple is not going to be worth keeping anyway.
const WALL_REPAIR_MAX_EXTRA_SHEETS: usize = 3;

/// How much slack (as a fraction of the sheet width) each retry of the wall repair leaves free at
/// the right edge of every sheet. Retry `i` uses `i * WALL_REPAIR_SLACK_STEP * W`.
const WALL_REPAIR_SLACK_STEP: f32 = 0.10;

/// Share of the plain pre-pass's budget granted to the wall-installation repair. Bounded so a
/// hopeless repair cannot eat the walled exploration's time; the fallback below it is cheap.
const WALL_REPAIR_BUDGET_RATIO: f32 = 0.5;

/// Strike limit for the wall-installation repair; see the use site.
const WALL_REPAIR_STRIKE_LIMIT: usize = 8;

/// No-improvement iteration limit for the wall-installation repair; see the use site.
const WALL_REPAIR_ITER_NO_IMPRV_LIMIT: usize = 400;
