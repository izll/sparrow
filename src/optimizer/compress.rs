use crate::config::{CompressionConfig, ShrinkDecayStrategy};
use crate::optimizer::separator::Separator;
use crate::optimizer::sheets::{compact_sheets_left, pack_down_sheets};
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::{BasicTerminator, Terminator};
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use jagua_rs::Instant;
use log::info;
use rand::{Rng, RngExt};

/// A terminator that fires as soon as **either** of its two components does.
///
/// The pack-down step gets its own (short) budget, but must of course also stop when the phase's
/// own terminator fires — including a Ctrl-C, which lives on the outer one.
struct PairTerminator<'a, T: Terminator>(BasicTerminator, &'a T);

impl<T: Terminator> Terminator for PairTerminator<'_, T> {
    fn kill(&self) -> bool {
        self.0.kill() || self.1.kill()
    }

    fn new_timeout(&mut self, timeout: std::time::Duration) {
        self.0.new_timeout(timeout);
    }

    fn timeout_at(&self) -> Option<Instant> {
        match (self.0.timeout_at(), self.1.timeout_at()) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }
}

/// Algorithm 13 from https://doi.org/10.48550/arXiv.2509.13329
pub fn compression_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    init_sol: &SPSolution,
    sol_listener: &mut impl SolutionListener,
    term: &impl Terminator,
    config: &CompressionConfig
) -> SPSolution {
    let mut best_sol = init_sol.clone();
    let start = Instant::now();
    let mut n_failed_attempts = 0;

    // --- Phase 8: cross-sheet pack-down (walled mode only) ------------------------------------
    // Before the fine compression starts squeezing the strip's right end, try to relocate the last
    // sheet's items into the earlier sheets. The fine compression cannot do this — its pressure
    // stops at the last wall — but it is exactly what turns a nearly-empty last sheet into a short
    // band (or lets the sheet disappear entirely when the width falls below a whole sheet).
    //
    // It runs first and gets at most `pack_down_time_ratio` of the phase's budget; the remainder
    // goes to the fine compression, which is what converts the freed space into strip width.
    if let Some(sheet) = config.sheet.as_ref()
        && sheet.pack_down
    {
        let budget = config.time_limit.mul_f32(sheet.pack_down_time_ratio.clamp(0.0, 1.0));
        let mut pd_term = BasicTerminator::new();
        pd_term.new_timeout(budget.min(config.time_limit.saturating_sub(start.elapsed())));
        let pd_term = PairTerminator(pd_term, term);

        let (packed, n_moved) = pack_down_sheets(sep, sheet, &best_sol, &pd_term, sol_listener);
        if n_moved > 0 {
            info!("[CMPR] [SHEET] pack-down moved {n_moved} item(s) off the last sheet ({:.3}%)",
                packed.density(instance) * 100.0);
            best_sol = packed;
        }
        // The pack-down never changes the strip width, so the fine compression below simply
        // continues from `best_sol` (which `pack_down_sheets` has already rolled the separator to).
        debug_assert!(sep.prob.strip_width() == best_sol.strip_width());
    }

    // Create the function to calculate the shrink step size.
    let shrink_step_size = |n_failed_attempts: i32| -> f32 {
        match config.shrink_decay {
            ShrinkDecayStrategy::TimeBased => {
                let range = config.shrink_range.1 - config.shrink_range.0;
                let elapsed = start.elapsed();
                let ratio = elapsed.as_secs_f32() / config.time_limit.as_secs_f32();
                config.shrink_range.0 + ratio * range
            }
            ShrinkDecayStrategy::FailureBased(r) => {
                config.shrink_range.0 * r.powi(n_failed_attempts)
            }
        }
    };

    // As long as the shrink step size is above the minimum, keep attempting to compress
    while !term.kill() && let step = shrink_step_size(n_failed_attempts) && step >= config.shrink_range.1 {
        match attempt_to_compress(sep, &best_sol, step, term, sol_listener) {
            Some(compacted_sol) => {
                info!("[CMPR] success at {:.3}% ({:.3} | {:.3}%)", step * 100.0, compacted_sol.strip_width(), compacted_sol.density(instance) * 100.0);
                sol_listener.report(ReportType::CmprFeas, &compacted_sol, instance);
                best_sol = compacted_sol;
            }
            None => {
                info!("[CMPR] failed at {:.3}%", step * 100.0);
                n_failed_attempts += 1;
            }
        }
    }
    info!("[CMPR] finished, compressed from {:.3}% to {:.3}% (+{:.3}%)", init_sol.density(instance) * 100.0, best_sol.density(instance) * 100.0, (best_sol.density(instance) - init_sol.density(instance)) * 100.0);

    // --- Phase 8: `--compact-sheets` post-pass ------------------------------------------------
    // Purely secondary: it cannot change the strip width or the sheet count, it only slides the
    // items of every sheet but the last as far left as they will go inside their own sheet, so the
    // slack of that sheet becomes one contiguous, reusable right-hand band. Runs last, after the
    // width is final, and only when explicitly requested.
    if let Some(sheet) = config.sheet.as_ref()
        && sheet.compact_sheets
    {
        let (compacted, n_moved) = compact_sheets_left(sep, sheet, &best_sol);
        if n_moved > 0 {
            debug_assert!(compacted.strip_width() == best_sol.strip_width(),
                "the left-compaction must not change the strip width");
            best_sol = compacted;
            sol_listener.report(ReportType::CmprFeas, &best_sol, instance);
        }
    }

    best_sol
}


fn attempt_to_compress(sep: &mut Separator, init_sol: &SPSolution, r_shrink: f32, term: &impl Terminator, sol_listener: &mut impl SolutionListener) -> Option<SPSolution> {
    // Restore to the initial solution and width
    sep.change_strip_width(init_sol.strip_width(), None);
    sep.rollback(init_sol, None);

    // Shrink the container by the provided amount at a random position
    let new_width = init_sol.strip_width() * (1.0 - r_shrink);
    let split_pos = sep.rng.random_range(0.0..sep.prob.strip_width());
    sep.change_strip_width(new_width, Some(split_pos));

    // Try to separate layout, if all collisions are eliminated, return the solution
    let (compacted_sol, ot) = sep.separate(term, sol_listener);
    match ot.get_total_loss() == 0.0 {
        true => Some(compacted_sol),
        false => None,
    }
}