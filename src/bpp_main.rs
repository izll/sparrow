//! `sparrow-bpp` — the bin packing (BPP) variant of the `sparrow` binary.
//!
//! Mirrors [`src/main.rs`](../main.rs): parse the CLI, load the instance, optionally warm start from
//! a previous solution, run the optimizer (with `-p` independent runs in parallel) and write the
//! best solution to `output/final_{name}.json` plus one SVG per bin.
//!
//! The only structural differences with the SPP binary are the input handling (a strip packing
//! instance can be turned into a bin packing one with `--bin WxH[:stock[:cost]]`) and the
//! comparison of parallel runs: the best run is the one with the lowest bin **cost**, ties broken
//! by the **lowest density of the least dense bin** (= the largest consolidated remainder).

use anyhow::{bail, Context, Result};
use clap::Parser as Clap;
use jagua_rs::io::import::Importer;
use jagua_rs::probs::bpp::entities::BPSolution;
use log::{info, warn};
use rand::rngs::Xoshiro256PlusPlus;
use rand::SeedableRng;
use sparrow::config::{BPConfig, DEFAULT_BPP_CONFIG};
use sparrow::consts::{DEFAULT_COMPRESS_TIME_RATIO, DEFAULT_EXPLORE_TIME_RATIO, DEFAULT_MAX_CONSEQ_FAILS_EXPL, LBF_SAMPLE_CONFIG, LOG_LEVEL_FILTER_DEBUG, LOG_LEVEL_FILTER_RELEASE};
use sparrow::optimizer::bpp::{optimize_bpp, BPLBFBuilder, BPShelfBuilder};
use sparrow::util::bpp_io::{self, BPSolutionListener, BPSvgExporter, BppCli, ExtBPOutput};
use sparrow::util::ctrlc_terminator::CtrlCTerminator;
use sparrow::util::io;
use sparrow::util::listener::ReportType;
use sparrow::util::verify;
use std::cmp::Ordering;
use std::fs;
use std::path::Path;
use std::time::Duration;

pub const OUTPUT_DIR: &str = "output";

pub const LIVE_DIR: &str = "data/live";

/// Largest total demand accepted, mirroring the SPP binary's cap. See `sparrow`'s
/// `MAX_TOTAL_DEMAND`.
pub const MAX_TOTAL_DEMAND: u64 = 1_000_000;

fn main() -> Result<()> {
    let mut config: BPConfig = DEFAULT_BPP_CONFIG;

    fs::create_dir_all(OUTPUT_DIR)?;
    let log_file_path = format!("{}/log.txt", OUTPUT_DIR);
    match cfg!(debug_assertions) {
        true => io::init_logger(LOG_LEVEL_FILTER_DEBUG, Path::new(&log_file_path))?,
        false => io::init_logger(LOG_LEVEL_FILTER_RELEASE, Path::new(&log_file_path))?,
    }

    let args = BppCli::parse();
    let input_file_path = &args.input;

    let (explore_dur, compress_dur) = match (args.global_time, args.exploration, args.compression) {
        (Some(gt), None, None) => (
            Duration::from_secs(gt).mul_f32(DEFAULT_EXPLORE_TIME_RATIO),
            Duration::from_secs(gt).mul_f32(DEFAULT_COMPRESS_TIME_RATIO),
        ),
        (None, Some(et), Some(ct)) => (Duration::from_secs(et), Duration::from_secs(ct)),
        (None, None, None) => {
            warn!("[MAIN] no time limit specified");
            (
                Duration::from_secs(600).mul_f32(DEFAULT_EXPLORE_TIME_RATIO),
                Duration::from_secs(600).mul_f32(DEFAULT_COMPRESS_TIME_RATIO),
            )
        }
        _ => bail!("invalid cli pattern (clap should have caught this)"),
    };
    config.expl_cfg.time_limit = explore_dur;
    config.cmpr_cfg.time_limit = compress_dur;
    if args.early_termination {
        config.expl_cfg.max_conseq_failed_attempts = Some(DEFAULT_MAX_CONSEQ_FAILS_EXPL);
        // Halve the stagnation limit too: `-x` means "give up early", and the stagnation stop is
        // the finer-grained of the two give-up rules.
        config.expl_cfg.stagnation_limit = config.expl_cfg.stagnation_limit.map(|n| (n / 2).max(1));
        // Also make the compression phase give up faster: halve the per-move budget of the
        // pack-down step and cut its separator's iteration/strike limits.
        config.cmpr_cfg.pack_down_move_time_limit /= 2;
        config.cmpr_cfg.pack_down_separator_config.iter_no_imprv_limit =
            (config.cmpr_cfg.pack_down_separator_config.iter_no_imprv_limit / 2).max(1);
        config.cmpr_cfg.pack_down_separator_config.strike_limit =
            (config.cmpr_cfg.pack_down_separator_config.strike_limit / 2).max(1);
        warn!("[MAIN] early termination enabled!");
    }
    if let Some(arg_rng_seed) = args.rng_seed {
        config.rng_seed = Some(arg_rng_seed as usize);
    }
    config.cmpr_cfg.pack_down_strategy = args.pack_down.into();
    info!("[MAIN] pack-down strategy: {:?}", config.cmpr_cfg.pack_down_strategy);

    info!("[MAIN] configured to explore for {}s and compress for {}s", explore_dur.as_secs(), compress_dur.as_secs());

    let seed = match config.rng_seed {
        Some(seed) => {
            info!("[MAIN] using seed: {}", seed);
            seed as u64
        }
        None => {
            let seed = rand::random();
            warn!("[MAIN] no seed provided, using: {}", seed);
            seed
        }
    };

    let n_runs = args.parallel_runs as usize;
    if n_runs > 1 {
        let n_threads = n_runs * config.expl_cfg.separator_config.n_workers.max(config.cmpr_cfg.separator_config.n_workers);
        info!("[MAIN] running {} independent optimizations in parallel (seeds {}..={}), keeping the best", n_runs, seed, seed + n_runs as u64 - 1);
        if n_threads > num_cpus::get() {
            warn!("[MAIN] {} runs x {} workers = {} threads > {} logical CPUs, runs will slow each other down",
                n_runs, n_threads / n_runs, n_threads, num_cpus::get());
        }
    }

    let (ext_instance, ext_solution) = bpp_io::read_bpp_input(Path::new(&input_file_path), &args.bins)?;

    // See `sparrow::MAX_TOTAL_DEMAND` on the SPP side: the BPP LBF constructor materialises one
    // `Vec` element per demanded copy too, so a huge demand is an allocation, not a number.
    let total_demand: u64 = ext_instance.items.iter().map(|it| it.demand).sum();
    if total_demand > MAX_TOTAL_DEMAND {
        bail!("the instance demands {total_demand} items, more than the supported maximum of \
               {MAX_TOTAL_DEMAND}; every copy is materialised individually, so this would exhaust \
               memory before the first placement");
    }

    // Minimum item separation: --min-sep > SPARROW_MIN_SEP env var > config default (shared with the SPP binary,
    // so both engines apply exactly the same inflation/deflation to identical inputs).
    let min_sep = sparrow::util::io::resolve_min_item_separation(args.min_item_separation, config.min_item_separation)?;
    config.min_item_separation = min_sep;
    if let Some(sep) = min_sep {
        info!("[MAIN] minimum item separation: {sep} (items inflated and bins deflated by {} each)", sep / 2.0);
    }
    let importer = Importer::new(config.cde_config, config.poly_simpl_tolerance, min_sep, config.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::bpp::io::import_instance(&importer, &ext_instance)?;

    let initial_solution = match ext_solution {
        Some(ext_sol) => Some(bpp_io::import_bp_solution(&instance, &ext_sol)?),
        None => None,
    };

    info!("[MAIN] loaded instance {} with #{} items and {} bin type(s)", ext_instance.name, instance.total_item_qty(), instance.bins.len());
    if let Some(init_sol) = &initial_solution {
        info!("[MAIN] warm start solution: {}", bpp_io::summarize(init_sol, &instance));

        // A warm start is fed straight into `BPProblem::restore`, which does not (and cannot)
        // invent placements for missing demand, nor reject extra ones. The check is **per item
        // id**, not on the total: a solution that places two copies of item 3 and none of item 4
        // has the right total and is still wrong. `verify_bpp_solution` also re-checks
        // collision-freedom (`restore` trusts its snapshots) and the bin stock.
        verify::verify_bpp_solution(init_sol, &instance, "the warm start solution")
            .context("the warm start solution given with -i is not usable")?;
    }

    let final_svg_path = format!("{OUTPUT_DIR}/final_{}.svg", ext_instance.name);
    let intermediate_svg_dir = match cfg!(feature = "only_final_svg") {
        true => None,
        false => Some(format!("{OUTPUT_DIR}/sols_{}", ext_instance.name)),
    };
    let live_svg_dir = match cfg!(feature = "live_svg") {
        true => Some(LIVE_DIR.to_string()),
        false => None,
    };

    // `optimize_bpp` *panics* if it cannot construct an initial solution, so validate the instance
    // once up front with the same LBF builder: it returns a proper `Err`, which becomes a CLI error.
    if initial_solution.is_none() {
        let probe_rng = Xoshiro256PlusPlus::seed_from_u64(seed);
        let probe = BPLBFBuilder::new(instance.clone(), probe_rng, LBF_SAMPLE_CONFIG).construct()
            .context("the instance cannot be packed: no initial solution could be constructed")?;
        info!("[MAIN] LBF probe: {} bin(s), cost: {}, density: {:.3}%",
            probe.prob.layouts.len(), probe.prob.bin_cost(), probe.prob.density() * 100.0);
        // The shelf constructor is only a *candidate* (see `Constructive::Best`), so a failure here
        // is informational: `optimize_bpp` falls back to the LBF solution it just validated.
        match BPShelfBuilder::new(instance.clone()).construct() {
            Ok(shelf) => info!("[MAIN] shelf probe: {} bin(s), cost: {}, density: {:.3}%",
                shelf.prob.layouts.len(), shelf.prob.bin_cost(), shelf.prob.density() * 100.0),
            Err(e) => warn!("[MAIN] shelf probe failed: {e}"),
        }
    }

    // Set up the Ctrl-C handler once (before spawning any runs); CtrlCTerminator is multi-instance safe.
    let ctrlc_terminator = CtrlCTerminator::new();

    // Runs one complete optimization (seed `run_seed`) and returns its final solution
    let run_optimization = |run_idx: usize, run_seed: u64| -> BPSolution {
        let rng = Xoshiro256PlusPlus::seed_from_u64(run_seed);
        // Every run gets its own SVG exporter; the final SVGs are written by the main thread for the best run only.
        let mut svg_exporter = BPSvgExporter::new(
            if n_runs == 1 { Some(final_svg_path.clone()) } else { None },
            intermediate_svg_dir.as_ref().map(|d| if n_runs == 1 { d.clone() } else { format!("{d}/run_{run_idx}") }),
            if run_idx == 0 { live_svg_dir.clone() } else { None },
        );
        let mut terminator = ctrlc_terminator.clone();

        optimize_bpp(
            instance.clone(),
            rng,
            &mut svg_exporter,
            &mut terminator,
            &config,
            initial_solution.as_ref(),
        )
    };

    let solution = if n_runs == 1 {
        run_optimization(0, seed)
    } else {
        // Run all optimizations concurrently on their own (named) threads and collect the results
        let solutions: Vec<(usize, BPSolution)> = std::thread::scope(|scope| {
            let handles = (0..n_runs)
                .map(|run_idx| {
                    let run_optimization = &run_optimization;
                    std::thread::Builder::new()
                        .name(format!("run-{run_idx}"))
                        .spawn_scoped(scope, move || (run_idx, run_optimization(run_idx, seed + run_idx as u64)))
                        .expect("failed to spawn optimization thread")
                })
                .collect::<Vec<_>>();
            handles.into_iter().map(|h| h.join().expect("optimization thread panicked")).collect()
        });

        // Only feasible runs may be selected. An infeasible solution packs into *fewer* bins exactly
        // because its parts overlap, so selecting on cost alone systematically prefers the broken
        // run over the correct ones.
        // The gate is the full export gate (demand per item id, feasibility, bin stock), not just
        // `is_feasible()`: a run that lost items packs into *fewer* bins precisely because it lost
        // them, so a cost-only selection prefers it over every correct run.
        let mut feasible = vec![];
        for (run_idx, sol) in solutions.into_iter() {
            let verdict = verify::verify_bpp_solution(&sol, &instance, "the solution");
            info!("[MAIN] run {} (seed {}): {}{}", run_idx, seed + run_idx as u64,
                bpp_io::summarize(&sol, &instance),
                match &verdict { Ok(()) => String::new(), Err(e) => format!("  <-- REJECTED: {e}") });
            match verdict {
                Ok(()) => feasible.push((run_idx, sol)),
                Err(e) => warn!("[MAIN] run {} (seed {}) produced an unusable solution and is excluded from the selection: {e}",
                    run_idx, seed + run_idx as u64),
            }
        }
        if feasible.is_empty() {
            bail!("all {} parallel runs produced unusable solutions; nothing to export", n_runs);
        }
        // Best = lowest cost, ties broken by the **lowest density of the least dense bin**: at equal
        // bin count the useful result is the one whose leftover material is concentrated in a single
        // bin (= the biggest consolidated remainder), not the one that spreads the same slack evenly.
        // `min_by` returns the first minimum.
        let (best_idx, best_sol) = feasible.into_iter()
            .min_by(|(_, a), (_, b)| {
                a.cost(&instance).cmp(&b.cost(&instance))
                    .then_with(|| min_bin_density(a, &instance)
                        .partial_cmp(&min_bin_density(b, &instance)).unwrap_or(Ordering::Equal))
            })
            .expect("the feasible list is non-empty");
        info!("[MAIN] best run: {} (seed {}), {}", best_idx, seed + best_idx as u64, bpp_io::summarize(&best_sol, &instance));

        // Export the final SVGs of the best run
        BPSvgExporter::new(Some(final_svg_path.clone()), None, None)
            .report(ReportType::Final, &best_sol, &instance);
        best_sol
    };

    info!("[MAIN] final solution: {}", bpp_io::summarize(&solution, &instance));

    // **The export gate**, the BPP twin of the SPP one in `main.rs`: nothing downstream can tell a
    // good JSON from a bad one, so the answer is fully re-verified (exact demand per item id,
    // per-layout collision-freedom, bin stock) immediately before the file is written. On failure
    // nothing is written and the process exits 1.
    verify::verify_bpp_solution(&solution, &instance, "the final solution")
        .context("refusing to export")?;

    let json_path = format!("{OUTPUT_DIR}/final_{}.json", ext_instance.name);
    let json_output = ExtBPOutput {
        instance: ext_instance,
        solution: bpp_io::export_bp(&instance, &solution),
    };
    bpp_io::write_bp_json(&json_output, Path::new(json_path.as_str()))?;

    Ok(())
}

/// The density of the **least dense** bin of a solution (`f32::INFINITY` for an empty solution).
///
/// Used as the tie-break between parallel runs of equal bin cost: the lower this value, the more of
/// the leftover material is concentrated in one bin, i.e. the larger the usable offcut.
fn min_bin_density(sol: &BPSolution, instance: &jagua_rs::probs::bpp::entities::BPInstance) -> f32 {
    sol.layout_snapshots.values()
        .map(|ls| ls.density(instance))
        .fold(f32::INFINITY, f32::min)
}
