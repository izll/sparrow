//! `sparrow-bpp` — the bin packing (BPP) variant of the `sparrow` binary.
//!
//! Mirrors [`src/main.rs`](../main.rs): parse the CLI, load the instance, optionally warm start from
//! a previous solution, run the optimizer (with `-p` independent runs in parallel) and write the
//! best solution to `output/final_{name}.json` plus one SVG per bin.
//!
//! The only structural differences with the SPP binary are the input handling (a strip packing
//! instance can be turned into a bin packing one with `--bin WxH[:stock[:cost]]`) and the
//! comparison of parallel runs: the best run is the one with the lowest bin **cost**, ties broken
//! by the higher density.

use anyhow::{bail, Context, Result};
use clap::Parser as Clap;
use jagua_rs::io::import::Importer;
use jagua_rs::probs::bpp::entities::BPSolution;
use log::{info, warn};
use rand::rngs::Xoshiro256PlusPlus;
use rand::SeedableRng;
use sparrow::config::{BPConfig, DEFAULT_BPP_CONFIG};
use sparrow::consts::{DEFAULT_COMPRESS_TIME_RATIO, DEFAULT_EXPLORE_TIME_RATIO, DEFAULT_MAX_CONSEQ_FAILS_EXPL, LBF_SAMPLE_CONFIG, LOG_LEVEL_FILTER_DEBUG, LOG_LEVEL_FILTER_RELEASE};
use sparrow::optimizer::bpp::{optimize_bpp, BPLBFBuilder};
use sparrow::util::bpp_io::{self, BPSolutionListener, BPSvgExporter, BppCli, ExtBPOutput};
use sparrow::util::ctrlc_terminator::CtrlCTerminator;
use sparrow::util::io;
use sparrow::util::listener::ReportType;
use std::cmp::Ordering;
use std::fs;
use std::path::Path;
use std::time::Duration;

pub const OUTPUT_DIR: &str = "output";

pub const LIVE_DIR: &str = "data/live";

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
        warn!("[MAIN] early termination enabled!");
    }
    if let Some(arg_rng_seed) = args.rng_seed {
        config.rng_seed = Some(arg_rng_seed as usize);
    }

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

    let importer = Importer::new(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::bpp::io::import_instance(&importer, &ext_instance)?;

    let initial_solution = match ext_solution {
        Some(ext_sol) => Some(bpp_io::import_bp_solution(&instance, &ext_sol)?),
        None => None,
    };

    info!("[MAIN] loaded instance {} with #{} items and {} bin type(s)", ext_instance.name, instance.total_item_qty(), instance.bins.len());
    if let Some(init_sol) = &initial_solution {
        info!("[MAIN] warm start solution: {}", bpp_io::summarize(init_sol, &instance));

        // A warm start is fed straight into `BPProblem::restore`, which does not (and cannot)
        // invent placements for missing demand. An incomplete solution would therefore be
        // optimized — and written out — with items silently missing, so reject it here.
        let n_placed: usize = init_sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
        if n_placed != instance.total_item_qty() {
            bail!("the warm start solution places {n_placed} item(s) but the instance demands {}; \
                   it does not cover the full demand", instance.total_item_qty());
        }
        // Likewise, `restore` trusts the snapshots: verify they are actually collision-free.
        for (lkey, ls) in init_sol.layout_snapshots.iter() {
            if !jagua_rs::entities::Layout::from_snapshot(ls).is_feasible() {
                bail!("layout {lkey:?} of the warm start solution is not collision-free");
            }
        }
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

        for (run_idx, sol) in solutions.iter() {
            info!("[MAIN] run {} (seed {}): {}", run_idx, seed + *run_idx as u64, bpp_io::summarize(sol, &instance));
        }
        // Best = lowest cost, ties broken by the higher density. `min_by` returns the first minimum.
        let (best_idx, best_sol) = solutions.into_iter()
            .min_by(|(_, a), (_, b)| {
                a.cost(&instance).cmp(&b.cost(&instance))
                    .then_with(|| b.density(&instance).partial_cmp(&a.density(&instance)).unwrap_or(Ordering::Equal))
            })
            .expect("at least one run");
        info!("[MAIN] best run: {} (seed {}), {}", best_idx, seed + best_idx as u64, bpp_io::summarize(&best_sol, &instance));

        // Export the final SVGs of the best run
        BPSvgExporter::new(Some(final_svg_path.clone()), None, None)
            .report(ReportType::Final, &best_sol, &instance);
        best_sol
    };

    info!("[MAIN] final solution: {}", bpp_io::summarize(&solution, &instance));

    let json_path = format!("{OUTPUT_DIR}/final_{}.json", ext_instance.name);
    let json_output = ExtBPOutput {
        instance: ext_instance,
        solution: bpp_io::export_bp(&instance, &solution),
    };
    bpp_io::write_bp_json(&json_output, Path::new(json_path.as_str()))?;

    Ok(())
}
