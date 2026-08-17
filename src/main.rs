extern crate core;

use clap::Parser as Clap;
use jagua_rs::io::import::Importer;
use log::{info, warn, Level};
use rand::SeedableRng;
use sparrow::config::*;
use sparrow::optimizer::optimize;
use sparrow::util::io;
use sparrow::util::io::{ExtSPOutput, MainCli};
use sparrow::EPOCH;
use std::fs;
use std::path::Path;
use std::time::Duration;

use anyhow::{bail, Result};
use rand::rngs::Xoshiro256PlusPlus;
use sparrow::consts::{DEFAULT_COMPRESS_TIME_RATIO, DEFAULT_EXPLORE_TIME_RATIO, DEFAULT_FAIL_DECAY_RATIO_CMPR, DEFAULT_MAX_CONSEQ_FAILS_EXPL, LOG_LEVEL_FILTER_DEBUG, LOG_LEVEL_FILTER_RELEASE};
use sparrow::util::ctrlc_terminator::CtrlCTerminator;
use sparrow::util::listener::{ReportType, SolutionListener};
use sparrow::util::svg_exporter::SvgExporter;

pub const OUTPUT_DIR: &str = "output";

pub const LIVE_DIR: &str = "data/live";

fn main() -> Result<()>{
    let mut config = DEFAULT_SPARROW_CONFIG;

    fs::create_dir_all(OUTPUT_DIR)?;
    let log_file_path = format!("{}/log.txt", OUTPUT_DIR);
    match cfg!(debug_assertions) {
        true => io::init_logger(LOG_LEVEL_FILTER_DEBUG, Path::new(&log_file_path))?,
        false => io::init_logger(LOG_LEVEL_FILTER_RELEASE, Path::new(&log_file_path))?,
    }

    let args = MainCli::parse();
    let input_file_path = &args.input;
    let (explore_dur, compress_dur) = match (args.global_time, args.exploration, args.compression) {
        (Some(gt), None, None) => {
            (Duration::from_secs(gt).mul_f32(DEFAULT_EXPLORE_TIME_RATIO), Duration::from_secs(gt).mul_f32(DEFAULT_COMPRESS_TIME_RATIO))
        },
        (None, Some(et), Some(ct)) => {
            (Duration::from_secs(et), Duration::from_secs(ct))
        },
        (None, None, None) => {
            warn!("[MAIN] no time limit specified");
            (Duration::from_secs(600).mul_f32(DEFAULT_EXPLORE_TIME_RATIO), Duration::from_secs(600).mul_f32(DEFAULT_COMPRESS_TIME_RATIO))
        },
        _ => bail!("invalid cli pattern (clap should have caught this)"),
    };
    config.expl_cfg.time_limit = explore_dur;
    config.cmpr_cfg.time_limit = compress_dur;
    if args.early_termination {
        config.expl_cfg.max_conseq_failed_attempts = Some(DEFAULT_MAX_CONSEQ_FAILS_EXPL);
        config.cmpr_cfg.shrink_decay = ShrinkDecayStrategy::FailureBased(DEFAULT_FAIL_DECAY_RATIO_CMPR);
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
        },
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

    let (ext_instance, ext_solution) = io::read_spp_input(Path::new(&input_file_path))?;

    let importer = Importer::new(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
    let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext_instance)?;

    let initial_solution = ext_solution.map(|e|
        jagua_rs::probs::spp::io::import_solution(&instance, &e)
    );

    info!("[MAIN] loaded instance {} with #{} items", ext_instance.name, instance.total_item_qty());
    
    let final_svg_path = format!("{OUTPUT_DIR}/final_{}.svg", ext_instance.name);
    let intermediate_svg_dir = match cfg!(feature = "only_final_svg") {
        true => None,
        false => Some(format!("{OUTPUT_DIR}/sols_{}", ext_instance.name))
    };
    let live_svg_path = match cfg!(feature = "live_svg") {
        true => Some(format!("{LIVE_DIR}/.live_solution.svg")),
        false => None
    };

    // Set up the Ctrl-C handler once (before spawning any runs)
    let ctrlc_terminator = CtrlCTerminator::new();

    // Runs one complete optimization (seed `run_seed`) and returns its final solution
    let run_optimization = |run_idx: usize, run_seed: u64| -> jagua_rs::probs::spp::entities::SPSolution {
        let rng = Xoshiro256PlusPlus::seed_from_u64(run_seed);
        // Every run gets its own SVG exporter; the final SVG is written by the main thread for the best run only.
        let mut svg_exporter = SvgExporter::new(
            if n_runs == 1 { Some(final_svg_path.clone()) } else { None },
            intermediate_svg_dir.as_ref().map(|d| if n_runs == 1 { d.clone() } else { format!("{d}/run_{run_idx}") }),
            if run_idx == 0 { live_svg_path.clone() } else { None },
        );
        let mut terminator = ctrlc_terminator.clone();
        optimize(
            instance.clone(),
            rng,
            &mut svg_exporter,
            &mut terminator,
            &config.expl_cfg,
            &config.cmpr_cfg,
            initial_solution.as_ref()
        )
    };

    let solution = if n_runs == 1 {
        run_optimization(0, seed)
    } else {
        // Run all optimizations concurrently on their own (named) threads and collect the results
        let solutions: Vec<(usize, jagua_rs::probs::spp::entities::SPSolution)> = std::thread::scope(|scope| {
            let handles = (0..n_runs).map(|run_idx| {
                let run_optimization = &run_optimization;
                std::thread::Builder::new()
                    .name(format!("run-{run_idx}"))
                    .spawn_scoped(scope, move || (run_idx, run_optimization(run_idx, seed + run_idx as u64)))
                    .expect("failed to spawn optimization thread")
            }).collect::<Vec<_>>();
            handles.into_iter().map(|h| h.join().expect("optimization thread panicked")).collect()
        });

        for (run_idx, sol) in solutions.iter() {
            info!("[MAIN] run {} (seed {}): width: {:.3}, density: {:.3}%", run_idx, seed + *run_idx as u64, sol.strip_width(), sol.density(&instance) * 100.0);
        }
        let (best_idx, best_sol) = solutions.into_iter()
            .min_by(|(_, a), (_, b)| a.strip_width().partial_cmp(&b.strip_width()).unwrap())
            .expect("at least one run");
        info!("[MAIN] best run: {} (seed {}), width: {:.3}, density: {:.3}%", best_idx, seed + best_idx as u64, best_sol.strip_width(), best_sol.density(&instance) * 100.0);

        // Export the final SVG of the best run
        SvgExporter::new(Some(final_svg_path.clone()), None, None)
            .report(ReportType::Final, &best_sol, &instance);
        best_sol
    };

    let json_path = format!("{OUTPUT_DIR}/final_{}.json", ext_instance.name);
    let json_output = ExtSPOutput {
        instance: ext_instance,
        solution: jagua_rs::probs::spp::io::export(&instance, &solution, *EPOCH)
    };
    io::write_json(&json_output, Path::new(json_path.as_str()), Level::Info)?;

    Ok(())
}
