extern crate core;

use clap::Parser as Clap;
use jagua_rs::io::import::Importer;
use log::{info, warn, Level};
use rand::SeedableRng;
use sparrow::config::*;
use sparrow::optimizer::optimize;
use sparrow::util::demand::total_demand;
use sparrow::util::io;
use sparrow::util::io::{ExtSPOutput, MainCli};
use sparrow::EPOCH;
use std::fs;
use std::path::Path;
use std::time::Duration;

use anyhow::{bail, Context, Result};
use rand::rngs::Xoshiro256PlusPlus;
use sparrow::util::verify;
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
    config.min_item_separation = io::resolve_min_item_separation(args.min_item_separation, config.min_item_separation)?;
    if let Some(sep) = config.min_item_separation {
        info!("[MAIN] minimum item separation: {sep} (items inflated and container deflated by {} each)", sep / 2.0);
    }

    // Multi-sheet ("walled") strip mode, if requested
    let sheet = match args.sheet_width {
        Some(width) => {
            let gap = SheetConfig::resolve_gap(args.sheet_gap, config.min_item_separation)?;
            let mut sc = SheetConfig::new(width, gap, args.compact_sheets);
            sc.pack_down = args.pack_down_sheets;
            if args.plain_first {
                sc.pipeline = SheetPipeline::PlainFirst;
            }
            Some(sc)
        }
        None => None,
    };
    config.apply_sheet(sheet);
    if let Some(sheet) = sheet {
        info!("[MAIN] multi-sheet (walled) mode: sheet width {} mm, wall/gap {} mm; \
               a wall is inserted at every multiple of {} mm so no item straddles a sheet boundary",
            sheet.width, sheet.gap, sheet.pitch());
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

    // A demand of many millions expands into one `Vec` entry per copy in both LBF constructors
    // (`iter::repeat_n(id, missing_qty).collect_vec()`), which is where a 100-million fixture went
    // to die: an 800 MB allocation and an abort with no message. Refuse it with one instead.
    let total_demand = total_demand(ext_instance.items.iter().map(|it| it.demand))
        .context("the instance's demand is not supportable")?;
    info!("[MAIN] total demand: {total_demand} item copies");

    // A malformed warm start reaches jagua's `import_solution`, which trusts it completely: an
    // unknown item id indexes out of bounds, an over-placed one underflows the demand counter and a
    // negative strip width builds an invalid `Rect` — all `panic = abort`, exit 134. Validate first.
    if let Some(ext_sol) = ext_solution.as_ref() {
        io::validate_spp_warm_start(&ext_instance, ext_sol)
            .context("the warm start solution given with -i is not usable")?;
    }

    let importer = Importer::new(config.cde_config, config.poly_simpl_tolerance, config.min_item_separation, config.narrow_concavity_cutoff_ratio);
    let mut instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext_instance)?;

    // Packability gate. Two input-level impossibilities used to surface as aborts deep inside the
    // engine: an item taller than the strip (LBF widens for ever -> "strip-width is running away")
    // and a `--min-sep` so large relative to the instance that jagua's 100 %-density starting width
    // deflates into an empty polygon. See `util::packability`.
    if let Some(min_width) = sparrow::util::packability::check_spp_packability(&instance, config.min_item_separation)? {
        info!("[MAIN] widening the initial strip {:.1} -> {:.1} mm so the container survives the \
               {:.1} mm deflation --min-sep applies to it",
            instance.base_strip.width, min_width, config.min_item_separation.unwrap_or(0.0) / 2.0);
        instance.base_strip.set_width(min_width);
    }
    let instance = instance;

    let initial_solution = ext_solution.map(|e|
        jagua_rs::probs::spp::io::import_solution(&instance, &e)
    );

    info!("[MAIN] loaded instance {} with #{} items", ext_instance.name, instance.total_item_qty());

    // In the walled mode an item wider than one sheet (in every allowed rotation) can never be
    // placed: it would have to straddle a wall. Left undetected this surfaces either as an LBF
    // "strip-width is running away" panic or — from a warm start, where the LBF is bypassed — as an
    // exported layout full of wall crossings. Fail loudly and early instead.
    if let Some(sheet) = sheet.as_ref() {
        let too_wide = sparrow::optimizer::sheets::items_too_wide_for_sheet(&instance, sheet);
        if !too_wide.is_empty() {
            let list = too_wide.iter()
                .map(|(id, w)| format!("item {id} ({:.1} mm)", w))
                .collect::<Vec<_>>()
                .join(", ");
            bail!("--sheet-width {} mm is too narrow for this instance: {} do(es) not fit inside a \
                   single sheet in any allowed rotation, so no walled solution can exist",
                sheet.width, list);
        }
    }

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
        // **No final path here.** The final SVG is written by the main thread *after* the export
        // gate, for whichever solution the gate accepts — see the end of `main`. Handing the
        // exporter a final path would put the answer on disk before it was verified.
        let mut svg_exporter = SvgExporter::new(
            None,
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

        // Pick the narrowest solution **among the feasible ones**. Selecting on width alone lets an
        // infeasible run win precisely because it is infeasible: overlapping parts pack into a
        // narrower strip than separated ones ever could, so the worst run is the most likely to be
        // chosen and exported.
        // The gate is the full export gate, not just `is_feasible()`: an *empty* layout is
        // perfectly collision-free, so a run that lost items (or started from a warm start that had
        // already lost them) is both "feasible" and narrower than every correct run — i.e. the
        // guaranteed winner of a width-only selection. Demand coverage and cuttability are
        // therefore checked here too.
        let mut feasible = vec![];
        for (run_idx, sol) in solutions.into_iter() {
            let verdict = verify::verify_spp_solution(&sol, &instance, sheet.as_ref(), "the solution");
            info!("[MAIN] run {} (seed {}): width: {:.3}, density: {:.3}%{}",
                run_idx, seed + run_idx as u64, sol.strip_width(), sol.density(&instance) * 100.0,
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
        let (best_idx, best_sol) = feasible.into_iter()
            .min_by(|(_, a), (_, b)| a.strip_width().partial_cmp(&b.strip_width()).unwrap())
            .expect("the feasible list is non-empty");
        info!("[MAIN] best run: {} (seed {}), width: {:.3}, density: {:.3}%", best_idx, seed + best_idx as u64, best_sol.strip_width(), best_sol.density(&instance) * 100.0);
        best_sol
    };

    if let Some(sheet) = sheet.as_ref() {
        sparrow::optimizer::sheets::log_sheet_report("FINAL", &solution, &instance, sheet);
    }

    // **The export gate.** Nothing downstream can tell a good JSON from a bad one, so the last
    // thing that happens before the file is written is a full re-verification of the answer:
    // exact demand per item id, collision-freedom, and — in the walled mode — that no item
    // straddles a sheet wall. All three had reproducible ways of reaching the file: a warm start
    // with a missing placement, `optimize`'s "possibly infeasible" fallback, and `--sheet-gap 0`
    // respectively. On failure nothing is written and the process exits 1.
    // Any final artefact left over from an *earlier* run in this directory is not this run's
    // answer. Remove it before the gate, so that a rejected run leaves no `final_*.svg`/`.json` at
    // all — neither its own (it never writes one) nor a stale one that a reader would attribute to
    // it. On success both are rewritten a few lines below.
    let json_path = format!("{OUTPUT_DIR}/final_{}.json", ext_instance.name);
    let _ = fs::remove_file(&final_svg_path);
    let _ = fs::remove_file(&json_path);

    verify::verify_spp_solution(&solution, &instance, sheet.as_ref(), "the final solution")
        .context("refusing to export")?;

    // Past the gate: *now* the solution may be published. The SVG is written first and the JSON
    // second, both to the same verified solution.
    SvgExporter::new(Some(final_svg_path.clone()), None, None)
        .report(ReportType::Final, &solution, &instance);

    let json_output = ExtSPOutput {
        instance: ext_instance,
        solution: jagua_rs::probs::spp::io::export(&instance, &solution, *EPOCH)
    };
    io::write_json(&json_output, Path::new(json_path.as_str()), Level::Info)?;

    Ok(())
}
