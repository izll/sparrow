use crate::EPOCH;
use anyhow::{Context, Result};
use clap::Parser;
use jagua_rs::probs::spp::io::ext_repr::{ExtSPInstance, ExtSPSolution};
use log::{log, Level, LevelFilter};
use serde::{Deserialize, Serialize};
use std::fs;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use svg::Document;

#[derive(Parser)]
pub struct MainCli {
    /// Path to input file (mandatory)
    #[arg(short = 'i', long, help = "Path to the input JSON file, or a solution JSON file for warm starting")]
    pub input: String,

    /// Global time limit in seconds (mutually exclusive with -e and -c)
    #[arg(short = 't', long, conflicts_with_all = &["exploration", "compression"], help = "Set a global time limit (in seconds)")]
    pub global_time: Option<u64>,

    /// Exploration time limit in seconds (requires compression time)
    #[arg(short = 'e', long, requires = "compression", help = "Set the exploration phase time limit (in seconds)")]
    pub exploration: Option<u64>,

    /// Compression time limit in seconds (requires exploration time)
    #[arg(short = 'c', long, requires = "exploration", help = "Set the compression phase time limit (in seconds)")]
    pub compression: Option<u64>,

    /// Enable early and automatic termination
    #[arg(short = 'x', long, help = "Enable early termination of the optimization process")]
    pub early_termination: bool,

    #[arg(short = 's', long, help = "Fixed seed for the random number generator")]
    pub rng_seed: Option<u64>,

    /// Minimum separation between items and between items and the container edge (mm)
    #[arg(long = "min-sep", value_name = "MM", value_parser = parse_non_negative_f32,
        help = "Minimum distance between items and between items and the container edge (mm). \
                Items are inflated and the container is deflated by half this value each. Overrides the SPARROW_MIN_SEP env var")]
    pub min_item_separation: Option<f32>,

    /// Usable width of one physical sheet (mm). Enables the multi-sheet ("walled") strip mode.
    #[arg(long = "sheet-width", value_name = "MM", value_parser = parse_positive_f32,
        help = "Enable multi-sheet (walled) strip packing: insert a wall at every multiple of this sheet width (mm), \
                so no item ever straddles a sheet boundary. The strip can then be cut into physical sheets of this width directly. \
                Without this flag the behaviour is the plain strip packing one")]
    pub sheet_width: Option<f32>,

    /// Thickness of the virtual wall between two consecutive sheets (mm)
    #[arg(long = "sheet-gap", value_name = "MM", requires = "sheet_width", value_parser = parse_sheet_gap,
        help = "Thickness of the (virtual) wall between two consecutive sheets (mm). The sheets are separate physical objects, \
                so this costs no material; a thicker wall gives the separator a better gradient to push items off a boundary. \
                Must be at least 1 mm: a zero-width wall is not representable as a collision hazard and would be dropped, \
                letting items straddle the sheet boundaries. Defaults to max(20, 2 * min-sep)")]
    pub sheet_gap: Option<f32>,

    /// Compact each sheet's items to the left within their own sheet after compression
    #[arg(long = "compact-sheets", requires = "sheet_width",
        help = "After compression, re-compact every sheet except the last one within its own sheet, so the leftover of each \
                sheet becomes one wide reusable band instead of many small gaps. Secondary objective only: it cannot reduce \
                the sheet count or the strip width")]
    pub compact_sheets: bool,

    /// Start the walled run from a wall-less pre-pass instead of exploring with the walls in place
    #[arg(long = "plain-first", requires = "sheet_width",
        help = "Run the exploration WITHOUT walls for half its budget first (the strip engine reaches a much higher \
                density that way), then cut the result into sheets, install the walls and repair the items that end up \
                on one. Falls back to a walled start if the repair cannot be made feasible. Off by default: measured \
                not to beat the walled-from-start pipeline on any reference instance")]
    pub plain_first: bool,

    /// Run the cross-sheet pack-down in the compression phase (off by default)
    #[arg(long = "pack-down-sheets", requires = "sheet_width",
        help = "Before the fine compression, cut the last sheet's leftover band back in large steps and relocate \
                whatever no longer fits into the earlier sheets. Off by default: on the measured instances no cut \
                was ever accepted and the attempts cost the fine compression its budget")]
    pub pack_down_sheets: bool,

    /// Number of independent optimizations to run in parallel (different seeds), keeping the best final solution
    #[arg(short = 'p', long, default_value_t = 1, value_parser = clap::value_parser!(u64).range(1..),
        help = "Run N independent optimizations in parallel (seed, seed+1, ...) within the same time limit and keep the best result. \
                Uses otherwise idle CPU cores; each run uses its own worker threads (see #workers in the log)")]
    pub parallel_runs: u64,
}

/// Environment variable read as a fallback for `--min-sep` (used by callers that cannot pass CLI flags).
pub const MIN_SEP_ENV_VAR: &str = "SPARROW_MIN_SEP";

/// Smallest wall thickness `--sheet-gap` accepts (mm).
///
/// A wall is modelled as a [`Hole`](jagua_rs::collision_detection::hazards::HazardEntity::Hole)
/// hazard built from a `Rect`, and a rectangle of zero width is not representable: `Rect::try_new`
/// rejects it, so the wall is silently *dropped* and nothing stops an item from straddling the
/// boundary. A run with `--sheet-gap 0` therefore looked like a walled run but was a plain strip
/// run whose export could not be cut into sheets at all. The gap is virtual (the sheets are
/// separate physical objects, so it costs no material), so requiring at least 1 mm of it costs
/// nothing and keeps the wall a real geometric obstacle.
pub const MIN_SHEET_GAP: f32 = 1.0;

/// Shared `clap` value parser for every floating point CLI argument of both binaries.
///
/// Rejects `NaN` and `±inf`, which `f32::from_str` happily accepts and which then propagate through
/// the whole pipeline: `--sheet-width NaN` aborted inside jagua, `--sheet-width inf` produced an
/// export full of `NaN`/`Infinity` metrics, and `--min-sep NaN` silently *disabled* the separation
/// (every `v > 0.0` comparison with a NaN is false). A `clap` parse error exits with code 2.
pub fn parse_finite_f32(s: &str) -> Result<f32, String> {
    let v: f32 = s.trim().parse().map_err(|_| format!("`{s}` is not a number"))?;
    match v.is_finite() {
        true => Ok(v),
        false => Err(format!("`{s}` is not a finite number")),
    }
}

/// [`parse_finite_f32`] plus a `> 0` requirement (`--sheet-width`).
pub fn parse_positive_f32(s: &str) -> Result<f32, String> {
    let v = parse_finite_f32(s)?;
    match v > 0.0 {
        true => Ok(v),
        false => Err(format!("`{s}` must be greater than 0")),
    }
}

/// [`parse_finite_f32`] plus a `>= 0` requirement (`--min-sep`).
pub fn parse_non_negative_f32(s: &str) -> Result<f32, String> {
    let v = parse_finite_f32(s)?;
    match v >= 0.0 {
        true => Ok(v),
        false => Err(format!("`{s}` must not be negative")),
    }
}

/// [`parse_finite_f32`] plus the `>= MIN_SHEET_GAP` requirement (`--sheet-gap`).
///
/// See [`MIN_SHEET_GAP`] for why zero is not accepted.
pub fn parse_sheet_gap(s: &str) -> Result<f32, String> {
    let v = parse_finite_f32(s)?;
    match v >= MIN_SHEET_GAP {
        true => Ok(v),
        false => Err(format!(
            "`{s}` is too thin: a sheet wall must be at least {MIN_SHEET_GAP} mm wide. A zero-width \
             wall is not representable as a collision hazard, so it would be dropped and items \
             would be free to straddle the sheet boundaries. The gap is virtual (the sheets are \
             separate physical objects), so it costs no material"
        )),
    }
}

/// Resolves the minimum item separation to use: CLI flag > `SPARROW_MIN_SEP` env var > config default.
/// Non-positive values disable the separation. Both binaries (`sparrow`, `sparrow-bpp`) use this, so an identical
/// input yields an identical fit/no-fit verdict regardless of the problem type.
///
/// Returns `Err` for a non-finite value: a `NaN` reaching this function used to fall through every
/// `v > 0.0` test and *silently disable* the separation the caller explicitly asked for.
pub fn resolve_min_item_separation(cli_value: Option<f32>, config_default: Option<f32>) -> Result<Option<f32>> {
    let env_raw = std::env::var(MIN_SEP_ENV_VAR).ok();
    let env_value = match env_raw.as_deref().map(str::trim).filter(|v| !v.is_empty()) {
        Some(raw) => match raw.parse::<f32>() {
            Ok(v) if v.is_finite() => Some(v),
            _ => anyhow::bail!("{MIN_SEP_ENV_VAR}={raw:?} is not a finite number"),
        },
        None => None,
    };
    let resolved = match cli_value.or(env_value) {
        Some(v) if !v.is_finite() => anyhow::bail!("the minimum item separation ({v}) is not a finite number"),
        Some(v) if v > 0.0 => Some(v),
        Some(_) => None,
        None => config_default,
    };
    Ok(resolved)
}

#[derive(Serialize, Deserialize, Clone)]
pub struct ExtSPOutput {
    #[serde(flatten)]
    pub instance: ExtSPInstance,
    pub solution: ExtSPSolution,
}

pub fn init_logger(level_filter: LevelFilter, log_file_path: &Path) -> Result<()> {
    //remove old log file
    let _ = fs::remove_file(log_file_path);
    fern::Dispatch::new()
        // Perform allocation-free log formatting
        .format(|out, message, record| {
            let handle = std::thread::current();
            let thread_name = handle.name().unwrap_or("-");

            let duration = EPOCH.elapsed();
            let sec = duration.as_secs() % 60;
            let min = (duration.as_secs() / 60) % 60;
            let hours = (duration.as_secs() / 60) / 60;

            let prefix = format!(
                "[{}] [{:0>2}:{:0>2}:{:0>2}] <{}>",
                record.level(),
                hours,
                min,
                sec,
                thread_name,
            );

            out.finish(format_args!("{:<25}{}", prefix, message))
        })
        // Add blanket level filter -
        .level(level_filter)
        .chain(std::io::stdout())
        .chain(fern::log_file(log_file_path)?)
        .apply()?;
    log!(
        Level::Info,
        "[EPOCH]: {}",
        jiff::Timestamp::now()
    );
    Ok(())
}


pub fn write_svg(document: &Document, path: &Path, log_lvl: Level) -> Result<()> {
    //make sure the parent directory exists
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).context("could not create parent directory for svg file")?;
    }
    svg::save(path, document)?;
    log!(log_lvl,
        "[IO] svg exported to file://{}",
        fs::canonicalize(path)
            .expect("could not canonicalize path")
            .to_str()
            .unwrap()
    );
    Ok(())
}

pub fn write_json(json: &impl Serialize, path: &Path, log_lvl: Level) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, json)?;
    log!(log_lvl,
        "[IO] json exported to file://{}",
        fs::canonicalize(path)
            .expect("could not canonicalize path")
            .to_str()
            .unwrap()
    );
    Ok(())
}

/// Validates an SPP warm-start solution **before** it is handed to
/// [`jagua_rs::probs::spp::io::import_solution`], which trusts its input completely.
///
/// The import indexes `instance.items` with the raw `item_id` and calls `place_item` for every
/// placement, so an unknown id, an over-placed item or a negative strip width all reach jagua as an
/// index-out-of-bounds / `usize` underflow / invalid-`Rect` **panic**. With `panic = "abort"` in the
/// release profile that is exit code `134` and a stack trace, where the user gave a merely malformed
/// file.
///
/// Checked here (all of it cheap, on the external representation):
/// * the strip width is finite and `> 0`;
/// * every `item_id` exists in the instance;
/// * every transformation is finite;
/// * the per-item-id placement counts match the demand **exactly**.
///
/// The last one is not merely defensive. `SPProblem::restore` cannot invent placements for missing
/// demand and does not reject extra ones, so an incomplete warm start used to be optimized and
/// exported with items silently missing (measured: demand 2, one placement, exit `0`, one item in
/// the output), and an over-complete one exported with items duplicated.
pub fn validate_spp_warm_start(ext_instance: &ExtSPInstance, ext_solution: &ExtSPSolution) -> Result<()> {
    use std::collections::BTreeMap;

    let width = ext_solution.strip_width;
    if !width.is_finite() || width <= 0.0 {
        anyhow::bail!("the warm start solution has an invalid strip width ({width}); it must be finite and > 0");
    }

    let demand: BTreeMap<u64, u64> = ext_instance.items.iter()
        .map(|it| (it.base.id, it.demand))
        .collect();
    // The declared orientations, per item id, for the rotation gate below.
    let orientations: BTreeMap<u64, Option<Vec<f32>>> = ext_instance.items.iter()
        .map(|it| (it.base.id, it.base.allowed_orientations.clone()))
        .collect();

    let mut placed: BTreeMap<u64, u64> = BTreeMap::new();
    for (idx, pi) in ext_solution.layout.placed_items.iter().enumerate() {
        if !demand.contains_key(&pi.item_id) {
            anyhow::bail!("the warm start solution places an unknown item id {} (placement #{idx}); \
                           the instance defines item id(s) {:?}",
                pi.item_id, demand.keys().collect::<Vec<_>>());
        }
        let t = &pi.transformation;
        if !t.rotation.is_finite() || !t.translation.0.is_finite() || !t.translation.1.is_finite() {
            anyhow::bail!("the warm start solution has a non-finite transformation for item {} (placement #{idx})", pi.item_id);
        }
        // **Rotation gate.** A warm start is replayed as-is, so a disallowed angle is carried
        // straight into the exported answer. It is invisible geometrically — the layout can be
        // perfectly collision-free — but an item declares `allowed_orientations` precisely because
        // the material has a grain/pattern direction, so an angle outside the list is scrap. The
        // audit's repro was exactly this: `allowed_orientations: [0.0]` with a 45° placement,
        // exported at exit 0.
        let allowed = orientations.get(&pi.item_id).and_then(|o| o.as_deref());
        if !crate::util::rotations::ext_orientation_ok(t.rotation, allowed, crate::util::verify::ROTATION_TOL_DEG) {
            anyhow::bail!("the warm start solution places item {} (placement #{idx}) at {}°, which is \
                           not one of its allowed_orientations ({}); a warm start is restored as-is, \
                           so this rotation would be carried straight through to the exported solution",
                pi.item_id, t.rotation,
                match allowed {
                    None => "any (continuous rotation)".to_string(),
                    Some([]) => "0° only (an empty list means a fixed orientation)".to_string(),
                    Some(list) => list.iter().map(|a| format!("{a}°")).collect::<Vec<_>>().join(", "),
                });
        }

        *placed.entry(pi.item_id).or_insert(0) += 1;
    }

    if placed != demand {
        let mut ids: Vec<u64> = demand.keys().chain(placed.keys()).copied().collect();
        ids.sort_unstable();
        ids.dedup();
        let detail = ids.into_iter()
            .filter_map(|id| {
                let (p, d) = (placed.get(&id).copied().unwrap_or(0), demand.get(&id).copied().unwrap_or(0));
                (p != d).then(|| format!("item {id}: placed {p}, demanded {d}"))
            })
            .collect::<Vec<_>>()
            .join("; ");
        anyhow::bail!("the warm start solution does not cover the instance demand exactly ({detail}); \
                       a warm start is restored as-is, so a missing or extra placement would be \
                       carried straight through to the exported solution");
    }
    Ok(())
}

pub fn read_spp_input(path: &Path) -> Result<(ExtSPInstance, Option<ExtSPSolution>)> {
    let input_str = fs::read_to_string(path).context("could not read input file")?;
    //try parsing a full output (instance + solution)
    match serde_json::from_str::<ExtSPOutput>(&input_str) {
        Ok(ext_output) => {
            Ok((ext_output.instance, Some(ext_output.solution)))
        }
        Err(_) => {
            //try parsing just the instance
            let ext_instance = serde_json::from_str::<ExtSPInstance>(&input_str)
                .context("could not parse instance from input file")?;
            Ok((ext_instance, None))
        }
    }
}
