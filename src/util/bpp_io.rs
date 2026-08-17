//! I/O for the bin packing (BPP) pipeline: input parsing, warm-start import, JSON/SVG export.
//!
//! Mirrors [`crate::util::io`] and [`crate::util::svg_exporter`], but for
//! [`jagua_rs::probs::bpp`] entities. A BPP solution consists of *multiple* layouts, so the
//! exporter writes one SVG per bin instead of a single one.
//!
//! The three accepted input formats (see [`read_bpp_input`]):
//! 1. [`ExtBPOutput`] — a previously written BPP solution file (instance + solution): warm start.
//! 2. [`ExtBPInstance`] — a plain bin packing instance.
//! 3. [`ExtSPInstance`] — a strip packing instance (items only), combined with `--bin` specs
//!    given on the command line, which are turned into rectangular bins.

use crate::consts::DRAW_OPTIONS;
use crate::util::io;
use crate::util::listener::ReportType;
use anyhow::{bail, Context, Result};
use clap::Parser;
use itertools::Itertools;
use jagua_rs::entities::Instance;
use jagua_rs::geometry::DTransformation;
use jagua_rs::io::ext_repr::{ExtContainer, ExtShape};
use jagua_rs::io::import::ext_to_int_transformation;
use jagua_rs::io::svg::s_layout_to_svg;
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPProblem, BPSolution};
use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBPSolution, ExtBin, ExtItem};
use jagua_rs::probs::spp::io::ext_repr::ExtSPInstance;
use log::Level;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::fs;
use std::path::Path;
use std::str::FromStr;

/// Default stock (number of available copies) of a bin type declared on the command line.
pub const DEFAULT_BIN_STOCK: usize = 1000;

/// Default cost of a bin type declared on the command line.
pub const DEFAULT_BIN_COST: u64 = 1;

/// Command line interface of the `sparrow-bpp` binary.
///
/// Mirrors [`crate::util::io::MainCli`], extended with the repeatable `--bin` flag which allows
/// running a plain strip packing instance (items only) as a bin packing problem.
#[derive(Parser)]
#[command(name = "sparrow-bpp", about = "Bin packing (BPP) variant of sparrow")]
pub struct BppCli {
    /// Path to input file (mandatory)
    #[arg(short = 'i', long, help = "Path to the input JSON file (BPP instance, SPP instance + --bin, or a BPP solution JSON file for warm starting)")]
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

    /// Number of independent optimizations to run in parallel (different seeds), keeping the best final solution
    #[arg(short = 'p', long, default_value_t = 1, value_parser = clap::value_parser!(u64).range(1..),
        help = "Run N independent optimizations in parallel (seed, seed+1, ...) within the same time limit and keep the best result. \
                Uses otherwise idle CPU cores; each run uses its own worker threads (see #workers in the log)")]
    pub parallel_runs: u64,

    /// Bin types (repeatable). Only used when the input file does not already define bins.
    #[arg(long = "bin", value_name = "WxH[:stock[:cost]]",
        help = "Declare a rectangular bin type, e.g. --bin 3200x3200:10:1 (stock defaults to 1000, cost to 1). Repeatable; bins get ids 0.. in the order given")]
    pub bins: Vec<BinSpec>,
}

/// A rectangular bin type as declared on the command line: `WxH[:stock[:cost]]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BinSpec {
    /// Width of the bin
    pub width: f32,
    /// Height of the bin
    pub height: f32,
    /// Number of available copies of this bin type
    pub stock: usize,
    /// Cost of using one copy of this bin type
    pub cost: u64,
}

impl BinSpec {
    /// Converts this spec into an external bin representation with the given id.
    pub fn to_ext_bin(self, id: usize) -> ExtBin {
        ExtBin {
            base: ExtContainer {
                id: id as u64,
                shape: ExtShape::Rectangle {
                    x_min: 0.0,
                    y_min: 0.0,
                    width: self.width,
                    height: self.height,
                },
                zones: vec![],
            },
            stock: self.stock,
            cost: self.cost,
        }
    }
}

impl fmt::Display for BinSpec {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}x{}:{}:{}", self.width, self.height, self.stock, self.cost)
    }
}

impl FromStr for BinSpec {
    type Err = String;

    /// Parses `WxH`, `WxH:stock` or `WxH:stock:cost`.
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut parts = s.split(':');
        let dims = parts.next().ok_or_else(|| format!("empty bin spec: '{s}'"))?;

        let (w_str, h_str) = dims
            .split_once(['x', 'X'])
            .ok_or_else(|| format!("invalid bin spec '{s}': expected WxH[:stock[:cost]]"))?;

        let width: f32 = w_str.trim().parse()
            .map_err(|_| format!("invalid bin width '{w_str}' in '{s}'"))?;
        let height: f32 = h_str.trim().parse()
            .map_err(|_| format!("invalid bin height '{h_str}' in '{s}'"))?;

        if !(width.is_finite() && width > 0.0 && height.is_finite() && height > 0.0) {
            return Err(format!("bin dimensions must be finite and positive: '{s}'"));
        }

        let stock = match parts.next() {
            Some(stock_str) => stock_str.trim().parse::<usize>()
                .map_err(|_| format!("invalid bin stock '{stock_str}' in '{s}'"))?,
            None => DEFAULT_BIN_STOCK,
        };
        if stock == 0 {
            return Err(format!("bin stock must be > 0: '{s}'"));
        }

        let cost = match parts.next() {
            Some(cost_str) => cost_str.trim().parse::<u64>()
                .map_err(|_| format!("invalid bin cost '{cost_str}' in '{s}'"))?,
            None => DEFAULT_BIN_COST,
        };

        if parts.next().is_some() {
            return Err(format!("too many ':' separated fields in bin spec '{s}': expected WxH[:stock[:cost]]"));
        }

        Ok(BinSpec { width, height, stock, cost })
    }
}

/// A complete BPP output file: the instance and its solution, serialised into one flat JSON object.
///
/// BPP counterpart of [`crate::util::io::ExtSPOutput`].
#[derive(Serialize, Deserialize, Clone)]
pub struct ExtBPOutput {
    #[serde(flatten)]
    pub instance: ExtBPInstance,
    pub solution: ExtBPSolution,
}

/// Reads a bin packing input file.
///
/// Three formats are tried in order:
/// 1. [`ExtBPOutput`] (instance + solution) → warm start,
/// 2. [`ExtBPInstance`] (bins included in the file) → `bins` is ignored,
/// 3. [`ExtSPInstance`] (items only) → combined with `bins` into an [`ExtBPInstance`] with
///    rectangular bins, ids `0..` in the order given.
///
/// Fails when a strip packing instance is given without any `--bin` spec.
pub fn read_bpp_input(path: &Path, bins: &[BinSpec]) -> Result<(ExtBPInstance, Option<ExtBPSolution>)> {
    let input_str = fs::read_to_string(path)
        .with_context(|| format!("could not read input file {}", path.display()))?;

    // 1. A full BPP output (instance + solution)
    if let Ok(ext_output) = serde_json::from_str::<ExtBPOutput>(&input_str) {
        log::info!("[BPIO] parsed input as a BPP solution file ({} layouts), warm starting", ext_output.solution.layouts.len());
        return Ok((ext_output.instance, Some(ext_output.solution)));
    }

    // 2. A plain BPP instance
    if let Ok(ext_instance) = serde_json::from_str::<ExtBPInstance>(&input_str) {
        log::info!("[BPIO] parsed input as a BPP instance with {} bin type(s)", ext_instance.bins.len());
        if !bins.is_empty() {
            log::warn!("[BPIO] the input file already defines bins, ignoring the {} --bin argument(s)", bins.len());
        }
        return Ok((ext_instance, None));
    }

    // 3. A strip packing instance + the bins from the command line
    let sp_instance = serde_json::from_str::<ExtSPInstance>(&input_str)
        .context("could not parse input file as a BPP solution, a BPP instance, or an SPP instance")?;

    if bins.is_empty() {
        bail!("'{}' is a strip packing instance (no bins); provide at least one --bin WxH[:stock[:cost]]", path.display());
    }

    let ext_instance = bp_instance_from_sp(&sp_instance, bins);
    log::info!("[BPIO] parsed input as an SPP instance, synthesised {} bin type(s): {}",
        bins.len(), bins.iter().map(|b| b.to_string()).join(", "));

    Ok((ext_instance, None))
}

/// Builds an [`ExtBPInstance`] from a strip packing instance (items only) and a list of bin specs.
///
/// The bins are rectangles anchored at the origin and receive ids `0..` in the order given.
pub fn bp_instance_from_sp(sp_instance: &ExtSPInstance, bins: &[BinSpec]) -> ExtBPInstance {
    ExtBPInstance {
        name: sp_instance.name.clone(),
        items: sp_instance.items.iter()
            .map(|it| ExtItem { base: it.base.clone(), demand: it.demand })
            .collect(),
        bins: bins.iter().enumerate()
            .map(|(id, spec)| spec.to_ext_bin(id))
            .collect(),
    }
}

/// Imports an external BPP solution into the library.
///
/// jagua-rs' own `bpp::io::import_solution` is `unimplemented!()`, so this mirrors the strip
/// packing logic in `jagua_rs::probs::spp::io::import_solution`: a fresh [`BPProblem`] is built and
/// every placement is replayed. Per layout the *first* item opens a new bin
/// ([`BPLayoutType::Closed`] with `bin_id = container_id`) and all remaining items go into that
/// same layout ([`BPLayoutType::Open`]).
///
/// Fails (`Err`) if the solution references an unknown bin/item id, if a bin's stock is exhausted
/// (`place_item(Closed{..})` itself does not check stock), or if it places more copies of an item
/// than the instance demands (`BPProblem::register_included_item` decrements an unchecked `usize`
/// and would panic with an arithmetic overflow instead of reporting the malformed input).
pub fn import_bp_solution(instance: &BPInstance, ext: &ExtBPSolution) -> Result<BPSolution> {
    let mut prob = BPProblem::new(instance.clone());

    for (idx, ext_layout) in ext.layouts.iter().enumerate() {
        let bin_id = ext_layout.container_id as usize;
        if bin_id >= instance.bins.len() {
            bail!("layout {idx} references unknown bin id {bin_id} (instance has {} bin types)", instance.bins.len());
        }
        if ext_layout.placed_items.is_empty() {
            log::warn!("[BPIO] layout {idx} (bin {bin_id}) is empty, skipping it");
            continue;
        }
        if prob.bin_stock_qtys[bin_id] == 0 {
            bail!("layout {idx}: no stock left for bin type {bin_id}");
        }

        let mut lkey = None;
        for ext_placement in ext_layout.placed_items.iter() {
            let item_id = ext_placement.item_id as usize;
            if item_id >= instance.items.len() {
                bail!("layout {idx} references unknown item id {item_id}");
            }
            // `place_item` -> `register_included_item` decrements `item_demand_qtys[item_id]`
            // without checking, so an over-placed item would panic (usize underflow) instead of
            // producing a readable error. Check the remaining demand up front.
            if prob.item_demand_qtys[item_id] == 0 {
                bail!("layout {idx} places more copies of item {item_id} than the instance demands ({})",
                    instance.item_qty(item_id));
            }
            let d_transf = {
                let ext_transf = DTransformation::from(ext_placement.transformation.clone());
                let item = instance.item(item_id);
                ext_to_int_transformation(&ext_transf, &item.shape_orig.pre_transform)
            };
            // The first item opens the bin, the rest join the layout it created.
            let layout_id = match lkey {
                None => BPLayoutType::Closed { bin_id },
                Some(lkey) => BPLayoutType::Open(lkey),
            };
            let (new_lkey, _) = prob.place_item(BPPlacement { layout_id, item_id, d_transf });
            lkey = Some(new_lkey);
        }
    }

    Ok(prob.save())
}

/// Exports a BPP solution into its external representation.
///
/// Thin wrapper around [`jagua_rs::probs::bpp::io::export`] so callers do not have to reach into
/// jagua-rs directly (and so the epoch handling stays in one place).
pub fn export_bp(instance: &BPInstance, solution: &BPSolution) -> ExtBPSolution {
    jagua_rs::probs::bpp::io::export(instance, solution, *crate::EPOCH)
}

/// Writes an [`ExtBPOutput`] to a JSON file.
pub fn write_bp_json(output: &ExtBPOutput, path: &Path) -> Result<()> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        fs::create_dir_all(parent).context("could not create parent directory for json file")?;
    }
    io::write_json(output, path, Level::Info)
}

/// Summarises a BPP solution for logging: cost, number of bins, overall density and the density of
/// each bin (in [`LayKey`](jagua_rs::probs::bpp::entities::LayKey) order, which is deterministic).
pub fn summarize(sol: &BPSolution, instance: &BPInstance) -> String {
    let per_bin = sol.layout_snapshots.iter()
        .map(|(_, ls)| format!("{:.1}%", ls.density(instance) * 100.0))
        .join(", ");
    format!(
        "cost: {}, #bins: {}, density: {:.3}%, per-bin: [{}]",
        sol.cost(instance),
        sol.layout_snapshots.len(),
        sol.density(instance) * 100.0,
        per_bin
    )
}

/// Trait for listeners that can receive BPP solutions during the optimization process.
///
/// BPP counterpart of [`crate::util::listener::SolutionListener`]; the [`ReportType`] variants are
/// shared with the strip packing pipeline.
pub trait BPSolutionListener {
    /// Called by the optimizer whenever a noteworthy solution is reached.
    fn report(&mut self, report: ReportType, solution: &BPSolution, instance: &BPInstance);
}

/// A dummy implementation of [`BPSolutionListener`] that does nothing.
pub struct DummyBPSolListener;

impl BPSolutionListener for DummyBPSolListener {
    fn report(&mut self, _report: ReportType, _solution: &BPSolution, _instance: &BPInstance) {
        // Do nothing
    }
}

/// Writes SVG files of BPP solutions: one file per bin.
///
/// BPP counterpart of [`crate::util::svg_exporter::SvgExporter`]. Because a BPP solution consists
/// of several layouts, every report produces *n* files, suffixed with `_bin{idx}` where `idx`
/// enumerates the layout snapshots in `LayKey` order (deterministic `SecondaryMap` iteration).
pub struct BPSvgExporter {
    svg_counter: usize,
    /// Path template for the final SVG files, if provided. `foo.svg` becomes `foo_bin0.svg`, ...
    pub final_path: Option<String>,
    /// Directory to write all intermediate solution SVG files to, if provided
    pub intermediate_dir: Option<String>,
    /// Directory to write the live SVG files to, if provided
    pub live_dir: Option<String>,
}

impl BPSvgExporter {
    /// Creates a new exporter. Any pre-existing `.svg` files in `intermediate_dir` are removed,
    /// exactly like the SPP exporter does.
    pub fn new(final_path: Option<String>, intermediate_dir: Option<String>, live_dir: Option<String>) -> Self {
        if let Some(intermediate_dir) = &intermediate_dir
            && let Ok(files_in_dir) = fs::read_dir(Path::new(intermediate_dir))
        {
            for file in files_in_dir.flatten() {
                if file.path().extension().unwrap_or_default() == "svg" {
                    let _ = fs::remove_file(file.path());
                }
            }
        }

        BPSvgExporter { svg_counter: 0, final_path, intermediate_dir, live_dir }
    }

    /// Writes one SVG per layout snapshot, named `{stem}_bin{idx}.svg` inside `dir`.
    fn write_all_bins(&self, solution: &BPSolution, instance: &BPInstance, dir: &Path, stem: &str, log_lvl: Level) {
        for (idx, (_lkey, snapshot)) in solution.layout_snapshots.iter().enumerate() {
            let name = format!("{stem}_bin{idx}");
            let svg = s_layout_to_svg(snapshot, instance, DRAW_OPTIONS, name.as_str());
            let path = dir.join(format!("{name}.svg"));
            if let Err(e) = io::write_svg(&svg, &path, log_lvl) {
                log::warn!("[BPIO] failed to write svg {}: {e}", path.display());
            }
        }
    }
}

impl BPSolutionListener for BPSvgExporter {
    fn report(&mut self, report_type: ReportType, solution: &BPSolution, instance: &BPInstance) {
        let suffix = match report_type {
            ReportType::CmprFeas => "cmpr",
            ReportType::ExplInfeas => "expl_nf",
            ReportType::ExplFeas => "expl_f",
            ReportType::Final => "final",
            ReportType::ExplImproving => "expl_i",
        };
        let stem = format!("{}_{}_{}", self.svg_counter, solution.cost(instance), suffix);

        if let Some(live_dir) = &self.live_dir {
            self.write_all_bins(solution, instance, Path::new(live_dir), ".live_solution", Level::Trace);
        }
        if let Some(intermediate_dir) = &self.intermediate_dir
            && report_type != ReportType::ExplImproving
        {
            self.write_all_bins(solution, instance, Path::new(intermediate_dir), &stem, Level::Trace);
            self.svg_counter += 1;
        }
        if let Some(final_path) = &self.final_path
            && report_type == ReportType::Final
        {
            let path = Path::new(final_path);
            let dir = path.parent().unwrap_or(Path::new("."));
            let final_stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("final");
            self.write_all_bins(solution, instance, dir, final_stem, Level::Info);
        }
    }
}
