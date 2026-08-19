use crate::optimizer::separator::SeparatorConfig;
use crate::sample::search::SampleConfig;
use jagua_rs::collision_detection::CDEConfig;
use jagua_rs::geometry::fail_fast::SPSurrogateConfig;
use crate::optimizer::bpp::shelf::Constructive;
use std::time::Duration;


/// Configuration of the **multi-sheet ("walled") strip packing** mode
/// (see [`crate::optimizer::sheets`]).
///
/// When present, a wall is inserted into the strip at every sheet boundary, so no item can straddle
/// a boundary and the strip can be cut into physical sheets of `width` without relocating anything.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SheetConfig {
    /// Usable width of one physical sheet (mm). Sheet height = the instance's strip height.
    pub width: f32,
    /// Thickness of the (virtual) wall between two consecutive sheets (mm).
    ///
    /// The sheets are separate physical objects, so this gap costs nothing; making it thick gives
    /// the wall a well-covered pole surrogate and therefore a smooth GLS loss gradient.
    /// See [`SheetConfig::resolve_gap`].
    pub gap: f32,
    /// **Per-sheet left-compaction post-pass.** When enabled, after the compression phase every
    /// sheet except the last one is re-compacted to the left *within its own sheet*, so the leftover
    /// of each sheet becomes one wide, reusable right-hand band instead of many small gaps between
    /// the parts.
    ///
    /// This is a purely *secondary* objective: it cannot reduce the sheet count (that is already
    /// minimised by the walled strip width) nor the strip width, it only redistributes the slack
    /// inside a sheet. See [`crate::optimizer::sheets::compact_sheets_left`].
    pub compact_sheets: bool,

    /// **Sheet-drop move** (phase 8). How many *failed* drop attempts are tolerated before the
    /// exploration phase gives up on dropping sheets and spends the rest of its budget on the plain
    /// fine shrink (which is what minimises the last sheet's band).
    ///
    /// Each strike is one full "relocate the last sheet's items into the earlier sheets and try to
    /// separate" attempt. See [`crate::optimizer::sheets::try_drop_sheet`].
    pub sheet_drop_strikes: usize,

    /// **Area bound** on the sheet-drop move, mirroring the BPP's
    /// [`BPExplorationConfig::max_reduction_density`]: if the total item area divided by the area
    /// of `n - 1` sheets exceeds this, dropping a sheet would require a nesting density that is out
    /// of reach in practice, and no attempt is made at all.
    pub max_reduction_density: f32,

    /// Whether the compression phase runs the **cross-sheet pack-down**: cut the last sheet's band
    /// back in large steps, relocating whatever no longer fits into the earlier sheets and making
    /// room with a short `separate()`. See [`crate::optimizer::sheets::pack_down_sheets`].
    ///
    /// **Defaults to `false`.** The step is correct and does what it says, but on all four measured
    /// instances (iso6, iso7, madisocad_iso, swim) not a single band cut was ever accepted, while
    /// the failed attempts cost 2–10 s of a 20 s compression budget — on madisocad_iso enough to
    /// leave the final band ~56 mm worse than without it. Since it is the fine compression that
    /// actually shortens the band on these instances, the step is opt-in rather than on by default;
    /// the sheet-drop move in the exploration phase is the operator that pays off.
    pub pack_down: bool,

    /// Wall-clock budget for a single pack-down band cut.
    pub pack_down_move_time_limit: Duration,

    /// Fraction of the compression phase's budget the pack-down step may use at most; the rest is
    /// left to the fine compression that shortens the last band.
    ///
    /// Deliberately a minority share: the pack-down either finds a cut in its first attempt or two,
    /// or there is none to find, and on an instance where there is none every second it spends is a
    /// second the fine compression does not get.
    pub pack_down_time_ratio: f32,

    /// How the walled run is *started*. See [`SheetPipeline`].
    pub pipeline: SheetPipeline,

    /// Fraction of the exploration budget the [`SheetPipeline::PlainFirst`] pipeline spends on its
    /// wall-less pre-pass. The remainder goes to the walled exploration that follows.
    pub plain_first_ratio: f32,
}

/// How a walled (`--sheet-width`) run gets its starting layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SheetPipeline {
    /// **Walls from the start** (default): the starting layout is built with the walls already in
    /// place (LBF respects them) and the exploration works with them from its first iteration.
    ///
    /// This is the phase-7 behaviour, and it remains the default because
    /// [`SheetPipeline::PlainFirst`] was measured **not to beat it** on any of the four reference
    /// instances once the wall-installation repair is required to actually produce a feasible
    /// layout (see `docs/sheets.md`).
    #[default]
    WalledFromStart,
    /// **Plain strip first, walls after.** The exploration phase is run *without* walls for
    /// `plain_first_ratio` of its budget, which lets the strip engine do what it is best at — it
    /// reaches a far higher density when it is not fighting the walls (iso7: 83.1 % vs 61.5 %). The
    /// resulting layout is then cut into sheets: the strip is chopped into chunks of `W - slack`,
    /// every item is translated right onto its own sheet (so the *whole chunk* moves together and
    /// its internal neighbour relationships survive), the walls are installed, and the separator
    /// repairs the items that were straddling a chunk boundary.
    ///
    /// The repair is **verified**: if it does not reach zero loss, the cut is retried with more
    /// slack per sheet, and if that still fails the run falls back to a walled LBF start. Handing an
    /// unrepaired layout to the exploration phase is not an option — that phase trusts its starting
    /// solution without testing it and would report the wall-straddling start as the final answer.
    PlainFirst,
}

/// Default for [`SheetConfig::sheet_drop_strikes`].
pub const DEFAULT_SHEET_DROP_STRIKES: usize = 3;

/// Default for [`SheetConfig::max_reduction_density`]; same value (and same reasoning) as the BPP's
/// [`BPExplorationConfig::max_reduction_density`].
pub const DEFAULT_SHEET_MAX_REDUCTION_DENSITY: f32 = 0.90;

/// Default wall thickness when `--sheet-gap` is not given: at least [`MIN_DEFAULT_SHEET_GAP`] mm,
/// and never less than twice the minimum item separation (so the wall stays thicker than the
/// clearance it has to enforce).
pub const MIN_DEFAULT_SHEET_GAP: f32 = 20.0;

impl SheetConfig {
    /// A sheet configuration with the default phase-8 settings (sheet-drop enabled with
    /// [`DEFAULT_SHEET_DROP_STRIKES`] strikes, pack-down enabled).
    pub fn new(width: f32, gap: f32, compact_sheets: bool) -> Self {
        Self {
            width,
            gap,
            compact_sheets,
            sheet_drop_strikes: DEFAULT_SHEET_DROP_STRIKES,
            max_reduction_density: DEFAULT_SHEET_MAX_REDUCTION_DENSITY,
            pack_down: false,
            pack_down_move_time_limit: Duration::from_secs(2),
            pack_down_time_ratio: 0.3,
            pipeline: SheetPipeline::WalledFromStart,
            plain_first_ratio: 0.5,
        }
    }

    /// Distance between the left edges of two consecutive sheets: `width + gap`.
    pub fn pitch(&self) -> f32 {
        self.width + self.gap
    }

    /// Resolves the gap to use: **any** explicit CLI value if given (including `0`), otherwise
    /// `max(MIN_DEFAULT_SHEET_GAP, 2 * min_item_separation)`.
    ///
    /// `--sheet-gap 0` is a legitimate request — sheets butt up against each other with no kerf —
    /// and used to be swallowed by the `g > 0.0` guard, silently substituting the 20 mm default and
    /// producing a layout laid out on a pitch the caller never asked for. It is honoured now, with a
    /// warning: a zero-width wall cannot separate the sheets geometrically, so an item may sit
    /// exactly on a boundary and the cut has no kerf allowance.
    pub fn resolve_gap(cli_gap: Option<f32>, min_item_separation: Option<f32>) -> f32 {
        match cli_gap {
            Some(g) if g > 0.0 => g,
            Some(0.0) => {
                log::warn!("[CFG] --sheet-gap 0: the sheet boundaries get zero-width walls, so items \
                            may touch a boundary exactly and the cut has no kerf allowance");
                0.0
            }
            Some(g) => {
                log::warn!("[CFG] ignoring a negative --sheet-gap ({g}); using the default instead");
                MIN_DEFAULT_SHEET_GAP.max(2.0 * min_item_separation.unwrap_or(0.0))
            }
            None => MIN_DEFAULT_SHEET_GAP.max(2.0 * min_item_separation.unwrap_or(0.0)),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SparrowConfig {
    pub rng_seed: Option<usize>,
    pub expl_cfg: ExplorationConfig,
    pub cmpr_cfg: CompressionConfig,
    /// Configuration for the collision detection engine.
    /// See [`CDEConfig`] for more details.
    pub cde_config: CDEConfig,
    /// Defines the polygon simplification tolerance: maximum allowable inflation of items when simplifying their shape.
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub poly_simpl_tolerance: Option<f32>,
    /// Defines the minimum distance between items and other hazards.
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub min_item_separation: Option<f32>,
    /// Defines a maximum distance and area of a concavity to be considered "narrow" (which will be closed).
    /// Disabled if `None`.
    /// See [`jagua_rs::io::parser::Parser::new`] for more details.
    pub narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
    /// Multi-sheet ("walled") strip packing mode. `None` = plain strip packing (default), in which
    /// case the behaviour is bit-identical to before this mode existed.
    pub sheet: Option<SheetConfig>,
}

#[derive(Debug, Clone, Copy)]
pub struct ExplorationConfig {
    pub shrink_step: f32,
    pub time_limit: Duration,
    pub max_conseq_failed_attempts: Option<usize>,
    pub solution_pool_distribution_stddev: f32,
    pub separator_config: SeparatorConfig,
    pub large_item_ch_area_cutoff_percentile: f32,
    /// See [`SparrowConfig::sheet`]. Propagated from there by [`SparrowConfig::apply_sheet`].
    pub sheet: Option<SheetConfig>,
}

#[derive(Debug, Clone, Copy)]
pub struct CompressionConfig {
    pub shrink_range: (f32, f32),
    pub time_limit: Duration,
    pub shrink_decay: ShrinkDecayStrategy,
    pub separator_config: SeparatorConfig,
    /// See [`SparrowConfig::sheet`]. Propagated from there by [`SparrowConfig::apply_sheet`].
    pub sheet: Option<SheetConfig>,
}

#[derive(Debug, Clone, Copy)]
pub enum ShrinkDecayStrategy {
    /// The shrink ratio decays linearly with time
    TimeBased,
    /// The shrink ratio decays by a fixed ratio every time it fails to compress into a feasible solution
    FailureBased(f32),
}

pub const DEFAULT_SPARROW_CONFIG: SparrowConfig = SparrowConfig {
    rng_seed: None,
    expl_cfg: ExplorationConfig {
        shrink_step: 0.001,
        time_limit: Duration::from_secs(9 * 60),
        max_conseq_failed_attempts: None,
        solution_pool_distribution_stddev: 0.25,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 200,
            strike_limit: 3,
            log_level: log::Level::Info,
            n_workers: 3,
            sample_config: SampleConfig {
                n_container_samples: 50,
                n_focussed_samples: 25,
                n_coord_descents: 3,
            },
        },
        large_item_ch_area_cutoff_percentile: 0.75,
        sheet: None,
    },
    cmpr_cfg: CompressionConfig {
        shrink_range: (0.0005, 0.00001),
        time_limit: Duration::from_secs(60),
        shrink_decay: ShrinkDecayStrategy::TimeBased,
        separator_config: SeparatorConfig {
            iter_no_imprv_limit: 100,
            strike_limit: 5,
            log_level: log::Level::Debug,
            n_workers: 3,
            sample_config: SampleConfig {
                n_container_samples: 50,
                n_focussed_samples: 25,
                n_coord_descents: 3,
            },
        },
        sheet: None,
    },
    cde_config: CDEConfig {
        quadtree_depth: 4,
        cd_threshold: 16,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(64, 0.0), (16, 0.8), (8, 0.9)],
            n_ff_poles: 1,
            n_ff_piers: 0,
        },
    },
    poly_simpl_tolerance: Some(0.001),
    narrow_concavity_cutoff_ratio: Some((0.01, 0.01)),
    min_item_separation: None,
    sheet: None,
};

impl SparrowConfig {
    /// Enables the multi-sheet mode and propagates the setting to both phase configurations.
    pub fn apply_sheet(&mut self, sheet: Option<SheetConfig>) {
        self.sheet = sheet;
        self.expl_cfg.sheet = sheet;
        self.cmpr_cfg.sheet = sheet;
    }
}
// ---------------------------------------------------------------------------------------------
// Bin Packing Problem (BPP) configuration
// ---------------------------------------------------------------------------------------------

/// Top-level configuration of the BPP pipeline ([`crate::optimizer::bpp::optimize_bpp`]).
///
/// Mirrors [`SparrowConfig`]: the geometry-related settings (`cde_config`, the shape-modification
/// tolerances) are identical, only the phase configurations differ because the BPP explores a
/// discrete objective (the number/cost of bins) instead of a continuous one (the strip width).
#[derive(Debug, Clone, Copy)]
pub struct BPConfig {
    pub rng_seed: Option<usize>,
    /// Which constructive heuristic builds the starting solution. See [`Constructive`].
    pub constructive: Constructive,
    pub expl_cfg: BPExplorationConfig,
    pub cmpr_cfg: BPCompressionConfig,
    /// Configuration for the collision detection engine. See [`CDEConfig`].
    pub cde_config: CDEConfig,
    /// Polygon simplification tolerance: maximum allowable inflation of items when simplifying
    /// their shape. Disabled if `None`.
    pub poly_simpl_tolerance: Option<f32>,
    /// Minimum distance between items and other hazards. Disabled if `None`.
    pub min_item_separation: Option<f32>,
    /// Maximum distance and area of a concavity to be considered "narrow" (which will be closed).
    /// Disabled if `None`.
    pub narrow_concavity_cutoff_ratio: Option<(f32, f32)>,
}

/// Configuration of the BPP exploration phase ([`crate::optimizer::bpp::explore::exploration_phase`]).
#[derive(Debug, Clone, Copy)]
pub struct BPExplorationConfig {
    /// Wall-clock budget for the exploration phase.
    pub time_limit: Duration,
    /// Stop after this many consecutive failed bin-removal attempts. Unlimited if `None`.
    pub max_conseq_failed_attempts: Option<usize>,
    /// Reuses the SPP [`SeparatorConfig`] as-is.
    pub separator_config: SeparatorConfig,
    /// Standard deviation of the half-normal distribution used to pick a solution from the pool of
    /// infeasible solutions (0 = always the best one, larger = more diverse).
    pub solution_pool_distribution_stddev: f32,
    /// Which items count as 'large' during disruption: the top percentile of the cumulative convex
    /// hull area of all items.
    pub large_item_ch_area_cutoff_percentile: f32,
    /// How many different scatter targets are tried before falling back to the least dense bin
    /// again. Consecutive attempts use the 1st, 2nd, ... least dense open layout as the bin to close.
    pub n_scatter_retries: usize,
    /// **Area-based feasibility bound** on the bin-count reduction.
    ///
    /// Before attempting to eliminate a bin, the exploration phase computes the density the
    /// remaining `n - 1` *densest* bins would have to reach to hold all the placed item area. If
    /// that required density exceeds this cap, the reduction is provably (area-wise) hopeless or
    /// hopelessly unlikely, and the phase returns immediately instead of burning its whole budget
    /// on impossible attempts.
    ///
    /// * `1.0` = pure area bound (only mathematically impossible reductions are skipped),
    /// * `0.90` (default) = also skip reductions that would need a nesting density above 90 %,
    ///   which is out of reach for irregular parts in practice.
    pub max_reduction_density: f32,
    /// **Phase-wise stagnation stop.** If this many consecutive attempts at the *current* bin count
    /// fail without the best min-loss seen at that level improving by at least
    /// [`STAGNATION_MIN_IMPROVEMENT`], the exploration phase gives up early and hands its remaining
    /// budget to the compression phase.
    ///
    /// This is strictly finer-grained than `max_conseq_failed_attempts`: that one only counts
    /// failures, this one also looks at whether those failures are *getting anywhere*. A run that
    /// keeps lowering its min loss is making progress and is left alone; one whose loss oscillates
    /// around the same value never will. Disabled if `None`.
    pub stagnation_limit: Option<usize>,
}

/// Relative improvement in the best min-loss that counts as "progress" for the stagnation stop.
pub const STAGNATION_MIN_IMPROVEMENT: f32 = 0.02;

/// In which *direction* the pack-down step ([`crate::optimizer::bpp::compress::pack_down`]) moves
/// items between bins.
///
/// Both strategies leave the bin count and the total density untouched — they only decide **where
/// the slack sits**, which is the secondary objective. They are exact mirror images of each other:
/// the same code path is used, only the source/destination orderings are reversed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PackDownStrategy {
    /// **Concentrate** (default, recommended): sources are the *sparsest* bins, destinations the
    /// *densest* ones that still have room. The slack migrates out of the sparse bins into the
    /// dense ones, so the leftover ends up as **few, large** offcuts (in the limit: one nearly
    /// empty bin, i.e. a full sheet back in stock).
    #[default]
    Concentrate,
    /// **Spread**: the reverse direction — sources are the *densest* bins, destinations the
    /// *sparsest* ones with the most free area. This evens the leftover out over all bins (many
    /// medium-sized bands instead of a few large ones). Useful only when downstream processing
    /// wants a similar offcut in every sheet; for reusable material `Concentrate` is almost always
    /// the better choice.
    Spread,
}

impl PackDownStrategy {
    /// Whether the *source* bins are visited in ascending density order (`Concentrate`) or in
    /// descending order (`Spread`).
    pub fn source_ascending(self) -> bool {
        self == PackDownStrategy::Concentrate
    }

    /// Whether a layout of density `dst_density` is a valid destination for a source of density
    /// `src_density`: strictly denser under `Concentrate`, strictly sparser under `Spread`.
    ///
    /// Requiring strictness is what makes the step terminate — a move always goes "downhill" in a
    /// fixed direction, so two bins can never keep swapping the same item back and forth.
    pub fn accepts_destination(self, src_density: f32, dst_density: f32) -> bool {
        match self {
            PackDownStrategy::Concentrate => dst_density > src_density,
            PackDownStrategy::Spread => dst_density < src_density,
        }
    }
}

/// Configuration of the BPP compression phase ([`crate::optimizer::bpp::compress::compression_phase`]).
#[derive(Debug, Clone, Copy)]
pub struct BPCompressionConfig {
    /// Wall-clock budget for the compression phase.
    pub time_limit: Duration,
    /// Reuses the SPP [`SeparatorConfig`] as-is.
    pub separator_config: SeparatorConfig,
    /// Whether to attempt the (strip-packing based) consolidation of the least dense bin.
    /// If `false`, the compression phase only reports the per-bin statistics.
    pub consolidate_remainder: bool,
    /// Configuration of the *strip packing* sub-optimization used for the consolidation.
    /// Its `time_limit` acts as the budget for a single consolidation attempt.
    pub consolidation_expl_cfg: ExplorationConfig,
    /// Whether to run the **pack-down** step before the strip consolidation:
    /// repeatedly try to move items out of the least dense bin into the other (denser) bins, so
    /// the leftover material concentrates in a single bin (which may even empty out completely,
    /// reducing the bin count).
    pub pack_down: bool,
    /// Wall-clock budget for a single pack-down move attempt (one item into one destination bin).
    /// This caps the `separate()` call that has to "make room" for the newcomer.
    pub pack_down_move_time_limit: Duration,
    /// Fraction of the compression phase's budget the pack-down step may use at most. The rest is
    /// reserved for the strip consolidation that follows it, which is what actually turns the
    /// emptied bin's remainder into a single offcut.
    pub pack_down_time_ratio: f32,
    /// Separator configuration used by the pack-down step. Deliberately much cheaper than the
    /// exploration one: a pack-down attempt is a *local* repair (one extra item in one bin), and
    /// hundreds of them are made, so few iterations and few strikes per attempt.
    pub pack_down_separator_config: SeparatorConfig,
    /// Which *direction* the pack-down step moves items in. See [`PackDownStrategy`].
    pub pack_down_strategy: PackDownStrategy,
    /// Minimum budget granted to a single bin's strip consolidation when the remaining compression
    /// budget is shared fairly over all bins (`remaining / n_remaining_bins`, floored at this).
    pub consolidation_min_time_per_bin: Duration,
}

/// The BPP counterpart of [`DEFAULT_SPARROW_CONFIG`]: identical separator, sampling and geometry
/// settings, only the phase-specific knobs differ.
pub const DEFAULT_BPP_CONFIG: BPConfig = BPConfig {
    rng_seed: None,
    constructive: Constructive::Best,
    expl_cfg: BPExplorationConfig {
        time_limit: Duration::from_secs(9 * 60),
        max_conseq_failed_attempts: None,
        separator_config: DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config,
        solution_pool_distribution_stddev: 0.25,
        large_item_ch_area_cutoff_percentile: 0.75,
        n_scatter_retries: 3,
        max_reduction_density: 0.90,
        stagnation_limit: Some(8),
    },
    cmpr_cfg: BPCompressionConfig {
        time_limit: Duration::from_secs(60),
        separator_config: DEFAULT_SPARROW_CONFIG.cmpr_cfg.separator_config,
        consolidate_remainder: true,
        consolidation_expl_cfg: ExplorationConfig {
            shrink_step: 0.005,
            // Budget for a *single* consolidation attempt; the phase's own `time_limit` caps the total.
            time_limit: Duration::from_secs(30),
            max_conseq_failed_attempts: Some(crate::consts::DEFAULT_MAX_CONSEQ_FAILS_EXPL),
            solution_pool_distribution_stddev: 0.25,
            separator_config: DEFAULT_SPARROW_CONFIG.cmpr_cfg.separator_config,
            large_item_ch_area_cutoff_percentile: 0.75,
            // The BPP pipeline packs into real bins; sheet walls are an SPP-only concept.
            sheet: None,
        },
        pack_down: true,
        pack_down_move_time_limit: Duration::from_secs(2),
        pack_down_time_ratio: 0.6,
        pack_down_separator_config: SeparatorConfig {
            iter_no_imprv_limit: 50,
            strike_limit: 2,
            log_level: log::Level::Debug,
            n_workers: DEFAULT_SPARROW_CONFIG.cmpr_cfg.separator_config.n_workers,
            sample_config: DEFAULT_SPARROW_CONFIG.cmpr_cfg.separator_config.sample_config,
        },
        pack_down_strategy: PackDownStrategy::Concentrate,
        consolidation_min_time_per_bin: Duration::from_secs(1),
    },
    cde_config: DEFAULT_SPARROW_CONFIG.cde_config,
    poly_simpl_tolerance: DEFAULT_SPARROW_CONFIG.poly_simpl_tolerance,
    min_item_separation: DEFAULT_SPARROW_CONFIG.min_item_separation,
    narrow_concavity_cutoff_ratio: DEFAULT_SPARROW_CONFIG.narrow_concavity_cutoff_ratio,
};
