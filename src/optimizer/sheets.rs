//! Multi-sheet ("walled") strip packing.
//!
//! # What this is
//!
//! The plain strip-packing engine minimises the width of one continuous strip. If the material is
//! actually delivered as a series of **fixed-size sheets** (e.g. 2000 x 1000 mm), a strip solution
//! cannot be used directly: cutting the strip at multiples of the sheet width slices right through
//! the parts that happen to straddle a boundary, and every straddling part has to be relocated —
//! typically costing a whole extra sheet.
//!
//! This module makes the sheet boundaries part of the *optimization problem* instead of a
//! post-processing step. A vertical **wall** is inserted into the container at every sheet
//! boundary, as a [`HazardEntity::Hole`](jagua_rs::collision_detection::hazards::HazardEntity::Hole)
//! (a quality-0 [`InferiorQualityZone`]). Items may not overlap a wall, so no part can ever straddle
//! a boundary, while the separator keeps its full global GLS compaction: sheet 1 fills up, then
//! sheet 2, and only the *last* sheet ends up with an unused end band. Minimising the strip width
//! automatically minimises the sheet count `ceil(width / (W + gap))`.
//!
//! # Geometry and coordinate mapping
//!
//! With sheet width `W` and gap `g`, the walled strip of width `L` looks like:
//!
//! ```text
//!   sheet 0          wall 1     sheet 1          wall 2     sheet 2
//! |<----- W ----->|<-- g -->|<----- W ----->|<-- g -->|<-- ... -->|
//! 0               W        W+g            2W+g      2W+2g
//! ```
//!
//! Wall `k` (for `k = 1, 2, ...`) covers the x-interval `[k*(W+g) - g, k*(W+g)]`. A point at strip
//! coordinate `x` therefore lies on sheet
//!
//! ```text
//!   k       = floor(x / (W + g))
//!   x_local = x - k * (W + g)          (in [0, W], the coordinate on the physical sheet)
//! ```
//!
//! The y coordinate is unchanged: sheets share the strip's fixed height.
//!
//! The gap `g` is **virtual** — the sheets are separate physical objects, so nothing is lost by
//! making the gap generous. A comfortably thick wall is in fact desirable: the wall's polygon gets
//! a well-covered pole surrogate, which gives the GLS separator a smooth loss gradient to push
//! straddling items off the boundary. See [`DEFAULT_SHEET_GAP`] and [`SheetConfig::resolve_gap`].
//!
//! # Minimum separation at walls
//!
//! Walls are built with [`ShapeModifyMode::Inflate`] using the *same* [`ShapeModifyConfig`] as the
//! strip, so a wall is inflated by `min_sep / 2` exactly like a placed item is. Combined with the
//! item's own inflation this reproduces the full `min_sep` clearance between a part and a sheet
//! edge — identical to the clearance the deflated strip border already enforces.

use crate::config::SheetConfig;
use jagua_rs::entities::{Container, InferiorQualityZone, Instance};
use jagua_rs::geometry::primitives::{Rect, SPolygon};
use jagua_rs::geometry::shape_modification::ShapeModifyMode;
use jagua_rs::geometry::{DTransformation, OriginalShape};
use jagua_rs::probs::spp::entities::{SPInstance, SPProblem, SPSolution};
use jagua_rs::Instant;
use log::{debug, info, warn};

/// Vertical margin by which a wall extends past the strip's height, so that it always cuts the
/// container cleanly from top to bottom (rather than leaving a hairline gap at the border).
const WALL_Y_OVERSHOOT: f32 = 1.0;

/// Number of sheets a walled strip of `width` decomposes into: `ceil(width / (W + gap))`.
///
/// The last sheet is generally only partially used; see [`last_sheet_used_width`].
pub fn n_sheets(width: f32, sheet: &SheetConfig) -> usize {
    let pitch = sheet.pitch();
    (width / pitch).ceil().max(1.0) as usize
}

/// How much of the *last* sheet is occupied, i.e. `width - (n_sheets - 1) * (W + gap)`, clamped to
/// `[0, W]`. This is the length of material that would be consumed from the final sheet.
pub fn last_sheet_used_width(width: f32, sheet: &SheetConfig) -> f32 {
    let pitch = sheet.pitch();
    let used = width - (n_sheets(width, sheet) - 1) as f32 * pitch;
    used.clamp(0.0, sheet.width)
}

/// The x-intervals `[x_min, x_max]` of the walls of a strip of `width`.
///
/// Only walls that actually fall inside the strip are returned (`x_min < width`), so a strip
/// shorter than one sheet has none.
pub fn wall_intervals(width: f32, sheet: &SheetConfig) -> Vec<(f32, f32)> {
    let pitch = sheet.pitch();
    (1..=n_sheets(width, sheet))
        .map(|k| {
            let x_max = k as f32 * pitch;
            (x_max - sheet.gap, x_max)
        })
        .filter(|(x_min, _)| *x_min < width)
        .collect()
}

/// Items of `instance` that cannot fit inside a single sheet in **any** allowed rotation.
///
/// The walled mode forbids straddling a boundary, so such an item makes the whole instance
/// unsolvable: no amount of separation or widening can ever place it. Detecting that up front turns
/// what would otherwise be an LBF "strip-width is running away" panic (or, worse, an infeasible
/// exported layout) into a clear error message at startup.
///
/// The test is the item's **minimum bounding-box width over its allowed rotations**: an item fits a
/// sheet only if some rotation makes it no wider than the sheet. Continuous rotation is sampled on
/// the same grid the placement sampler uses, so the answer agrees with what the engine can actually
/// achieve.
pub fn items_too_wide_for_sheet(instance: &SPInstance, sheet: &SheetConfig) -> Vec<(usize, f32)> {
    use jagua_rs::geometry::Transformation;
    use jagua_rs::geometry::geo_traits::TransformableFrom;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use std::f32::consts::PI;

    // Same rotation grid as `UniformBBoxSampler` uses for continuous rotation.
    const ROT_N_SAMPLES: usize = 24;

    instance.items.iter()
        .filter_map(|(item, _)| {
            let rotations: Vec<f32> = match &item.allowed_rotation {
                RotationRange::None => vec![0.0],
                RotationRange::Discrete(r) => r.clone(),
                RotationRange::Continuous => (0..ROT_N_SAMPLES)
                    .map(|i| i as f32 * (2.0 * PI) / ROT_N_SAMPLES as f32)
                    .collect(),
            };
            let mut buffer = item.shape_cd.as_ref().clone();
            let min_width = rotations.iter()
                .map(|&r| {
                    let bbox = buffer
                        .transform_from(item.shape_cd.as_ref(), &Transformation::from_rotation(r))
                        .bbox;
                    OrderedFloat(bbox.width())
                })
                .min()
                .map(|w| w.0)
                .unwrap_or(f32::INFINITY);
            (min_width > sheet.width).then_some((item.id, min_width))
        })
        .collect()
}

/// Replaces the problem's container with an otherwise identical one that has a **wall at every
/// sheet boundary**, modelled as quality-0 zones (holes).
///
/// Must be called after every jagua-level change of the strip width
/// ([`SPProblem::change_strip_width`], [`SPProblem::fit_strip`], [`SPProblem::new`]), because those
/// swap in a plain, wall-less container built from [`Strip`](jagua_rs::probs::spp::entities::strip::Strip).
///
/// The new container deliberately reuses **the same `id` jagua-rs would have given it**
/// (`width.to_bits()`), so that [`SPProblem::restore`] — which compares `container.id` — keeps
/// working: restoring a snapshot taken at the same width hits the cheap
/// [`Layout::restore`](jagua_rs::entities::Layout::restore) path, and the snapshot itself carries
/// the walled container along, so no walls are ever lost across a save/restore round-trip.
///
/// Placed items are preserved: [`Layout::swap_container`](jagua_rs::entities::Layout::swap_container)
/// re-registers all dynamic hazards on the rebuilt CDE.
pub fn apply_sheet_walls(prob: &mut SPProblem, sheet: &SheetConfig) {
    let strip = prob.strip;
    let width = strip.width;
    let height = strip.fixed_height;

    let walls = wall_intervals(width, sheet).into_iter()
        // With `--sheet-gap 0` the wall interval is degenerate (`x_min == x_max`) and `Rect::try_new`
        // rejects it. There is nothing to model in that case — the sheets touch, so no strip of
        // material is forbidden — and the boundary is enforced by the strip width alone.
        .filter(|(x_min, x_max)| x_max > x_min)
        .map(|(x_min, x_max)| OriginalShape {
            shape: SPolygon::from(
                Rect::try_new(x_min, -WALL_Y_OVERSHOOT, x_max, height + WALL_Y_OVERSHOOT)
                    .expect("wall rectangle should be valid (x_max > x_min checked above)"),
            ),
            pre_transform: DTransformation::empty(),
            // Inflate (like an item), so items keep `min_sep` from a wall exactly as they do from
            // the deflated strip border.
            modify_mode: ShapeModifyMode::Inflate,
            modify_config: strip.shape_modify_config,
        })
        .collect::<Vec<_>>();

    let quality_zones = match walls.is_empty() {
        true => vec![],
        false => vec![
            InferiorQualityZone::new(0, walls).expect("quality-0 zone of wall rectangles should be valid")
        ],
    };

    let container = Container::new(
        // MUST match the id jagua-rs derives from the strip width, see the doc comment above.
        width.to_bits() as usize,
        OriginalShape {
            shape: SPolygon::from(
                Rect::try_new(0.0, 0.0, width, height).expect("strip rectangle should be valid"),
            ),
            pre_transform: DTransformation::empty(),
            modify_mode: ShapeModifyMode::Deflate,
            modify_config: strip.shape_modify_config,
        },
        quality_zones,
        strip.cde_config,
    )
    .expect("walled container should be valid");

    prob.layout.swap_container(container);
}

/// Applies the walls only if a sheet configuration is present; a no-op in plain strip-packing mode.
pub fn apply_sheet_walls_opt(prob: &mut SPProblem, sheet: Option<&SheetConfig>) {
    if let Some(sheet) = sheet {
        apply_sheet_walls(prob, sheet);
    }
}

/// Prepares a **warm start** for the walled mode by widening the strip to a whole number of sheets
/// plus one spare.
///
/// A solution imported from a plain (wall-less) run — or from any other source — has no reason to
/// respect the sheet boundaries, so items may sit right on top of a wall. Separating that in place
/// is usually hopeless: at the original, tightly packed width there is simply nowhere for a
/// straddling item to go, and the exploration phase would spend its whole budget failing.
///
/// Widening first gives the separator the room it needs; because the exploration phase only ever
/// shrinks, the extra width is given back immediately, and the warm start still saves all the work
/// of arranging the items in the first place.
pub fn widen_for_walls(prob: &mut SPProblem, sheet: &SheetConfig) {
    let width = prob.strip_width();
    // Round up to a whole number of sheets, then add one more as working room.
    let widened = (n_sheets(width, sheet) + 1) as f32 * sheet.pitch();
    if widened > width {
        info!(
            "[SHEET] warm start: widening the strip {:.1} -> {:.1} mm ({} -> {} sheets) so straddling items can be separated off the walls",
            width, widened, n_sheets(width, sheet), n_sheets(widened, sheet),
        );
        prob.change_strip_width(widened);
        apply_sheet_walls(prob, sheet);
    }
}

/// A one-line human readable summary of how a strip of `width` decomposes into sheets.
pub fn sheet_summary(width: f32, sheet: &SheetConfig) -> String {
    format!(
        "{} sheet(s) of {} (+{} gap), last sheet used width {:.1}",
        n_sheets(width, sheet),
        sheet.width,
        sheet.gap,
        last_sheet_used_width(width, sheet),
    )
}

/// Statistics of a single physical sheet of a walled solution.
///
/// The **leftover band** is the secondary objective: the rectangular strip
/// `[used_width, W] x [0, H]` at the right edge of the sheet. Unlike the gaps *between* the parts,
/// this band is one contiguous rectangle and can go straight back into stock.
// Not `Copy`: `straddling_item_ids` is a `Vec`.
#[derive(Debug, Clone)]
pub struct SheetStats {
    /// Index of the sheet, `0` = the first (leftmost) one.
    pub index: usize,
    /// Number of items whose collision shape lies on this sheet.
    pub n_items: usize,
    /// How far into the sheet the items reach: `max(x_max) - k*(W+gap)`, `0.0` for an empty sheet.
    pub used_width: f32,
    /// Total area of the items on this sheet (their *original* shape area).
    pub item_area: f32,
    /// Usable width of one physical sheet.
    pub sheet_width: f32,
    /// Height of one physical sheet (= the strip height).
    pub sheet_height: f32,
    /// Ids of the items assigned to this sheet whose bbox **straddles** a wall, i.e. whose two
    /// edges fall on different sheets. Empty for any feasible walled layout; a non-empty list means
    /// the layout is not cuttable and the stats below it are only indicative.
    pub straddling_item_ids: Vec<usize>,
}

impl SheetStats {
    /// Full area of one physical sheet: `W * H`.
    pub fn sheet_area(&self) -> f32 {
        self.sheet_width * self.sheet_height
    }

    /// Width of the reusable rectangular band left at the right edge of the sheet.
    pub fn leftover_band_width(&self) -> f32 {
        (self.sheet_width - self.used_width).max(0.0)
    }

    /// Area of the reusable rectangular band.
    pub fn leftover_band_area(&self) -> f32 {
        self.leftover_band_width() * self.sheet_height
    }

    /// Area lost *inside* the used part of the sheet (the gaps between the parts): everything that
    /// is neither an item nor the reusable band.
    pub fn internal_gap_area(&self) -> f32 {
        (self.sheet_area() - self.item_area - self.leftover_band_area()).max(0.0)
    }

    /// Density of the sheet relative to the *full* physical sheet (`W * H`).
    pub fn density(&self) -> f32 {
        self.item_area / self.sheet_area()
    }
}

/// Per-sheet statistics of a solution, one entry per physical sheet (including empty ones).
///
/// An item is assigned to the sheet its **right** edge (`x_max`) falls on, which is the edge that
/// determines how far into a sheet the material is consumed. In a feasible walled solution both
/// edges lie on the same sheet, so the choice is immaterial.
///
/// In an *infeasible* one they need not, and an item that straddles a wall is recorded in
/// [`SheetStats::straddling_item_ids`] of the sheet it is assigned to rather than being silently
/// clamped into it: the old code derived the index from `x_min` and clamped, which quietly reported
/// e.g. "used 2611 mm" on a 700 mm sheet with no indication that the layout was uncuttable.
pub fn sheet_stats(sol: &SPSolution, instance: &SPInstance, sheet: &SheetConfig) -> Vec<SheetStats> {
    let width = sol.strip_width();
    let height = sol.layout_snapshot.container.outer_orig.bbox().height();
    let pitch = sheet.pitch();
    let n = n_sheets(width, sheet);

    let mut stats = (0..n)
        .map(|index| SheetStats {
            index,
            n_items: 0,
            used_width: 0.0,
            item_area: 0.0,
            sheet_width: sheet.width,
            sheet_height: height,
            straddling_item_ids: vec![],
        })
        .collect::<Vec<_>>();

    // Which sheet a coordinate falls on, without clamping into range.
    let sheet_of = |x: f32| (x / pitch).floor().max(0.0) as usize;

    for (_, pi) in sol.layout_snapshot.placed_items.iter() {
        let bbox = pi.shape.bbox;
        // `x_max` sits exactly on a boundary for an item flush with the sheet's right edge, which
        // `floor` would push into the next sheet; nudge it back by taking the sheet of the last
        // point strictly inside the item.
        let k_max = sheet_of(bbox.x_max).min(n - 1);
        let k_min = sheet_of(bbox.x_min).min(n - 1);
        let k = k_max;
        let s = &mut stats[k];
        s.n_items += 1;
        s.used_width = s.used_width.max(bbox.x_max - k as f32 * pitch);
        s.item_area += instance.item(pi.item_id).area();
        if k_min != k_max {
            s.straddling_item_ids.push(pi.item_id);
        }
    }
    stats
}

/// Logs the sheet decomposition of `sol`: the summary line, one line per sheet, and the split of the
/// wasted area into the reusable right-hand **bands** and the **internal gaps** between the parts.
pub fn log_sheet_report(phase: &str, sol: &SPSolution, instance: &SPInstance, sheet: &SheetConfig) {
    let width = sol.strip_width();
    info!("[SHEET] [{phase}] {}", sheet_summary(width, sheet));

    let stats = sheet_stats(sol, instance, sheet);
    for s in &stats {
        info!(
            "[SHEET] sheet {}: {} items, used {:.1}/{} mm, dens {:.1}%, leftover band {:.1} mm",
            s.index,
            s.n_items,
            s.used_width,
            sheet.width,
            s.density() * 100.0,
            s.leftover_band_width(),
        );
    }

    // A straddling item means the strip cannot be cut into sheets at all, which matters far more
    // than any of the numbers above — so say so explicitly rather than letting it hide behind a
    // >100 % density or a used width larger than the sheet.
    let n_straddling: usize = stats.iter().map(|s| s.straddling_item_ids.len()).sum();
    if n_straddling > 0 {
        let ids = stats.iter()
            .flat_map(|s| s.straddling_item_ids.iter().map(move |id| format!("{id}@sheet{}", s.index)))
            .collect::<Vec<_>>()
            .join(", ");
        warn!("[SHEET] [{phase}] {n_straddling} item(s) STRADDLE a sheet wall — this layout cannot \
               be cut into sheets: {ids}");
    }

    let band_area: f32 = stats.iter().map(|s| s.leftover_band_area()).sum();
    let gap_area: f32 = stats.iter().map(|s| s.internal_gap_area()).sum();
    let total_area: f32 = stats.iter().map(|s| s.sheet_area()).sum();
    info!(
        "[SHEET] leftover: {:.0} mm2 total ({:.1}%) = band (reusable) {:.0} mm2 ({:.1}%) + internal gaps {:.0} mm2 ({:.1}%)",
        band_area + gap_area,
        (band_area + gap_area) / total_area * 100.0,
        band_area,
        band_area / total_area * 100.0,
        gap_area,
        gap_area / total_area * 100.0,
    );
}

// =================================================================================================
// Phase 8: cross-sheet relocation
// =================================================================================================
//
// # Why a dedicated move is needed
//
// The exploration phase's only move on the *objective* is the 0.1 % fine shrink. In the walled
// mode that shrink narrows the strip's right-hand end, so the pressure it applies is felt **only by
// the last sheet's boundary**: sheets `0..n-2` are separated from the shrinking end by the walls
// and never feel any compaction at all. Their holes are filled only by lucky random samples of the
// separator, which is not a systematic mechanism.
//
// Measured consequence (phase 7, iso7, 33 parts, `--sheet-width 1995 --min-sep 5`): the free strip
// reaches 83.2 % (3466 mm = 1.74 sheets), but the walled run jams at 3 sheets with used widths
// 1990 / 1990 / 686 — the first two sheets sit at ~63 % density while the third holds 686 mm of
// parts that would fit into their holes if room were made for them.
//
// Phase 8 adds the missing operator, in the two places where it can pay off:
//
// * [`try_drop_sheet`] — the **sheet-drop / scatter** move for the exploration phase. It is the
//   direct SPP analogue of the BPP's `close_bin_and_scatter` + `separate()`: shrink the strip by a
//   *whole sheet* at once and relocate the dropped sheet's items to random positions over the
//   surviving sheets, then let the separator resolve the overlap that creates. Where the fine
//   shrink is a local move that can never make a part jump a wall, this one does exactly that.
// * [`pack_down_sheets`] — the **cross-sheet pack-down** for the compression phase: the same idea
//   at single-item granularity, used once the sheet count is fixed.

use crate::optimizer::separator::Separator;
use crate::sample::search::search_placement_in;
use crate::sample::uniform_sampler::UniformBBoxSampler;
use crate::eval::sep_evaluator::SeparationEvaluator;
use crate::quantify::tracker::CollisionTracker;
use crate::util::assertions::tracker_matches_layout;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::terminator::{BasicTerminator, Terminator};
use itertools::Itertools;
use jagua_rs::entities::PItemKey;
use rand::{Rng, RngExt, SeedableRng};
use rand::rngs::Xoshiro256PlusPlus;
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::time::Duration;

/// The strip width that holds exactly `n` sheets: `(n-1) * (W + gap) + W`.
///
/// This is the *tightest* width for `n` sheets — one full sheet less than `n+1` sheets need, and
/// with no unused tail. Shrinking the strip to `target_width_for(n-1)` is exactly the sheet-drop
/// move's objective step.
pub fn target_width_for(n_sheets: usize, sheet: &SheetConfig) -> f32 {
    debug_assert!(n_sheets >= 1);
    (n_sheets - 1) as f32 * sheet.pitch() + sheet.width
}

/// The bounding box of sheet `k` inside the strip: `[k*(W+g), k*(W+g) + W] x [0, H]`.
///
/// Used as the sampling window when an item is to be relocated *into a specific sheet*.
pub fn sheet_bbox(k: usize, sheet: &SheetConfig, height: f32) -> Rect {
    let x_min = k as f32 * sheet.pitch();
    Rect::try_new(x_min, 0.0, x_min + sheet.width, height)
        .expect("sheet bbox should be valid (W > 0, H > 0)")
}

/// Which sheet an item's collision bbox lies on, derived from its **left** edge.
///
/// In a feasible walled layout both edges are on the same sheet, so the choice does not matter;
/// during a drop attempt items may temporarily overlap a wall, and the left edge is the stable
/// choice (it keeps items that hang off the strip's right end assigned to the last sheet).
fn sheet_index_of(bbox: Rect, sheet: &SheetConfig, n: usize) -> usize {
    ((bbox.x_min / sheet.pitch()).floor().max(0.0) as usize).min(n.saturating_sub(1))
}

/// The density the *whole* solution would have to reach to fit into `n_target` sheets.
///
/// `Σ item area / (n_target * W * H)`. The SPP counterpart of the BPP's
/// `required_density_for_reduction`: a value above 1.0 proves the reduction impossible, and a value
/// above a realistic packing density ([`SheetConfig::max_reduction_density`]) makes it hopeless in
/// practice, so no attempt is made.
pub fn required_density_for(sep: &Separator, n_target: usize, sheet: &SheetConfig) -> f32 {
    if n_target == 0 {
        return f32::INFINITY;
    }
    let height = sep.prob.layout.container.outer_orig.bbox().height();
    let item_area = sep.prob.layout.placed_item_area(&sep.instance);
    item_area / (n_target as f32 * sheet.width * height)
}

/// Shrinks a sampling window so that an item whose **translation** is drawn from it keeps its whole
/// shape inside the original window.
///
/// [`UniformBBoxSampler`] constrains the translation, correcting only `container_bbox` for the
/// rotated shape's bounding box — its `sample_bbox` argument is used as-is. To make a sample window
/// mean "the item lands inside this box", the box has to be deflated by the item's extent first.
///
/// The deflation is derived from the item's rotated bounding boxes exactly the way the sampler
/// derives its container range — offset by `-bbox.x_min` on the left and `-bbox.x_max` on the right
/// — taking the worst case over all allowed rotations so that one window is valid for whichever
/// rotation is drawn. That is conservative (a given rotation could legally sit closer to an edge),
/// which is the right trade here: the alternative is a per-rotation window the sampler's API cannot
/// express.
///
/// Returns `None` when the item cannot fit the window at all, which the caller treats as "sample
/// the raw window and let the separator sort it out".
fn shrink_bbox_for_item(bbox: Rect, item: &jagua_rs::entities::Item) -> Option<Rect> {
    use jagua_rs::geometry::Transformation;
    use jagua_rs::geometry::geo_traits::TransformableFrom;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use std::f32::consts::PI;

    // Same rotation grid the sampler uses for continuous rotation.
    const ROT_N_SAMPLES: usize = 24;
    let rotations: Vec<f32> = match &item.allowed_rotation {
        RotationRange::None => vec![0.0],
        RotationRange::Discrete(r) => r.clone(),
        RotationRange::Continuous => (0..ROT_N_SAMPLES)
            .map(|i| i as f32 * (2.0 * PI) / ROT_N_SAMPLES as f32)
            .collect(),
    };

    let mut buffer = item.shape_cd.as_ref().clone();
    // Worst-case offsets over the rotations: how far the shape reaches left/below its origin
    // (`min_*`, negative-most) and right/above it (`max_*`, positive-most).
    let (mut lo_x, mut lo_y, mut hi_x, mut hi_y) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    for &r in &rotations {
        let b = buffer.transform_from(item.shape_cd.as_ref(), &Transformation::from_rotation(r)).bbox;
        lo_x = lo_x.min(b.x_min);
        lo_y = lo_y.min(b.y_min);
        hi_x = hi_x.max(b.x_max);
        hi_y = hi_y.max(b.y_max);
    }
    Rect::try_new(bbox.x_min - lo_x, bbox.y_min - lo_y, bbox.x_max - hi_x, bbox.y_max - hi_y).ok()
}

/// The **sheet-drop / scatter** move: try to make the current solution fit into one sheet fewer.
///
/// Precondition: the separator holds a **feasible** solution occupying `n` sheets.
///
/// 1. The strip is narrowed to [`target_width_for`]`(n - 1)` — a whole sheet at once, not the
///    0.1 % fine step. `Separator::change_strip_width` is deliberately **not** used for the item
///    shifting: its linear "shift everything right of the split" would smear the dropped sheet's
///    items across the new right-hand end, where they would all pile up on top of each other in the
///    same place they already were. Instead every item that lived on the dropped sheet is
///    **relocated to a uniformly random position inside the surviving sheets** (a feasible rotation
///    is chosen by [`UniformBBoxSampler`]), which spreads them over exactly the holes the fine
///    shrink can never reach. Overlap is expected and is the separator's job.
/// 2. The tracker is rebuilt and the workers reseeded (the width changed, so the containers did).
/// 3. `separate()` is run: if it reaches zero loss the solution now genuinely fits in `n - 1`
///    sheets and `true` is returned with the separator holding it. Otherwise `false` is returned
///    and the separator holds the least-infeasible attempt — the caller decides whether to pool it
///    and roll back.
///
/// Determinism: the items are collected in `SlotMap` order and sorted largest-area-first with the
/// `PItemKey` as tie-break, the destination sheet is assigned round-robin, and the only randomness
/// is `sep.rng`. Same seed ⇒ same attempt.
///
/// Returns `(succeeded, best_attempt, loss)`.
pub fn try_drop_sheet(
    sep: &mut Separator,
    sheet: &SheetConfig,
    term: &impl Terminator,
    sol_listener: &mut impl SolutionListener,
) -> (bool, SPSolution, f32) {
    let width = sep.prob.strip_width();
    let n = n_sheets(width, sheet);
    debug_assert!(n >= 2, "try_drop_sheet requires at least 2 sheets");
    scatter_and_shrink(sep, sheet, target_width_for(n - 1, sheet), "drop", term, sol_listener)
}

/// Strike limit used while repairing a scatter; see [`scatter_and_shrink`].
const SCATTER_STRIKE_LIMIT: usize = 8;

/// No-improvement iteration limit used while repairing a scatter; see [`scatter_and_shrink`].
const SCATTER_ITER_NO_IMPRV_LIMIT: usize = 400;

/// The engine shared by the sheet-drop and the pack-down: **shrink the strip to `new_width` and
/// scatter everything that no longer fits over the sheets that survive**, then separate.
///
/// `label` only names the move in the log.
///
/// Both callers rely on the same two properties:
/// * every item whose collision bbox starts beyond `new_width` is *re-placed*, uniformly at random,
///   inside one of the surviving sheets — which is what gets a part across a wall, the thing the
///   fine shrink can never do;
/// * the strip really does get narrower, so the vacated tail **stops existing**. That matters more
///   than it sounds: the separator's own placement search is global, and as long as a large empty
///   band is still part of the container it will happily park every relocated item straight back
///   into it, undoing the move. Removing the space is what makes the relocation stick.
///
/// Determinism: items are collected in `SlotMap` order and sorted largest-area-first with the
/// `PItemKey` as tie-break, destinations are assigned round-robin, and the only randomness is
/// `sep.rng`. Same seed ⇒ same attempt.
///
/// Returns `(succeeded, best_attempt, loss)`; on success the separator holds a feasible solution at
/// `new_width`, on failure the least-infeasible attempt (the caller decides what to do with it).
fn scatter_and_shrink(
    sep: &mut Separator,
    sheet: &SheetConfig,
    new_width: f32,
    label: &str,
    term: &impl Terminator,
    sol_listener: &mut impl SolutionListener,
) -> (bool, SPSolution, f32) {
    let width = sep.prob.strip_width();
    let n = n_sheets(width, sheet);
    let n_target = n_sheets(new_width, sheet);
    let height = sep.prob.layout.container.outer_orig.bbox().height();
    // Everything must fit left of the new right edge; the last surviving sheet is only usable up to
    // `new_width`, so its sampling window is clipped there.
    let last_usable = new_width - (n_target - 1) as f32 * sheet.pitch();

    // --- 1. Which items no longer fit? --------------------------------------------------------
    // Deterministic order: largest original area first, ties broken by PItemKey.
    let mut doomed = sep.prob.layout.placed_items.iter()
        .filter(|(_, pi)| pi.shape.bbox.x_max > new_width)
        .map(|(pk, pi)| (pk, pi.item_id))
        .collect_vec();
    doomed.sort_by_key(|(pk, item_id)| {
        (Reverse(OrderedFloat(sep.instance.item(*item_id).area())), *pk)
    });

    if doomed.is_empty() {
        // Nothing sticks out: a plain width change is all that is needed.
        sep.change_strip_width(new_width, Some(width + 1.0));
        let (attempt, ct) = sep.separate(term, sol_listener);
        let loss = ct.get_total_loss();
        return (loss == 0.0, attempt, loss);
    }

    info!("[SHEET] {label} attempt: {} -> {} sheet(s) (width {:.1} -> {:.1}), scattering {} item(s)",
        n, n_target, width, new_width, doomed.len());

    // --- 2. Relocate them, uniformly at random, over the surviving sheets ----------------------
    // Round-robin over the destination sheets keeps the scatter balanced without needing a
    // density computation that would immediately be invalidated by the previous placement.
    for (i, (pk, item_id)) in doomed.iter().enumerate() {
        let dst_k = i % n_target;
        let item = sep.instance.item(*item_id);
        let mut bbox = sheet_bbox(dst_k, sheet, height);
        if dst_k == n_target - 1 {
            // The last surviving sheet is only usable up to the new strip end.
            bbox = Rect::try_new(bbox.x_min, bbox.y_min, bbox.x_min + last_usable, bbox.y_max)
                .unwrap_or(bbox);
        }
        // NOTE on what `bbox` actually constrains. `UniformBBoxSampler` treats its `sample_bbox` as
        // the range of the **translation** (the item's origin), not as a box the item's *shape* is
        // kept inside — only `container_bbox` is corrected for the rotated shape's extent. Passing
        // the raw sheet bbox therefore samples origins across the whole sheet, and an item whose
        // origin lands near the right edge sticks out well past it, straight onto the next wall.
        //
        // Since the point of the scatter is to land items *on a given sheet*, the sampling window is
        // shrunk here to the set of origins that keep the item inside the sheet, using the same
        // correction the sampler applies to the container: the widest rotated half-extent. That is a
        // conservative (rotation-independent) bound, which is what keeps this cheap — the sampler
        // still intersects per rotation against the container, so nothing leaves the strip either.
        let sample_bbox = shrink_bbox_for_item(bbox, item).unwrap_or(bbox);
        // The container is still the *old*, wider one here, so an item sampled inside the shrunken
        // window is inside the container as well.
        match UniformBBoxSampler::new(sample_bbox, item, sep.prob.layout.container.outer_cd.bbox) {
            Some(sampler) => {
                // A uniform random position first — that is what spreads the relocated items over
                // the *whole* of the surviving sheets rather than over one corner of them...
                let dt = sampler.sample(&mut sep.rng);
                let pk = sep.move_item(*pk, dt);
                // ...and then a local search inside the same sheet, so the item at least starts in
                // the least-colliding spot that sheet has to offer instead of on top of whatever
                // the random draw happened to hit. This is pure head start for the separation that
                // follows; it cannot move the item out of the sheet (the sampling window is the
                // sheet), and it costs one placement search per relocated item.
                search_best_position_in_sheet(sep, pk, bbox);
            }
            None => {
                // The item does not fit inside a single sheet in any rotation. That can only happen
                // for an instance whose parts are wider than a sheet, which the walled mode cannot
                // handle at all; leave it where it is and let the separator deal with it.
                warn!("[SHEET] item {item_id} does not fit inside one sheet, left in place");
            }
        }
    }

    // --- 3. Narrow the strip. No item may be shifted: they have all been re-placed already, and
    //        the survivors must stay exactly where they are (that is the layout being reused).
    //        A split position past the new right end means `change_strip_width` finds nothing to
    //        shift, while still rebuilding the container (with the correct number of walls), the
    //        tracker and the workers.
    sep.change_strip_width(new_width, Some(width + 1.0));
    debug_assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout));

    // --- 4. Let the separator try to resolve the overlap the scatter introduced ----------------
    // The scatter is a *global* perturbation (a whole sheet's worth of items land on top of the
    // others), so the repair is a full re-nesting job, not the local touch-up the fine shrink's
    // separation does. It therefore gets a deliberately more patient separator: more strikes and
    // more no-improvement iterations before giving up. Anything less and the attempt reports
    // "impossible" after a couple of seconds without ever having tried.
    let outer_cfg = sep.config;
    sep.config.strike_limit = outer_cfg.strike_limit.max(SCATTER_STRIKE_LIMIT);
    sep.config.iter_no_imprv_limit = outer_cfg.iter_no_imprv_limit.max(SCATTER_ITER_NO_IMPRV_LIMIT);
    let (attempt, ct) = sep.separate(term, sol_listener);
    sep.config = outer_cfg;
    let loss = ct.get_total_loss();
    (loss == 0.0, attempt, loss)
}

/// Rolls the separator back to a solution taken at a **different** strip width.
///
/// [`Separator::rollback`] asserts that the width matches, so restoring a wider (pre-drop) solution
/// needs the width change first. The split position is past the right end so that no item is
/// shifted — the solution being restored immediately overwrites every placement anyway.
pub fn rollback_to_width(sep: &mut Separator, sol: &SPSolution) {
    if sep.prob.strip_width() != sol.strip_width() {
        let past_end = sep.prob.strip_width().max(sol.strip_width()) + 1.0;
        sep.change_strip_width(sol.strip_width(), Some(past_end));
    }
    sep.rollback(sol, None);
}

/// The **cross-sheet pack-down** for the compression phase.
///
/// Once the sheet count is fixed, the remaining objective is the *last* sheet's leftover band. The
/// fine compression shortens it in 0.05 % steps by squeezing the strip's right end — but that
/// pressure stops at the last wall, so it can only ever compact the last sheet's own contents, not
/// move any of them into the holes of the earlier sheets.
///
/// This step attacks the same band in **large** steps instead: the strip is cut back by
/// `PACK_DOWN_BAND_STEP` of the last sheet's used width at once, and every item that no longer fits
/// is relocated into an earlier sheet ([`scatter_and_shrink`]), after which a short separation has
/// to make the result feasible. On success the band is permanently shorter and the step repeats
/// from there; on failure the cut is halved and retried, down to `PACK_DOWN_MIN_STEP`.
///
/// Why the transfer is bundled with a width cut instead of being done item by item: the separator's
/// placement search is **global over the whole strip**. If the vacated tail of the last sheet is
/// still part of the container, the search will simply put the relocated item back there — measured
/// on iso7, every single single-item transfer was undone this way within a fraction of a second.
/// Removing the space in the same move is what makes the relocation stick.
///
/// The result is always feasible and never wider than `init_sol`, so the fine compression that
/// follows can only improve on it.
///
/// Returns the best (feasible) solution found and how many band cuts were accepted.
pub fn pack_down_sheets(
    sep: &mut Separator,
    sheet: &SheetConfig,
    init_sol: &SPSolution,
    term: &impl Terminator,
    sol_listener: &mut impl SolutionListener,
) -> (SPSolution, usize) {
    // The exploration phase leaves the separator at the *last attempted* (narrower, infeasible)
    // width, not at `init_sol`'s, so the rollback has to change the width first.
    rollback_to_width(sep, init_sol);
    let mut best_sol = init_sol.clone();

    let n = n_sheets(sep.prob.strip_width(), sheet);
    if n < 2 {
        info!("[SHEET] pack-down: only one sheet, nothing to pack down");
        return (best_sol, 0);
    }
    let start = Instant::now();
    let mut n_accepted = 0usize;
    // Relative size of the next cut, halved on every failure.
    let mut step = PACK_DOWN_BAND_STEP;
    // Consecutive failures; the step gives up entirely after `PACK_DOWN_MAX_FAILS` of them.
    let mut n_failed = 0usize;

    info!("[SHEET] pack-down: {} sheet(s), last sheet used {:.1} mm, budget {:.1}s",
        n, last_sheet_used_width(best_sol.strip_width(), sheet),
        term.timeout_at().map_or(f32::INFINITY,
            |d| d.saturating_duration_since(Instant::now()).as_secs_f32()));

    while !term.kill() && step >= PACK_DOWN_MIN_STEP && n_failed < PACK_DOWN_MAX_FAILS {
        let width = best_sol.strip_width();
        let used = last_sheet_used_width(width, sheet);
        if used <= PACK_DOWN_MIN_BAND {
            info!("[SHEET] pack-down: the last sheet is (almost) empty, done");
            break;
        }
        // Cut `step` of the last sheet's *used* width away. Never cut past the sheet's left edge:
        // dropping a whole sheet is the exploration phase's job, not this one's.
        let target = (width - used * step).max(width - used);

        let sub_term = short_term(term, sheet.pack_down_move_time_limit);
        let (ok, attempt, loss) = scatter_and_shrink(sep, sheet, target, "pack-down", &sub_term, sol_listener);

        if ok {
            info!("[SHEET] pack-down: band cut accepted, last sheet {:.1} -> {:.1} mm ({:.3}%)",
                used, last_sheet_used_width(attempt.strip_width(), sheet),
                attempt.density(&sep.instance) * 100.0);
            best_sol = attempt;
            n_accepted += 1;
            n_failed = 0;
            sol_listener.report(ReportType::CmprFeas, &best_sol, &sep.instance);
            rollback_to_width(sep, &best_sol);
        } else {
            n_failed += 1;
            debug!("[SHEET] pack-down: band cut of {:.1}% failed (min loss {}), halving",
                step * 100.0, crate::FMT().fmt2(loss));
            step *= 0.5;
            // Always resume from the best feasible solution; the failed attempt is discarded.
            rollback_to_width(sep, &best_sol);
        }
    }

    rollback_to_width(sep, &best_sol);
    info!("[SHEET] pack-down finished: {n_accepted} band cut(s) accepted in {:.1}s, last sheet now {:.1} mm",
        start.elapsed().as_secs_f32(), last_sheet_used_width(best_sol.strip_width(), sheet));
    (best_sol, n_accepted)
}

/// Fraction of the last sheet's used width a pack-down band cut removes at once. Large on purpose:
/// the point of this step is to force a *cross-sheet* relocation, and a small cut can be answered by
/// the last sheet's own items shuffling closer together, which the fine compression does better and
/// far more cheaply.
const PACK_DOWN_BAND_STEP: f32 = 0.5;

/// Smallest relative band cut still worth attempting; below this the fine compression takes over.
const PACK_DOWN_MIN_STEP: f32 = 0.05;

/// How many band cuts may fail in a row before the pack-down gives up and hands the rest of the
/// compression budget to the fine compression.
///
/// Kept deliberately small. Measured on madisocad_iso, where no cut is possible at all: three
/// failing attempts cost 11 of the phase's 20 seconds and left the fine compression with a worse
/// final band than it reached without the step. The pack-down only pays off where a cut succeeds,
/// and where one does, it succeeds early.
const PACK_DOWN_MAX_FAILS: usize = 2;

/// Last-sheet used width (mm) below which the band is considered gone.
const PACK_DOWN_MIN_BAND: f32 = 1.0;


/// Places `pk` at the best position [`search_placement_in`] finds **inside `bbox`**, and returns its
/// (new) key. If no sample is found at all the item is left untouched.
///
/// Note `ref_pk = None`: the item's *current* placement is deliberately **not** offered to the
/// search as a candidate. In the pack-down the item starts out on the last sheet with a loss of
/// zero (the layout is feasible), so seeding the search with it would make it the unbeatable
/// optimum and the "transfer" would be a no-op every single time. Dropping the reference forces the
/// item into the best position the *target sheet* has to offer, overlap included — which is exactly
/// the move being attempted, and which the `separate()` that follows is there to repair.
fn search_best_position_in_sheet(sep: &mut Separator, pk: PItemKey, bbox: Rect) -> PItemKey {
    // The search needs the layout and the tracker immutably while wanting an RNG mutably, so it
    // gets its own RNG seeded from the master stream (which keeps the step deterministic).
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(sep.rng.next_u64());

    let best = {
        let layout = &sep.prob.layout;
        let item = sep.instance.item(layout.placed_items[pk].item_id);
        let evaluator = SeparationEvaluator::new(layout, item, pk, &sep.ct);
        let (best, _) = search_placement_in(
            layout, item, None, evaluator, sep.config.sample_config, &mut rng, Some(bbox),
        );
        best.map(|(dt, _)| dt)
    };

    match best {
        Some(dt) => sep.move_item(pk, dt),
        None => pk,
    }
}

/// A private terminator granting `min(remaining budget, limit)`; the SPP twin of the BPP helper of
/// the same name.
fn short_term(term: &impl Terminator, limit: Duration) -> BasicTerminator {
    let budget = match term.timeout_at() {
        Some(deadline) => deadline.saturating_duration_since(Instant::now()).min(limit),
        None => limit,
    };
    let mut t = BasicTerminator::new();
    t.new_timeout(budget);
    t
}

/// **Install the walls into a wall-less (plain strip) solution**, the core of
/// [`SheetPipeline::PlainFirst`](crate::config::SheetPipeline::PlainFirst).
///
/// The plain strip engine produces a much denser layout than a walled run of the same budget —
/// on iso7, 83.2 % versus 61.5 % — because it never has to fight the walls. That layout cannot be
/// used directly (parts straddle the sheet boundaries), but it is a far better *starting point*
/// than anything a walled run reaches on its own, and turning it into a walled one is a **local**
/// repair rather than a global re-nest:
///
/// 1. The plain strip is cut into chunks of `W - slack` mm; `n = ceil(width / (W - slack))` is how
///    many sheets that takes. Note this uses a *width*, not the pitch: the gaps do not exist yet.
/// 2. Every item is assigned to a chunk by `k = floor(bbox.x_min / (W - slack))` and translated
///    right onto sheet `k`. This opens exactly the gap the walls need and, crucially, **moves every
///    item together with its own chunk**, so all the neighbour relationships inside a chunk survive
///    untouched. The only items that end up overlapping a wall are the ones that were straddling a
///    chunk boundary in the first place.
/// 3. The strip is set to `target_width_for(n)` and the walls are installed.
///
/// `slack` is the caller's retry knob: with `slack = 0` the cut is as tight as the material allows
/// and the repair has nothing but the existing gaps to work with; a positive slack leaves that many
/// mm free at the right edge of every sheet for the wall-crossers to slide into, at the cost of
/// needing more sheets.
///
/// The caller then runs `separate()`: each wall-crosser has to slide into the free space of its own
/// or the next sheet, which is a small local move. Returns `n`, the sheet count that was installed.
///
/// This function only *prepares* the problem; it does not separate, and the layout it leaves behind
/// is generally infeasible (that is the point).
pub fn install_walls_into_plain(sep: &mut Separator, sheet: &SheetConfig, slack: f32) -> usize {
    let width = sep.prob.strip_width();
    // The *effective* sheet width the plain layout is cut at. With `slack = 0` this is the real
    // sheet width and the cut is as tight as possible; a positive slack cuts the plain strip into
    // shorter chunks, so each chunk has `slack` mm of room inside its sheet for the repair to use.
    // That costs sheets but is what makes a repair possible when the tight cut is hopeless.
    let eff_width = (sheet.width - slack).max(sheet.width * 0.25);
    // Sheets are counted against the effective width: the gaps are about to be inserted.
    let n = ((width / eff_width).ceil().max(1.0)) as usize;
    let new_width = target_width_for(n, sheet);

    // Deterministic order (SlotMap iteration is stable, and the shifts are independent anyway).
    // Item on chunk `k` moves to sheet `k`: from `k*eff_width` to `k*(W+gap)`.
    let shifts = sep.prob.layout.placed_items.iter()
        .map(|(pk, pi)| {
            let k = ((pi.shape.bbox.x_min / eff_width).floor().max(0.0) as usize).min(n - 1);
            (pk, pi.d_transf, k as f32 * (sheet.pitch() - eff_width))
        })
        .filter(|(_, _, shift)| *shift > 0.0)
        .collect_vec();

    info!("[SHEET] installing walls into the plain strip solution: width {:.1} -> {:.1} ({} sheet(s), \
           cut every {:.1} mm), shifting {} item(s) right onto their own sheet",
        width, new_width, n, eff_width, shifts.len());

    // 1. Widen first, so every shifted item stays inside the container at all times. The split
    //    position is past the right end, so `change_strip_width` shifts nothing by itself.
    sep.change_strip_width(new_width.max(width), Some(width + 1.0));

    // 2. Apply the per-sheet offsets.
    for (pk, dt, shift) in shifts {
        let (x, y) = dt.translation();
        sep.move_item(pk, DTransformation::new(dt.rotation(), (x + shift, y)));
    }

    // 3. Settle on the exact target width (a no-op when it already matched) and (re)install the
    //    walls; `change_strip_width` does the container swap, the tracker rebuild and the worker
    //    reseed in one go.
    sep.change_strip_width(new_width, Some(new_width + 1.0));
    debug_assert!(tracker_matches_layout(&sep.ct, &sep.prob.layout));
    n
}

/// **Per-sheet left-compaction** (`--compact-sheets`), the secondary post-pass.
///
/// For every sheet except the last, every item is translated as far to the **left inside its own
/// sheet** as it can go without colliding, largest-x first. The many small gaps between the parts
/// thus migrate into a single wide reusable band at the right edge of the sheet.
///
/// This is deliberately a pure *translation* pass rather than a re-nesting: it cannot make the
/// solution worse (every step is verified against the CDE and skipped if it collides), it cannot
/// change the sheet count, and it is cheap enough to always run. A move is only kept when it is
/// collision-free *and* stays inside the source sheet, so no item can be pushed into a wall.
///
/// Returns the compacted solution and the number of items that actually moved.
pub fn compact_sheets_left(
    sep: &mut Separator,
    sheet: &SheetConfig,
    init_sol: &SPSolution,
) -> (SPSolution, usize) {
    rollback_to_width(sep, init_sol);
    let width = sep.prob.strip_width();
    let n = n_sheets(width, sheet);
    if n < 2 {
        return (init_sol.clone(), 0);
    }

    let mut n_moved = 0usize;
    // Left to right: an item may only slide into space an already-processed item has vacated, so a
    // single left-to-right sweep per sheet is the right order (and is deterministic).
    let mut order = sep.prob.layout.placed_items.iter()
        .filter(|(_, pi)| sheet_index_of(pi.shape.bbox, sheet, n) < n - 1)
        .map(|(pk, pi)| (pk, OrderedFloat(pi.shape.bbox.x_min)))
        .collect_vec();
    order.sort_by_key(|(pk, x)| (*x, *pk));

    for (pk, _) in order {
        if !sep.prob.layout.placed_items.contains_key(pk) {
            continue;
        }
        let (dt, bbox) = {
            let pi = &sep.prob.layout.placed_items[pk];
            (pi.d_transf, pi.shape.bbox)
        };
        let k = sheet_index_of(bbox, sheet, n);
        let sheet_x_min = k as f32 * sheet.pitch();
        // How far left the item could go at most before hitting its own sheet's left edge.
        let max_shift = bbox.x_min - sheet_x_min;
        if max_shift <= COMPACT_MIN_SHIFT {
            continue;
        }

        // Binary search on the shift: the largest collision-free displacement, to within
        // `COMPACT_MIN_SHIFT`. `try_shift` restores the item on failure, so the layout is unchanged
        // whenever the probe collides — but *every* probe re-keys the item (`move_item` removes and
        // re-places it), so `cur_pk` has to be updated from both branches. Dropping the key of the
        // undo branch used to leave the loop probing a dangling key on its next iteration, which
        // panics with "invalid SlotMap key used".
        let (mut lo, mut hi) = (0.0f32, max_shift);
        let mut cur_pk = pk;
        let mut best_shift = 0.0f32;
        // Whether the item currently sits at `best_shift` (true) or back at `dt` (false, after a
        // failed probe undid itself).
        let mut at_best = true;
        while hi - lo > COMPACT_MIN_SHIFT {
            let mid = 0.5 * (lo + hi);
            let (new_pk, accepted) = try_shift(sep, cur_pk, dt, mid);
            cur_pk = new_pk;
            if accepted {
                // `mid` works; keep it and try to go further left.
                best_shift = mid;
                at_best = true;
                lo = mid;
            } else {
                // The probe put the item back at `dt`, i.e. it is no longer at `best_shift`.
                at_best = false;
                hi = mid;
            }
        }
        // The search may well have ended on a *failed* probe, which restored the item to its
        // original position — so the best accepted shift has to be re-applied before moving on.
        // Without this the item silently stays put while `n_moved` claims it moved.
        if best_shift > 0.0 && !at_best {
            let (new_pk, accepted) = try_shift(sep, cur_pk, dt, best_shift);
            cur_pk = new_pk;
            if !accepted {
                // Should not happen (the same shift was accepted earlier and nothing else moved in
                // between), but if it does the item is back at `dt` and genuinely did not move.
                best_shift = 0.0;
            }
        }
        let _ = cur_pk;
        if best_shift > 0.0 {
            n_moved += 1;
        }
    }

    let sol = sep.prob.save();
    info!("[SHEET] per-sheet left-compaction: {n_moved} item(s) shifted left within their sheet");
    (sol, n_moved)
}

/// Smallest displacement the left-compaction bothers with (mm). Also the binary search's tolerance.
const COMPACT_MIN_SHIFT: f32 = 0.5;

/// Tries to place the item currently at `pk` at `dt` shifted `shift` mm to the left.
///
/// Returns `(key, accepted)`: the key the item lives under **after** the probe — which is a *new*
/// key in both branches, because [`Separator::move_item`] removes and re-places the item and the
/// undo is itself a `move_item` — and whether the shifted position was collision-free. The caller
/// must always adopt the returned key: the key it passed in is dangling on return, and reusing it
/// panics with "invalid SlotMap key used".
fn try_shift(sep: &mut Separator, pk: PItemKey, dt: DTransformation, shift: f32) -> (PItemKey, bool) {
    let (x, y) = dt.translation();
    let shifted = DTransformation::new(dt.rotation(), (x - shift, y));
    let new_pk = sep.move_item(pk, shifted);
    if sep.ct.get_loss(new_pk) == 0.0 {
        (new_pk, true)
    } else {
        // Undo: put it back exactly where it was. This mints yet another key, which is the one the
        // item is actually reachable under from now on.
        (sep.move_item(new_pk, dt), false)
    }
}
