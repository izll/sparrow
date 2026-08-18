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
use log::info;

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
        .map(|(x_min, x_max)| OriginalShape {
            shape: SPolygon::from(
                Rect::try_new(x_min, -WALL_Y_OVERSHOOT, x_max, height + WALL_Y_OVERSHOOT)
                    .expect("wall rectangle should be valid (gap > 0)"),
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
#[derive(Debug, Clone, Copy)]
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
/// Items are assigned to a sheet by the *right* edge of their collision-shape bounding box, which
/// is unambiguous in a walled solution: no item crosses a wall, so both edges lie on the same sheet.
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
        })
        .collect::<Vec<_>>();

    for (_, pi) in sol.layout_snapshot.placed_items.iter() {
        let bbox = pi.shape.bbox;
        let k = ((bbox.x_min / pitch).floor().max(0.0) as usize).min(n - 1);
        let s = &mut stats[k];
        s.n_items += 1;
        s.used_width = s.used_width.max(bbox.x_max - k as f32 * pitch);
        s.item_area += instance.item(pi.item_id).area();
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
