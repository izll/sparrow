//! Deterministic bounding-box shelf constructor: an alternative to [`BPLBFBuilder`] that ignores
//! the exact item contours and packs their **collision-shape bounding boxes** instead.
//!
//! [`BPLBFBuilder`](crate::optimizer::bpp::BPLBFBuilder) samples positions with the same
//! (randomised) machinery the separation loop uses. That is the right tool for irregular parts,
//! where the contour is what decides whether two items nest into each other. It is a *bad* tool for
//! rectangular parts: the sampler has no notion of "column" or "shelf", so it leaves the neat
//! guillotine structure such a part set wants on the table.
//!
//! [`BPShelfBuilder`] is the complementary heuristic. It works purely on
//! [`Item::shape_cd`]`.bbox` — which already carries the `min_item_separation` inflation — inside
//! the container's [`Container::outer_cd`](jagua_rs::entities::Container::outer_cd)`.bbox` — which
//! already carries the matching deflation. So a placement that is collision-free *in the bbox
//! model* is collision-free in the real geometry too, with the requested separation respected.
//! (The converse does not hold, which is exactly why this is a complement and not a replacement:
//! for irregular shapes the bbox model wastes everything the contour would have saved.)
//!
//! Two classic first-fit-decreasing variants are run and the better result is kept:
//!
//! * **column mode** — vertical stacks packed left → right (an item goes on *top* of an open
//!   column when it fits, otherwise a new column starts to the right),
//! * **shelf/row mode** — horizontal shelves packed bottom → top (the transpose of the above).
//!
//! Both are fully deterministic: no RNG is involved anywhere, and every ordering is a total order.
//!
//! The result is verified against jagua-rs itself ([`Layout::is_feasible`]) before it is returned,
//! so a bug in the bbox arithmetic fails loudly instead of producing a colliding "solution".

use anyhow::{Result, bail};
use itertools::Itertools;
use jagua_rs::entities::{Instance, Item, Layout};
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::Transformable;
use jagua_rs::geometry::primitives::Rect;
use jagua_rs::geometry::{DTransformation, Transformation};
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPProblem};
use log::{debug, info};
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::f32::consts::PI;

/// Numerical slack used by all "does it fit" comparisons.
///
/// The bbox arithmetic is exact in principle, but the rotated bbox of a polygon is recomputed from
/// its (rotated) vertices, so it drifts by a few ULPs. Comparing with a hair of slack would risk
/// producing a real collision, so the slack is applied in the *conservative* direction: a candidate
/// must fit with `FIT_EPS` to spare.
const FIT_EPS: f32 = 1e-4;

/// Gap inserted between neighbouring bboxes (and against the container edge).
///
/// jagua-rs' collision detection treats **touching** shapes as colliding, so two bboxes that share
/// an edge exactly — which is precisely what a perfect shelf packing produces — would be reported
/// as a collision. Every item is therefore inset by this much on both axes. It is orders of
/// magnitude below the `min_item_separation` the geometry already carries, so it costs nothing
/// real; it only breaks the exact-touch tie. Float drift in the rotated bbox (a few ULPs at these
/// magnitudes) stays well inside it.
const PLACEMENT_GAP: f32 = 1e-3;

/// Which constructive heuristic [`optimize_bpp`](crate::optimizer::bpp::optimize_bpp) starts from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Constructive {
    /// Build both and start from the better one (fewer bins; tie → lower min-bin density).
    #[default]
    Best,
    /// Always use [`BPLBFBuilder`](crate::optimizer::bpp::BPLBFBuilder).
    Lbf,
    /// Always use [`BPShelfBuilder`].
    Shelf,
}

/// Which of the two packing directions a [`BPShelfBuilder`] run uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Vertical stacks, packed left → right.
    Column,
    /// Horizontal shelves, packed bottom → top.
    Shelf,
}

/// One item, reduced to the bbox model: its id plus the chosen orientation's bbox extents.
#[derive(Debug, Clone, Copy)]
struct BoxItem {
    item_id: usize,
    /// Rotation (radians) this item is placed with.
    rotation: f32,
    /// Width of the item's collision-shape bbox **after** applying `rotation`.
    w: f32,
    /// Height of the item's collision-shape bbox **after** applying `rotation`.
    h: f32,
    /// Min corner of the item's collision-shape bbox **after** applying `rotation` (no
    /// translation). The translation that lands the bbox on a target corner is
    /// `target - rotated_min`, see [`ShelfBin::commit`].
    rotated_min: (f32, f32),
}

/// A single open column (or shelf), i.e. one "band" of the bin.
///
/// In [`Mode::Column`] `cross` is the column's x-offset and `extent` its width; `along` is how much
/// of the bin height is already consumed. In [`Mode::Shelf`] the two axes are swapped.
#[derive(Debug, Clone, Copy)]
struct Band {
    /// Offset of the band on the *cross* axis (x for columns, y for shelves).
    cross: f32,
    /// Size of the band on the cross axis (the widest item put into it).
    extent: f32,
    /// How much of the *along* axis (y for columns, x for shelves) is used up.
    along: f32,
}

/// One bin under construction: the bin type it was opened from plus its open bands and placements.
#[derive(Debug, Clone)]
struct ShelfBin {
    bin_id: usize,
    /// Usable area of this bin (the deflated container bbox).
    bbox: Rect,
    bands: Vec<Band>,
    /// Total cross-axis extent consumed by `bands`.
    cross_used: f32,
    /// The finished placements: `(item_id, d_transf)`, in placement order.
    placements: Vec<(usize, DTransformation)>,
    /// Sum of the placed items' bbox areas — used only for the tie-break statistics.
    box_area: f32,
}

/// The result of one full construction run, before it is turned into a [`BPProblem`].
#[derive(Debug, Clone)]
struct Packing {
    mode: Mode,
    bins: Vec<ShelfBin>,
}

impl Packing {
    /// Density of the *least dense* bin, in the bbox model. Used as the tie-break between the two
    /// modes: at an equal bin count the better packing is the one that concentrates its slack.
    fn min_bin_density(&self) -> f32 {
        self.bins.iter()
            .map(|b| b.box_area / b.bbox.area())
            .fold(f32::INFINITY, f32::min)
    }
}

/// BPP constructor that packs the items' **bounding boxes** into columns or shelves.
///
/// See the [module documentation](self) for what this is for and why it complements
/// [`BPLBFBuilder`](crate::optimizer::bpp::BPLBFBuilder). Construct with [`Self::new`] and run
/// [`Self::construct`].
pub struct BPShelfBuilder {
    pub instance: BPInstance,
    pub prob: BPProblem,
}

impl BPShelfBuilder {
    /// Creates a builder for `instance` with an empty problem.
    pub fn new(instance: BPInstance) -> Self {
        let prob = BPProblem::new(instance.clone());
        Self { instance, prob }
    }

    /// Packs the full demand into bins and leaves the result in [`Self::prob`].
    ///
    /// Runs both [`Mode::Column`] and [`Mode::Shelf`], keeps the better one (fewer bins; on a tie
    /// the lower least-dense-bin density), materialises it into the [`BPProblem`] and verifies the
    /// outcome with jagua-rs' own collision detection.
    ///
    /// # Errors
    /// * an item does not fit into any bin type in any allowed orientation,
    /// * the bin stock runs out,
    /// * the verification fails — that is a bug in the bbox arithmetic, never a property of the
    ///   input, so it is reported loudly rather than silently repaired.
    pub fn construct(mut self) -> Result<Self> {
        let column = self.pack(Mode::Column)?;
        let shelf = self.pack(Mode::Shelf)?;

        // Fewer bins wins; on a tie the packing whose emptiest bin is emptiest (the slack is more
        // concentrated, which is what the compression phase wants). Deterministic total order.
        let best = [column, shelf].into_iter()
            .min_by_key(|p| (p.bins.len(), OrderedFloat(p.min_bin_density())))
            .expect("two candidate packings");

        info!("[BPSHELF] {:?} mode wins: {} bin(s)", best.mode, best.bins.len());
        self.materialise(&best)?;
        Ok(self)
    }

    /// Runs one first-fit-decreasing pass in the given mode and returns the resulting packing.
    fn pack(&self, mode: Mode) -> Result<Packing> {
        // Every bin type's usable (deflated) area, so the orientation choice can reject items that
        // fit nowhere before any packing work starts.
        let bin_boxes = self.instance.bins.iter()
            .map(|bin| (bin.id, bin.container.outer_cd.bbox))
            .collect_vec();

        // --- 1. Reduce every demanded item to one oriented bbox ------------------------------
        let mut boxes: Vec<BoxItem> = vec![];
        for (item, qty) in self.instance.items.iter() {
            let chosen = self.choose_orientation(item, &bin_boxes, mode)?;
            for _ in 0..*qty {
                boxes.push(chosen);
            }
        }

        // --- 2. Sort: decreasing "along-axis" size, then cross-axis size, then id ------------
        // FFDH sorts by height for shelves; the column variant is the transpose, so the *along*
        // axis of the mode is the one that drives the order. Ties fall back to the cross size and
        // finally the item id, which makes the order a total one (hence deterministic).
        boxes.sort_by_key(|b| {
            let (along, cross) = along_cross(b.w, b.h, mode);
            (Reverse(OrderedFloat(along)), Reverse(OrderedFloat(cross)), b.item_id)
        });

        // --- 3. First-fit into open bands, opening bands and bins as needed ------------------
        let mut stock = self.instance.bins.iter().map(|b| self.prob.bin_stock_qtys[b.id]).collect_vec();
        let mut bins: Vec<ShelfBin> = vec![];

        for b in boxes {
            if Self::place_into_open_bin(&mut bins, &b, mode) {
                continue;
            }
            // Nothing open can take it: open the cheapest bin per unit area that has stock left
            // *and* can host this item at all.
            let bin_id = self.cheapest_bin_for(&b, &bin_boxes, &stock)?;
            let idx = self.instance.bins.iter().position(|bin| bin.id == bin_id)
                .expect("bin id comes from the instance");
            stock[idx] -= 1;

            let bbox = bin_boxes.iter().find(|(id, _)| *id == bin_id).expect("bin id exists").1;
            debug_assert!(b.w + PLACEMENT_GAP <= bbox.width() + FIT_EPS
                && b.h + PLACEMENT_GAP <= bbox.height() + FIT_EPS);
            let mut bin = ShelfBin {
                bin_id,
                bbox,
                bands: vec![],
                cross_used: 0.0,
                placements: vec![],
                box_area: 0.0,
            };
            if !Self::place_into_bin(&mut bin, &b, mode) {
                bail!("[BPSHELF] item {} does not fit into an empty bin of type {bin_id} \
                       (bbox {:.3}x{:.3} vs usable {:.3}x{:.3})",
                    b.item_id, b.w, b.h, bbox.width(), bbox.height());
            }
            bins.push(bin);
        }

        debug!("[BPSHELF] {:?} mode: {} bin(s)", mode, bins.len());
        Ok(Packing { mode, bins })
    }

    /// Picks the orientation an item is packed in.
    ///
    /// The rule is deliberately simple and deterministic, and it is the one that matters for the
    /// column/shelf model: **keep the item slim along the cross axis and tall along the along
    /// axis**, because that is what lets many of them sit side by side while each band stacks
    /// several. Among the orientations that fit in the largest bin, the one with the largest
    /// along-axis extent wins (ties → smallest cross extent → smallest angle), which for a
    /// rectangle means "stand it up" in column mode and "lay it down" in shelf mode.
    ///
    /// Candidate rotations are the item's [`RotationRange::Discrete`] angles; a
    /// [`RotationRange::Continuous`] item is only tried at 0° and 90° (the two orientations whose
    /// bbox a rectilinear model can reason about), and a [`RotationRange::None`] item only at 0°.
    fn choose_orientation(&self, item: &Item, bin_boxes: &[(usize, Rect)], mode: Mode) -> Result<BoxItem> {
        // The largest usable container decides what "fits at all" means.
        let (max_w, max_h) = bin_boxes.iter()
            .map(|(_, r)| (OrderedFloat(r.width()), OrderedFloat(r.height())))
            .fold((OrderedFloat(0.0), OrderedFloat(0.0)), |(w, h), (rw, rh)| (w.max(rw), h.max(rh)));

        let candidate = candidate_rotations(item).into_iter()
            .map(|r| {
                let bbox = rotated_bbox(item, r);
                BoxItem {
                    item_id: item.id,
                    rotation: r,
                    w: bbox.width(),
                    h: bbox.height(),
                    rotated_min: (bbox.x_min, bbox.y_min),
                }
            })
            .filter(|b| b.w + PLACEMENT_GAP <= max_w.into_inner() + FIT_EPS
                && b.h + PLACEMENT_GAP <= max_h.into_inner() + FIT_EPS)
            // Largest along-axis extent first, then the slimmest cross extent, then the smallest
            // angle: a total order, so the choice is deterministic.
            .min_by_key(|b| {
                let (along, cross) = along_cross(b.w, b.h, mode);
                (Reverse(OrderedFloat(along)), OrderedFloat(cross), OrderedFloat(b.rotation))
            });

        match candidate {
            Some(c) => Ok(c),
            None => bail!("[BPSHELF] item {} fits in no bin type in any allowed orientation \
                           (largest usable area: {:.3}x{:.3})", item.id, max_w, max_h),
        }
    }

    /// The cheapest (by `cost / area`) bin type that still has stock and can host `b`.
    fn cheapest_bin_for(&self, b: &BoxItem, bin_boxes: &[(usize, Rect)], stock: &[usize]) -> Result<usize> {
        let choice = self.instance.bins.iter().enumerate()
            .filter(|(idx, _)| stock[*idx] > 0)
            .filter(|(_, bin)| {
                let bbox = bin_boxes.iter().find(|(id, _)| *id == bin.id).expect("bin id exists").1;
                b.w + PLACEMENT_GAP <= bbox.width() + FIT_EPS
                    && b.h + PLACEMENT_GAP <= bbox.height() + FIT_EPS
            })
            // `min_by_key` returns the first minimum → ties resolve to the lowest bin id.
            .min_by_key(|(_, bin)| OrderedFloat(bin.cost as f32 / bin.container.area()))
            .map(|(_, bin)| bin.id);

        match choice {
            Some(bin_id) => Ok(bin_id),
            None => bail!("[BPSHELF] no bin stock left that can host item {} ({:.3}x{:.3})",
                b.item_id, b.w, b.h),
        }
    }

    /// First-fit over the already open bins (in opening order). Returns whether `b` was placed.
    fn place_into_open_bin(bins: &mut [ShelfBin], b: &BoxItem, mode: Mode) -> bool {
        bins.iter_mut().any(|bin| Self::place_into_bin(bin, b, mode))
    }

    /// First-fit inside a single bin: try the open bands in order, else open a new band.
    ///
    /// A band accepts the item when the item's cross extent fits within the band's extent and the
    /// remaining along-axis space is enough. A *new* band is opened when the untouched cross-axis
    /// remainder of the bin can host it. Returns whether the item was placed.
    fn place_into_bin(bin: &mut ShelfBin, b: &BoxItem, mode: Mode) -> bool {
        // Reserve the separating gap along with the item itself, so neighbours never touch.
        let (along, cross) = along_cross(b.w + PLACEMENT_GAP, b.h + PLACEMENT_GAP, mode);
        let (bin_along, bin_cross) = along_cross(
            bin.bbox.width() - PLACEMENT_GAP, bin.bbox.height() - PLACEMENT_GAP, mode);

        // 1. Stack onto an open band.
        for band_idx in 0..bin.bands.len() {
            let band = bin.bands[band_idx];
            if cross <= band.extent + FIT_EPS && band.along + along <= bin_along + FIT_EPS {
                let (cross_off, along_off) = (band.cross, band.along);
                bin.bands[band_idx].along += along;
                bin.commit(b, cross_off, along_off, mode);
                return true;
            }
        }

        // 2. Open a new band on the untouched cross-axis remainder.
        if bin.cross_used + cross <= bin_cross + FIT_EPS && along <= bin_along + FIT_EPS {
            let cross_off = bin.cross_used;
            bin.bands.push(Band { cross: cross_off, extent: cross, along });
            bin.cross_used += cross;
            bin.commit(b, cross_off, 0.0, mode);
            return true;
        }

        false
    }

    /// Turns the winning [`Packing`] into real placements on [`Self::prob`] and verifies them.
    fn materialise(&mut self, packing: &Packing) -> Result<()> {
        for bin in packing.bins.iter() {
            if self.prob.bin_stock_qtys[bin.bin_id] == 0 {
                bail!("[BPSHELF] bin type {} ran out of stock while materialising the packing", bin.bin_id);
            }
            // The first placement opens the layout (`Closed`), the rest join it (`Open`).
            let mut lkey = None;
            for (item_id, d_transf) in bin.placements.iter() {
                let layout_id = match lkey {
                    None => BPLayoutType::Closed { bin_id: bin.bin_id },
                    Some(l) => BPLayoutType::Open(l),
                };
                let (l, _) = self.prob.place_item(BPPlacement { layout_id, item_id: *item_id, d_transf: *d_transf });
                lkey = Some(l);
            }
        }

        // --- Verification: this must hold by construction, so failure is a bug ----------------
        if !self.prob.item_demand_qtys.iter().all(|&d| d == 0) {
            bail!("[BPSHELF] the packing does not cover the full demand (missing: {:?})",
                self.prob.item_demand_qtys);
        }
        for (lkey, layout) in self.prob.layouts.iter() {
            if !layout.is_feasible() {
                bail!("[BPSHELF] layout {lkey:?} of the bbox packing collides — the bbox model must \
                       never produce a collision, this is a bug in the shelf constructor");
            }
        }

        info!("[BPSHELF] placed all {} item(s) into {} bin(s) (cost: {}, dens: {:.3}%)",
            self.prob.n_placed_items(), self.prob.layouts.len(), self.prob.bin_cost(),
            self.prob.density() * 100.0);
        Ok(())
    }
}

impl ShelfBin {
    /// Records a placement at the given band offsets, converting them into a [`DTransformation`].
    ///
    /// The item's collision shape is centred by its `pre_transform`, so the translation that lands
    /// its **rotated** bbox's min corner on `(x, y)` is `(x, y) - rotated_bbox.min_corner`.
    fn commit(&mut self, b: &BoxItem, cross_off: f32, along_off: f32, mode: Mode) {
        let (x_off, y_off) = match mode {
            Mode::Column => (cross_off, along_off),
            Mode::Shelf => (along_off, cross_off),
        };
        let target_x = self.bbox.x_min + x_off + PLACEMENT_GAP;
        let target_y = self.bbox.y_min + y_off + PLACEMENT_GAP;

        // `rotated_min` is where the bbox min corner sits when only the rotation is applied.
        let (min_x, min_y) = b.rotated_min;
        let d_transf = DTransformation::new(b.rotation, (target_x - min_x, target_y - min_y));

        self.placements.push((b.item_id, d_transf));
        self.box_area += b.w * b.h;
    }
}

/// Maps `(w, h)` onto `(along, cross)` for the given mode.
///
/// Columns run along **y** (they stack upwards) and are laid out along **x**; shelves are the
/// transpose. Keeping this in one place is what makes the two modes share all the packing code.
fn along_cross(w: f32, h: f32, mode: Mode) -> (f32, f32) {
    match mode {
        Mode::Column => (h, w),
        Mode::Shelf => (w, h),
    }
}

/// The rotations to consider for an item.
///
/// [`RotationRange::Discrete`] contributes its angles verbatim, [`RotationRange::Continuous`] the
/// two rectilinear ones (0° and 90°) — a bbox model cannot exploit an arbitrary angle — and
/// [`RotationRange::None`] only 0°. The result is deduplicated and sorted so the caller's choice is
/// deterministic.
pub(crate) fn candidate_rotations(item: &Item) -> Vec<f32> {
    let mut rotations = match &item.allowed_rotation {
        RotationRange::None => vec![0.0],
        RotationRange::Continuous => vec![0.0, PI / 2.0],
        RotationRange::Discrete(angles) => angles.clone(),
    };
    rotations.sort_by_key(|r| OrderedFloat(*r));
    rotations.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
    rotations
}

/// The bbox of `item.shape_cd` after applying `rotation` (no translation).
pub(crate) fn rotated_bbox(item: &Item, rotation: f32) -> Rect {
    if rotation == 0.0 {
        return item.shape_cd.bbox;
    }
    item.shape_cd.transform_clone(&Transformation::from_rotation(rotation)).bbox
}
