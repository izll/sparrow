//! **Packability pre-checks**: turn a structurally impossible (or merely awkward) instance into a
//! clear error message before the engine walks into a panic.
//!
//! The strip-packing pipeline has two failure modes that are indistinguishable from a crash:
//!
//! * an item **taller than the strip** can never be placed, so the LBF constructor keeps widening
//!   the strip by 20 % looking for room that does not exist, until
//!   [`assertions::strip_width_is_in_check`](crate::util::assertions::strip_width_is_in_check)
//!   fires: `strip-width is running away (>165.113), item 0 does not seem to fit into the strip`,
//!   `panic = abort`, exit `134`;
//! * a **`--min-sep` larger than the instance's own scale** deflates the initial container (whose
//!   width jagua derives as `total_item_area / strip_height`, i.e. the 100 %-density width) into an
//!   empty polygon: `called Result::unwrap() on an Err value: Offset resulted in an empty polygon`,
//!   again exit `134`. Measured with two 10x10 items in a 100 mm strip at `--min-sep 20`, where a
//!   wider starting strip would have solved the instance perfectly well.
//!
//! Both are *input* problems, and both are cheap to detect up front. The walled mode already had
//! its own version of the first check
//! ([`items_too_wide_for_sheet`](crate::optimizer::sheets::items_too_wide_for_sheet)); this module
//! generalises it to the strip height, which every SPP run has, walled or not.

use anyhow::{bail, Result};
use jagua_rs::entities::Instance;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::TransformableFrom;
use jagua_rs::geometry::Transformation;
use jagua_rs::probs::spp::entities::SPInstance;
use ordered_float::OrderedFloat;
use std::f32::consts::PI;

/// Rotation grid used to test a continuously rotatable item, matching the one
/// [`items_too_wide_for_sheet`](crate::optimizer::sheets::items_too_wide_for_sheet) and the
/// placement sampler use.
const ROT_N_SAMPLES: usize = 24;

/// The rotations to test for an item: its allowed set, or a uniform grid for continuous rotation.
fn rotations_of(item: &jagua_rs::entities::Item) -> Vec<f32> {
    match &item.allowed_rotation {
        RotationRange::None => vec![0.0],
        RotationRange::Discrete(r) => r.clone(),
        RotationRange::Continuous => (0..ROT_N_SAMPLES)
            .map(|i| i as f32 * (2.0 * PI) / ROT_N_SAMPLES as f32)
            .collect(),
    }
}

/// Items of `instance` that are **taller than the strip in every allowed rotation**, with that
/// minimum height.
///
/// The test uses `shape_cd` — the *collision* shape, i.e. the contour already inflated by
/// `min_sep / 2` — because that is the shape the engine actually has to fit between the strip's
/// deflated borders. An item that fits its raw contour into the strip but not its inflated one is
/// just as unplaceable, and reporting it here is far more useful than the runaway-width panic.
pub fn items_too_tall_for_strip(instance: &SPInstance) -> Vec<(usize, f32)> {
    let height = instance.base_strip.fixed_height;
    instance.items.iter()
        .filter_map(|(item, _)| {
            let mut buffer = item.shape_cd.as_ref().clone();
            let min_height = rotations_of(item).iter()
                .map(|&r| {
                    let bbox = buffer
                        .transform_from(item.shape_cd.as_ref(), &Transformation::from_rotation(r))
                        .bbox;
                    OrderedFloat(bbox.height())
                })
                .min()
                .map(|h| h.0)
                .unwrap_or(f32::INFINITY);
            (min_height > height).then_some((item.id, min_height))
        })
        .collect()
}

/// The **smallest strip width** at which the container survives its `min_sep / 2` deflation.
///
/// jagua derives the initial strip width from the item area (`total_item_area / strip_height`), and
/// deflating a `w x h` rectangle by `d` leaves a `(w - 2d) x (h - 2d)` one — empty as soon as
/// `w <= 2d`. This returns the width the strip has to have for the deflated container to be
/// non-degenerate, or `None` when no deflation is applied.
pub fn min_viable_strip_width(min_item_separation: Option<f32>) -> Option<f32> {
    let sep = min_item_separation?;
    if sep <= 0.0 {
        return None;
    }
    // Deflation is `min_sep / 2` per side, so a width of exactly `min_sep` collapses to zero.
    // Ask for a little more than that so the result is a usable container, not a degenerate one.
    Some(sep * 1.5)
}

/// The full SPP packability gate, run once at start-up.
///
/// Returns an `Err` describing the problem, or `Ok(Some(w))` when the caller should **widen the
/// initial strip to `w`** before constructing anything (the deflation case), or `Ok(None)` when the
/// instance is fine as it is.
pub fn check_spp_packability(
    instance: &SPInstance,
    min_item_separation: Option<f32>,
) -> Result<Option<f32>> {
    let height = instance.base_strip.fixed_height;

    let too_tall = items_too_tall_for_strip(instance);
    if !too_tall.is_empty() {
        let list = too_tall.iter()
            .map(|(id, h)| format!("item {id} ({:.1} mm)", h))
            .collect::<Vec<_>>()
            .join(", ");
        let sep_note = match min_item_separation {
            Some(sep) if sep > 0.0 => format!(" (heights include the {:.1} mm inflation --min-sep {sep} applies to every item)", sep / 2.0),
            _ => String::new(),
        };
        bail!("the strip is only {height:.1} mm high, but {list} do(es) not fit inside it in any \
               allowed rotation{sep_note}, so no solution can exist");
    }

    // The container has to survive its own deflation, or `Strip -> Container` aborts.
    if let Some(min_width) = min_viable_strip_width(min_item_separation) {
        let sep = min_item_separation.expect("min_viable_strip_width returns None without a separation");
        if height <= sep {
            bail!("--min-sep {sep} is at least as large as the {height:.1} mm strip height: deflating \
                   the container by {:.1} mm on each side leaves nothing to pack into. Use a smaller \
                   separation or a taller strip", sep / 2.0);
        }
        if instance.base_strip.width < min_width {
            return Ok(Some(min_width));
        }
    }
    Ok(None)
}
