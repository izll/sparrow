//! **Final-answer gates** shared by the `sparrow` and `sparrow-bpp` binaries.
//!
//! Everything in this module answers one question: *may this solution be written to disk?* The
//! optimizer pipelines are free to hold infeasible intermediate states — that is what the separator
//! is for — but the moment a solution becomes the program's answer it has to be checked, because
//! nothing downstream can tell a good export from a bad one. The audited failures this module
//! exists to stop were all of the same shape: a warm start (or a fallback path) that was never
//! verified, exported with exit code `0`.
//!
//! Four independent properties are checked, in both problem variants:
//!
//! 1. **Exact demand coverage per item id.** Not the total count — per id. A solution that places
//!    two copies of item 3 and none of item 4 has the right total and is still wrong.
//! 2. **Geometric feasibility**, via [`Layout::from_snapshot(..).is_feasible()`].
//! 3. **Allowed rotations.** Every placement's angle has to be one the item actually permits. This
//!    is the one property that is *invisible to geometry*: a part placed at 45° when only 0° is
//!    allowed can be perfectly collision-free and still be scrap, because `allowed_orientations`
//!    is how a grain, a pattern or a laminate direction is expressed. The audit exported exactly
//!    that at exit `0` — a warm start whose single placement carried a 45° rotation for an item
//!    declared `allowed_orientations: [0.0]` — and only the Python validator caught it.
//! 4. **Cuttability** (walled SPP only): no item's collision bbox may straddle a sheet wall.
//!
//! A violation is an `Err`, which both binaries turn into an exit code of `1` *before* any JSON is
//! written. Failing loudly and writing nothing is the only safe answer: a layout with overlapping
//! parts or missing pieces that reaches a CNC is worse than no layout at all.

use crate::config::SheetConfig;
use crate::optimizer::sheets::sheet_stats;
use anyhow::{bail, Result};
use jagua_rs::entities::{Instance, Layout};
use jagua_rs::probs::bpp::entities::{BPInstance, BPSolution};
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use std::collections::BTreeMap;

/// Formats a demand mismatch table into a readable one-line message.
fn demand_error(placed: &BTreeMap<usize, usize>, demand: &BTreeMap<usize, usize>) -> String {
    let mut ids: Vec<usize> = demand.keys().chain(placed.keys()).copied().collect();
    ids.sort_unstable();
    ids.dedup();
    ids.into_iter()
        .filter_map(|id| {
            let (p, d) = (placed.get(&id).copied().unwrap_or(0), demand.get(&id).copied().unwrap_or(0));
            (p != d).then(|| format!("item {id}: placed {p}, demanded {d}"))
        })
        .collect::<Vec<_>>()
        .join("; ")
}

/// Checks that `placed` matches `demand` **for every item id**, and that no unknown id occurs.
fn check_demand(placed: &BTreeMap<usize, usize>, demand: &BTreeMap<usize, usize>, what: &str) -> Result<()> {
    if placed != demand {
        bail!("{what} does not cover the instance demand exactly ({})", demand_error(placed, demand));
    }
    Ok(())
}

/// The per-item-id demand of an SPP instance.
pub fn spp_demand(instance: &SPInstance) -> BTreeMap<usize, usize> {
    instance.items.iter().map(|(item, qty)| (item.id, *qty)).collect()
}

/// The per-item-id demand of a BPP instance.
pub fn bpp_demand(instance: &BPInstance) -> BTreeMap<usize, usize> {
    instance.items.iter().map(|(item, qty)| (item.id, *qty)).collect()
}

/// How many copies of each item id an SPP solution places.
pub fn spp_placed(sol: &SPSolution) -> BTreeMap<usize, usize> {
    let mut placed = BTreeMap::new();
    for (_, pi) in sol.layout_snapshot.placed_items.iter() {
        *placed.entry(pi.item_id).or_insert(0) += 1;
    }
    placed
}

/// How many copies of each item id a BPP solution places, summed over all its layouts.
pub fn bpp_placed(sol: &BPSolution) -> BTreeMap<usize, usize> {
    let mut placed = BTreeMap::new();
    for ls in sol.layout_snapshots.values() {
        for (_, pi) in ls.placed_items.iter() {
            *placed.entry(pi.item_id).or_insert(0) += 1;
        }
    }
    placed
}

/// Angular tolerance of the rotation gate, in **degrees**.
///
/// The exported/imported angles are `f32` and go through degree↔radian conversions on both sides, so
/// exact equality is unusable: a nominal 180° round-trips as 179.99998. 1e-3° is four orders of
/// magnitude below any rotation a human would declare and far above that noise. It matches
/// `scripts/validate_solution.py`'s `ANGLE_TOL`, so the Rust gate and the independent validator
/// agree on the boundary rather than disagreeing by a hair.
pub const ROTATION_TOL_DEG: f32 = 1e-3;

/// The same tolerance in radians, which is what the library's angles are in.
pub const ROTATION_TOL_RAD: f32 = ROTATION_TOL_DEG * std::f32::consts::PI / 180.0;

/// Checks every placement's rotation against the item's [`RotationRange`](jagua_rs::geometry::geo_enums::RotationRange).
///
/// `placements` yields `(item_id, rotation_in_radians, where)`, `where` naming the layout for the
/// error message (`""` for the single-layout SPP case).
///
/// * `None` ⇒ the rotation must be 0° modulo 360°,
/// * `Discrete(a)` ⇒ it must equal one of `a` modulo 360°,
/// * `Continuous` ⇒ anything finite.
///
/// Modulo 360° is not a nicety: the engine exports the angle it happens to hold, and an item with
/// `[0, 180]` is routinely written out as `-180`. A comparison without the wrap-around would reject
/// correct solutions, which is how a gate gets disabled.
fn check_rotations<'a>(
    placements: impl Iterator<Item = (usize, f32, &'a str)>,
    item_of: impl Fn(usize) -> Option<&'a jagua_rs::entities::Item>,
    what: &str,
) -> Result<()> {
    use crate::util::rotations::{describe_allowed, rotation_is_allowed};

    let mut bad: Vec<String> = vec![];
    for (item_id, rotation, whereabouts) in placements {
        let Some(item) = item_of(item_id) else {
            bail!("{what} places an unknown item id {item_id}");
        };
        if !rotation_is_allowed(item, rotation, ROTATION_TOL_RAD) {
            bad.push(format!(
                "item {item_id}{whereabouts} at {:.3}° (allowed: {})",
                rotation.to_degrees(),
                describe_allowed(item)
            ));
            if bad.len() >= 8 {
                break;
            }
        }
    }
    if !bad.is_empty() {
        bail!(
            "{what} places {} item(s) at a rotation the item does not allow: {}. \
             An angle outside `allowed_orientations` is not a geometric defect — the layout can be \
             perfectly collision-free — but it is unmanufacturable whenever the material has a \
             grain or pattern direction, which is exactly why the item declares the list",
            bad.len(),
            bad.join("; ")
        );
    }
    Ok(())
}

/// The rotation gate for an SPP solution. See [`check_rotations`].
pub fn verify_spp_rotations(sol: &SPSolution, instance: &SPInstance, what: &str) -> Result<()> {
    check_rotations(
        sol.layout_snapshot.placed_items.iter()
            .map(|(_, pi)| (pi.item_id, pi.d_transf.rotation(), "")),
        |id| instance.items.get(id).map(|(item, _)| item),
        what,
    )
}

/// The rotation gate for a BPP solution. See [`check_rotations`].
pub fn verify_bpp_rotations(sol: &BPSolution, instance: &BPInstance, what: &str) -> Result<()> {
    let placements: Vec<(usize, f32, String)> = sol.layout_snapshots.iter()
        .flat_map(|(lkey, ls)| {
            ls.placed_items.iter()
                .map(move |(_, pi)| (pi.item_id, pi.d_transf.rotation(), format!(" in layout {lkey:?}")))
        })
        .collect();
    check_rotations(
        placements.iter().map(|(id, r, w)| (*id, *r, w.as_str())),
        |id| instance.items.get(id).map(|(item, _)| item),
        what,
    )
}

/// Ids of the items whose collision bbox **straddles a sheet wall** in a walled SPP solution.
///
/// Empty for any cuttable layout. A non-empty result means the strip cannot be cut into physical
/// sheets at all: at least one part sits across a boundary and would be sliced in two.
pub fn straddling_items(sol: &SPSolution, instance: &SPInstance, sheet: &SheetConfig) -> Vec<usize> {
    sheet_stats(sol, instance, sheet).into_iter()
        .flat_map(|s| s.straddling_item_ids)
        .collect()
}

/// The full **export gate** for an SPP solution: demand, feasibility and (in walled mode)
/// cuttability.
///
/// `what` names the solution in the error message ("the final solution", "the warm start", ...).
///
/// This is the gate that used to be missing entirely in the single-run path. `optimize` had a
/// "possibly infeasible" fallback that handed the separator's *current* layout back as the answer
/// whenever the exploration phase never reached feasibility; with a zero-time budget and an
/// overlapping warm start that path exported a layout with 1 185 179.5 mm² of overlap at exit `0`.
pub fn verify_spp_solution(
    sol: &SPSolution,
    instance: &SPInstance,
    sheet: Option<&SheetConfig>,
    what: &str,
) -> Result<()> {
    check_demand(&spp_placed(sol), &spp_demand(instance), what)?;

    verify_spp_rotations(sol, instance, what)?;

    if !Layout::from_snapshot(&sol.layout_snapshot).is_feasible() {
        bail!("{what} is not collision-free: items overlap each other, the strip border or a sheet wall");
    }

    if let Some(sheet) = sheet {
        let straddling = straddling_items(sol, instance, sheet);
        if !straddling.is_empty() {
            bail!("{what} has {} item(s) straddling a sheet wall (item id(s): {}), so the strip \
                   cannot be cut into {} mm sheets",
                straddling.len(),
                straddling.iter().map(usize::to_string).collect::<Vec<_>>().join(", "),
                sheet.width);
        }
    }
    Ok(())
}

/// The full **export gate** for a BPP solution: demand coverage, per-layout feasibility and bin
/// stock.
pub fn verify_bpp_solution(sol: &BPSolution, instance: &BPInstance, what: &str) -> Result<()> {
    check_demand(&bpp_placed(sol), &bpp_demand(instance), what)?;

    verify_bpp_rotations(sol, instance, what)?;

    for (lkey, ls) in sol.layout_snapshots.iter() {
        if !Layout::from_snapshot(ls).is_feasible() {
            bail!("{what}: layout {lkey:?} is not collision-free");
        }
    }

    // Never use more copies of a bin type than the instance has in stock.
    let mut used: BTreeMap<usize, usize> = BTreeMap::new();
    for ls in sol.layout_snapshots.values() {
        *used.entry(ls.container.id).or_insert(0) += 1;
    }
    for (bin_id, n) in used {
        let stock = instance.bins.iter().find(|b| b.id == bin_id).map(|b| b.stock);
        match stock {
            Some(stock) if n > stock => bail!("{what} uses {n} copies of bin type {bin_id}, but only {stock} are in stock"),
            None => bail!("{what} uses an unknown bin type {bin_id}"),
            _ => {}
        }
    }
    Ok(())
}
