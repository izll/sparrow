//! **Final-answer gates** shared by the `sparrow` and `sparrow-bpp` binaries.
//!
//! Everything in this module answers one question: *may this solution be written to disk?* The
//! optimizer pipelines are free to hold infeasible intermediate states — that is what the separator
//! is for — but the moment a solution becomes the program's answer it has to be checked, because
//! nothing downstream can tell a good export from a bad one. The audited failures this module
//! exists to stop were all of the same shape: a warm start (or a fallback path) that was never
//! verified, exported with exit code `0`.
//!
//! Three independent properties are checked, in both problem variants:
//!
//! 1. **Exact demand coverage per item id.** Not the total count — per id. A solution that places
//!    two copies of item 3 and none of item 4 has the right total and is still wrong.
//! 2. **Geometric feasibility**, via [`Layout::from_snapshot(..).is_feasible()`].
//! 3. **Cuttability** (walled SPP only): no item's collision bbox may straddle a sheet wall.
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
