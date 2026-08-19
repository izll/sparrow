//! **The demand cap, in one place.** Both binaries ask the same question — "is this instance's
//! total demand something we can actually build?" — so they ask the same function.
//!
//! It was two copies, one per binary, and they were two copies of a bug. The gate read:
//!
//! ```ignore
//! let total_demand: u64 = ext_instance.items.iter().map(|it| it.demand).sum();
//! if total_demand > MAX_TOTAL_DEMAND { bail!(..) }
//! ```
//!
//! `sum::<u64>()` **wraps** in release. The audit's fixture — one item demanding `u64::MAX`, plus one
//! demanding `1` — therefore summed to `0`, sailed straight past a cap it exceeded by the widest
//! possible margin, and died deep inside the constructor instead: `no pole found` for SPP,
//! `capacity overflow` for BPP, both exit `134`, neither with a message a user could act on. The
//! check that existed to turn an abort into an error was itself producing the abort.
//!
//! [`total_demand`] makes the overflow the error rather than the thing that hides it, and caps each
//! individual demand as well: a single `u64::MAX` item cannot overflow a sum it is the only term
//! of, so a total-only check would still have let it through to the same allocation.

use anyhow::{bail, Result};

/// Largest total demand (summed over all item types) the binaries accept.
///
/// Both LBF constructors materialise **one `Vec` element per demanded copy**
/// (`iter::repeat_n(id, missing_qty)`), so a demand is an allocation size, not just a number: a
/// 100-million fixture reserved ~800 MB before placing a single item and aborted. The cap sits far
/// above any real nesting job and turns that abort into a message.
pub const MAX_TOTAL_DEMAND: u64 = 1_000_000;

/// Largest **single-item** demand accepted.
///
/// The same number as [`MAX_TOTAL_DEMAND`] — one item cannot legitimately exceed the total either —
/// but checked separately and *first*, because a per-item check is the only one that can catch a
/// lone `u64::MAX` before it reaches the addition.
pub const MAX_ITEM_DEMAND: u64 = MAX_TOTAL_DEMAND;

/// Sums per-item demands, rejecting an over-large individual demand, an over-large total, and any
/// arithmetic overflow — **without ever wrapping**.
///
/// The index in the error messages is the item's position in the instance's item list, which is what
/// a user editing the JSON is looking at.
pub fn total_demand(demands: impl Iterator<Item = u64>) -> Result<u64> {
    demands.enumerate().try_fold(0u64, |total, (idx, d)| {
        if d > MAX_ITEM_DEMAND {
            bail!("item #{idx} demands {d} copies, more than the supported maximum of \
                   {MAX_ITEM_DEMAND} for a single item; every copy is materialised individually, \
                   so this would exhaust memory before the first placement");
        }
        let total = total.checked_add(d).ok_or_else(|| anyhow::anyhow!(
            "the instance's total demand overflows a 64-bit counter at item #{idx} (running total \
             {total}, this item {d}); such an instance cannot be packed, and the unchecked sum this \
             replaces wrapped around to a small number and slipped straight past the demand limit"))?;
        if total > MAX_TOTAL_DEMAND {
            bail!("the instance demands more than {MAX_TOTAL_DEMAND} items in total (already \
                   {total} at item #{idx}); every copy is materialised individually, so this would \
                   exhaust memory before the first placement");
        }
        Ok(total)
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_normal_instance_sums_normally() {
        assert_eq!(total_demand([3, 4, 5].into_iter()).unwrap(), 12);
        assert_eq!(total_demand(std::iter::empty()).unwrap(), 0);
    }

    #[test]
    fn the_cap_itself_is_accepted_and_one_more_is_not() {
        assert_eq!(total_demand([MAX_TOTAL_DEMAND].into_iter()).unwrap(), MAX_TOTAL_DEMAND);
        assert!(total_demand([MAX_TOTAL_DEMAND + 1].into_iter()).is_err());
        // Split across two items, so only the *running total* exceeds it.
        assert!(total_demand([MAX_TOTAL_DEMAND, 1].into_iter()).is_err());
    }

    /// The audit's fixture. The old `sum()` wrapped this to `0` and let it through.
    #[test]
    fn the_audit_overflow_fixture_is_an_error_not_a_wrap() {
        let err = total_demand([u64::MAX, 1].into_iter()).unwrap_err().to_string();
        assert!(err.contains("item #0"), "the first item is the culprit: {err}");
        // And the wrap it replaces really did produce 0.
        assert_eq!(u64::MAX.wrapping_add(1), 0);
    }

    /// A lone `u64::MAX` overflows nothing, so only the per-item cap can catch it.
    #[test]
    fn a_single_enormous_demand_is_caught_by_the_per_item_cap() {
        assert!(total_demand([u64::MAX].into_iter()).is_err());
    }
}
