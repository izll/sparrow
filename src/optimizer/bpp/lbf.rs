use crate::eval::lbf_evaluator::LBFEvaluator;
use crate::eval::sample_eval::SampleEval;
use crate::sample::search::{search_placement, SampleConfig};
use anyhow::{bail, Result};
use itertools::Itertools;
use jagua_rs::entities::{Instance, Item, Layout};
use jagua_rs::probs::bpp::entities::{BPInstance, BPLayoutType, BPPlacement, BPProblem, LayKey};
use jagua_rs::Instant;
use jagua_rs::geometry::DTransformation;
use log::debug;
use ordered_float::OrderedFloat;
use rand::rngs::Xoshiro256PlusPlus;
use std::cmp::Reverse;
use std::iter;

/// BPP counterpart of [`crate::optimizer::lbf::LBFBuilder`]: constructs a first (feasible) solution
/// by placing the items one by one, left-bottom-first, into bins.
///
/// The item ordering is identical to the SPP builder (convex hull area × diameter, descending,
/// expanded by demand). The difference is what happens when an item does not fit: the SPP builder
/// widens the strip, the BPP builder opens a new bin.
pub struct BPLBFBuilder {
    pub instance: BPInstance,
    pub prob: BPProblem,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
}

impl BPLBFBuilder {
    pub fn new(instance: BPInstance, rng: Xoshiro256PlusPlus, sample_config: SampleConfig) -> Self {
        let prob = BPProblem::new(instance.clone());
        Self { instance, prob, rng, sample_config }
    }

    /// Places all demanded items into bins, largest first.
    ///
    /// For each item, the already open layouts are tried in creation order (deterministic SlotMap
    /// iteration) with the [`LBFEvaluator`]; the first [`SampleEval::Clear`] sample wins. If the
    /// item fits in none of them, a new bin is opened: the bin type with remaining stock and the
    /// lowest `cost / area` ratio.
    ///
    /// Fails (`Err`) if an item fits nowhere and no bin stock is left.
    pub fn construct(mut self) -> Result<Self> {
        let start = Instant::now();
        let n_items = self.instance.items.len();

        let sorted_item_indices = (0..n_items)
            .sorted_by_cached_key(|id| {
                let item_shape = self.instance.item(*id).shape_cd.as_ref();
                let convex_hull_area = item_shape.surrogate().convex_hull_area;
                let diameter = item_shape.diameter;
                Reverse(OrderedFloat(convex_hull_area * diameter))
            })
            .flat_map(|id| {
                let missing_qty = self.prob.item_demand_qtys[id];
                iter::repeat_n(id, missing_qty)
            })
            .collect_vec();

        debug!("[BPLBF] placing items in order: {:?}", sorted_item_indices);

        for item_id in sorted_item_indices {
            self.place_item(item_id)?;
        }

        debug!("[BPLBF] placed all {} items into {} bin(s) (cost: {}, dens: {:.3}%) in {:?}",
            self.prob.n_placed_items(), self.prob.layouts.len(), self.prob.bin_cost(),
            self.prob.density() * 100.0, start.elapsed());

        Ok(self)
    }

    /// Places a single item: first into an already open layout, otherwise into a freshly opened bin.
    fn place_item(&mut self, item_id: usize) -> Result<()> {
        // 1. Try all open layouts in creation order.
        if let Some((lkey, d_transf)) = self.find_placement_in_open_layouts(item_id) {
            self.prob.place_item(BPPlacement { layout_id: BPLayoutType::Open(lkey), item_id, d_transf });
            debug!("[BPLBF] placed item {} in existing layout {:?} at [{}]", item_id, lkey, d_transf);
            return Ok(());
        }

        // 2. No open layout can host it: open a new bin.
        //    Prefer the cheapest bin type per unit of area, among those with remaining stock.
        //    `min_by_key` returns the first minimum, so ties resolve to the lowest bin id.
        let bin_id = self.instance.bins.iter()
            .filter(|bin| self.prob.bin_stock_qtys[bin.id] > 0)
            .min_by_key(|bin| OrderedFloat(bin.cost as f32 / bin.container.area()))
            .map(|bin| bin.id);

        let bin_id = match bin_id {
            Some(bin_id) => bin_id,
            None => bail!("no bin stock left: item {item_id} does not fit in any open layout and no new bin can be opened"),
        };

        // Search for a placement in a *hypothetical* empty layout of this bin type, so we never
        // have to open (and possibly close again) a bin that cannot host the item.
        let empty_layout = Layout::new(self.instance.bins[bin_id].container.clone());
        let d_transf = match search_lbf(&empty_layout, self.instance.item(item_id), self.sample_config, &mut self.rng) {
            Some(d_transf) => d_transf,
            None => bail!("item {item_id} does not fit in an empty bin of type {bin_id}"),
        };

        // `place_item(Closed{..})` does not check stock itself, so the check above is mandatory.
        let (lkey, _) = self.prob.place_item(BPPlacement {
            layout_id: BPLayoutType::Closed { bin_id },
            item_id,
            d_transf,
        });
        debug!("[BPLBF] opened new bin {} (layout {:?}) for item {} at [{}]", bin_id, lkey, item_id, d_transf);
        Ok(())
    }

    /// Searches all open layouts (creation order) for a collision-free LBF placement of the item.
    fn find_placement_in_open_layouts(&mut self, item_id: usize) -> Option<(LayKey, DTransformation)> {
        let lkeys = self.prob.layouts.keys().collect_vec();
        for lkey in lkeys {
            let layout = &self.prob.layouts[lkey];
            let item = self.instance.item(item_id);
            if let Some(d_transf) = search_lbf(layout, item, self.sample_config, &mut self.rng) {
                return Some((lkey, d_transf));
            }
        }
        None
    }

}

/// Searches a layout for a collision-free (left-bottom-first) placement of `item`.
/// Free function so it can be used both on layouts owned by the problem and on detached ones.
fn search_lbf(layout: &Layout, item: &Item, sample_config: SampleConfig, rng: &mut Xoshiro256PlusPlus) -> Option<DTransformation> {
    let evaluator = LBFEvaluator::new(layout, item);
    let (best_sample, _) = search_placement(layout, item, None, evaluator, sample_config, rng);
    match best_sample {
        Some((d_transf, SampleEval::Clear { .. })) => Some(d_transf),
        _ => None,
    }
}
