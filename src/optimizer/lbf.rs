use crate::config::SheetConfig;
use crate::eval::lbf_evaluator::LBFEvaluator;
use crate::eval::sample_eval::SampleEval;
use crate::optimizer::sheets::apply_sheet_walls_opt;
use crate::sample::search::{search_placement, SampleConfig};
use crate::util::assertions;
use itertools::Itertools;
use jagua_rs::entities::Instance;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::Instant;
use log::debug;
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::iter;
use rand::rngs::Xoshiro256PlusPlus;

pub struct LBFBuilder {
    pub instance: SPInstance,
    pub prob: SPProblem,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
    /// Multi-sheet ("walled") mode; the walls are re-applied after every strip width change so that
    /// the constructor never places an item across a sheet boundary. See [`crate::optimizer::sheets`].
    pub sheet: Option<SheetConfig>,
}

impl LBFBuilder {
    pub fn new(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
    ) -> Self {
        Self::new_with_sheet(instance, rng, sample_config, None)
    }

    /// Like [`LBFBuilder::new`], but for the multi-sheet ("walled") mode.
    pub fn new_with_sheet(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
        sheet: Option<SheetConfig>,
    ) -> Self {
        let mut prob = SPProblem::new(instance.clone());
        apply_sheet_walls_opt(&mut prob, sheet.as_ref());

        Self {
            instance,
            prob,
            rng,
            sample_config,
            sheet,
        }
    }

    pub fn construct(mut self) -> Self {
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

        debug!("[CONSTR] placing items in order: {:?}",sorted_item_indices);

        for item_id in sorted_item_indices {
            self.place_item(item_id);
        }

        self.prob.fit_strip();
        apply_sheet_walls_opt(&mut self.prob, self.sheet.as_ref());
        debug!("[CONSTR] placed all items in width: {:.3} (in {:?})",self.prob.strip_width(), start.elapsed());
        self
    }

    fn place_item(&mut self, item_id: usize) {
        match self.find_placement(item_id) {
            Some(p_opt) => {
                self.prob.place_item(p_opt);
                debug!("[CONSTR] placing item {}/{} with id {} at [{}]",self.prob.layout.placed_items.len(),self.instance.total_item_qty(),p_opt.item_id,p_opt.d_transf);
            }
            None => {
                debug!("[CONSTR] failed to place item with id {}, expanding strip width",item_id);
                self.prob.change_strip_width(self.prob.strip_width() * 1.2);
                apply_sheet_walls_opt(&mut self.prob, self.sheet.as_ref());
                assert!(assertions::strip_width_is_in_check(&self.prob), "strip-width is running away (>{:.3}), item {item_id} does not seem to fit into the strip", self.prob.strip_width());          
                self.place_item(item_id);
            }
        }
    }

    fn find_placement(&mut self, item_id: usize) -> Option<SPPlacement> {
        let layout = &self.prob.layout;
        let item = self.instance.item(item_id);
        let evaluator = LBFEvaluator::new(layout, item);

        let (best_sample, _) = search_placement(layout, item, None, evaluator, self.sample_config, &mut self.rng);

        match best_sample {
            Some((d_transf, SampleEval::Clear { .. })) => {
                Some(SPPlacement { item_id, d_transf })
            }
            _ => None
        }
    }
}
