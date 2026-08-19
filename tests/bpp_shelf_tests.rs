//! Phase 5 integration tests: the deterministic bbox **shelf/column** constructor
//! ([`BPShelfBuilder`]).
//!
//! Two instance families are used:
//!
//! * an **o90-like** rectangular set (the real MADisoCAD case that motivated the constructor):
//!   7 rectangle types, all 600 wide, demand 7 each, all four orientations allowed, packed into
//!   1990 x 995 bins with `--min-sep 5`. This is where the bbox model is supposed to shine.
//! * the irregular **swim** set, which is where it is *not* supposed to shine — the test only
//!   asserts that it stays collision-free, since a bbox packing of irregular contours is by
//!   construction wasteful (that is exactly why `Constructive::Best` exists).

#[cfg(test)]
mod bpp_shelf_integration_tests {
    use anyhow::Result;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtItem as ExtBaseItem, ExtSPolygon, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::BPInstance;
    use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBin, ExtItem};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::{DEFAULT_BPP_CONFIG, DEFAULT_SPARROW_CONFIG};
    use sparrow::optimizer::bpp::optimize_bpp;
    use sparrow::optimizer::bpp::shelf::BPShelfBuilder;
    use sparrow::util::bpp_io::DummyBPSolListener;
    use sparrow::util::io;
    use sparrow::util::terminator::BasicTerminator;
    use std::path::Path;
    use std::time::Duration;

    /// The o90 part set: 7 rectangles, all 600 wide, these heights, demand 7 each.
    const O90_HEIGHTS: [f32; 7] = [992.0, 540.0, 102.0, 96.0, 102.0, 892.0, 508.0];
    const O90_WIDTH: f32 = 600.0;
    const O90_DEMAND: usize = 7;
    const O90_BIN_W: f32 = 1990.0;
    const O90_BIN_H: f32 = 995.0;
    const O90_MIN_SEP: f32 = 5.0;

    /// Builds the o90 instance in code: rectangles as explicit polygons, all four orientations
    /// allowed, one 1990 x 995 bin type, imported with `min_item_separation = 5`.
    fn build_o90_instance() -> Result<BPInstance> {
        let config = DEFAULT_SPARROW_CONFIG;

        let items = O90_HEIGHTS.iter().enumerate()
            .map(|(id, h)| ExtItem {
                base: ExtBaseItem {
                    id: id as u64,
                    allowed_orientations: Some(vec![0.0, 90.0, 180.0, 270.0]),
                    shape: ExtShape::SimplePolygon(ExtSPolygon(vec![
                        (0.0, 0.0), (O90_WIDTH, 0.0), (O90_WIDTH, *h), (0.0, *h),
                    ])),
                    min_quality: None,
                },
                demand: O90_DEMAND as u64,
            })
            .collect();

        let ext = ExtBPInstance {
            name: "o90".to_string(),
            items,
            bins: vec![ExtBin {
                base: ExtContainer {
                    id: 0,
                    shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: O90_BIN_W, height: O90_BIN_H },
                    zones: vec![],
                },
                stock: 100,
                cost: 1,
            }],
        };

        let importer = Importer::new(
            config.cde_config,
            config.poly_simpl_tolerance,
            Some(O90_MIN_SEP),
            config.narrow_concavity_cutoff_ratio,
        );
        jagua_rs::probs::bpp::io::import_instance(&importer, &ext)
    }

    /// Loads the `swim` items into a BPP instance with `w` x `h` bins.
    fn build_swim_instance(w: f32, h: f32) -> Result<BPInstance> {
        let config = DEFAULT_SPARROW_CONFIG;
        let (sp_instance, _) = io::read_spp_input(Path::new("data/input/swim.json"))?;

        let ext = ExtBPInstance {
            name: sp_instance.name.clone(),
            items: sp_instance.items.iter()
                .map(|it| ExtItem { base: it.base.clone(), demand: it.demand })
                .collect(),
            bins: vec![ExtBin {
                base: ExtContainer {
                    id: 0,
                    shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: w, height: h },
                    zones: vec![],
                },
                stock: 100,
                cost: 1,
            }],
        };

        let importer = Importer::new(
            config.cde_config,
            config.poly_simpl_tolerance,
            config.min_item_separation,
            config.narrow_concavity_cutoff_ratio,
        );
        jagua_rs::probs::bpp::io::import_instance(&importer, &ext)
    }

    /// (a) On the o90 instance the shelf constructor is feasible, places everything, and needs at
    /// most 12 bins.
    ///
    /// **12, not 10.** With `--min-sep 5` every item is inflated to 605 x (h+5) and the bin
    /// deflated to 1985 x 990, and in that geometry 12 is a *proven* lower bound: the 997-tall
    /// piece cannot stand upright, so it must lie (997 x 605) and then no second "big" piece
    /// (>= 513 in one dimension) fits beside it — while no bin can hold more than three big pieces
    /// at all. 7 bins are therefore occupied by the 7 copies of item 0 plus at most one big piece
    /// each, leaving 14 big pieces at 3 per bin = 5 more bins. The 10-bin figure a naive shelf
    /// heuristic reports is computed on the *raw* 600 x h geometry, i.e. it silently drops the
    /// requested 5 mm separation.
    #[test]
    fn shelf_packs_o90_within_the_lower_bound() -> Result<()> {
        let instance = build_o90_instance()?;
        let builder = BPShelfBuilder::new(instance.clone()).construct()?;
        let prob = &builder.prob;

        println!("[TEST] shelf on o90: {} bins, cost {}, density {:.2}%",
            prob.layouts.len(), prob.bin_cost(), prob.density() * 100.0);

        assert!(prob.item_demand_qtys.iter().all(|&d| d == 0),
            "not all demand was placed: {:?}", prob.item_demand_qtys);
        assert_eq!(prob.n_placed_items(), instance.total_item_qty());
        assert!(prob.layouts.values().all(|l| l.is_feasible()),
            "the bbox packing must be collision-free");
        assert!(prob.layouts.len() <= 12,
            "expected at most 12 bins (the proven lower bound for the inflated geometry), got {}",
            prob.layouts.len());
        Ok(())
    }

    /// (b) The full pipeline on o90 is feasible and never worse than the shelf seed it starts from.
    #[test]
    fn optimize_bpp_on_o90_is_feasible_and_not_worse_than_the_shelf() -> Result<()> {
        let instance = build_o90_instance()?;
        let shelf_cost = BPShelfBuilder::new(instance.clone()).construct()?.prob.bin_cost();

        let mut config = DEFAULT_BPP_CONFIG;
        config.expl_cfg.time_limit = Duration::from_secs(10);
        config.cmpr_cfg.time_limit = Duration::from_secs(10);

        let sol = optimize_bpp(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(42),
            &mut DummyBPSolListener,
            &mut BasicTerminator::new(),
            &config,
            None,
        )?;

        let densities = sol.layout_snapshots.values()
            .map(|ls| format!("{:.1}%", ls.density(&instance) * 100.0))
            .collect::<Vec<_>>();
        println!("[TEST] optimize_bpp on o90: cost {} (shelf: {shelf_cost}), per-bin densities: {densities:?}",
            sol.cost(&instance));

        let n_placed: usize = sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
        assert_eq!(n_placed, instance.total_item_qty(), "all demand must be placed");
        for (lkey, ls) in sol.layout_snapshots.iter() {
            assert!(jagua_rs::entities::Layout::from_snapshot(ls).is_feasible(),
                "layout {lkey:?} of the final solution collides");
        }
        assert!(sol.cost(&instance) <= shelf_cost,
            "the pipeline ({}) must not be worse than its shelf seed ({shelf_cost})", sol.cost(&instance));
        Ok(())
    }

    /// (c) The bbox model must stay collision-free on **irregular** shapes too (it will simply be
    /// wasteful there, which is why `Constructive::Best` also builds the LBF solution).
    #[test]
    fn shelf_packs_swim_without_collisions() -> Result<()> {
        let instance = build_swim_instance(3200.0, 3200.0)?;
        let builder = BPShelfBuilder::new(instance.clone()).construct()?;
        let prob = &builder.prob;

        println!("[TEST] shelf on swim 3200x3200: {} bins, density {:.2}%",
            prob.layouts.len(), prob.density() * 100.0);

        assert!(prob.item_demand_qtys.iter().all(|&d| d == 0), "all demand must be placed");
        assert_eq!(prob.n_placed_items(), instance.total_item_qty());
        assert!(prob.layouts.values().all(|l| l.is_feasible()),
            "the bbox packing must be collision-free even for irregular contours");
        Ok(())
    }

    /// (d) The constructor is deterministic: no RNG is involved, so two builds are identical down
    /// to the individual transformations.
    #[test]
    fn shelf_is_deterministic() -> Result<()> {
        let instance = build_o90_instance()?;
        let a = BPShelfBuilder::new(instance.clone()).construct()?;
        let b = BPShelfBuilder::new(instance.clone()).construct()?;

        assert_eq!(a.prob.layouts.len(), b.prob.layouts.len(), "bin count must match");

        // Compare the placements layout by layout, in SlotMap order (which is itself deterministic).
        let placements = |p: &jagua_rs::probs::bpp::entities::BPProblem| {
            p.layouts.values()
                .map(|l| l.placed_items.values()
                    .map(|pi| (pi.item_id, pi.d_transf))
                    .collect::<Vec<_>>())
                .collect::<Vec<_>>()
        };
        assert_eq!(placements(&a.prob), placements(&b.prob),
            "two shelf builds must produce identical placements");
        Ok(())
    }
}
