//! Phase 2b integration tests: the BPP exploration/compression loops and the `optimize_bpp`
//! orchestration.
//!
//! Like [`bpp_tests`], the `swim` strip packing instance is reused as a source of items and the
//! bins are synthesised as 3200 x 3200 rectangles (~40 % of the total item area), which makes the
//! LBF constructor open 4 bins.

#[cfg(test)]
mod bpp_explore_integration_tests {
    use anyhow::Result;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::BPInstance;
    use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBin, ExtItem};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::{DEFAULT_BPP_CONFIG, DEFAULT_SPARROW_CONFIG};
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::bpp::explore::exploration_phase;
    use sparrow::optimizer::bpp::lbf::BPLBFBuilder;
    use sparrow::optimizer::bpp::separator::BPSeparator;
    use sparrow::optimizer::bpp::optimize_bpp;
    use sparrow::util::bpp_io::DummyBPSolListener;
    use sparrow::util::io;
    use sparrow::util::terminator::{BasicTerminator, Terminator};
    use std::path::Path;
    use std::time::Duration;

    const INSTANCE_PATH: &str = "data/input/swim.json";
    const BIN_W: f32 = 3200.0;
    const BIN_H: f32 = 3200.0;
    const SEED: u64 = 0;

    /// Loads the `swim` items and wraps them in a `BPInstance` with a single rectangular bin type.
    fn build_bp_instance() -> Result<BPInstance> {
        let config = DEFAULT_SPARROW_CONFIG;
        let (sp_instance, _) = io::read_spp_input(Path::new(INSTANCE_PATH))?;

        let ext_bp_instance = ExtBPInstance {
            name: sp_instance.name.clone(),
            items: sp_instance.items.iter()
                .map(|it| ExtItem { base: it.base.clone(), demand: it.demand })
                .collect(),
            bins: vec![ExtBin {
                base: ExtContainer {
                    id: 0,
                    shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: BIN_W, height: BIN_H },
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
        jagua_rs::probs::bpp::io::import_instance(&importer, &ext_bp_instance)
    }

    /// The full pipeline: 20 s exploration + 5 s compression must produce a feasible solution with
    /// all demand placed and a cost no worse than the LBF's.
    #[test]
    fn bpp_optimize_reduces_bin_count() -> Result<()> {
        let instance = build_bp_instance()?;

        // Reference: what does the LBF alone achieve?
        let lbf = BPLBFBuilder::new(instance.clone(), Xoshiro256PlusPlus::seed_from_u64(SEED), LBF_SAMPLE_CONFIG)
            .construct()?;
        let lbf_cost = lbf.prob.bin_cost();
        let lbf_bins = lbf.prob.layouts.len();
        println!("[TEST] LBF reference: {lbf_bins} bins, cost {lbf_cost}, dens {:.2}%", lbf.prob.density() * 100.0);
        drop(lbf);

        let mut config = DEFAULT_BPP_CONFIG;
        config.expl_cfg.time_limit = Duration::from_secs(20);
        config.cmpr_cfg.time_limit = Duration::from_secs(5);
        config.cmpr_cfg.consolidation_expl_cfg.time_limit = Duration::from_secs(5);

        let mut term = BasicTerminator::new();
        let mut listener = DummyBPSolListener;
        let sol = optimize_bpp(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(SEED),
            &mut listener,
            &mut term,
            &config,
            None,
        );

        let cost = sol.cost(&instance);
        println!("[TEST] optimize_bpp: cost {cost} (LBF: {lbf_cost}), {} bins, total dens {:.2}%",
            sol.layout_snapshots.len(), sol.density(&instance) * 100.0);
        for (lkey, ls) in sol.layout_snapshots.iter() {
            println!("[TEST]   bin {lkey:?}: dens {:.2}%, {} items", ls.density(&instance) * 100.0, ls.placed_items.len());
        }

        // 1. Feasible: every layout collision-free (checked by rebuilding the layouts).
        for (lkey, ls) in sol.layout_snapshots.iter() {
            let layout = jagua_rs::entities::Layout::from_snapshot(ls);
            assert!(layout.is_feasible(), "layout {lkey:?} of the final solution is not feasible");
        }

        // 2. All demand placed.
        let n_placed: usize = sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
        assert_eq!(n_placed, instance.total_item_qty(), "not all items were placed");

        // 3. Never worse than the starting point.
        assert!(cost <= lbf_cost, "the final cost {cost} is worse than the LBF cost {lbf_cost}");
        Ok(())
    }

    /// The exploration phase in isolation: every returned solution must be feasible and there must
    /// be at least one (the starting solution).
    #[test]
    fn bpp_exploration_phase_returns_feasible_solutions() -> Result<()> {
        let instance = build_bp_instance()?;
        let builder = BPLBFBuilder::new(instance.clone(), Xoshiro256PlusPlus::seed_from_u64(SEED), LBF_SAMPLE_CONFIG)
            .construct()?;
        let rng = builder.rng.clone();

        let mut config = DEFAULT_BPP_CONFIG.expl_cfg;
        config.time_limit = Duration::from_secs(10);

        let mut sep = BPSeparator::new(instance.clone(), builder.prob, rng, config.separator_config);
        let mut term = BasicTerminator::new();
        term.new_timeout(config.time_limit);
        let mut listener = DummyBPSolListener;

        let sols = exploration_phase(&instance, &mut sep, &mut listener, &term, &config);

        assert!(!sols.is_empty(), "the exploration phase must return at least one solution");
        println!("[TEST] exploration returned {} feasible solution(s)", sols.len());

        let mut prev_cost = u64::MAX;
        for (i, sol) in sols.iter().enumerate() {
            let cost = sol.cost(&instance);
            println!("[TEST]   #{i}: cost {cost}, {} bins, dens {:.2}%",
                sol.layout_snapshots.len(), sol.density(&instance) * 100.0);

            for (lkey, ls) in sol.layout_snapshots.iter() {
                let layout = jagua_rs::entities::Layout::from_snapshot(ls);
                assert!(layout.is_feasible(), "solution #{i}, layout {lkey:?} is not feasible");
            }
            let n_placed: usize = sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
            assert_eq!(n_placed, instance.total_item_qty(), "solution #{i} does not place all items");

            assert!(cost < prev_cost || i == 0, "solutions must strictly improve: {prev_cost} -> {cost}");
            prev_cost = cost;
        }
        Ok(())
    }
}
