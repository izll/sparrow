//! Phase 1 integration tests for the bin packing (BPP) pipeline.
//!
//! The `swim` strip packing instance is reused as a source of items; the bins are synthesised
//! here as plain rectangles (which is exactly what the planned `--bin WxH[:stock[:cost]]` CLI flag
//! will do). The bin is sized so that its area is roughly 40 % of the total item area, which forces
//! the LBF constructor to open at least three bins.

#[cfg(test)]
mod bpp_integration_tests {
    use anyhow::Result;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::BPInstance;
    use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBin, ExtItem};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::DEFAULT_SPARROW_CONFIG;
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::bpp::lbf::BPLBFBuilder;
    use sparrow::optimizer::bpp::separator::BPSeparator;
    use sparrow::util::io;
    use sparrow::util::terminator::{BasicTerminator, Terminator};
    use std::path::Path;
    use std::time::Duration;

    const INSTANCE_PATH: &str = "data/input/swim.json";
    /// Bin dimensions: 3200 x 3200 = 10.24M ~= 40 % of the total swim item area (~25.4M),
    /// so at least 3 bins are needed. The largest item is 1940 x 1577, so everything fits.
    const BIN_W: f32 = 3200.0;
    const BIN_H: f32 = 3200.0;
    const SEPARATE_TIMEOUT: Duration = Duration::from_secs(15);
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
        let instance = jagua_rs::probs::bpp::io::import_instance(&importer, &ext_bp_instance)?;
        Ok(instance)
    }

    /// (a) The LBF constructor produces a feasible solution with all demand placed, in >= 3 bins.
    #[test]
    fn bpp_lbf_constructs_feasible_solution() -> Result<()> {
        let instance = build_bp_instance()?;
        let rng = Xoshiro256PlusPlus::seed_from_u64(SEED);

        let builder = BPLBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG).construct()?;
        let prob = &builder.prob;

        println!("[TEST] LBF: {} bins, cost {}, density {:.2}%",
            prob.layouts.len(), prob.bin_cost(), prob.density() * 100.0);

        // All demand placed
        assert!(prob.item_demand_qtys.iter().all(|&d| d == 0), "not all demand was placed: {:?}", prob.item_demand_qtys);
        assert_eq!(prob.n_placed_items(), instance.total_item_qty());

        // Every layout is collision-free
        for (lkey, layout) in prob.layouts.iter() {
            assert!(layout.is_feasible(), "layout {lkey:?} is not feasible after LBF");
        }

        // The bin was sized so that at least 3 are needed
        assert!(prob.layouts.len() >= 3, "expected >= 3 bins, got {}", prob.layouts.len());
        Ok(())
    }

    /// (b) Closing the least dense bin and scattering its items creates overlap (loss > 0);
    ///     `separate()` must then reduce that loss.
    #[test]
    fn bpp_close_bin_and_separate_reduces_loss() -> Result<()> {
        let instance = build_bp_instance()?;
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(SEED);

        let builder = BPLBFBuilder::new(instance.clone(), rng.clone(), LBF_SAMPLE_CONFIG).construct()?;
        let n_bins_lbf = builder.prob.layouts.len();
        rng = builder.rng.clone();

        let mut sep = BPSeparator::new(
            instance.clone(),
            builder.prob,
            rng,
            DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config,
        );

        // Starting point must be feasible
        assert_eq!(sep.total_loss(), 0.0, "the LBF solution should have zero loss");

        // Close the least dense bin and scatter its content over the remaining ones
        let target = sep.least_dense_layout().expect("there should be at least one layout");
        let closed = sep.close_bin_and_scatter(target);
        assert!(closed, "close_bin_and_scatter should have closed a bin");
        assert_eq!(sep.prob.layouts.len(), n_bins_lbf - 1, "one bin should have been closed");

        let loss_before = sep.total_loss();
        println!("[TEST] after scatter: {} bins, loss = {loss_before}", sep.prob.layouts.len());
        assert!(loss_before > 0.0, "scattering items should introduce overlap");

        // Separate for a bounded amount of time
        let mut term = BasicTerminator::new();
        term.new_timeout(SEPARATE_TIMEOUT);
        let (sol, _cts) = sep.separate(&term);

        let loss_after = sep.total_loss();
        println!("[TEST] after separate: loss {loss_before} -> {loss_after} (reached 0: {})", loss_after == 0.0);

        assert!(loss_after < loss_before, "separate() should reduce the loss: {loss_before} -> {loss_after}");

        if loss_after == 0.0 {
            // Feasibility reached with one bin fewer: double-check with jagua-rs' own check
            sep.rollback(&sol, None);
            for (lkey, layout) in sep.prob.layouts.iter() {
                assert!(layout.is_feasible(), "layout {lkey:?} reports zero loss but is not feasible");
            }
            println!("[TEST] separation succeeded, {} bins remain (was {n_bins_lbf})", sep.prob.layouts.len());
        } else {
            println!("[TEST] separation did not reach feasibility within {SEPARATE_TIMEOUT:?} (allowed)");
        }
        Ok(())
    }

    /// A no-op guard: `close_bin_and_scatter` must refuse to close the only remaining bin.
    #[test]
    fn bpp_close_bin_is_noop_with_single_layout() -> Result<()> {
        let instance = build_bp_instance()?;
        let rng = Xoshiro256PlusPlus::seed_from_u64(SEED);
        let builder = BPLBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG).construct()?;

        let mut sep = BPSeparator::new(
            instance,
            builder.prob,
            builder.rng,
            DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config,
        );

        // Remove all layouts but one
        while sep.prob.layouts.len() > 1 {
            let lkey = sep.prob.layouts.keys().next().unwrap();
            sep.prob.remove_layout(lkey);
        }
        sep.rebuild_trackers();

        let lkey = sep.prob.layouts.keys().next().unwrap();
        assert!(!sep.close_bin_and_scatter(lkey), "closing the only bin must be a no-op");
        assert_eq!(sep.prob.layouts.len(), 1);
        Ok(())
    }
}
