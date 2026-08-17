//! Phase 4 integration tests: the cross-layout **pack-down** step and the area-based bound on the
//! bin-count reduction.
//!
//! As in [`bpp_explore_tests`], the `swim` strip packing instance supplies the items and the bins
//! are synthesised as rectangles, so no extra test data is needed.

#[cfg(test)]
mod bpp_packdown_integration_tests {
    use anyhow::Result;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::{BPInstance, BPSolution};
    use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBin, ExtItem};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::{DEFAULT_BPP_CONFIG, DEFAULT_SPARROW_CONFIG};
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::bpp::explore::{exploration_phase, required_density_for_reduction};
    use sparrow::optimizer::bpp::lbf::BPLBFBuilder;
    use sparrow::optimizer::bpp::optimize_bpp;
    use sparrow::optimizer::bpp::separator::BPSeparator;
    use sparrow::util::bpp_io::DummyBPSolListener;
    use sparrow::util::io;
    use sparrow::util::terminator::{BasicTerminator, Terminator};
    use std::path::Path;
    use std::time::{Duration, Instant};

    const INSTANCE_PATH: &str = "data/input/swim.json";
    const SEED: u64 = 0;

    /// Loads the `swim` items and wraps them in a `BPInstance` with a single `w` x `h` bin type.
    fn build_bp_instance(w: f32, h: f32) -> Result<BPInstance> {
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
        jagua_rs::probs::bpp::io::import_instance(&importer, &ext_bp_instance)
    }

    /// The per-bin densities of a solution, ascending.
    fn bin_densities(sol: &BPSolution, instance: &BPInstance) -> Vec<f32> {
        let mut d = sol.layout_snapshots.values()
            .map(|ls| ls.density(instance))
            .collect::<Vec<_>>();
        d.sort_by(|a, b| a.partial_cmp(b).unwrap());
        d
    }

    /// The full pipeline with the pack-down step: the leftover must end up concentrated in the
    /// sparsest bin (or a bin must have disappeared entirely).
    #[test]
    fn pack_down_empties_the_sparsest_bin() -> Result<()> {
        let instance = build_bp_instance(3200.0, 3200.0)?;

        let mut config = DEFAULT_BPP_CONFIG;
        config.expl_cfg.time_limit = Duration::from_secs(10);
        config.cmpr_cfg.time_limit = Duration::from_secs(25);
        config.cmpr_cfg.consolidation_expl_cfg.time_limit = Duration::from_secs(5);

        // Reference: the exploration phase alone (i.e. the state the compression phase starts from).
        let pre_min_density = {
            let builder = BPLBFBuilder::new(
                instance.clone(), Xoshiro256PlusPlus::seed_from_u64(SEED), LBF_SAMPLE_CONFIG,
            ).construct()?;
            let rng = builder.rng.clone();
            let mut sep = BPSeparator::new(
                instance.clone(), builder.prob, rng, config.expl_cfg.separator_config,
            );
            let mut term = BasicTerminator::new();
            term.new_timeout(config.expl_cfg.time_limit);
            let sols = exploration_phase(
                &instance, &mut sep, &mut DummyBPSolListener, &term, &config.expl_cfg,
            );
            let expl_sol = sols.last().expect("at least one solution").clone();
            let densities = bin_densities(&expl_sol, &instance);
            println!("[TEST] after exploration: cost {}, per-bin densities {:?}",
                expl_sol.cost(&instance),
                densities.iter().map(|d| format!("{:.2}%", d * 100.0)).collect::<Vec<_>>());
            densities[0]
        };

        // The real run.
        let mut term = BasicTerminator::new();
        let sol = optimize_bpp(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(SEED),
            &mut DummyBPSolListener,
            &mut term,
            &config,
            None,
        );

        let cost = sol.cost(&instance);
        let densities = bin_densities(&sol, &instance);
        println!("[TEST] optimize_bpp: cost {cost}, total dens {:.2}%, per-bin densities {:?}",
            sol.density(&instance) * 100.0,
            densities.iter().map(|d| format!("{:.2}%", d * 100.0)).collect::<Vec<_>>());
        println!("[TEST] min per-bin density: {:.2}% (pre-compression: {:.2}%)",
            densities[0] * 100.0, pre_min_density * 100.0);

        // 1. Feasible: every layout collision-free.
        for (lkey, ls) in sol.layout_snapshots.iter() {
            let layout = jagua_rs::entities::Layout::from_snapshot(ls);
            assert!(layout.is_feasible(), "layout {lkey:?} of the final solution is not feasible");
        }

        // 2. All demand placed.
        let n_placed: usize = sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
        assert_eq!(n_placed, instance.total_item_qty(), "not all items were placed");

        // 3. At most 4 bins.
        assert!(cost <= 4, "the final cost {cost} is worse than the expected 4 bins");

        // 4. Either a bin was eliminated, or the pack-down emptied the sparsest bin further.
        assert!(
            cost < 4 || densities[0] < pre_min_density,
            "pack-down neither reduced the bin count (cost {cost}) nor emptied the sparsest bin \
             ({:.3}% vs the pre-compression {:.3}%)",
            densities[0] * 100.0, pre_min_density * 100.0,
        );
        Ok(())
    }

    /// The area bound: with bins sized so that `n - 1` of them would need well over 90 % density,
    /// the exploration phase must return (almost) immediately instead of burning its budget.
    #[test]
    fn area_bound_stops_exploration_quickly() -> Result<()> {
        // The swim items total ~25.4 M area units. With 3200 x 7300 bins (23.4 M each) the LBF
        // opens exactly two, and squeezing them into one would require ~109 % density: not merely
        // above the 90 % cap but above 100 %, so the reduction is provably impossible.
        let instance = build_bp_instance(3200.0, 7300.0)?;

        let builder = BPLBFBuilder::new(
            instance.clone(), Xoshiro256PlusPlus::seed_from_u64(SEED), LBF_SAMPLE_CONFIG,
        ).construct()?;
        let rng = builder.rng.clone();

        let mut config = DEFAULT_BPP_CONFIG.expl_cfg;
        // A generous budget the phase must *not* consume.
        config.time_limit = Duration::from_secs(60);

        let mut sep = BPSeparator::new(
            instance.clone(), builder.prob, rng, config.separator_config,
        );
        let required = required_density_for_reduction(&sep);
        println!("[TEST] {} bins, reduction to {} would need {:.2}% density (cap {:.2}%)",
            sep.prob.layouts.len(), sep.prob.layouts.len() - 1,
            required * 100.0, config.max_reduction_density * 100.0);
        assert!(required > config.max_reduction_density,
            "test setup: the reduction must be above the cap, got {required:.4}");

        let mut term = BasicTerminator::new();
        term.new_timeout(config.time_limit);

        let start = Instant::now();
        let sols = exploration_phase(
            &instance, &mut sep, &mut DummyBPSolListener, &term, &config,
        );
        let elapsed = start.elapsed();
        println!("[TEST] exploration returned after {:.3}s with {} solution(s)",
            elapsed.as_secs_f32(), sols.len());

        assert!(elapsed < Duration::from_secs(3),
            "the area bound must make the exploration return immediately, took {elapsed:?}");
        assert_eq!(sols.len(), 1, "no reduction should have been attempted");
        Ok(())
    }

    /// `required_density_for_reduction` returns `INFINITY` when there is nothing to reduce.
    #[test]
    fn area_bound_is_infinite_for_a_single_bin() -> Result<()> {
        // One huge bin holds everything, so the LBF opens exactly one layout.
        let instance = build_bp_instance(20000.0, 20000.0)?;
        let builder = BPLBFBuilder::new(
            instance.clone(), Xoshiro256PlusPlus::seed_from_u64(SEED), LBF_SAMPLE_CONFIG,
        ).construct()?;
        let rng = builder.rng.clone();
        let sep = BPSeparator::new(
            instance.clone(), builder.prob, rng, DEFAULT_BPP_CONFIG.expl_cfg.separator_config,
        );
        assert_eq!(sep.prob.layouts.len(), 1, "test setup: a single bin was expected");
        assert!(required_density_for_reduction(&sep).is_infinite());
        Ok(())
    }
}
