//! Phase 4 integration tests: the cross-layout **pack-down** step and the area-based bound on the
//! bin-count reduction.
//!
//! As in [`bpp_explore_tests`], the `swim` strip packing instance supplies the items and the bins
//! are synthesised as rectangles, so no extra test data is needed.

#[cfg(test)]
mod bpp_packdown_integration_tests {
    use anyhow::Result;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtItem as ExtBaseItem, ExtSPolygon, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::{BPInstance, BPSolution};
    use jagua_rs::probs::bpp::io::ext_repr::{ExtBPInstance, ExtBin, ExtItem};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::{DEFAULT_BPP_CONFIG, DEFAULT_SPARROW_CONFIG, PackDownStrategy};
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::bpp::explore::{exploration_phase, required_density_for_reduction};
    use sparrow::optimizer::bpp::lbf::BPLBFBuilder;
    use sparrow::optimizer::bpp::optimize_bpp;
    use sparrow::optimizer::bpp::compress::pack_down;
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
        )?;

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

    // ---------------------------------------------------------------------------------------
    // Phase 6: pack-down over *all* source bins + per-bin consolidation, and the strategy switch
    // ---------------------------------------------------------------------------------------

    /// The `iso6` part set (the real customer instance that motivated phase 6): 7 rectangle types,
    /// all ~600 wide, into 1990 x 995 bins with `--min-sep 5`.
    ///
    /// `(width, height, demand)`. The 9 copies of 600 x 992 are the ones that force 9 bins: inflated
    /// to 997 x 605 they must lie down, and two of them do not fit side by side in 1985.
    const ISO6_PARTS: [(f32, f32, usize); 7] = [
        (600.0, 992.0, 9),
        (600.0, 540.0, 1),
        (600.0, 101.8, 7),
        (599.7, 95.8, 21),
        (600.0, 101.9, 7),
        (600.0, 892.0, 1),
        (600.0, 508.0, 3),
    ];
    const ISO6_MIN_SEP: f32 = 5.0;
    const ISO6_BIN: (f32, f32) = (1990.0, 995.0);
    /// 9 bins is a proven lower bound for this instance (see `docs/bpp.md`).
    const ISO6_OPTIMAL_COST: u64 = 9;

    /// Builds the iso6 instance in code: rectangles as explicit polygons, all four orientations
    /// allowed, one 1990 x 995 bin type, imported with `min_item_separation = 5`.
    fn build_iso6_instance() -> Result<BPInstance> {
        let config = DEFAULT_SPARROW_CONFIG;

        let items = ISO6_PARTS.iter().enumerate()
            .map(|(id, (w, h, demand))| ExtItem {
                base: ExtBaseItem {
                    id: id as u64,
                    allowed_orientations: Some(vec![0.0, 90.0, 180.0, 270.0]),
                    shape: ExtShape::SimplePolygon(ExtSPolygon(vec![
                        (0.0, 0.0), (*w, 0.0), (*w, *h), (0.0, *h),
                    ])),
                    min_quality: None,
                },
                demand: *demand as u64,
            })
            .collect();

        let ext = ExtBPInstance {
            name: "iso6".to_string(),
            items,
            bins: vec![ExtBin {
                base: ExtContainer {
                    id: 0,
                    shape: ExtShape::Rectangle {
                        x_min: 0.0, y_min: 0.0, width: ISO6_BIN.0, height: ISO6_BIN.1,
                    },
                    zones: vec![],
                },
                stock: 25,
                cost: 1,
            }],
        };

        let importer = Importer::new(
            config.cde_config,
            config.poly_simpl_tolerance,
            Some(ISO6_MIN_SEP),
            config.narrow_concavity_cutoff_ratio,
        );
        jagua_rs::probs::bpp::io::import_instance(&importer, &ext)
    }

    /// The phase-6 config used by both iso6 tests: 5 s exploration + 15 s compression, seed 42.
    fn iso6_config() -> sparrow::config::BPConfig {
        let mut config = DEFAULT_BPP_CONFIG;
        config.expl_cfg.time_limit = Duration::from_secs(5);
        config.cmpr_cfg.time_limit = Duration::from_secs(15);
        // Keep a single consolidation attempt short enough that every bin gets a turn.
        config.cmpr_cfg.consolidation_expl_cfg.time_limit = Duration::from_secs(5);
        config
    }

    /// Asserts the solution is feasible with the full demand placed, and returns its per-bin
    /// densities (ascending) and item counts.
    fn assert_feasible(sol: &BPSolution, instance: &BPInstance) {
        for (lkey, ls) in sol.layout_snapshots.iter() {
            let layout = jagua_rs::entities::Layout::from_snapshot(ls);
            assert!(layout.is_feasible(), "layout {lkey:?} of the final solution is not feasible");
        }
        let n_placed: usize = sol.layout_snapshots.values().map(|ls| ls.placed_items.len()).sum();
        assert_eq!(n_placed, instance.total_item_qty(), "not all items were placed");
    }

    /// How many bins hold more than one item.
    fn n_multi_item_bins(sol: &BPSolution) -> usize {
        sol.layout_snapshots.values().filter(|ls| ls.placed_items.len() > 1).count()
    }

    /// **The phase-6 headline test**, rewritten as a *deterministic* property.
    ///
    /// The property pack-down exists to guarantee is: **every open bin takes a turn as source.** On
    /// iso6 the old (phase-4) version took only the single least dense bin — a lonely 997 x 605
    /// piece that fits nowhere — tried one destination and returned in 0.0 s with 10 s of budget
    /// untouched, while the 45 % bins and the gap-riddled 67-79 % bins were never looked at.
    ///
    /// The previous version of this test asserted that instead by wall clock: "after
    /// `optimize_bpp`, at least one per-bin density differs from the exploration result", i.e. *a
    /// cross-bin move was accepted within the budget*. That is a different property, and a
    /// non-deterministic one — whether a given transfer's separation converges inside its per-move
    /// time limit depends on machine load. The second audit measured exactly that: FAILED with
    /// "no cross-bin move happened" in a clean release suite, PASS on an immediate isolated rerun,
    /// **with an identical density vector**. A release gate that flips on load is not a gate.
    ///
    /// So the assertion is now on [`PackDownStats::sources_visited`], which the loop records as it
    /// goes: every layout that was enumerated as a source, in visit order. Whether any particular
    /// transfer is accepted is left to the heuristic; that *every bin was tried* is not.
    #[test]
    #[cfg_attr(debug_assertions, ignore = "pack-down needs release speed (tracker_matches_layout debug asserts starve the time budget); run with `cargo test --release`")]
    fn pack_down_uses_every_bin_as_source_on_iso6() -> Result<()> {
        let instance = build_iso6_instance()?;
        let config = iso6_config();

        // Drive the exploration phase to the 9-bin state the compression phase starts from, then
        // run `pack_down` on it directly. Going through `optimize_bpp` would work too, but this way
        // the test observes the loop it is about rather than the whole pipeline around it.
        let builder = BPLBFBuilder::new(
            instance.clone(), Xoshiro256PlusPlus::seed_from_u64(42), LBF_SAMPLE_CONFIG,
        ).construct()?;
        let rng = builder.rng.clone();
        let mut sep = BPSeparator::new(
            instance.clone(), builder.prob, rng, config.expl_cfg.separator_config,
        );
        let mut term = BasicTerminator::new();
        term.new_timeout(config.expl_cfg.time_limit);
        let sols = exploration_phase(&instance, &mut sep, &mut DummyBPSolListener, &term, &config.expl_cfg);
        let expl_sol = sols.last().expect("at least one solution").clone();
        assert_eq!(expl_sol.cost(&instance), ISO6_OPTIMAL_COST,
            "test setup: the exploration phase must reach the proven 9-bin optimum");

        // The set of bins the pack-down is supposed to visit.
        let n_bins_before = sep.prob.layouts.len();
        assert_eq!(n_bins_before, ISO6_OPTIMAL_COST as usize,
            "test setup: the separator must hold the 9-bin solution");
        let bins_before: std::collections::BTreeSet<_> = sep.prob.layouts.keys().collect();

        let mut cmpr_sep = BPSeparator::new(
            sep.instance.clone(), sep.prob, Xoshiro256PlusPlus::seed_from_u64(43),
            config.cmpr_cfg.separator_config,
        );
        let mut pd_term = BasicTerminator::new();
        pd_term.new_timeout(config.cmpr_cfg.time_limit);
        let (sol, stats) = pack_down(
            &mut cmpr_sep, &pd_term, &config.cmpr_cfg, &mut DummyBPSolListener, &instance,
        );

        println!("[TEST] pack-down: {} pass(es), {} source visit(s) over {} distinct bin(s), {} move(s)",
            stats.n_passes, stats.sources_visited.len(), stats.distinct_sources().len(), stats.n_moved);

        // 1. **The property.** Every bin that existed when the pass started was taken as a source.
        //    A bin may be *closed* by an accepted move (it empties out), which is a success, not a
        //    miss — so the visited set has to cover every bin that survived, plus every bin that
        //    was closed, which together is exactly `bins_before`.
        let visited = stats.distinct_sources();
        let missed: Vec<_> = bins_before.difference(&visited).collect();
        assert!(missed.is_empty(),
            "pack-down must take EVERY open bin as a source; {} of {n_bins_before} never had a turn: {missed:?}",
            missed.len());

        // 2. It really did make a full pass, not a single source and out (the phase-4 behaviour).
        assert!(stats.n_passes >= 1, "at least one full pass must have run");
        assert!(stats.sources_visited.len() >= n_bins_before,
            "a full pass visits every source once, so at least {n_bins_before} visits were expected, \
             got {}", stats.sources_visited.len());

        // 3. The result is still a valid solution and did not cost bins.
        assert_feasible(&sol, &instance);
        assert!(sol.cost(&instance) <= ISO6_OPTIMAL_COST,
            "pack-down may close a bin, never open one");
        Ok(())
    }

    /// The **long-budget** variant of the above: the wall-clock-dependent half of the old test,
    /// kept because "a cross-bin move actually happens on iso6" is worth knowing — just not worth
    /// gating a release on. `#[ignore]`d, so it is opt-in:
    ///
    /// ```bash
    /// cargo test --release --test bpp_packdown_tests -- --ignored --nocapture
    /// ```
    #[test]
    #[ignore = "wall-clock dependent (the audit measured it flaky); run with `cargo test --release --test bpp_packdown_tests -- --ignored`"]
    fn pack_down_moves_items_across_bins_on_iso6_with_a_long_budget() -> Result<()> {
        let instance = build_iso6_instance()?;
        let mut config = iso6_config();
        // Three times the budget of the deterministic test, so the heuristic has room to land a
        // move even on a loaded machine.
        config.cmpr_cfg.time_limit = Duration::from_secs(45);

        let (expl_sol, expl_densities) = {
            let builder = BPLBFBuilder::new(
                instance.clone(), Xoshiro256PlusPlus::seed_from_u64(42), LBF_SAMPLE_CONFIG,
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
            let sol = sols.last().expect("at least one solution").clone();
            let d = bin_densities(&sol, &instance);
            println!("[TEST] after exploration: cost {}, {} multi-item bin(s), densities {:?}",
                sol.cost(&instance), n_multi_item_bins(&sol),
                d.iter().map(|x| format!("{:.1}%", x * 100.0)).collect::<Vec<_>>());
            (sol, d)
        };

        let start = Instant::now();
        let mut term = BasicTerminator::new();
        let sol = optimize_bpp(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(42),
            &mut DummyBPSolListener,
            &mut term,
            &config,
            None,
        )?;
        let elapsed = start.elapsed();

        let cost = sol.cost(&instance);
        let densities = bin_densities(&sol, &instance);
        println!("[TEST] optimize_bpp: cost {cost}, {} multi-item bin(s), densities {:?}, {:.1}s",
            n_multi_item_bins(&sol),
            densities.iter().map(|x| format!("{:.1}%", x * 100.0)).collect::<Vec<_>>(),
            elapsed.as_secs_f32());

        assert_feasible(&sol, &instance);
        assert_eq!(cost, ISO6_OPTIMAL_COST, "iso6 must stay at its proven optimum of 9 bins");

        // A cross-bin move is the *only* mechanism in the compression phase that changes any bin's
        // density (intra-layout consolidation relocates items strictly within one bin, and the
        // total density is fixed once the bin count is), so a changed density vector proves one.
        assert_eq!(densities.len(), expl_densities.len(), "the bin count must not have changed");
        let moved = densities.iter().zip(expl_densities.iter()).any(|(a, b)| (a - b).abs() > 1e-4);
        assert!(moved,
            "no cross-bin move happened: the per-bin densities are unchanged from the exploration \
             result ({expl_densities:?})");

        let range = |d: &[f32]| d.last().copied().unwrap_or(0.0) - d.first().copied().unwrap_or(0.0);
        let spread_sum = |d: &[f32]| -> f32 {
            let mean = d.iter().sum::<f32>() / d.len() as f32;
            d.iter().map(|x| (x - mean).abs()).sum()
        };
        println!("[TEST] density spread: {:.3} -> {:.3} (range {:.1} pp -> {:.1} pp)",
            spread_sum(&expl_densities), spread_sum(&densities),
            range(&expl_densities) * 100.0, range(&densities) * 100.0);
        assert!(spread_sum(&densities) > spread_sum(&expl_densities),
            "Concentrate must pull the per-bin densities apart: spread {:.4} -> {:.4}",
            spread_sum(&expl_densities), spread_sum(&densities));

        assert_eq!(expl_sol.cost(&instance), ISO6_OPTIMAL_COST);
        Ok(())
    }

    /// The `Spread` strategy is the mirror image of `Concentrate`: it evens the leftover out over
    /// all bins instead of concentrating it, so the **spread of the per-bin densities shrinks**.
    ///
    /// Asserting on the *minimum* bin density alone would be flaky on iso6: four of the nine bins
    /// hold a single 997 x 605 piece that fits into no other bin in any rotation, so the minimum is
    /// pinned at 30.1 % whatever the strategy does. What `Spread` provably changes is the
    /// **range** (max - min) — that is the property the strategy exists for.
    #[test]
    #[cfg_attr(debug_assertions, ignore = "pack-down needs release speed (tracker_matches_layout debug asserts starve the time budget); run with `cargo test --release`")]
    fn spread_evens_out_the_leftover() -> Result<()> {
        let instance = build_iso6_instance()?;

        let run = |strategy: PackDownStrategy| -> (u64, Vec<f32>) {
            let mut config = iso6_config();
            config.cmpr_cfg.pack_down_strategy = strategy;
            let mut term = BasicTerminator::new();
            let sol = optimize_bpp(
                instance.clone(),
                Xoshiro256PlusPlus::seed_from_u64(42),
                &mut DummyBPSolListener,
                &mut term,
                &config,
                None,
            ).expect("this instance is packable, so a start always exists");
            assert_feasible(&sol, &instance);
            (sol.cost(&instance), bin_densities(&sol, &instance))
        };

        let (conc_cost, conc) = run(PackDownStrategy::Concentrate);
        let (spread_cost, spread) = run(PackDownStrategy::Spread);

        let range = |d: &[f32]| d.last().copied().unwrap_or(0.0) - d.first().copied().unwrap_or(0.0);
        println!("[TEST] Concentrate: cost {conc_cost}, range {:.1} pp, densities {:?}",
            range(&conc) * 100.0, conc.iter().map(|x| format!("{:.1}%", x * 100.0)).collect::<Vec<_>>());
        println!("[TEST] Spread:      cost {spread_cost}, range {:.1} pp, densities {:?}",
            range(&spread) * 100.0, spread.iter().map(|x| format!("{:.1}%", x * 100.0)).collect::<Vec<_>>());

        // Neither strategy may cost bins.
        assert_eq!(conc_cost, ISO6_OPTIMAL_COST);
        assert_eq!(spread_cost, ISO6_OPTIMAL_COST);

        // `Spread` levels the bins out; `Concentrate` pulls them apart.
        assert!(range(&spread) < range(&conc),
            "Spread must even the per-bin densities out: range {:.2} pp vs Concentrate's {:.2} pp",
            range(&spread) * 100.0, range(&conc) * 100.0);
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
