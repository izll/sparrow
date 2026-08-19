//! Phase 7 integration tests: the multi-sheet ("walled") strip packing mode
//! ([`sparrow::optimizer::sheets`]).
//!
//! The mode inserts a wall (a quality-0 [`HazardEntity::Hole`]) at every sheet boundary of the
//! strip, so that no item can straddle a boundary and the strip can be cut into physical sheets
//! without relocating anything.
//!
//! Covered here:
//! * **walls exist**: after [`apply_sheet_walls`] the CDE reports the expected number of hole
//!   hazards, and an item placed across a boundary is detected as colliding;
//! * **full pipeline**: a real instance (`swim.json`) run end-to-end with walls stays feasible per
//!   jagua and has zero straddling items (the debug assertions in the tracker also run, since the
//!   test profile keeps them on);
//! * **sheet count**: a synthetic 3-rectangle instance whose optimum is obviously 2 sheets;
//! * **no regression**: with `sheet = None` the tracker has no hole entries at all.

#[cfg(test)]
mod sheet_integration_tests {
    use anyhow::Result;
    use jagua_rs::collision_detection::hazards::HazardEntity;
    use jagua_rs::collision_detection::hazards::collector::{BasicHazardCollector, HazardCollector};
    use jagua_rs::collision_detection::hazards::filter::NoFilter;
    use jagua_rs::entities::Instance;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::geometry::geo_traits::TransformableFrom;
    use jagua_rs::io::ext_repr::{ExtItem as ExtBaseItem, ExtSPolygon, ExtShape};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
    use jagua_rs::probs::spp::io::ext_repr::{ExtItem, ExtSPInstance};
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::{DEFAULT_SPARROW_CONFIG, SheetConfig};
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::compress::compression_phase;
    use sparrow::optimizer::explore::exploration_phase;
    use sparrow::optimizer::lbf::LBFBuilder;
    use sparrow::optimizer::separator::Separator;
    use sparrow::optimizer::sheets::{
        apply_sheet_walls, n_sheets, sheet_stats, wall_intervals,
    };
    use sparrow::quantify::tracker::{CollisionTracker, n_holes_of};
    use sparrow::util::io;
    use sparrow::util::listener::DummySolListener;
    use sparrow::util::terminator::{BasicTerminator, Terminator};
    use std::path::Path;
    use std::time::Duration;

    const INSTANCE_BASE_PATH: &str = "data/input";

    fn sheet(width: f32, gap: f32) -> SheetConfig {
        SheetConfig::new(width, gap, false)
    }

    fn import_spp(path: &str, min_sep: Option<f32>) -> Result<SPInstance> {
        let config = DEFAULT_SPARROW_CONFIG;
        let (ext, _) = io::read_spp_input(Path::new(&format!("{INSTANCE_BASE_PATH}/{path}")))?;
        let importer = Importer::new(
            config.cde_config,
            config.poly_simpl_tolerance,
            min_sep.or(config.min_item_separation),
            config.narrow_concavity_cutoff_ratio,
        );
        jagua_rs::probs::spp::io::import_instance(&importer, &ext)
    }

    /// Every item of `sol` must lie strictly on one sheet: its collision-shape bbox may not overlap
    /// any wall interval `[k(W+g) - g, k(W+g)]`. Returns the number of straddling items.
    fn count_straddling(sol: &SPSolution, sc: &SheetConfig) -> usize {
        let walls = wall_intervals(sol.strip_width(), sc);
        sol.layout_snapshot.placed_items.iter()
            .filter(|(_, pi)| {
                let b = pi.shape.bbox;
                walls.iter().any(|(x_min, x_max)| b.x_max > *x_min && b.x_min < *x_max)
            })
            .count()
    }

    /// (a) The walls are actually present as hole hazards, and they actually block placements.
    #[test]
    fn walls_exist_and_block_placements() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        // Widen the strip so that several sheets fit in it
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        // 10000mm at a 3020mm pitch is 4 sheets, separated by 3 *interior* walls
        // (at [3000,3020], [6020,6040], [9040,9060]); the 4th boundary lies past the strip end.
        assert_eq!(n_sheets(prob.strip_width(), &sc), 4, "sanity: 10000mm at a 3020mm pitch is 4 sheets");
        let walls = wall_intervals(prob.strip_width(), &sc);
        assert_eq!(walls.len(), 3, "only the walls inside the strip are materialised");
        assert_eq!(walls[0], (3000.0, 3020.0));
        let expected = walls.len();
        assert_eq!(n_holes_of(&prob.layout), expected, "every wall must be a Hole hazard");

        // The CDE must report exactly these holes
        let holes = prob.layout.cde().hazards_map.values()
            .filter(|h| matches!(h.entity, HazardEntity::Hole { .. }))
            .count();
        assert_eq!(holes, expected, "CDE must contain one Hole hazard per wall");

        // Place an item right across the first wall -> must collide
        let item = instance.item(0);
        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        let wall_center = (x_min + x_max) / 2.0;
        let mut buff = item.shape_cd.as_ref().clone();

        let straddling = DTransformation::new(0.0, (wall_center, instance.base_strip.fixed_height / 2.0));
        buff.transform_from(&item.shape_cd, &straddling.compose());
        assert!(
            prob.layout.cde().detect_poly_collision(&buff, &NoFilter),
            "an item centred on a wall must collide with it"
        );

        // The same item well inside a sheet must not collide
        let clear = DTransformation::new(0.0, (x_min / 2.0, instance.base_strip.fixed_height / 2.0));
        buff.transform_from(&item.shape_cd, &clear.compose());
        assert!(
            !prob.layout.cde().detect_poly_collision(&buff, &NoFilter),
            "an item in the middle of an empty sheet must not collide"
        );
        Ok(())
    }

    /// The tracker must see the wall collision as a *hole* loss (not panic on the hazard, and not
    /// silently ignore it).
    #[test]
    fn tracker_accounts_for_hole_losses() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        let wall_center = (x_min + x_max) / 2.0;

        // Deliberately place an item on top of the first wall
        let pk = prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, (wall_center, instance.base_strip.fixed_height / 2.0)),
        });

        let ct = CollisionTracker::new(&prob.layout);
        assert_eq!(ct.n_holes, wall_intervals(prob.strip_width(), &sc).len());
        assert!(ct.get_hole_loss(pk, 0) > 0.0, "the item straddling wall 0 must have a hole loss");
        assert!(ct.get_loss(pk) > 0.0, "the total loss must include the hole loss");
        assert!(ct.get_total_loss() > 0.0);
        Ok(())
    }

    /// With no sheet configuration the tracker must be exactly as before: no hole entries at all.
    #[test]
    fn no_sheet_means_no_holes() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let mut prob = SPProblem::new(instance.clone());
        prob.place_item(SPPlacement { item_id: 0, d_transf: DTransformation::new(0.0, (500.0, 500.0)) });

        let ct = CollisionTracker::new(&prob.layout);
        assert_eq!(n_holes_of(&prob.layout), 0);
        assert_eq!(ct.n_holes, 0);
        assert!(ct.hole_collisions.is_empty(), "no allocation for holes in plain strip mode");
        Ok(())
    }

    /// (b) Full pipeline with walls: feasible per jagua and zero straddling items.
    #[test]
    fn full_pipeline_keeps_items_off_the_walls() -> Result<()> {
        let config = {
            let mut c = DEFAULT_SPARROW_CONFIG;
            c.apply_sheet(Some(sheet(3000.0, 20.0)));
            c
        };
        let sc = config.sheet.unwrap();
        let instance = import_spp("swim.json", None)?;
        let rng = Xoshiro256PlusPlus::seed_from_u64(0);

        let mut terminator = BasicTerminator::new();
        let mut listener = DummySolListener;

        terminator.new_timeout(Duration::from_secs(15));
        let builder = LBFBuilder::new_with_sheet(instance.clone(), rng, LBF_SAMPLE_CONFIG, config.sheet)
            .construct();
        assert_eq!(count_straddling(&builder.prob.save(), &sc), 0, "LBF must not straddle walls");

        let mut sep = Separator::new_with_sheet(
            builder.instance, builder.prob, builder.rng,
            config.expl_cfg.separator_config, config.sheet,
        );
        let sols = exploration_phase(&instance, &mut sep, &mut listener, &terminator, &config.expl_cfg);
        let last = sols.last().expect("exploration must yield a solution");

        terminator.new_timeout(Duration::from_secs(5));
        let final_sol = compression_phase(&instance, &mut sep, last, &mut listener, &terminator, &config.cmpr_cfg);

        // The final layout must be feasible according to jagua itself...
        let layout = jagua_rs::entities::Layout::from_snapshot(&final_sol.layout_snapshot);
        assert!(layout.is_feasible(), "the final walled solution must be feasible");
        assert_eq!(n_holes_of(&layout), wall_intervals(final_sol.strip_width(), &sc).len(),
            "the walls must survive the whole pipeline (save/restore round-trips)");
        assert!(n_holes_of(&layout) + 1 >= n_sheets(final_sol.strip_width(), &sc),
            "there must be a wall between every pair of consecutive sheets");

        // ...and no item may straddle a sheet boundary
        assert_eq!(count_straddling(&final_sol, &sc), 0, "no item may straddle a wall");

        // The per-sheet metrics must be self-consistent
        let stats = sheet_stats(&final_sol, &instance, &sc);
        assert_eq!(stats.len(), n_sheets(final_sol.strip_width(), &sc));
        assert_eq!(stats.iter().map(|s| s.n_items).sum::<usize>(), final_sol.layout_snapshot.placed_items.len());
        for s in &stats {
            assert!(s.used_width <= sc.width + 1e-3, "sheet {} overflows its width", s.index);
            assert!(s.leftover_band_width() >= 0.0);
            assert!(s.density() <= 1.0);
        }
        Ok(())
    }

    /// (c) A synthetic instance whose optimum is obviously 2 sheets: three 900 x 900 squares in
    /// 1000-high sheets of width 2000. Two fit side by side on one sheet (1800 <= 2000), the third
    /// needs a second sheet — and three sheets are never necessary.
    #[test]
    fn synthetic_three_rectangles_fit_in_two_sheets() -> Result<()> {
        const SIDE: f32 = 900.0;
        let ext = ExtSPInstance {
            name: "sheet_synth".to_string(),
            strip_height: 1000.0,
            items: vec![ExtItem {
                base: ExtBaseItem {
                    id: 0,
                    allowed_orientations: Some(vec![0.0, 90.0, 180.0, 270.0]),
                    shape: ExtShape::SimplePolygon(ExtSPolygon(vec![
                        (0.0, 0.0), (SIDE, 0.0), (SIDE, SIDE), (0.0, SIDE),
                    ])),
                    min_quality: None,
                },
                demand: 3,
            }],
        };
        let config = {
            let mut c = DEFAULT_SPARROW_CONFIG;
            c.apply_sheet(Some(sheet(2000.0, 20.0)));
            c
        };
        let sc = config.sheet.unwrap();
        let importer = Importer::new(
            config.cde_config, config.poly_simpl_tolerance,
            config.min_item_separation, config.narrow_concavity_cutoff_ratio,
        );
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext)?;

        let mut terminator = BasicTerminator::new();
        let mut listener = DummySolListener;
        terminator.new_timeout(Duration::from_secs(10));

        let builder = LBFBuilder::new_with_sheet(
            instance.clone(), Xoshiro256PlusPlus::seed_from_u64(0), LBF_SAMPLE_CONFIG, config.sheet,
        ).construct();
        let mut sep = Separator::new_with_sheet(
            builder.instance, builder.prob, builder.rng,
            config.expl_cfg.separator_config, config.sheet,
        );
        let sols = exploration_phase(&instance, &mut sep, &mut listener, &terminator, &config.expl_cfg);
        let final_sol = sols.last().expect("exploration must yield a solution").clone();

        let sheets = n_sheets(final_sol.strip_width(), &sc);
        assert!(sheets <= 2, "three 900x900 squares must fit in 2 sheets of 2000x1000, got {sheets}");
        assert_eq!(count_straddling(&final_sol, &sc), 0, "no item may straddle a wall");
        let layout = jagua_rs::entities::Layout::from_snapshot(&final_sol.layout_snapshot);
        assert!(layout.is_feasible());
        Ok(())
    }

    /// (d) **Phase 8 sheet-drop.** A synthetic instance whose LBF start needs three sheets while
    /// two are comfortably enough: 8 rectangles of 900 x 450 in 2000 x 1000 sheets. Two sheets hold
    /// 4,000,000 mm2 and the parts total 3,240,000 mm2, i.e. 81 % — but laid out as 2 columns x 2
    /// rows per sheet the fit is exact and obvious, so the walled optimizer must get to 2 sheets.
    ///
    /// This is the regression test for the cross-sheet relocation operator: without it the run gets
    /// stuck at whatever sheet count the fine shrink happens to land on.
    #[test]
    fn sheet_drop_reaches_two_sheets() -> Result<()> {
        const W: f32 = 900.0;
        const H: f32 = 450.0;
        let ext = ExtSPInstance {
            name: "sheet_drop_synth".to_string(),
            strip_height: 1000.0,
            items: vec![ExtItem {
                base: ExtBaseItem {
                    id: 0,
                    allowed_orientations: Some(vec![0.0, 90.0, 180.0, 270.0]),
                    shape: ExtShape::SimplePolygon(ExtSPolygon(vec![
                        (0.0, 0.0), (W, 0.0), (W, H), (0.0, H),
                    ])),
                    min_quality: None,
                },
                demand: 8,
            }],
        };
        let config = {
            let mut c = DEFAULT_SPARROW_CONFIG;
            c.apply_sheet(Some(sheet(2000.0, 20.0)));
            c.expl_cfg.time_limit = Duration::from_secs(20);
            c.cmpr_cfg.time_limit = Duration::from_secs(10);
            c
        };
        let sc = config.sheet.unwrap();
        let importer = Importer::new(
            config.cde_config, config.poly_simpl_tolerance,
            config.min_item_separation, config.narrow_concavity_cutoff_ratio,
        );
        let instance = jagua_rs::probs::spp::io::import_instance(&importer, &ext)?;

        let mut listener = DummySolListener;
        let mut terminator = BasicTerminator::new();
        let final_sol = sparrow::optimizer::optimize(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(0),
            &mut listener,
            &mut terminator,
            &config.expl_cfg,
            &config.cmpr_cfg,
            None,
        );

        let sheets = n_sheets(final_sol.strip_width(), &sc);
        assert!(sheets <= 2, "8 parts of 900x450 must fit in 2 sheets of 2000x1000, got {sheets}");
        assert_eq!(count_straddling(&final_sol, &sc), 0, "no item may straddle a wall");
        let layout = jagua_rs::entities::Layout::from_snapshot(&final_sol.layout_snapshot);
        assert!(layout.is_feasible(), "the final walled solution must be feasible");
        Ok(())
    }

    /// The wall geometry helpers must agree with the documented coordinate mapping.
    #[test]
    fn wall_geometry_matches_the_coordinate_mapping() {
        let sc = sheet(2000.0, 20.0);
        assert_eq!(sc.pitch(), 2020.0);

        // A strip shorter than one sheet has no wall inside it
        assert_eq!(n_sheets(1500.0, &sc), 1);
        assert!(wall_intervals(1500.0, &sc).is_empty());

        // Just past the first sheet: 2 sheets, one wall at [2000, 2020]
        assert_eq!(n_sheets(2500.0, &sc), 2);
        let w = wall_intervals(2500.0, &sc);
        assert_eq!(w.len(), 1);
        assert_eq!(w[0], (2000.0, 2020.0));

        // Exactly one full sheet: still 1 sheet, no wall
        assert_eq!(n_sheets(2000.0, &sc), 1);
        assert!(wall_intervals(2000.0, &sc).is_empty());

        // x -> (k, x_local)
        for (x, k, x_local) in [(0.0, 0, 0.0), (1999.0, 0, 1999.0), (2020.0, 1, 0.0), (3000.0, 1, 980.0)] {
            assert_eq!((x / sc.pitch()).floor() as usize, k, "sheet index of x={x}");
            assert!((x - k as f32 * sc.pitch() - x_local).abs() < 1e-3, "local x of x={x}");
        }
    }

    /// A save/restore round-trip at the same width must keep the walls (the container id trick).
    #[test]
    fn walls_survive_save_restore() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);
        let expected = n_holes_of(&prob.layout);
        assert!(expected > 0);

        let snapshot = prob.save();
        // The snapshot must carry the walled container
        assert_eq!(
            snapshot.layout_snapshot.container.quality_zones[0].as_ref().map_or(0, |z| z.shapes_cd.len()),
            expected,
        );

        // Same-width restore keeps the walls (cheap Layout::restore path)
        prob.restore(&snapshot);
        assert_eq!(n_holes_of(&prob.layout), expected, "same-width restore must keep the walls");

        // Different-width restore rebuilds from the snapshot, which also carries them
        let mut other = SPProblem::new(instance.clone());
        other.change_strip_width(5000.0);
        apply_sheet_walls(&mut other, &sc);
        other.restore(&snapshot);
        assert_eq!(n_holes_of(&other.layout), expected, "cross-width restore must keep the walls");
        Ok(())
    }

    /// The GLS weights of hole entries must be updated like every other entry, and stay clamped.
    #[test]
    fn hole_weights_participate_in_gls() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        let pk = prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, ((x_min + x_max) / 2.0, instance.base_strip.fixed_height / 2.0)),
        });

        let mut ct = CollisionTracker::new(&prob.layout);
        let w_before = ct.get_hole_weight(pk, 0);
        ct.update_weights();
        let w_after = ct.get_hole_weight(pk, 0);
        assert!(w_after > w_before, "a colliding hole entry's weight must grow: {w_before} -> {w_after}");

        // ...and the weighted loss must reflect the weight
        assert!(ct.get_weighted_loss(pk) >= ct.get_loss(pk) * w_before);

        // A large number of updates must stay finite (GLS_WEIGHT_MAX clamp)
        for _ in 0..500 {
            ct.update_weights();
        }
        assert!(ct.get_hole_weight(pk, 0).is_finite(), "hole weights must stay clamped/finite");
        assert!(ct.get_total_weighted_loss().is_finite());
        Ok(())
    }

    /// Sanity of the collision collector on a walled layout: the hazards an item collides with must
    /// include the wall, and the specialized separator pipeline must agree with jagua's own.
    #[test]
    fn collector_reports_hole_hazards() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        let pk = prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, ((x_min + x_max) / 2.0, instance.base_strip.fixed_height / 2.0)),
        });

        let pi = &prob.layout.placed_items[pk];
        let mut collector = BasicHazardCollector::new();
        prob.layout.cde().collect_poly_collisions(&pi.shape, &mut collector);
        collector.remove_by_entity(&HazardEntity::from((pk, pi)));

        assert!(
            collector.iter().any(|(_, he)| matches!(he, HazardEntity::Hole { idx: 0 })),
            "the collector must report the wall as a Hole hazard"
        );
        Ok(())
    }

    // =============================================================================================
    // Regression tests for the review findings (see docs/sheets.md).
    //
    // All of these assert on *returned data* (solutions, stats, counts) rather than relying on
    // debug assertions, so they are meaningful under `cargo test --release` as well — which is
    // where the bugs they cover actually bit, debug_asserts being compiled out there.
    // =============================================================================================

    /// CRITICAL 1 — a **walled warm start** must never hand an infeasible layout to the
    /// exploration phase (and hence to the exported JSON).
    ///
    /// A wall-less solution is optimized first, then re-imported as the warm start of a *walled*
    /// run whose sheets are narrow enough that items are guaranteed to straddle the new walls.
    /// Before the fix the straddling layout was seeded into `exploration_phase` as "feasible" and
    /// came straight back out; now it is repaired first (or the warm start is discarded for a
    /// walled LBF), so the returned solution is feasible and clear of every wall.
    #[test]
    fn walled_warm_start_never_returns_an_infeasible_layout() -> Result<()> {
        let instance = import_spp("swim.json", None)?;

        // 1. A plain (wall-less) solution: the warm start.
        let mut terminator = BasicTerminator::new();
        let mut listener = DummySolListener;
        let plain_cfg = DEFAULT_SPARROW_CONFIG;
        terminator.new_timeout(Duration::from_secs(6));
        let builder = LBFBuilder::new(instance.clone(), Xoshiro256PlusPlus::seed_from_u64(0), LBF_SAMPLE_CONFIG)
            .construct();
        let mut plain_sep = Separator::new(
            builder.instance, builder.prob, builder.rng, plain_cfg.expl_cfg.separator_config,
        );
        let plain_sols = exploration_phase(&instance, &mut plain_sep, &mut listener, &terminator, &plain_cfg.expl_cfg);
        let warm_start = plain_sols.last().expect("the plain run must yield a solution").clone();
        assert!(jagua_rs::entities::Layout::from_snapshot(&warm_start.layout_snapshot).is_feasible(),
            "sanity: the wall-less warm start itself is feasible");

        // 2. Feed it into a walled run whose sheets are narrow enough that the wall-less layout
        //    certainly has items sitting on a boundary, but wide enough that every part still fits
        //    on one sheet, so a correct answer exists.
        let sc = sheet(2000.0, 20.0);
        assert!(sparrow::optimizer::sheets::items_too_wide_for_sheet(&instance, &sc).is_empty(),
            "sanity: at 2000 mm every part fits a sheet, so the instance is solvable");
        // The warm start really does straddle the walls of the sheet config it is about to be fed
        // into — otherwise this test would be vacuous.
        assert!(count_straddling(&warm_start, &sc) > 0,
            "sanity: the wall-less warm start must straddle the walls it is imported against");

        let mut walled_cfg = DEFAULT_SPARROW_CONFIG;
        walled_cfg.apply_sheet(Some(sc));
        walled_cfg.expl_cfg.time_limit = Duration::from_secs(10);
        walled_cfg.cmpr_cfg.time_limit = Duration::from_secs(4);

        // 2a. The core guarantee, asserted directly and deterministically: the separator that
        //     `optimize` hands to `exploration_phase` for a walled warm start must be **feasible**.
        //     This is the invariant `exploration_phase` relies on when it seeds its feasible-solution
        //     list with the start, and the one the missing repair used to break — in release, where
        //     the debug assertion that would have caught it is compiled out.
        {
            use sparrow::optimizer::sheets::{apply_sheet_walls_opt, widen_for_walls};

            let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone());
            apply_sheet_walls_opt(&mut prob, Some(&sc));
            prob.restore(&warm_start);
            apply_sheet_walls_opt(&mut prob, Some(&sc));
            widen_for_walls(&mut prob, &sc);

            let raw = Separator::new_with_sheet(
                instance.clone(), prob, Xoshiro256PlusPlus::seed_from_u64(3),
                walled_cfg.expl_cfg.separator_config, Some(sc),
            );
            // Without a repair this is exactly what used to reach `exploration_phase`.
            assert!(raw.ct.get_total_loss() > 0.0,
                "sanity: the un-repaired walled warm start really is infeasible");

            let mut next_rng = {
                let mut r = Xoshiro256PlusPlus::seed_from_u64(3);
                move || {
                    use rand::RngExt;
                    Xoshiro256PlusPlus::seed_from_u64(r.random())
                }
            };
            let repaired = sparrow::optimizer::repair_walled_warm_start(
                &instance, raw, &mut next_rng, &mut listener, &walled_cfg.expl_cfg, sc,
            );
            assert_eq!(repaired.ct.get_total_loss(), 0.0,
                "the separator handed to exploration_phase must be feasible: either the warm start \
                 was repaired, or it was replaced by a walled LBF construction");
            assert_eq!(count_straddling(&repaired.prob.save(), &sc), 0,
                "the repaired start must be clear of every wall");
            assert_eq!(repaired.prob.layout.placed_items.len(), instance.total_item_qty(),
                "the repair (or its LBF fallback) must still place all demand");
        }

        // 2b. And end-to-end: the solution `optimize` actually returns (and exports).
        let mut term = BasicTerminator::new();
        term.new_timeout(Duration::from_secs(30));
        let final_sol = sparrow::optimizer::optimize(
            instance.clone(),
            Xoshiro256PlusPlus::seed_from_u64(3),
            &mut listener,
            &mut term,
            &walled_cfg.expl_cfg,
            &walled_cfg.cmpr_cfg,
            Some(&warm_start),
        );

        // The returned solution — the one that gets exported — must be genuinely feasible...
        let layout = jagua_rs::entities::Layout::from_snapshot(&final_sol.layout_snapshot);
        assert!(layout.is_feasible(),
            "a walled warm start must never yield an infeasible layout (this is what got exported before the fix)");
        // ...and no item may sit on a wall.
        assert_eq!(count_straddling(&final_sol, &sc), 0,
            "no item may straddle a sheet wall after a walled warm start");
        // All demand must still be placed.
        assert_eq!(final_sol.layout_snapshot.placed_items.len(), instance.total_item_qty(),
            "the warm start must not lose items");
        Ok(())
    }

    /// CRITICAL 1 (unit) — `exploration_phase` must not record an infeasible start as a feasible
    /// solution, in **release** semantics (where the debug assertion that used to "cover" this is
    /// compiled out).
    ///
    /// An item is deliberately parked on a wall, so the start has a non-zero loss. The phase must
    /// either separate its way to a genuinely feasible layout or return none at all — what it must
    /// never do is hand back the overlapping start.
    ///
    /// **Release-only.** Feeding an infeasible start is precisely what the `debug_assert!` in
    /// `exploration_phase` forbids, so under the test profile (debug assertions ON) this test would
    /// trip that assertion before reaching the behaviour it checks. The bug it guards against is a
    /// release-mode bug — debug_asserts are compiled out there — so the test runs where the bug
    /// lives: `cargo test --release`.
    #[test]
    #[cfg(not(debug_assertions))]
    fn exploration_does_not_seed_an_infeasible_start_as_feasible() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        // Park one item squarely on the first wall: the start is now infeasible by construction.
        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, ((x_min + x_max) / 2.0, instance.base_strip.fixed_height / 2.0)),
        });

        let mut config = DEFAULT_SPARROW_CONFIG;
        config.apply_sheet(Some(sc));
        config.expl_cfg.time_limit = Duration::from_secs(3);

        let mut sep = Separator::new_with_sheet(
            instance.clone(), prob, Xoshiro256PlusPlus::seed_from_u64(0),
            config.expl_cfg.separator_config, config.sheet,
        );
        assert!(sep.ct.get_total_loss() > 0.0, "sanity: the start really is infeasible");

        let mut terminator = BasicTerminator::new();
        let mut listener = DummySolListener;
        terminator.new_timeout(Duration::from_secs(3));
        let sols = exploration_phase(&instance, &mut sep, &mut listener, &terminator, &config.expl_cfg);

        // Whatever comes back must be feasible — the infeasible start must not be among it.
        for sol in &sols {
            let layout = jagua_rs::entities::Layout::from_snapshot(&sol.layout_snapshot);
            assert!(layout.is_feasible(),
                "exploration_phase returned a solution that is not collision-free (width {})",
                sol.strip_width());
            assert_eq!(count_straddling(sol, &sc), 0, "a returned solution may not straddle a wall");
        }
        Ok(())
    }

    /// CRITICAL 2 — `--compact-sheets` must run to completion.
    ///
    /// `try_shift`'s undo path minted a fresh `PItemKey` and threw it away, so the binary search's
    /// next probe reused a dangling key and the run died with "invalid SlotMap key used". The
    /// compaction is exercised here directly on a multi-sheet layout: it must not panic, the result
    /// must stay feasible and wall-clear, `n_moved` must be truthful, and no item may end up
    /// further **right** than it started (the pass only ever moves items left).
    #[test]
    fn compact_sheets_completes_and_only_moves_items_left() -> Result<()> {
        use sparrow::optimizer::sheets::compact_sheets_left;
        use std::collections::HashMap;

        let instance = import_spp("swim.json", None)?;
        let sc = sheet(2500.0, 20.0);
        let mut config = DEFAULT_SPARROW_CONFIG;
        config.apply_sheet(Some(sc));

        let mut terminator = BasicTerminator::new();
        let mut listener = DummySolListener;
        terminator.new_timeout(Duration::from_secs(12));

        let builder = LBFBuilder::new_with_sheet(
            instance.clone(), Xoshiro256PlusPlus::seed_from_u64(0), LBF_SAMPLE_CONFIG, config.sheet,
        ).construct();
        let mut sep = Separator::new_with_sheet(
            builder.instance, builder.prob, builder.rng,
            config.expl_cfg.separator_config, config.sheet,
        );
        let sols = exploration_phase(&instance, &mut sep, &mut listener, &terminator, &config.expl_cfg);
        let before = sols.last().expect("exploration must yield a solution").clone();
        assert!(n_sheets(before.strip_width(), &sc) >= 2,
            "sanity: the compaction only does anything with at least 2 sheets");

        // The x_min of every item before the pass, keyed by PItemKey (stable across the rollback
        // `compact_sheets_left` performs first).
        let x_before: HashMap<_, _> = before.layout_snapshot.placed_items.iter()
            .map(|(pk, pi)| (pk, pi.shape.bbox.x_min))
            .collect();

        // THE call that used to panic with "invalid SlotMap key used".
        let (after, n_moved) = compact_sheets_left(&mut sep, &sc, &before, &BasicTerminator::new());

        // Feasible, wall-clear, and nothing lost.
        let layout = jagua_rs::entities::Layout::from_snapshot(&after.layout_snapshot);
        assert!(layout.is_feasible(), "the compacted layout must be collision-free");
        assert_eq!(count_straddling(&after, &sc), 0, "compaction may not push an item onto a wall");
        assert_eq!(after.layout_snapshot.placed_items.len(), before.layout_snapshot.placed_items.len(),
            "compaction may not lose or duplicate items");
        assert_eq!(n_sheets(after.strip_width(), &sc), n_sheets(before.strip_width(), &sc),
            "compaction may not change the sheet count");

        // `n_moved` must be truthful: it may not exceed the number of items, and if it claims
        // movement then the layout must genuinely differ.
        assert!(n_moved <= after.layout_snapshot.placed_items.len(), "n_moved is out of range");

        // No item may have moved RIGHT. Items are matched by (item_id, y) since keys are re-minted.
        let mut n_actually_moved = 0usize;
        let after_by_item: Vec<_> = after.layout_snapshot.placed_items.iter()
            .map(|(_, pi)| (pi.item_id, pi.shape.bbox.x_min, pi.shape.bbox.y_min))
            .collect();
        for (_, pi) in before.layout_snapshot.placed_items.iter() {
            // The counterpart is the item of the same id at (essentially) the same y.
            let found = after_by_item.iter()
                .filter(|(id, _, y)| *id == pi.item_id && (y - pi.shape.bbox.y_min).abs() < 1e-3)
                .map(|(_, x, _)| *x)
                .collect::<Vec<_>>();
            if let Some(&x_after) = found.iter().min_by(|a, b| a.partial_cmp(b).unwrap()) {
                assert!(x_after <= pi.shape.bbox.x_min + 1e-2,
                    "item {} moved RIGHT ({} -> {}); the compaction only shifts items left",
                    pi.item_id, pi.shape.bbox.x_min, x_after);
                if x_after < pi.shape.bbox.x_min - 1e-2 {
                    n_actually_moved += 1;
                }
            }
        }
        // If the pass reported movement, at least one item must really have moved — the old code
        // could report `n_moved > 0` while the final probe had silently restored everything.
        if n_moved > 0 {
            assert!(n_actually_moved > 0,
                "n_moved = {n_moved} but no item actually changed position (the accepted shift was not re-applied)");
        }
        let _ = x_before;
        Ok(())
    }

    /// MEDIUM — `sheet_stats` must derive the sheet from the item's **right** edge and report
    /// straddling items explicitly instead of clamping them into a sheet silently.
    #[test]
    fn sheet_stats_reports_straddling_items() -> Result<()> {
        let instance = import_spp("swim.json", None)?;
        let sc = sheet(3000.0, 20.0);

        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(10_000.0);
        apply_sheet_walls(&mut prob, &sc);

        // One item entirely on sheet 0, one parked across the first wall.
        prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, (1200.0, instance.base_strip.fixed_height / 2.0)),
        });
        let (x_min, x_max) = wall_intervals(prob.strip_width(), &sc)[0];
        prob.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, ((x_min + x_max) / 2.0, instance.base_strip.fixed_height / 2.0)),
        });

        let sol = prob.save();
        let stats = sheet_stats(&sol, &instance, &sc);
        let n_straddling: usize = stats.iter().map(|s| s.straddling_item_ids.len()).sum();
        assert_eq!(n_straddling, 1,
            "the item sitting on wall 0 must be reported as straddling, not clamped into a sheet");
        assert_eq!(stats.iter().map(|s| s.n_items).sum::<usize>(), 2, "every item is still counted once");

        // A clean solution must report nothing.
        let mut clean = SPProblem::new(instance.clone());
        clean.change_strip_width(10_000.0);
        apply_sheet_walls(&mut clean, &sc);
        clean.place_item(SPPlacement {
            item_id: 0,
            d_transf: DTransformation::new(0.0, (1200.0, instance.base_strip.fixed_height / 2.0)),
        });
        let clean_stats = sheet_stats(&clean.save(), &instance, &sc);
        assert_eq!(clean_stats.iter().map(|s| s.straddling_item_ids.len()).sum::<usize>(), 0,
            "a wall-clear layout must report no straddling items");
        Ok(())
    }

    /// MEDIUM — an item wider than a sheet in every rotation makes the walled mode unsolvable and
    /// must be detected up front (this is what turns the 700 mm repro into a clear error instead of
    /// an infeasible export or an LBF runaway panic).
    #[test]
    fn items_too_wide_for_a_sheet_are_detected() -> Result<()> {
        use sparrow::optimizer::sheets::items_too_wide_for_sheet;
        let instance = import_spp("swim.json", None)?;

        // swim's parts are well over 700 mm wide, so a 700 mm sheet is impossible...
        let narrow = items_too_wide_for_sheet(&instance, &sheet(700.0, 20.0));
        assert!(!narrow.is_empty(), "700 mm sheets must be reported as too narrow for swim");
        for (_, w) in &narrow {
            assert!(*w > 700.0, "a reported item must genuinely exceed the sheet width");
        }

        // ...while a 2500 mm sheet fits every part.
        let wide = items_too_wide_for_sheet(&instance, &sheet(2500.0, 20.0));
        assert!(wide.is_empty(), "at 2500 mm every swim part fits a sheet, got {wide:?}");
        Ok(())
    }

    /// CRITICAL (audit) — `--sheet-gap 0` must be **rejected**, not honoured.
    ///
    /// This test used to assert the opposite ("a zero gap yields degenerate wall intervals, which
    /// `apply_sheet_walls` filters out"), which is precisely the bug: filtering the walls out turns
    /// a walled run into a plain strip run that still reports sheets. The audited reproduction
    /// (`--sheet-width 1995 --sheet-gap 0`) exported a layout with 10 items straddling a boundary
    /// at exit 0. A zero-thickness wall is not representable as a `Rect` hazard, and since the gap
    /// is virtual — the sheets are separate physical objects — refusing it costs nothing.
    #[test]
    fn zero_sheet_gap_is_rejected() {
        use sparrow::config::SheetConfig;

        assert!(SheetConfig::resolve_gap(Some(0.0), None).is_err(),
            "--sheet-gap 0 cannot be modelled and must be rejected");
        assert!(SheetConfig::resolve_gap(Some(-5.0), None).is_err(),
            "a negative gap must be an error, not a silent fallback to the 20 mm default");
        assert_eq!(SheetConfig::resolve_gap(Some(7.5), None).unwrap(), 7.5, "an explicit legal gap is used as given");
        assert!(SheetConfig::resolve_gap(None, None).unwrap() > 0.0, "no --sheet-gap still gets the default");
        assert_eq!(SheetConfig::resolve_gap(None, Some(5.0)).unwrap(), 20.0_f32.max(10.0),
            "the default still respects 2 * min_item_separation");
    }

}
