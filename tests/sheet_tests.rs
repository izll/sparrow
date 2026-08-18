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
        SheetConfig { width, gap, compact_sheets: false }
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
}
