//! Regression tests for the findings of the independent `feat/bin-packing` audit
//! (`docs/feat-bin-packing-independent-audit.md`).
//!
//! Every test in here failed (or aborted the process) before the fix and passes after it. They are
//! grouped by the audit's own severity levels, and each carries the audit's reproduction in its doc
//! comment so the connection survives.
//!
//! Two kinds of test live here:
//! * **unit-level** ones that call the guard directly (`resolve_gap`, `validate_spp_warm_start`,
//!   `verify_spp_solution`, the packability checks, the CLI value parsers). These are the bulk,
//!   because the guards are the fix and they are what must not regress.
//! * **end-to-end** ones that run the *release* binary and assert on its exit code and on whether it
//!   wrote a JSON file. The audit's criticals were all "exit 0 and export something wrong", which is
//!   a property of the whole program and cannot be tested any other way. They are `#[ignore]`d by
//!   default and run explicitly (see [`release_binary`]) so that a plain `cargo test` never depends
//!   on a release build being present.

#[cfg(test)]
mod audit_regression_tests {
    use anyhow::Result;
    use jagua_rs::entities::Instance;
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
    use jagua_rs::probs::spp::io::ext_repr::{ExtItem, ExtSPInstance, ExtSPSolution};
    use jagua_rs::io::ext_repr::{
        ExtItem as ExtBaseItem, ExtLayout, ExtPlacedItem, ExtSPolygon, ExtShape, ExtTransformation,
    };
    use jagua_rs::geometry::DTransformation;
    use sparrow::config::{SheetConfig, DEFAULT_SPARROW_CONFIG};
    use sparrow::util::io::{
        parse_finite_f32, parse_non_negative_f32, parse_positive_f32, parse_sheet_gap,
        resolve_min_item_separation, validate_spp_warm_start, MIN_SHEET_GAP,
    };
    use sparrow::util::packability::{check_spp_packability, items_too_tall_for_strip};
    use sparrow::util::verify::{spp_placed, straddling_items, verify_spp_solution};

    // ---------------------------------------------------------------------------------------
    // fixtures
    // ---------------------------------------------------------------------------------------

    /// A `w x h` axis-aligned rectangle with its lower-left corner at the origin.
    fn rect_shape(w: f32, h: f32) -> ExtShape {
        ExtShape::SimplePolygon(ExtSPolygon(vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]))
    }

    fn ext_item(id: u64, demand: u64, w: f32, h: f32) -> ExtItem {
        ExtItem {
            base: ExtBaseItem {
                id,
                allowed_orientations: Some(vec![0.0]),
                shape: rect_shape(w, h),
                min_quality: None,
            },
            demand,
        }
    }

    /// An instance of `demand` copies of one `w x h` rectangle in a strip of the given height.
    fn ext_instance(demand: u64, w: f32, h: f32, strip_height: f32) -> ExtSPInstance {
        ExtSPInstance {
            name: "audit".into(),
            items: vec![ext_item(0, demand, w, h)],
            strip_height,
        }
    }

    fn placement(item_id: u64, x: f32, y: f32) -> ExtPlacedItem {
        ExtPlacedItem {
            item_id,
            transformation: ExtTransformation { rotation: 0.0, translation: (x, y) },
        }
    }

    fn ext_solution(strip_width: f32, placed: Vec<ExtPlacedItem>) -> ExtSPSolution {
        ExtSPSolution {
            strip_width,
            layout: ExtLayout { container_id: 0, placed_items: placed, density: 0.0 },
            density: 0.0,
            run_time_sec: 0,
        }
    }

    fn import(ext: &ExtSPInstance, min_sep: Option<f32>) -> Result<SPInstance> {
        let cfg = DEFAULT_SPARROW_CONFIG;
        let importer = Importer::new(cfg.cde_config, cfg.poly_simpl_tolerance, min_sep, cfg.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&importer, ext)
    }

    /// Builds a solution by placing the items at the given transformations, bypassing every check —
    /// this is how a bad warm start looks *after* it has been restored, which is the state the
    /// export gate has to catch.
    ///
    /// The coordinates are **item-shape coordinates**, i.e. where the item's own origin corner
    /// ends up, exactly like the external JSON representation uses. jagua re-centres every item's
    /// shape on its centroid via `shape_orig.pre_transform`, so the raw `DTransformation` the
    /// problem wants is not the same number; `ext_to_int_transformation` performs that conversion
    /// and is what the real warm-start import uses too.
    ///
    /// Note the fixtures keep a millimetre clear of every container edge: jagua counts an *exact*
    /// border touch as a collision, so an item flush at `x = 0` is "infeasible" for reasons that
    /// have nothing to do with what these tests are about.
    fn solution_of(instance: &SPInstance, width: f32, placements: &[(usize, f32, f32)]) -> jagua_rs::probs::spp::entities::SPSolution {
        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(width);
        for &(item_id, x, y) in placements {
            let ext = DTransformation::new(0.0, (x, y));
            let d_transf = jagua_rs::io::import::ext_to_int_transformation(
                &ext, &instance.item(item_id).shape_orig.pre_transform,
            );
            prob.place_item(SPPlacement { item_id, d_transf });
        }
        prob.save()
    }

    // =======================================================================================
    // CRITICAL 1 — `--sheet-gap 0` exported an uncuttable layout
    // =======================================================================================

    /// A zero-width wall is not representable as a `Rect` hazard, so it was silently dropped and
    /// items straddled the boundaries freely. Reproduction (audit):
    /// `sparrow -i iso7.json --sheet-width 1995 --sheet-gap 0 --min-sep 5 -e 0 -c 0 -s 42`
    /// -> exit 0, "10 items STRADDLE a sheet wall" in the log, JSON written anyway.
    ///
    /// The gap is now rejected at the CLI/config level.
    #[test]
    fn sheet_gap_zero_is_rejected() {
        let err = SheetConfig::resolve_gap(Some(0.0), Some(5.0))
            .expect_err("--sheet-gap 0 must be rejected: a zero-width wall cannot be modelled");
        let msg = err.to_string();
        assert!(msg.contains("too thin"), "the error must say why: {msg}");
        assert!(msg.contains("straddle"), "the error must explain the consequence: {msg}");
    }

    /// Anything below 1 mm goes the same way — including negative values, which used to be
    /// *silently replaced by the 20 mm default*, laying the run out on a pitch nobody asked for.
    #[test]
    fn sheet_gap_below_minimum_is_rejected() {
        for gap in [-20.0f32, -0.001, 0.0, 0.5, 0.999] {
            assert!(SheetConfig::resolve_gap(Some(gap), None).is_err(),
                "--sheet-gap {gap} is below the {MIN_SHEET_GAP} mm minimum and must be rejected");
        }
    }

    /// ...while a legal gap is still honoured exactly, and the default is unchanged.
    #[test]
    fn sheet_gap_legal_values_unchanged() -> Result<()> {
        assert_eq!(SheetConfig::resolve_gap(Some(1.0), None)?, 1.0, "the minimum itself is legal");
        assert_eq!(SheetConfig::resolve_gap(Some(20.0), Some(5.0))?, 20.0, "an explicit gap wins");
        assert_eq!(SheetConfig::resolve_gap(None, None)?, 20.0, "the default is max(20, 2*min_sep)");
        assert_eq!(SheetConfig::resolve_gap(None, Some(15.0))?, 30.0, "...and 2*min_sep when that is larger");
        Ok(())
    }

    /// The straddling gate itself: a layout with an item across a wall must be refused by the export
    /// gate even if it is otherwise collision-free and covers the demand. This is the mandatory
    /// pre-export check the sheets mode was missing entirely.
    #[test]
    fn export_gate_rejects_straddling_items() -> Result<()> {
        let ext = ext_instance(2, 60.0, 45.0, 100.0);
        let instance = import(&ext, None)?;
        let sc = SheetConfig::new(100.0, 20.0, false);

        // Sheet 0 is [0,100], wall 1 is [100,120], sheet 1 starts at 120. An item 60 wide placed at
        // x=80 spans [80,140]: straight across the wall.
        let straddling = solution_of(&instance, 240.0, &[(0, 80.0, 1.0), (0, 130.0, 50.0)]);
        assert!(!straddling_items(&straddling, &instance, &sc).is_empty(),
            "the fixture must actually straddle a wall");
        let err = verify_spp_solution(&straddling, &instance, Some(&sc), "the solution")
            .expect_err("a straddling layout must never be exported");
        assert!(err.to_string().contains("straddling"), "{err}");

        // The same two items, each wholly inside a sheet and clear of the strip's own border, must
        // pass. (The container is deflated slightly by the shape-modification config, so the items
        // are nudged a millimetre off every edge.)
        let clean = solution_of(&instance, 240.0, &[(0, 10.0, 1.0), (0, 130.0, 1.0)]);
        assert!(straddling_items(&clean, &instance, &sc).is_empty(), "sanity: nothing straddles here");
        verify_spp_solution(&clean, &instance, Some(&sc), "the solution")
            .expect("a cuttable, collision-free, complete layout must pass the gate");
        Ok(())
    }

    // =======================================================================================
    // CRITICAL 2 — SPP warm start demand gate
    // =======================================================================================

    /// demand 2, one placement -> used to be optimized and exported with the item silently missing
    /// (audit: "demand 1, placement 0 -> exit 0").
    #[test]
    fn warm_start_with_missing_placement_is_rejected() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let sol = ext_solution(100.0, vec![placement(0, 0.0, 0.0)]);
        let err = validate_spp_warm_start(&ext, &sol).expect_err("an incomplete warm start must be rejected");
        let msg = err.to_string();
        assert!(msg.contains("placed 1"), "the error must name the mismatch: {msg}");
        assert!(msg.contains("demanded 2"), "{msg}");
    }

    /// demand 2, three placements -> used to be exported with the item duplicated
    /// (audit: "demand 1, placement 2 -> exit 0").
    #[test]
    fn warm_start_with_extra_placement_is_rejected() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let sol = ext_solution(200.0, vec![
            placement(0, 0.0, 0.0), placement(0, 0.0, 50.0), placement(0, 60.0, 0.0),
        ]);
        let err = validate_spp_warm_start(&ext, &sol).expect_err("an over-complete warm start must be rejected");
        assert!(err.to_string().contains("placed 3"), "{err}");
    }

    /// The check is **per item id**, not on the total. Two copies of item 0 and none of item 1 has
    /// the right total (2) and is still wrong — a total-only check waves it straight through.
    #[test]
    fn warm_start_demand_is_checked_per_item_id() {
        let ext = ExtSPInstance {
            name: "two".into(),
            items: vec![ext_item(0, 1, 40.0, 40.0), ext_item(1, 1, 30.0, 30.0)],
            strip_height: 100.0,
        };
        let sol = ext_solution(200.0, vec![placement(0, 0.0, 0.0), placement(0, 60.0, 0.0)]);
        let err = validate_spp_warm_start(&ext, &sol)
            .expect_err("the right total with the wrong per-id split must still be rejected");
        let msg = err.to_string();
        assert!(msg.contains("item 0: placed 2, demanded 1"), "{msg}");
        assert!(msg.contains("item 1: placed 0, demanded 1"), "{msg}");
    }

    /// An exactly-matching warm start must of course still be accepted.
    #[test]
    fn warm_start_with_exact_demand_is_accepted() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let sol = ext_solution(100.0, vec![placement(0, 0.0, 0.0), placement(0, 50.0, 0.0)]);
        validate_spp_warm_start(&ext, &sol).expect("an exact warm start must be accepted");
    }

    /// The **export** gate enforces the same property on the final answer, not just on the import:
    /// an empty layout is perfectly collision-free and narrower than every correct one, so a
    /// width-only `-p` selection would pick it every single time.
    #[test]
    fn export_gate_rejects_incomplete_solution() -> Result<()> {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let instance = import(&ext, None)?;

        let incomplete = solution_of(&instance, 100.0, &[(0, 1.0, 1.0)]);
        assert_eq!(spp_placed(&incomplete).get(&0).copied(), Some(1), "sanity");
        let err = verify_spp_solution(&incomplete, &instance, None, "the solution")
            .expect_err("a solution missing an item must never be exported");
        assert!(err.to_string().contains("demand"), "{err}");

        // An entirely empty layout is the pathological case the width-only selection preferred.
        let empty = solution_of(&instance, 100.0, &[]);
        assert!(verify_spp_solution(&empty, &instance, None, "the solution").is_err(),
            "an empty layout is collision-free but covers no demand at all");

        let complete = solution_of(&instance, 100.0, &[(0, 1.0, 1.0), (0, 50.0, 1.0)]);
        verify_spp_solution(&complete, &instance, None, "the solution")
            .expect("a complete, collision-free solution must pass");
        Ok(())
    }

    // =======================================================================================
    // CRITICAL 3 — geometrically infeasible warm start reached the export
    // =======================================================================================

    /// The audit's reproduction exported a layout with 1 185 179.5 mm² of overlap at exit 0,
    /// because `optimize`'s "possibly infeasible" fallback handed the separator's current layout
    /// back as the answer. The fallback is gone and the export gate is the backstop.
    #[test]
    fn export_gate_rejects_overlapping_solution() -> Result<()> {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let instance = import(&ext, None)?;

        // Two 40x40 items one millimetre apart: a ~1 521 mm² overlap.
        let overlapping = solution_of(&instance, 45.0, &[(0, 1.0, 1.0), (0, 2.0, 2.0)]);
        assert_eq!(spp_placed(&overlapping).get(&0).copied(), Some(2), "the demand IS covered...");
        let err = verify_spp_solution(&overlapping, &instance, None, "the solution")
            .expect_err("...but the items overlap, so it must be refused");
        assert!(err.to_string().contains("collision-free"), "{err}");
        Ok(())
    }

    // =======================================================================================
    // HIGH 5 — a malformed warm start aborted instead of erroring
    // =======================================================================================

    /// `item_id = 999` used to index `instance.items` out of bounds inside jagua's
    /// `import_solution`: `panic = abort`, exit 134. Audit: `spp_foreign.json`.
    #[test]
    fn warm_start_with_unknown_item_id_is_rejected() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let sol = ext_solution(100.0, vec![placement(999, 0.0, 0.0), placement(0, 0.0, 50.0)]);
        let err = validate_spp_warm_start(&ext, &sol).expect_err("an unknown item id must be a clean error");
        assert!(err.to_string().contains("unknown item id 999"), "{err}");
    }

    /// A negative strip width builds an invalid `Rect` deep inside jagua: abort, exit 134.
    #[test]
    fn warm_start_with_negative_strip_width_is_rejected() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        for width in [-5.0f32, 0.0, f32::NAN, f32::INFINITY] {
            assert!(validate_spp_warm_start(&ext, &ext_solution(width, vec![
                placement(0, 0.0, 0.0), placement(0, 50.0, 0.0),
            ])).is_err(), "a strip width of {width} must be rejected");
        }
    }

    /// Same as above but asserting the message, split out because the loop above only asserts that
    /// *something* is rejected.
    #[test]
    fn warm_start_negative_width_message() {
        let ext = ext_instance(2, 40.0, 40.0, 100.0);
        let sol = ext_solution(-5.0, vec![placement(0, 0.0, 0.0), placement(0, 50.0, 0.0)]);
        let err = validate_spp_warm_start(&ext, &sol).expect_err("a negative strip width must be rejected");
        assert!(err.to_string().contains("invalid strip width"), "{err}");
    }

    /// A non-finite transformation is just as unusable and just as fatal downstream.
    #[test]
    fn warm_start_with_non_finite_transformation_is_rejected() {
        let ext = ext_instance(1, 40.0, 40.0, 100.0);
        let mut sol = ext_solution(100.0, vec![placement(0, 0.0, 0.0)]);
        sol.layout.placed_items[0].transformation.translation.0 = f32::NAN;
        let err = validate_spp_warm_start(&ext, &sol).expect_err("a NaN translation must be rejected");
        assert!(err.to_string().contains("non-finite"), "{err}");
    }

    // =======================================================================================
    // HIGH 6 — NaN / Infinity were accepted as CLI floats
    // =======================================================================================

    /// `--sheet-width NaN` aborted (exit 134); `--sheet-width inf` exited 0 and wrote a JSON whose
    /// metrics were `NaN`/`Infinity`; `--min-sep NaN` *silently disabled* the separation, because
    /// every `v > 0.0` test against a NaN is false.
    #[test]
    fn cli_float_parsers_reject_non_finite() {
        for bad in ["NaN", "nan", "inf", "-inf", "Infinity", "-Infinity"] {
            assert!(parse_finite_f32(bad).is_err(), "`{bad}` must not parse as a finite float");
            assert!(parse_positive_f32(bad).is_err(), "`{bad}` must be rejected for --sheet-width");
            assert!(parse_non_negative_f32(bad).is_err(), "`{bad}` must be rejected for --min-sep");
            assert!(parse_sheet_gap(bad).is_err(), "`{bad}` must be rejected for --sheet-gap");
        }
    }

    #[test]
    fn cli_float_parsers_enforce_ranges() {
        // --sheet-width must be > 0
        assert!(parse_positive_f32("0").is_err(), "--sheet-width 0 is meaningless");
        assert!(parse_positive_f32("-1").is_err(), "--sheet-width may not be negative");
        assert_eq!(parse_positive_f32("1995").unwrap(), 1995.0);

        // --min-sep must be >= 0
        assert!(parse_non_negative_f32("-1").is_err(), "--min-sep may not be negative");
        assert_eq!(parse_non_negative_f32("0").unwrap(), 0.0, "0 is legal: it disables the separation");
        assert_eq!(parse_non_negative_f32("5").unwrap(), 5.0);

        // --sheet-gap must be >= MIN_SHEET_GAP
        assert!(parse_sheet_gap("0").is_err(), "a zero-width wall is not representable");
        assert!(parse_sheet_gap("0.5").is_err(), "below the 1 mm minimum");
        assert_eq!(parse_sheet_gap("1").unwrap(), MIN_SHEET_GAP);
        assert_eq!(parse_sheet_gap("20").unwrap(), 20.0);

        // Garbage is still garbage.
        assert!(parse_finite_f32("").is_err());
        assert!(parse_finite_f32("abc").is_err());
    }

    /// `resolve_min_item_separation` is the last line of defence for the env-var path, which no
    /// clap parser sees. A NaN there used to disable the separation without a word.
    #[test]
    fn resolve_min_item_separation_rejects_non_finite() {
        assert!(resolve_min_item_separation(Some(f32::NAN), None).is_err(), "a NaN --min-sep must error, not silently disable");
        assert!(resolve_min_item_separation(Some(f32::INFINITY), None).is_err());
        // The normal paths are unchanged.
        assert_eq!(resolve_min_item_separation(Some(5.0), None).unwrap(), Some(5.0));
        assert_eq!(resolve_min_item_separation(Some(0.0), Some(3.0)).unwrap(), None, "0 disables it");
        assert_eq!(resolve_min_item_separation(None, Some(3.0)).unwrap(), Some(3.0), "the config default applies");
    }

    // =======================================================================================
    // HIGH 7 — unpackable / large-min-sep instances aborted
    // =======================================================================================

    /// An item taller than the strip made the LBF widen for ever until
    /// `strip-width is running away` aborted the process (exit 134). Audit: a 20x80 item in a
    /// 50 mm strip.
    #[test]
    fn item_taller_than_strip_is_a_clean_error() -> Result<()> {
        let ext = ExtSPInstance {
            name: "tall".into(),
            items: vec![ext_item(0, 1, 20.0, 80.0)],
            strip_height: 50.0,
        };
        let instance = import(&ext, None)?;

        let too_tall = items_too_tall_for_strip(&instance);
        assert_eq!(too_tall.len(), 1, "the 80 mm item does not fit a 50 mm strip");
        assert_eq!(too_tall[0].0, 0);

        let err = check_spp_packability(&instance, None).expect_err("an unpackable instance must be an error");
        let msg = err.to_string();
        assert!(msg.contains("50.0 mm high"), "the message must name the strip height: {msg}");
        assert!(msg.contains("item 0"), "...and the offending item: {msg}");
        Ok(())
    }

    /// A rotatable item that fits in *some* orientation is packable and must not be flagged.
    #[test]
    fn item_that_fits_when_rotated_is_accepted() -> Result<()> {
        let mut ext = ExtSPInstance {
            name: "rot".into(),
            items: vec![ext_item(0, 1, 20.0, 80.0)],
            strip_height: 100.0,
        };
        // 90 degrees turns the 20x80 into an 80x20, which fits a 100 mm strip either way.
        ext.items[0].base.allowed_orientations = Some(vec![0.0, 90.0]);
        let instance = import(&ext, None)?;
        assert!(items_too_tall_for_strip(&instance).is_empty(), "it fits at 0 degrees already");
        assert!(check_spp_packability(&instance, None).is_ok());
        Ok(())
    }

    /// Two 10x10 items in a 100 mm strip with `--min-sep 20`: jagua derives the starting width from
    /// the item area (200/100 = 2 mm), deflating it by 10 mm per side leaves an empty polygon and
    /// the process aborts with `Offset resulted in an empty polygon` (exit 134) — even though the
    /// instance is perfectly solvable at a sane width. The gate now asks for a wider start instead.
    #[test]
    fn large_min_sep_widens_the_strip_instead_of_aborting() -> Result<()> {
        let ext = ext_instance(2, 10.0, 10.0, 100.0);
        let instance = import(&ext, Some(20.0))?;
        assert!(instance.base_strip.width <= 20.0, "sanity: jagua's 100%-density start is tiny here");

        let widen = check_spp_packability(&instance, Some(20.0))?
            .expect("the container would not survive its deflation, so a wider start is required");
        assert!(widen > 20.0, "the new width must exceed the {}mm the deflation eats: got {widen}", 20.0);

        // With the widened strip the container is constructible, which is what used to abort.
        let mut widened = instance.clone();
        widened.base_strip.set_width(widen);
        let _prob = SPProblem::new(widened);
        Ok(())
    }

    /// A `--min-sep` at least as large as the strip height is genuinely unsolvable and must say so
    /// rather than aborting inside jagua.
    ///
    /// Which of the two gate branches speaks up depends on the instance: inflating every item by
    /// `min_sep / 2` also makes it taller, so a separation that large usually pushes the items past
    /// the strip height first. Both are correct answers and both name the real cause, so the
    /// assertion checks that the message mentions the separation *or* the strip height rather than
    /// pinning one specific branch.
    #[test]
    fn min_sep_larger_than_strip_height_is_an_error() -> Result<()> {
        let ext = ext_instance(2, 10.0, 10.0, 20.0);
        let instance = import(&ext, Some(30.0))?;
        let err = check_spp_packability(&instance, Some(30.0))
            .expect_err("a separation wider than the strip leaves nothing to pack into");
        let msg = err.to_string();
        assert!(msg.contains("strip") && (msg.contains("min-sep") || msg.contains("inflation")),
            "the message must blame the separation/strip, not abort: {msg}");
        Ok(())
    }

    /// A normal instance needs no intervention at all — the gate must be invisible in the common case.
    #[test]
    fn packability_gate_is_a_no_op_for_a_normal_instance() -> Result<()> {
        let ext = ext_instance(10, 10.0, 10.0, 100.0);
        let instance = import(&ext, Some(2.0))?;
        assert_eq!(check_spp_packability(&instance, Some(2.0))?, None, "nothing to fix here");
        assert_eq!(check_spp_packability(&instance, None)?, None);
        Ok(())
    }

    // =======================================================================================
    // MEDIUM 10 — the sheet operators overran their deadline
    // =======================================================================================

    /// The separator's inner loop had **no terminator check at all**: only the outer strike loop
    /// looked at it, so an expired deadline was noticed at most once per strike — and with the
    /// patient scatter/wall-repair settings a strike is up to 400 no-improvement iterations. The
    /// audited `-e 1 -c 1` iso7 run gave the pack-down a 0.3 s budget and it took ~2.5 s, with the
    /// whole run at ~3.8 s against a 2 s budget.
    ///
    /// The test asks the separator for a deliberately tiny budget on a layout that cannot possibly
    /// be separated in it, and requires it to come back inside a slack allowance. Without the fix
    /// this runs for as long as a full strike takes.
    #[test]
    fn separator_honours_its_deadline() -> Result<()> {
        use sparrow::optimizer::separator::Separator;
        use sparrow::util::listener::DummySolListener;
        use sparrow::util::terminator::{BasicTerminator, Terminator};
        use std::time::{Duration, Instant};
        use rand::SeedableRng;
        use rand::rngs::Xoshiro256PlusPlus;

        // A strip that provably cannot hold what is in it: 30 copies of a 40x40 part (48 000 mm²
        // of item) in a 200x200 strip (40 000 mm²). No arrangement is feasible, so the separator
        // grinds away at a loss it can never drive to zero and only a terminator can stop it.
        //
        // The strip is kept comfortably wider than a single part on purpose — a strip narrower
        // than the item itself trips a jagua quadtree debug assertion that has nothing to do with
        // what is being measured here.
        let ext = ext_instance(30, 40.0, 40.0, 200.0);
        let instance = import(&ext, None)?;
        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(200.0);
        for i in 0..30 {
            let e = DTransformation::new(0.0, (2.0 + (i % 6) as f32 * 25.0, 2.0 + (i / 6) as f32 * 25.0));
            let d_transf = jagua_rs::io::import::ext_to_int_transformation(
                &e, &instance.item(0).shape_orig.pre_transform,
            );
            prob.place_item(SPPlacement { item_id: 0, d_transf });
        }

        // A deliberately patient configuration — this is what the scatter and the wall repair use,
        // and it is exactly the setting that made the overrun so large.
        let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config;
        cfg.strike_limit = 8;
        cfg.iter_no_imprv_limit = 400;

        let mut sep = Separator::new(instance, prob, Xoshiro256PlusPlus::seed_from_u64(42), cfg);
        let budget = Duration::from_millis(200);
        let mut term = BasicTerminator::new();
        term.new_timeout(budget);

        let start = Instant::now();
        let (_sol, _ct) = sep.separate(&term, &mut DummySolListener);
        let elapsed = start.elapsed();

        // The check is per iteration, and one iteration (all workers, one pass over every item)
        // cannot be interrupted, so some overshoot is expected. A whole strike is not.
        //
        // The allowance is profile-dependent because the *unit* being bounded is one iteration, and
        // a debug iteration is far slower than a release one (debug assertions re-verify the whole
        // tracker against the layout on every move). The property under test — "it stops after an
        // iteration rather than after a strike" — is the same in both; only the constant differs.
        // Before the fix this ran for a full strike, i.e. up to 400 iterations, in either profile:
        // measured 2.56 s in debug and 1.20 s in release against this 200 ms budget, versus 0.45 s
        // and 0.20 s after. The allowances below sit between those two pairs, so the test
        // discriminates in both profiles rather than merely passing.
        let allowance = budget + match cfg!(debug_assertions) {
            true => Duration::from_millis(1200),
            false => Duration::from_millis(600),
        };
        assert!(elapsed <= allowance,
            "separate() overran its {budget:?} budget by too much: took {elapsed:?} (allowed up to {allowance:?})");
        Ok(())
    }

    // =======================================================================================
    // end-to-end: the release binary's exit code and whether it wrote a file
    // =======================================================================================
    //
    // The criticals were all "exit 0 and export something wrong", which only the whole program
    // exhibits. They run **by default**: every one uses `-e 0 -c 0`, so the whole group finishes in
    // well under a second, and a gate that is `#[ignore]`d is a gate that a plain `cargo test`
    // silently skips — which is exactly what the second audit found.
    //
    // The binary comes from `env!("CARGO_BIN_EXE_sparrow")`, which Cargo defines for every
    // integration test as the path of the `sparrow` binary **built for this test run**, in this
    // profile and this target directory. The previous version hardcoded
    // `CARGO_MANIFEST_DIR/target/release/sparrow` and returned early when it was absent, so under a
    // custom `CARGO_TARGET_DIR` it tested a stale binary from an earlier build, and with no binary
    // present at all every test passed having asserted nothing.

    mod end_to_end {
        use super::*;
        use std::path::{Path, PathBuf};
        use std::process::Command;

        /// The `sparrow` binary built for **this** test run. A compile-time constant: it cannot be
        /// missing, and it cannot be a different build than the one under test.
        fn sparrow_binary() -> PathBuf {
            PathBuf::from(env!("CARGO_BIN_EXE_sparrow"))
        }

        /// Every file in `dir`, sorted.
        fn files_in(dir: &Path) -> Vec<String> {
            let Ok(entries) = std::fs::read_dir(dir) else { return vec![] };
            let mut names: Vec<String> = entries.flatten()
                .filter(|e| e.path().is_file())
                .filter_map(|e| e.file_name().into_string().ok())
                .collect();
            names.sort();
            names
        }

        /// What a run left behind.
        pub struct Outcome {
            pub code: i32,
            pub wrote_json: bool,
            /// Any `final_*.svg` in `output/`. A rejected run must leave none: the optimizer's
            /// listener used to write the final SVG *before* the export gate, so a run that
            /// correctly refused to export its JSON still left an SVG of the rejected layout.
            pub wrote_final_svg: bool,
            pub files: Vec<String>,
            pub stderr: String,
        }

        /// Runs the binary in a fresh temp dir and reports what it did.
        fn run(args: &[&str], input: &ExtSPInstance, solution: Option<&ExtSPSolution>) -> Outcome {
            let bin = sparrow_binary();
            let dir = std::env::temp_dir().join(format!("sparrow-audit-e2e-{}", std::process::id()))
                .join(format!("{:x}", std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
            std::fs::create_dir_all(&dir).expect("could not create the test working directory");

            #[derive(serde::Serialize)]
            struct Out<'a> {
                #[serde(flatten)]
                instance: &'a ExtSPInstance,
                #[serde(skip_serializing_if = "Option::is_none")]
                solution: Option<&'a ExtSPSolution>,
            }
            let input_path = dir.join("input.json");
            std::fs::write(&input_path, serde_json::to_string(&Out { instance: input, solution })
                .expect("the fixture must serialise")).expect("could not write the test input");

            let output = Command::new(&bin)
                .current_dir(&dir)
                .arg("-i").arg(&input_path)
                .args(args)
                .output()
                .unwrap_or_else(|e| panic!("could not run {}: {e}", bin.display()));

            let files = files_in(&dir.join("output"));
            let outcome = Outcome {
                code: output.status.code().unwrap_or(-1),
                wrote_json: files.iter().any(|f| f == &format!("final_{}.json", input.name)),
                wrote_final_svg: files.iter().any(|f| f.starts_with("final_") && f.ends_with(".svg")),
                files,
                stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
            };
            let _ = std::fs::remove_dir_all(&dir);
            outcome
        }

        /// CRITICAL 1: `--sheet-gap 0` must be a clap error (exit 2) and write nothing.
        #[test]
        fn e2e_sheet_gap_zero_exits_2() {
            let ext = ext_instance(6, 60.0, 45.0, 100.0);
            let out = run(&["--sheet-width", "100", "--sheet-gap", "0", "-e", "0", "-c", "0", "-s", "42"], &ext, None);
            assert_eq!(out.code, 2, "a rejected CLI value must exit 2: {}", out.stderr);
            assert!(!out.wrote_json, "nothing may be exported: {:?}", out.files);
            assert!(!out.wrote_final_svg, "not even an SVG: {:?}", out.files);
        }

        /// CRITICAL 2: an incomplete warm start must exit 1 and write nothing (was: exit 0 + JSON).
        #[test]
        fn e2e_incomplete_warm_start_exits_1() {
            let ext = ext_instance(2, 40.0, 40.0, 100.0);
            let sol = ext_solution(100.0, vec![placement(0, 1.0, 1.0)]);
            let out = run(&["-e", "0", "-c", "0", "-s", "42"], &ext, Some(&sol));
            assert_eq!(out.code, 1, "an unusable warm start must exit 1: {}", out.stderr);
            assert!(!out.wrote_json, "nothing may be exported: {:?}", out.files);
            assert!(!out.wrote_final_svg, "not even an SVG: {:?}", out.files);
        }

        /// CRITICAL 3: an overlapping warm start must exit 1 and write nothing (was: exit 0 + JSON
        /// with 1 185 179.5 mm² of overlap).
        #[test]
        fn e2e_overlapping_warm_start_exits_1() {
            let ext = ext_instance(2, 40.0, 40.0, 100.0);
            let sol = ext_solution(45.0, vec![placement(0, 0.0, 0.0), placement(0, 1.0, 1.0)]);
            let out = run(&["-e", "0", "-c", "0", "-s", "42"], &ext, Some(&sol));
            // In release the export gate turns this into exit 1. A debug build dies earlier, on the
            // `[EXPL] exploration must start from a feasible layout` debug assertion (exit 101);
            // the property both share — and the one that matters — is that nothing is exported.
            assert_ne!(out.code, 0, "an infeasible solution must never be exported: {}", out.stderr);
            if !cfg!(debug_assertions) {
                assert_eq!(out.code, 1, "in release the gate must make it exit 1: {}", out.stderr);
            }
            assert!(!out.wrote_json, "{:?}", out.files);
            assert!(!out.wrote_final_svg, "not even an SVG: {:?}", out.files);
        }

        /// HIGH 5: an unknown item id must exit 1, not abort with 134.
        #[test]
        fn e2e_unknown_item_id_exits_1_not_134() {
            let ext = ext_instance(2, 40.0, 40.0, 100.0);
            let sol = ext_solution(100.0, vec![placement(999, 1.0, 1.0), placement(0, 1.0, 50.0)]);
            let out = run(&["-e", "0", "-c", "0", "-s", "42"], &ext, Some(&sol));
            assert_eq!(out.code, 1, "a malformed warm start must be an error, not an abort (was 134): {}", out.stderr);
            assert!(!out.wrote_json, "{:?}", out.files);
            assert!(!out.wrote_final_svg, "{:?}", out.files);
        }

        /// HIGH 6: non-finite CLI floats must exit 2, not abort or produce NaN output.
        #[test]
        fn e2e_non_finite_cli_floats_exit_2() {
            let ext = ext_instance(2, 40.0, 40.0, 100.0);
            for args in [
                vec!["--sheet-width", "NaN", "-e", "0", "-c", "0", "-s", "42"],
                vec!["--sheet-width", "inf", "-e", "0", "-c", "0", "-s", "42"],
                vec!["--min-sep", "NaN", "-e", "0", "-c", "0", "-s", "42"],
            ] {
                let out = run(&args, &ext, None);
                assert_eq!(out.code, 2, "{args:?} must be a clap error: {}", out.stderr);
                assert!(!out.wrote_json, "{args:?} must not export anything: {:?}", out.files);
                assert!(!out.wrote_final_svg, "{args:?} must not leave an SVG: {:?}", out.files);
            }
        }

        /// HIGH 7: an item taller than the strip must exit 1, not abort with 134.
        #[test]
        fn e2e_item_taller_than_strip_exits_1_not_134() {
            // A *fixed-orientation* part (`allowed_orientations: [0.0]`, as `ext_item` builds), so
            // no rotation can rescue it — with continuous rotation the pre-check deliberately keeps
            // quiet, because its 16-step grid samples a continuum and cannot prove non-fitting.
            let ext = ExtSPInstance { name: "tall".into(), items: vec![ext_item(0, 1, 20.0, 80.0)], strip_height: 50.0 };
            let out = run(&["-e", "0", "-c", "0", "-s", "42"], &ext, None);
            assert_eq!(out.code, 1, "an unpackable instance must be an error, not an abort (was 134): {}", out.stderr);
            assert!(!out.wrote_json, "{:?}", out.files);
            assert!(!out.wrote_final_svg, "{:?}", out.files);
        }

        /// HIGH 7 (second half): a large `--min-sep` must now *succeed* rather than abort — the
        /// instance is solvable, only jagua's starting width was too small to survive the deflation.
        #[test]
        fn e2e_large_min_sep_succeeds() {
            let ext = ext_instance(2, 10.0, 10.0, 100.0);
            let out = run(&["--min-sep", "20", "-e", "0", "-c", "0", "-s", "42"], &ext, None);
            assert_eq!(out.code, 0, "this instance is solvable at a sane width (was: abort 134): {}", out.stderr);
            assert!(out.wrote_json, "and it must produce a solution: {:?}", out.files);
            assert!(out.wrote_final_svg, "a successful run writes the final SVG too: {:?}", out.files);
        }

        /// MEDIUM 9: an absurd demand must be a message, not an 800 MB allocation and an abort.
        #[test]
        fn e2e_huge_demand_exits_1() {
            let ext = ext_instance(100_000_000, 10.0, 10.0, 100.0);
            let out = run(&["-e", "0", "-c", "0", "-s", "42"], &ext, None);
            assert_eq!(out.code, 1, "an unsupportable demand must be a clean error: {}", out.stderr);
            assert!(!out.wrote_json, "{:?}", out.files);
            assert!(!out.wrote_final_svg, "{:?}", out.files);
        }

        /// CRITICAL 2, sheets half: the exact-demand gate must apply to the **walled** warm-start
        /// path too, which the audit found had no such check at all.
        #[test]
        fn e2e_sheets_warm_start_demand_is_gated() {
            let ext = ext_instance(4, 40.0, 40.0, 100.0);
            let sol = ext_solution(200.0, vec![placement(0, 1.0, 1.0), placement(0, 60.0, 1.0)]);
            let out = run(
                &["--sheet-width", "100", "--sheet-gap", "20", "-e", "0", "-c", "0", "-s", "42"],
                &ext, Some(&sol));
            assert_eq!(out.code, 1, "2 of 4 items in a walled warm start must be rejected: {}", out.stderr);
            assert!(!out.wrote_json, "nothing may be exported: {:?}", out.files);
            assert!(!out.wrote_final_svg, "not even an SVG: {:?}", out.files);
        }

        /// MEDIUM 10 end-to-end: the audit's own reproduction shape — `-e 1 -c 1` with the
        /// pack-down and compact-sheets operators enabled — must finish inside `budget + 1.5 s`.
        /// It measured ~3.8 s against a 2 s budget before the deadline checks were added.
        ///
        /// The only `#[ignore]`d test in this module, and for a reason the others do not share: it
        /// asserts on **wall-clock time**, so it is meaningless in a debug build (where one
        /// separator iteration is several times slower) and unreliable on a loaded machine. Run it
        /// deliberately:
        ///
        /// ```bash
        /// cargo test --release --test audit_regression_tests -- --ignored --nocapture
        /// ```
        ///
        /// or via `scripts/ci.sh`, which runs the whole gate.
        #[test]
        #[ignore = "wall-clock assertion: only meaningful in release. `cargo test --release --test audit_regression_tests -- --ignored`"]
        fn e2e_pack_down_honours_the_time_budget() {
            use std::time::{Duration, Instant};

            // Enough parts, and tight enough, that both operators have real work to do.
            let ext = ext_instance(24, 40.0, 40.0, 200.0);
            let start = Instant::now();
            let out = run(
                &["--sheet-width", "150", "--sheet-gap", "20", "--min-sep", "5",
                  "--compact-sheets", "--pack-down-sheets", "-e", "1", "-c", "1", "-s", "42"],
                &ext, None);
            let elapsed = start.elapsed();
            assert_eq!(out.code, 0, "this instance is packable and must succeed: {}", out.stderr);

            // 1 s explore + 1 s compress, plus the allowance the task asks for. Process start-up,
            // instance import and the final SVG/JSON write all sit outside the phase budgets.
            let allowance = Duration::from_secs(2) + Duration::from_millis(1500);
            assert!(elapsed <= allowance,
                "the run overran its 2 s budget: took {elapsed:?} (allowed up to {allowance:?})");
        }
    }
}

