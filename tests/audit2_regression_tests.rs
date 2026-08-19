//! Regression tests for the findings of the **second** independent `feat/bin-packing` audit
//! (`docs/feat-bin-packing-second-audit.md`, HEAD `2c48460`).
//!
//! Every test here fails (or lets a wrong answer through) on `2c48460` and passes after the fix.
//! Each carries the audit's own reproduction in its doc comment.
//!
//! As in `audit_regression_tests.rs`, there are two kinds:
//! * **unit-level** tests calling the guard directly — the bulk, because the guard *is* the fix;
//! * **end-to-end** tests running the release binary. Those live in
//!   [`end_to_end`] and use `env!("CARGO_BIN_EXE_sparrow")` / `..._sparrow-bpp`, which Cargo sets
//!   for every integration test to the binary **built for this very test run**. They are cheap
//!   (`-e 0 -c 0`) so they run by default; nothing here is `#[ignore]`d.

#[cfg(test)]
mod audit2_regression_tests {
    use anyhow::Result;
    use jagua_rs::entities::Instance;
    use jagua_rs::geometry::geo_enums::RotationRange;
    use jagua_rs::geometry::DTransformation;
    use jagua_rs::io::ext_repr::{
        ExtItem as ExtBaseItem, ExtLayout, ExtPlacedItem, ExtSPolygon, ExtShape, ExtTransformation,
    };
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, SPSolution};
    use jagua_rs::probs::spp::io::ext_repr::{ExtItem, ExtSPInstance, ExtSPSolution};
    use sparrow::config::{SheetConfig, DEFAULT_SPARROW_CONFIG};
    use sparrow::util::io::validate_spp_warm_start;
    use sparrow::util::packability::{check_spp_packability, items_too_tall_for_strip};
    use sparrow::util::rotations::{candidate_rotations, ext_orientation_ok, rotation_is_allowed, ROT_N_SAMPLES};
    use sparrow::util::verify::{verify_spp_solution, ROTATION_TOL_DEG, ROTATION_TOL_RAD};
    use std::f32::consts::PI;

    // ---------------------------------------------------------------------------------------
    // fixtures
    // ---------------------------------------------------------------------------------------

    fn rect_shape(w: f32, h: f32) -> ExtShape {
        ExtShape::SimplePolygon(ExtSPolygon(vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]))
    }

    /// One `w x h` rectangle, with the given `allowed_orientations` (degrees; `None` = continuous).
    fn ext_item_rot(id: u64, demand: u64, w: f32, h: f32, orientations: Option<Vec<f32>>) -> ExtItem {
        ExtItem {
            base: ExtBaseItem { id, allowed_orientations: orientations, shape: rect_shape(w, h), min_quality: None },
            demand,
        }
    }

    fn ext_instance_rot(demand: u64, w: f32, h: f32, strip_height: f32, orientations: Option<Vec<f32>>) -> ExtSPInstance {
        ExtSPInstance {
            name: "audit2".into(),
            items: vec![ext_item_rot(0, demand, w, h, orientations)],
            strip_height,
        }
    }

    fn import(ext: &ExtSPInstance, min_sep: Option<f32>) -> Result<SPInstance> {
        let cfg = DEFAULT_SPARROW_CONFIG;
        let importer = Importer::new(cfg.cde_config, cfg.poly_simpl_tolerance, min_sep, cfg.narrow_concavity_cutoff_ratio);
        jagua_rs::probs::spp::io::import_instance(&importer, ext)
    }

    /// Places items at the given `(item_id, rotation_rad, x, y)` transformations with no checks at
    /// all — the state a bad warm start leaves behind once it has been restored, which is exactly
    /// what the export gate has to catch.
    fn solution_of(instance: &SPInstance, width: f32, placements: &[(usize, f32, f32, f32)]) -> SPSolution {
        let mut prob = SPProblem::new(instance.clone());
        prob.change_strip_width(width);
        for &(item_id, rot, x, y) in placements {
            let ext = DTransformation::new(rot, (x, y));
            let d_transf = jagua_rs::io::import::ext_to_int_transformation(
                &ext, &instance.item(item_id).shape_orig.pre_transform,
            );
            prob.place_item(SPPlacement { item_id, d_transf });
        }
        prob.save()
    }

    fn ext_solution(strip_width: f32, placed: Vec<ExtPlacedItem>) -> ExtSPSolution {
        ExtSPSolution {
            strip_width,
            layout: ExtLayout { container_id: 0, placed_items: placed, density: 0.0 },
            density: 0.0,
            run_time_sec: 0,
        }
    }

    fn placement_rot(item_id: u64, rotation_deg: f32, x: f32, y: f32) -> ExtPlacedItem {
        ExtPlacedItem { item_id, transformation: ExtTransformation { rotation: rotation_deg, translation: (x, y) } }
    }

    // =======================================================================================
    // CRITICAL — a disallowed rotation was exported at exit 0
    // =======================================================================================

    /// The audit's repro, at the level of the gate itself: an item declared
    /// `allowed_orientations: [0.0]` placed at 45°. The layout is geometrically perfect — one small
    /// rectangle alone in a large strip — so **every** check the export gate used to run passed it,
    /// and `sparrow-bpp` wrote JSON and SVG at exit `0`. Only the Python validator objected.
    ///
    /// A rotation outside the declared set is not a geometry bug, which is precisely why nothing
    /// geometric caught it: it is a *manufacturing* defect. An item declares
    /// `allowed_orientations` when the material has a grain, a pattern or a laminate direction, and
    /// a part cut at 45° from such a sheet is scrap however cleanly it was nested.
    #[test]
    fn export_gate_rejects_a_disallowed_rotation() -> Result<()> {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0]));
        let instance = import(&ext, None)?;

        // 45° = PI/4 rad, comfortably inside a 100 mm strip: no overlap, no border crossing.
        let sol = solution_of(&instance, 50.0, &[(0, PI / 4.0, 20.0, 20.0)]);

        // Sanity: the layout really is geometrically fine, so this test is about the rotation only.
        assert!(jagua_rs::entities::Layout::from_snapshot(&sol.layout_snapshot).is_feasible(),
            "test setup: the layout must be collision-free, or the gate would reject it for the wrong reason");

        let err = verify_spp_solution(&sol, &instance, None, "the final solution")
            .expect_err("a 45° placement of a 0°-only item must be refused");
        let msg = err.to_string();
        assert!(msg.contains("rotation"), "the error must name the problem: {msg}");
        assert!(msg.contains("45"), "the error must name the offending angle: {msg}");
        Ok(())
    }

    /// The same gate must not fire on a *legal* rotation, or it is a gate that gets switched off.
    #[test]
    fn export_gate_accepts_an_allowed_rotation() -> Result<()> {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0, 90.0]));
        let instance = import(&ext, None)?;
        for rot in [0.0, PI / 2.0] {
            let sol = solution_of(&instance, 50.0, &[(0, rot, 20.0, 20.0)]);
            verify_spp_solution(&sol, &instance, None, "the final solution")
                .unwrap_or_else(|e| panic!("{}° is declared allowed and must pass: {e}", rot.to_degrees()));
        }
        Ok(())
    }

    /// **Modulo 360°, and the wrap-around is load-bearing.** The engine exports the angle it
    /// happens to hold, so an item declared `[0, 180]` is routinely written out as `-180`. A gate
    /// that compared angles literally would reject correct solutions on every second run — the
    /// fastest possible route to the gate being disabled again.
    #[test]
    fn rotation_gate_is_modulo_360() -> Result<()> {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0, 180.0]));
        let instance = import(&ext, None)?;
        let item = &instance.items[0].0;
        for rot_deg in [-180.0f32, 180.0, 360.0, -360.0, 540.0] {
            assert!(rotation_is_allowed(item, rot_deg.to_radians(), ROTATION_TOL_RAD),
                "{rot_deg}° is 0° or 180° modulo 360 and must be accepted");
        }
        for rot_deg in [45.0f32, 90.0, -90.0, 179.0] {
            assert!(!rotation_is_allowed(item, rot_deg.to_radians(), ROTATION_TOL_RAD),
                "{rot_deg}° is neither 0° nor 180° modulo 360 and must be rejected");
        }
        Ok(())
    }

    /// A continuously rotatable item (`allowed_orientations` absent) may sit at **any** angle,
    /// including one nowhere near the sampling grid — the separator's own moves produce such angles
    /// routinely. The gate must let them all through.
    #[test]
    fn rotation_gate_lets_continuous_items_through() -> Result<()> {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, None);
        let instance = import(&ext, None)?;
        let item = &instance.items[0].0;
        assert!(matches!(item.allowed_rotation, RotationRange::Continuous),
            "test setup: an absent allowed_orientations must import as Continuous");
        for rot_deg in [0.0f32, 13.7, 45.0, 123.456, -77.0, 359.99] {
            assert!(rotation_is_allowed(item, rot_deg.to_radians(), ROTATION_TOL_RAD),
                "{rot_deg}° must be accepted for a continuously rotatable item");
        }
        // ...but not a NaN: a non-finite angle is a corrupt input, not a free rotation.
        assert!(!rotation_is_allowed(item, f32::NAN, ROTATION_TOL_RAD));
        Ok(())
    }

    /// The tolerance has to absorb `f32` round-trip noise (a nominal 180° comes back as 179.99998)
    /// without waving through an angle a human would call different.
    #[test]
    fn rotation_gate_tolerance_is_tight_but_not_brittle() -> Result<()> {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0]));
        let instance = import(&ext, None)?;
        let item = &instance.items[0].0;
        assert!(rotation_is_allowed(item, (0.0009f32).to_radians(), ROTATION_TOL_RAD),
            "float32 noise below the {ROTATION_TOL_DEG}° tolerance must pass");
        assert!(!rotation_is_allowed(item, (0.5f32).to_radians(), ROTATION_TOL_RAD),
            "half a degree is a real rotation and must not pass");
        Ok(())
    }

    /// The gate at the **warm-start import**, not just at export: a warm start is replayed as-is,
    /// so catching it at the door gives the far better error message and costs nothing.
    /// This is the audit's SPP-side twin of the BPP repro.
    #[test]
    fn spp_warm_start_with_a_disallowed_rotation_is_rejected() {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0]));
        let sol = ext_solution(50.0, vec![placement_rot(0, 45.0, 20.0, 20.0)]);
        let err = validate_spp_warm_start(&ext, &sol)
            .expect_err("a 45° placement of a 0°-only item must be refused at import");
        let msg = err.to_string();
        assert!(msg.contains("45"), "the error must name the angle: {msg}");
        assert!(msg.contains("allowed_orientations"), "the error must name the field: {msg}");
    }

    /// And it must still accept a legal warm start, or every warm start becomes unusable.
    #[test]
    fn spp_warm_start_with_an_allowed_rotation_is_accepted() {
        let ext = ext_instance_rot(2, 10.0, 5.0, 100.0, Some(vec![0.0, 90.0]));
        let sol = ext_solution(50.0, vec![placement_rot(0, 0.0, 1.0, 1.0), placement_rot(0, -270.0, 20.0, 20.0)]);
        validate_spp_warm_start(&ext, &sol)
            .expect("0° and -270° (= 90° mod 360) are both declared allowed");
    }

    // =======================================================================================
    // HIGH 1 — `allowed_orientations: []` means "fixed 0°", not "nothing allowed"
    // =======================================================================================

    /// jagua-rs' importer maps `[]` and `[0.0]` to the *same* `RotationRange::None`. The Rust gate
    /// has to agree, or the two halves of the same rule disagree about the same file.
    ///
    /// The Python validator's copy of this table is exercised by `validate_solution.py --self-test`
    /// (case 6), where `[]` used to be read as "no orientation is permitted", rejecting **both**
    /// engines' legitimate output and taking `nest_race.py` down with it.
    #[test]
    fn empty_orientation_list_means_fixed_zero() -> Result<()> {
        let empty = import(&ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![])), None)?;
        let zero = import(&ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0])), None)?;
        for inst in [&empty, &zero] {
            assert!(matches!(inst.items[0].0.allowed_rotation, RotationRange::None),
                "both [] and [0.0] must import as RotationRange::None");
        }
        // and the gate treats them identically
        for inst in [&empty, &zero] {
            let item = &inst.items[0].0;
            assert!(rotation_is_allowed(item, 0.0, ROTATION_TOL_RAD));
            assert!(rotation_is_allowed(item, (360.0f32).to_radians(), ROTATION_TOL_RAD));
            assert!(!rotation_is_allowed(item, (45.0f32).to_radians(), ROTATION_TOL_RAD));
        }
        Ok(())
    }

    /// The same table at the JSON level, which is what `validate_solution.py` implements and what
    /// `validate_spp_warm_start` uses. `[]` ⇒ fixed 0°, `null`/absent ⇒ continuous.
    #[test]
    fn ext_orientation_table_matches_jagua() {
        // absent / null: continuous
        for rot in [0.0f32, 45.0, -123.4] {
            assert!(ext_orientation_ok(rot, None, ROTATION_TOL_DEG), "{rot}° must pass for a continuous item");
        }
        // empty list: fixed 0°, exactly like [0.0] — NOT "nothing allowed"
        for (rot, expected) in [(0.0f32, true), (360.0, true), (-0.0005, true), (45.0, false), (90.0, false)] {
            assert_eq!(ext_orientation_ok(rot, Some(&[]), ROTATION_TOL_DEG), expected,
                "{rot}° against []: the empty list is jagua's fixed 0°");
            assert_eq!(ext_orientation_ok(rot, Some(&[0.0]), ROTATION_TOL_DEG), expected,
                "{rot}° against [0.0] must give the same answer as against []");
        }
        // a genuine discrete set
        assert!(ext_orientation_ok(-180.0, Some(&[0.0, 180.0]), ROTATION_TOL_DEG));
        assert!(!ext_orientation_ok(90.0, Some(&[0.0, 180.0]), ROTATION_TOL_DEG));
        // non-finite is never acceptable
        assert!(!ext_orientation_ok(f32::NAN, None, ROTATION_TOL_DEG));
        assert!(!ext_orientation_ok(f32::INFINITY, Some(&[0.0]), ROTATION_TOL_DEG));
    }

    /// An **empty** warm-start orientation list must not reject a 0° placement — that is the bug
    /// on the Rust side of the same table.
    #[test]
    fn warm_start_accepts_zero_for_an_empty_orientation_list() {
        let ext = ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![]));
        let sol = ext_solution(50.0, vec![placement_rot(0, 0.0, 20.0, 20.0)]);
        validate_spp_warm_start(&ext, &sol)
            .expect("[] means a fixed 0° orientation, so a 0° placement is legal");

        let bad = ext_solution(50.0, vec![placement_rot(0, 45.0, 20.0, 20.0)]);
        validate_spp_warm_start(&ext, &bad)
            .expect_err("[] means fixed 0°, so 45° must still be refused");
    }

    // =======================================================================================
    // HIGH 2 — the demand limit was bypassable by overflowing u64
    // =======================================================================================
    //
    // The binaries' `total_demand()` is duplicated (one per binary target, they are separate
    // crates), so it cannot be imported here. The behaviour is asserted end-to-end below in
    // `e2e_demand_overflow_*`; this unit test pins the arithmetic property that made it possible.

    /// `iter().sum::<u64>()` wraps in release. `u64::MAX + 1 == 0`, which is how a demand of
    /// 18 446 744 073 709 551 615 sailed past a `> 1_000_000` check and aborted the process deep
    /// inside the constructor (`no pole found` / `capacity overflow`, exit 134) instead of
    /// producing a message.
    ///
    /// This test documents *why* the fix has to be `checked_add`, and fails loudly if a future Rust
    /// release ever makes the plain sum panic instead (in which case the fix is still right, but
    /// this reasoning would need updating).
    #[test]
    fn the_unchecked_sum_really_does_wrap() {
        let demands = [u64::MAX, 1u64];
        // The wrapping sum is what the old gate saw:
        let wrapped = demands.iter().fold(0u64, |a, b| a.wrapping_add(*b));
        assert_eq!(wrapped, 0, "u64::MAX + 1 wraps to 0 — this is the bypass");
        assert!(wrapped <= 1_000_000, "...and 0 passes any sane demand limit");
        // The checked fold refuses instead:
        let checked = demands.iter().try_fold(0u64, |a, b| a.checked_add(*b));
        assert!(checked.is_none(), "checked_add must report the overflow rather than wrap");

        // ...and the function actually shipped — the one both binaries call — refuses it too. The
        // assertions above reason about `u64` arithmetic; this one is the regression test, because
        // it fails if `util::demand` ever goes back to a plain `sum()`.
        assert!(sparrow::util::demand::total_demand(demands.into_iter()).is_err(),
            "the shipped gate must reject the audit's overflow fixture");
    }

    /// The cap itself, on the real function: `MAX_TOTAL_DEMAND` is fine, one more is not — including
    /// when the excess is spread across items so that no single entry is over the per-item cap.
    #[test]
    fn the_demand_cap_boundary_is_exact() {
        use sparrow::util::demand::{total_demand, MAX_TOTAL_DEMAND};

        assert_eq!(total_demand([MAX_TOTAL_DEMAND].into_iter()).unwrap(), MAX_TOTAL_DEMAND,
            "the cap itself is a legal instance");
        assert!(total_demand([MAX_TOTAL_DEMAND + 1].into_iter()).is_err(),
            "one over the cap must be rejected");
        assert!(total_demand([MAX_TOTAL_DEMAND - 1, 2].into_iter()).is_err(),
            "the TOTAL is capped, not merely each item");
        // A lone u64::MAX overflows nothing, so only the per-item cap can catch it.
        assert!(total_demand([u64::MAX].into_iter()).is_err(),
            "a single enormous demand must be caught by the per-item cap");
    }

    // =======================================================================================
    // HIGH 3 — the pre-checks used a different rotation grid than the sampler
    // =======================================================================================

    /// **The audit's repro.** A 100 x 10 rectangle pre-rotated by 22.5°, in a 10.2 mm strip.
    ///
    /// The sampler's 16-step grid contains 337.5° (= -22.5°), which un-rotates the part into a
    /// 10.0 mm tall box — it fits, and `949ec94` (before the pre-check existed) solved this
    /// instance at exit 0. The pre-check's private 24-step grid has no sample within 7.5° of that,
    /// so it computed a minimum height of 23 mm and killed the run:
    ///
    /// ```text
    /// Error: the strip is only 10.2 mm high, but item 0 (23.0 mm) does not fit
    /// exit 1
    /// ```
    ///
    /// A rejection gate that reasons on a grid the engine does not use is answering a different
    /// question than the one it is gating.
    // The vertices below are the audit's, copied verbatim so that this test reproduces the reported
    // input exactly. They carry more decimals than an `f32` can hold (`88.561119` rounds to
    // `88.56112`), which is precisely why they must not be "tidied": the point of the fixture is to
    // be the same numbers the audit typed, and rounding them by hand would silently make it a
    // different, un-cited shape.
    #[allow(clippy::excessive_precision)]
    #[test]
    fn the_rotated_rectangle_of_the_audit_is_not_rejected() -> Result<()> {
        let ext = ExtSPInstance {
            name: "audit_rotation_grid".into(),
            items: vec![ExtItem {
                base: ExtBaseItem {
                    id: 0,
                    allowed_orientations: None, // continuous
                    shape: ExtShape::SimplePolygon(ExtSPolygon(vec![
                        (0.0, 0.0), (92.387953, 38.268343), (88.561119, 47.507138), (-3.826834, 9.238795),
                    ])),
                    min_quality: None,
                },
                demand: 1,
            }],
            strip_height: 10.2,
        };
        let instance = import(&ext, None)?;

        let too_tall = items_too_tall_for_strip(&instance);
        assert!(too_tall.is_empty(),
            "the 22.5°-rotated part fits the 10.2 mm strip at the sampler's -22.5°; the pre-check \
             rejected it with {too_tall:?} because it used a 24-step grid the engine never samples");
        check_spp_packability(&instance, None)
            .expect("this instance is solvable and the packability gate must let it through");
        Ok(())
    }

    /// The two grids are now literally the same object, so they cannot drift apart again.
    /// `ROT_N_SAMPLES` is the sampler's, and the pre-checks consume `candidate_rotations`.
    #[test]
    fn the_continuous_grid_is_the_samplers_grid() -> Result<()> {
        let instance = import(&ext_instance_rot(1, 10.0, 5.0, 100.0, None), None)?;
        let item = &instance.items[0].0;
        let rots: Vec<f32> = candidate_rotations(item).collect();
        assert_eq!(rots.len(), ROT_N_SAMPLES, "the continuous grid must have exactly ROT_N_SAMPLES steps");
        assert_eq!(ROT_N_SAMPLES, 16, "the sampler's grid is 16 steps; the pre-checks' old private copy was 24");
        // 22.5° must be on it — that is the angle the audit's part needs.
        let step = (2.0 * PI) / ROT_N_SAMPLES as f32;
        assert!(rots.iter().any(|r| (r - step).abs() < 1e-5), "22.5° (one step) must be a sample");
        Ok(())
    }

    /// The grid honours a **discrete** set exactly (no sampling, no rounding), and a fixed item
    /// gets exactly `0.0`.
    #[test]
    fn the_grid_honours_declared_orientations() -> Result<()> {
        let discrete = import(&ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0, 90.0, 270.0])), None)?;
        let rots: Vec<f32> = candidate_rotations(&discrete.items[0].0).collect();
        assert_eq!(rots.len(), 3, "a discrete set is used as declared: {rots:?}");

        let fixed = import(&ext_instance_rot(1, 10.0, 5.0, 100.0, Some(vec![0.0])), None)?;
        let rots: Vec<f32> = candidate_rotations(&fixed.items[0].0).collect();
        assert_eq!(rots, vec![0.0]);
        Ok(())
    }

    /// A **genuinely** unpackable item is still rejected — the fix must not turn the gate off.
    /// A 200 mm tall fixed-orientation part cannot enter a 50 mm strip at any angle it may use.
    #[test]
    fn a_genuinely_too_tall_item_is_still_rejected() -> Result<()> {
        let instance = import(&ext_instance_rot(1, 20.0, 200.0, 50.0, Some(vec![0.0])), None)?;
        let too_tall = items_too_tall_for_strip(&instance);
        assert_eq!(too_tall.len(), 1, "a 200 mm part in a 50 mm strip is unpackable and must be reported");
        check_spp_packability(&instance, None)
            .expect_err("the packability gate must still refuse a truly impossible instance");
        Ok(())
    }

    /// The walled pre-check gets the same treatment: a fixed-orientation part wider than one sheet
    /// is still refused, so `items_too_wide_for_sheet` did not lose its teeth either.
    #[test]
    fn a_genuinely_too_wide_item_is_still_rejected_for_sheets() -> Result<()> {
        let instance = import(&ext_instance_rot(1, 300.0, 20.0, 100.0, Some(vec![0.0])), None)?;
        let sheet = SheetConfig::new(100.0, 20.0, false);
        let too_wide = sparrow::optimizer::sheets::items_too_wide_for_sheet(&instance, &sheet);
        assert_eq!(too_wide.len(), 1, "a 300 mm part cannot fit a 100 mm sheet at 0°, its only orientation");
        Ok(())
    }

    // =======================================================================================
    // end-to-end: the release binary's exit code and what it left on disk
    // =======================================================================================
    //
    // `env!("CARGO_BIN_EXE_<name>")` is set by Cargo for every integration test, and points at the
    // binary **built for this test run** — in the same profile, in the same target directory. The
    // previous suite hardcoded `CARGO_MANIFEST_DIR/target/release/sparrow` and returned early when
    // the file was missing, so under a custom `CARGO_TARGET_DIR` it tested a stale binary and with
    // no binary at all it passed while asserting nothing. Neither is possible here: the path is a
    // compile-time constant that cannot be absent.

    mod end_to_end {
        use std::path::{Path, PathBuf};
        use std::process::Command;

        /// The `sparrow` binary built for **this** test run.
        fn sparrow_bin() -> PathBuf {
            PathBuf::from(env!("CARGO_BIN_EXE_sparrow"))
        }

        /// The `sparrow-bpp` binary built for **this** test run.
        fn sparrow_bpp_bin() -> PathBuf {
            PathBuf::from(env!("CARGO_BIN_EXE_sparrow-bpp"))
        }

        /// A private working directory for one end-to-end case.
        fn work_dir(tag: &str) -> PathBuf {
            let dir = std::env::temp_dir()
                .join(format!("sparrow-audit2-{}", std::process::id()))
                .join(format!("{tag}-{:x}", std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos()));
            std::fs::create_dir_all(&dir).expect("could not create the test working directory");
            dir
        }

        /// What a run left behind: its exit code and the names of the files in `output/`.
        struct Outcome {
            code: i32,
            files: Vec<String>,
            stderr: String,
        }

        impl Outcome {
            fn wrote_json(&self, instance_name: &str) -> bool {
                self.files.iter().any(|f| f == &format!("final_{instance_name}.json"))
            }
            /// Any `final_*.svg` at all — for BPP that is one file per bin, so the test asks about
            /// the *prefix*, not an exact name.
            fn wrote_final_svg(&self) -> bool {
                self.files.iter().any(|f| f.starts_with("final_") && f.ends_with(".svg"))
            }
        }

        fn files_in(dir: &Path) -> Vec<String> {
            let Ok(entries) = std::fs::read_dir(dir) else { return vec![] };
            let mut names: Vec<String> = entries.flatten()
                .filter(|e| e.path().is_file())
                .filter_map(|e| e.file_name().into_string().ok())
                .collect();
            names.sort();
            names
        }

        /// Runs a binary in a fresh directory with `input_json` as its `-i` file.
        fn run_with_json(bin: &Path, tag: &str, input_json: &str, args: &[&str]) -> Outcome {
            let dir = work_dir(tag);
            let input_path = dir.join("input.json");
            std::fs::write(&input_path, input_json).expect("could not write the test input");

            let out = Command::new(bin)
                .current_dir(&dir)
                .arg("-i").arg(&input_path)
                .args(args)
                .output()
                .unwrap_or_else(|e| panic!("could not run {}: {e}", bin.display()));

            let outcome = Outcome {
                code: out.status.code().unwrap_or(-1),
                files: files_in(&dir.join("output")),
                stderr: String::from_utf8_lossy(&out.stderr).into_owned(),
            };
            let _ = std::fs::remove_dir_all(&dir);
            outcome
        }

        // ---- CRITICAL: the disallowed warm-start rotation ---------------------------------

        /// **The audit's CRITICAL repro, verbatim.** Before the fix: exit `0`, `rotation: 45.0` in
        /// the exported JSON, an SVG next to it, and only `validate_solution.py` objecting.
        #[test]
        fn e2e_bpp_warm_start_rotation_is_refused() {
            const INPUT: &str = r#"{
              "name": "audit_bpp_warm_rotation",
              "items": [{"id": 0, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 5}}}],
              "bins": [{"id": 0,
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 30, "height": 30}},
                "zones": [], "stock": 1, "cost": 1}],
              "solution": {"cost": 1, "density": 0.0555556, "run_time_sec": 0,
                "layouts": [{"container_id": 0, "density": 0.0555556,
                  "placed_items": [{"item_id": 0, "transformation": {"rotation": 45.0, "translation": [10.0, 10.0]}}]}]}
            }"#;
            let out = run_with_json(&sparrow_bpp_bin(), "bpp-rot", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "a 45° placement of a 0°-only item must exit 1 (was 0): {}", out.stderr);
            assert!(!out.wrote_json("audit_bpp_warm_rotation"), "no JSON may be written: {:?}", out.files);
            assert!(!out.wrote_final_svg(), "no final SVG may be written either: {:?}", out.files);
        }

        /// The SPP twin of the same input.
        #[test]
        fn e2e_spp_warm_start_rotation_is_refused() {
            const INPUT: &str = r#"{
              "name": "audit_spp_rot",
              "items": [{"id": 0, "demand": 1, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 5}}}],
              "strip_height": 100.0,
              "solution": {"strip_width": 50.0, "density": 0.01, "run_time_sec": 0,
                "layout": {"container_id": 0, "density": 0.01,
                  "placed_items": [{"item_id": 0, "transformation": {"rotation": 45.0, "translation": [20.0, 20.0]}}]}}
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-rot", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "a 45° placement of a 0°-only item must exit 1: {}", out.stderr);
            assert!(!out.wrote_json("audit_spp_rot"), "no JSON may be written: {:?}", out.files);
            assert!(!out.wrote_final_svg(), "no final SVG may be written either: {:?}", out.files);
        }

        // ---- HIGH 4: no final artefacts survive a rejected run ------------------------------

        /// **The audit's SVG repro.** An overlapping warm start: the run correctly exits 1 and
        /// writes no JSON — and used to leave `final_audit_one.svg` of the rejected layout behind
        /// anyway, because the optimizer's listener wrote the final SVG *before* the export gate.
        /// A file called `final_*` that is not the final answer is worse than no file.
        #[test]
        fn e2e_rejected_spp_run_leaves_no_final_artefacts() {
            const INPUT: &str = r#"{
              "name": "audit_one",
              "items": [{"id": 0, "demand": 2, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 40, "height": 40}}}],
              "strip_height": 100.0,
              "solution": {"strip_width": 45.0, "density": 0.5, "run_time_sec": 0,
                "layout": {"container_id": 0, "density": 0.5, "placed_items": [
                  {"item_id": 0, "transformation": {"rotation": 0.0, "translation": [0.0, 0.0]}},
                  {"item_id": 0, "transformation": {"rotation": 0.0, "translation": [1.0, 1.0]}}]}}
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-overlap", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            // Release exits 1 through the export gate. A **debug** build never gets that far: the
            // `[EXPL] exploration must start from a feasible layout` debug assertion fires first
            // and the process dies at 101. Either way the run failed and — the point of this test —
            // left nothing behind; only the release path has a meaningful exit code to pin.
            assert_ne!(out.code, 0, "an overlapping warm start must never succeed: {}", out.stderr);
            if !cfg!(debug_assertions) {
                assert_eq!(out.code, 1, "in release the export gate must turn it into exit 1: {}", out.stderr);
            }
            assert!(!out.wrote_json("audit_one"), "no JSON may be written: {:?}", out.files);
            assert!(!out.wrote_final_svg(),
                "a rejected run must leave NO final_*.svg — the listener used to write it before the \
                 export gate ran: {:?}", out.files);
        }

        /// An incomplete warm start (1 of 2 items) goes the same way, SVG included.
        #[test]
        fn e2e_incomplete_spp_warm_start_leaves_no_final_artefacts() {
            const INPUT: &str = r#"{
              "name": "audit_incomplete",
              "items": [{"id": 0, "demand": 2, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 40, "height": 40}}}],
              "strip_height": 100.0,
              "solution": {"strip_width": 100.0, "density": 0.5, "run_time_sec": 0,
                "layout": {"container_id": 0, "density": 0.5, "placed_items": [
                  {"item_id": 0, "transformation": {"rotation": 0.0, "translation": [1.0, 1.0]}}]}}
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-incomplete", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "an incomplete warm start must exit 1: {}", out.stderr);
            assert!(!out.wrote_json("audit_incomplete"), "{:?}", out.files);
            assert!(!out.wrote_final_svg(), "no final SVG either: {:?}", out.files);
        }

        /// A rejected **BPP** run must be equally clean. BPP writes one SVG per bin, so the check
        /// is over the whole `final_*` family.
        #[test]
        fn e2e_rejected_bpp_run_leaves_no_final_artefacts() {
            // Two items, only one placed: the demand gate refuses it.
            const INPUT: &str = r#"{
              "name": "audit_bpp_incomplete",
              "items": [{"id": 0, "demand": 2, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 5}}}],
              "bins": [{"id": 0,
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 30, "height": 30}},
                "zones": [], "stock": 2, "cost": 1}],
              "solution": {"cost": 1, "density": 0.05, "run_time_sec": 0,
                "layouts": [{"container_id": 0, "density": 0.05,
                  "placed_items": [{"item_id": 0, "transformation": {"rotation": 0.0, "translation": [10.0, 10.0]}}]}]}
            }"#;
            let out = run_with_json(&sparrow_bpp_bin(), "bpp-incomplete", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "1 of 2 items placed must exit 1: {}", out.stderr);
            assert!(!out.wrote_json("audit_bpp_incomplete"), "{:?}", out.files);
            assert!(!out.wrote_final_svg(), "no final SVG may be written: {:?}", out.files);
        }

        /// **A successful run still writes both.** Without this, "leaves no artefacts" could be
        /// satisfied by never writing anything at all.
        #[test]
        fn e2e_successful_spp_run_writes_json_and_svg() {
            const INPUT: &str = r#"{
              "name": "audit_ok",
              "items": [{"id": 0, "demand": 2, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 10}}}],
              "strip_height": 100.0
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-ok", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 0, "this instance is trivially packable: {}", out.stderr);
            assert!(out.wrote_json("audit_ok"), "the JSON must be written: {:?}", out.files);
            assert!(out.wrote_final_svg(), "the final SVG must be written: {:?}", out.files);
        }

        /// The BPP twin: a successful run writes JSON and at least one bin SVG.
        #[test]
        fn e2e_successful_bpp_run_writes_json_and_svg() {
            const INPUT: &str = r#"{
              "name": "audit_bpp_ok",
              "items": [{"id": 0, "demand": 2, "allowed_orientations": [0.0],
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 5}}}],
              "bins": [{"id": 0,
                "shape": {"type": "rectangle", "data": {"x_min": 0, "y_min": 0, "width": 30, "height": 30}},
                "zones": [], "stock": 2, "cost": 1}]
            }"#;
            let out = run_with_json(&sparrow_bpp_bin(), "bpp-ok", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 0, "this instance is trivially packable: {}", out.stderr);
            assert!(out.wrote_json("audit_bpp_ok"), "the JSON must be written: {:?}", out.files);
            assert!(out.wrote_final_svg(), "at least one bin SVG must be written: {:?}", out.files);
        }

        // ---- HIGH 2: the demand overflow ----------------------------------------------------

        /// **The audit's overflow repro.** `u64::MAX + 1` wraps to 0, so the demand gate never
        /// fired and the process aborted (exit 134, `no pole found`) instead of reporting the
        /// input. Both binaries must now exit 1 with a message and write nothing.
        #[test]
        fn e2e_demand_overflow_is_a_clean_error_spp() {
            const INPUT: &str = r#"{
              "name": "audit_demand_overflow",
              "items": [
                {"id": 0, "demand": 18446744073709551615, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}},
                {"id": 1, "demand": 1, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}}
              ],
              "strip_height": 100.0
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-overflow", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "an overflowing demand must be a clean error, not an abort (was 134)");
            assert!(!out.wrote_json("audit_demand_overflow"), "{:?}", out.files);
            assert!(!out.wrote_final_svg(), "{:?}", out.files);
        }

        #[test]
        fn e2e_demand_overflow_is_a_clean_error_bpp() {
            const INPUT: &str = r#"{
              "name": "audit_demand_overflow",
              "items": [
                {"id": 0, "demand": 18446744073709551615, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}},
                {"id": 1, "demand": 1, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}}
              ],
              "strip_height": 100.0
            }"#;
            let out = run_with_json(&sparrow_bpp_bin(), "bpp-overflow", INPUT,
                &["--bin", "100x100", "-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "an overflowing demand must be a clean error, not an abort (was 134)");
            assert!(!out.wrote_json("audit_demand_overflow"), "{:?}", out.files);
        }

        /// `MAX_TOTAL_DEMAND + 1`, spread over two items so neither one alone is over the per-item
        /// cap: the *total* has to be gated too, not just each entry.
        #[test]
        fn e2e_demand_just_over_the_cap_is_rejected() {
            // MAX_TOTAL_DEMAND is 1_000_000; 600_000 + 400_001 = 1_000_001.
            const INPUT: &str = r#"{
              "name": "audit_demand_cap",
              "items": [
                {"id": 0, "demand": 600000, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}},
                {"id": 1, "demand": 400001, "allowed_orientations": [0.0],
                 "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}}
              ],
              "strip_height": 100.0
            }"#;
            let out = run_with_json(&sparrow_bin(), "spp-cap", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "1 000 001 > MAX_TOTAL_DEMAND must be rejected: {}", out.stderr);
            assert!(!out.wrote_json("audit_demand_cap"), "{:?}", out.files);

            let out = run_with_json(&sparrow_bpp_bin(), "bpp-cap", INPUT,
                &["--bin", "100x100", "-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 1, "the BPP binary must apply the same cap: {}", out.stderr);
        }

        // ---- HIGH 3: the rotation grid, end to end ------------------------------------------

        /// **The audit's rotation-grid repro, end to end.** `2c48460`:
        /// `Error: the strip is only 10.2 mm high, but item 0 (23.0 mm) does not fit`, exit 1.
        /// `949ec94` (before the pre-check): exit 0 with a solution. It must solve again.
        #[test]
        fn e2e_rotated_rectangle_is_solved_again() {
            const INPUT: &str = r#"{
              "name": "audit_rotation_grid",
              "items": [{"id": 0, "demand": 1,
                "shape": {"type": "simple_polygon", "data": [
                  [0.0, 0.0], [92.387953, 38.268343], [88.561119, 47.507138], [-3.826834, 9.238795]]}}],
              "strip_height": 10.2
            }"#;
            let out = run_with_json(&sparrow_bin(), "rot-grid", INPUT, &["-e", "0", "-c", "0", "-s", "42"]);
            assert_eq!(out.code, 0,
                "the 22.5°-rotated part fits the 10.2 mm strip at the sampler's -22.5°; the run must \
                 not be rejected by a pre-check on a different grid. stderr: {}", out.stderr);
            assert!(out.wrote_json("audit_rotation_grid"), "and it must produce a solution: {:?}", out.files);
        }
    }
}
