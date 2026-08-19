//! Regression test for the "does not fit" parity between the strip (SPP) and bin (BPP) engines.
//!
//! Background: with `min_item_separation = s`, jagua-rs inflates every item by `s/2` and deflates every container
//! (strip and bin alike) by `s/2`. A 992 mm tall item therefore needs a container of at least 992 + 2*(s/2) + 2*(s/2)
//! = 1002 mm (+ epsilon) when `s = 5`. Both engines must reach the same verdict for identical inputs; historically
//! only `sparrow-bpp` honoured the `SPARROW_MIN_SEP` env var, which made the two binaries *look* inconsistent.
#[cfg(test)]
mod fit_parity_tests {
    use jagua_rs::entities::{Container, Layout};
    use jagua_rs::geometry::shape_modification::ShapeModifyConfig;
    use jagua_rs::io::ext_repr::{ExtContainer, ExtShape, ExtSPolygon};
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::spp::entities::Strip;
    use rand::rngs::Xoshiro256PlusPlus;
    use rand::SeedableRng;
    use sparrow::config::DEFAULT_SPARROW_CONFIG;
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::eval::lbf_evaluator::LBFEvaluator;
    use sparrow::eval::sample_eval::SampleEval;
    use sparrow::sample::search::search_placement;
    use sparrow::util::io::resolve_min_item_separation;
    use test_case::test_case;

    const BIN_W: f32 = 1995.0;
    const ITEM_W: f32 = 600.0;
    const ITEM_H: f32 = 992.0;

    fn importer(min_sep: Option<f32>) -> Importer {
        let c = DEFAULT_SPARROW_CONFIG;
        Importer::new(c.cde_config, c.poly_simpl_tolerance, min_sep, c.narrow_concavity_cutoff_ratio)
    }

    fn item(importer: &Importer) -> jagua_rs::entities::Item {
        let ext_item = jagua_rs::io::ext_repr::ExtItem {
            id: 0,
            allowed_orientations: Some(vec![0.0, 180.0]),
            shape: ExtShape::SimplePolygon(ExtSPolygon(vec![(0.0, 0.0), (ITEM_W, 0.0), (ITEM_W, ITEM_H), (0.0, ITEM_H)])),
            min_quality: None,
        };
        importer.import_item(&ext_item).unwrap()
    }

    /// Strip container exactly as `jagua_rs::probs::spp::io::import_instance` builds it (only the offset is applied).
    fn strip_container(importer: &Importer, height: f32) -> Container {
        let strip = Strip::new(
            height,
            importer.cde_config,
            ShapeModifyConfig { offset: importer.shape_modify_config.offset, simplify_tolerance: None, narrow_concavity_cutoff: None },
            BIN_W,
        ).unwrap();
        Container::from(strip)
    }

    /// Bin container exactly as `jagua_rs::probs::bpp::io::import_instance` builds it (rectangular `--bin`).
    fn bin_container(importer: &Importer, height: f32) -> Container {
        let ext = ExtContainer {
            id: 0,
            shape: ExtShape::Rectangle { x_min: 0.0, y_min: 0.0, width: BIN_W, height },
            zones: vec![],
        };
        importer.import_container(&ext).unwrap()
    }

    /// Whether the LBF search finds a collision-free placement for the item in an empty layout of `container`.
    fn fits(container: Container, item: &jagua_rs::entities::Item) -> bool {
        let layout = Layout::new(container);
        let evaluator = LBFEvaluator::new(&layout, item);
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(42);
        let (best, _) = search_placement(&layout, item, None, evaluator, LBF_SAMPLE_CONFIG, &mut rng);
        matches!(best, Some((_, SampleEval::Clear { .. })))
    }

    // container height, min separation, expected verdict (same for strip and bin)
    #[test_case(995.0, None, true; "no separation: 992 fits in 995")]
    #[test_case(995.0, Some(5.0), false; "5 mm separation: 992 + 5 + 5 does not fit in 995")]
    #[test_case(1001.0, Some(5.0), false; "5 mm separation: just below the 1002 threshold")]
    #[test_case(1003.0, Some(5.0), true; "5 mm separation: just above the 1002 threshold")]
    #[test_case(1005.0, Some(5.0), true; "5 mm separation: 1005 fits")]
    fn strip_and_bin_reach_the_same_verdict(height: f32, min_sep: Option<f32>, expected: bool) {
        let importer = importer(min_sep);
        let item = item(&importer);
        let strip_verdict = fits(strip_container(&importer, height), &item);
        let bin_verdict = fits(bin_container(&importer, height), &item);
        assert_eq!(strip_verdict, bin_verdict, "strip ({strip_verdict}) and bin ({bin_verdict}) verdicts differ for height {height}, min_sep {min_sep:?}");
        assert_eq!(strip_verdict, expected, "unexpected verdict for height {height}, min_sep {min_sep:?}");
    }

    #[test]
    fn min_sep_resolution_prefers_cli_then_env_then_default() {
        // (the env var is process-global; only exercise the CLI/default paths here)
        // The resolver returns a `Result` now: a non-finite value is an error rather than a
        // silent "separation disabled" (a NaN fails every `v > 0.0` test).
        assert_eq!(resolve_min_item_separation(Some(5.0), None).unwrap(), Some(5.0));
        assert_eq!(resolve_min_item_separation(Some(0.0), Some(3.0)).unwrap(), None, "non-positive CLI value disables the separation");
        assert!(resolve_min_item_separation(Some(f32::NAN), Some(3.0)).is_err(), "a NaN must be an error, not a silent opt-out");
        if std::env::var("SPARROW_MIN_SEP").is_err() {
            assert_eq!(resolve_min_item_separation(None, Some(3.0)).unwrap(), Some(3.0));
            assert_eq!(resolve_min_item_separation(None, None).unwrap(), None);
        }
    }
}
