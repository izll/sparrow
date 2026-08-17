//! Phase 2a integration tests for the bin packing (BPP) I/O layer.
//!
//! Covers the three responsibilities of [`sparrow::util::bpp_io`]:
//! * parsing `--bin WxH[:stock[:cost]]` specs,
//! * turning a strip packing instance + bin specs into an importable [`ExtBPInstance`],
//! * the export → import round trip of a solution (jagua-rs' own `bpp` importer is unimplemented).

#[cfg(test)]
mod bpp_io_tests {
    use anyhow::Result;
    use jagua_rs::entities::Layout;
    use jagua_rs::io::import::Importer;
    use jagua_rs::probs::bpp::entities::BPInstance;
    use jagua_rs::probs::bpp::io::ext_repr::ExtBPInstance;
    use rand::SeedableRng;
    use rand::rngs::Xoshiro256PlusPlus;
    use sparrow::config::DEFAULT_SPARROW_CONFIG;
    use sparrow::consts::LBF_SAMPLE_CONFIG;
    use sparrow::optimizer::bpp::BPLBFBuilder;
    use sparrow::util::bpp_io::{self, BinSpec, DEFAULT_BIN_COST, DEFAULT_BIN_STOCK};
    use sparrow::util::io;
    use std::path::Path;
    use std::str::FromStr;

    const INSTANCE_PATH: &str = "data/input/swim.json";
    /// Same bin as in `tests/bpp_tests.rs`: ~40 % of the total swim item area, so >= 3 bins are needed.
    const BIN_SPEC: &str = "3200x3200:100:1";
    const SEED: u64 = 0;

    fn importer() -> Importer {
        let config = DEFAULT_SPARROW_CONFIG;
        Importer::new(
            config.cde_config,
            config.poly_simpl_tolerance,
            config.min_item_separation,
            config.narrow_concavity_cutoff_ratio,
        )
    }

    /// Reads the swim SPP instance and combines it with `BIN_SPEC` into a BPP instance.
    fn build_ext_bp_instance() -> Result<ExtBPInstance> {
        let bins = vec![BinSpec::from_str(BIN_SPEC).unwrap()];
        let (ext_instance, ext_solution) = bpp_io::read_bpp_input(Path::new(INSTANCE_PATH), &bins)?;
        assert!(ext_solution.is_none(), "a plain SPP instance carries no solution");
        Ok(ext_instance)
    }

    fn build_bp_instance() -> Result<BPInstance> {
        let ext_instance = build_ext_bp_instance()?;
        jagua_rs::probs::bpp::io::import_instance(&importer(), &ext_instance)
    }

    /// (a) `BinSpec` parses all four accepted forms and rejects malformed input.
    #[test]
    fn bin_spec_parsing() {
        assert_eq!(
            BinSpec::from_str("100x200").unwrap(),
            BinSpec { width: 100.0, height: 200.0, stock: DEFAULT_BIN_STOCK, cost: DEFAULT_BIN_COST }
        );
        assert_eq!(
            BinSpec::from_str("100x200:7").unwrap(),
            BinSpec { width: 100.0, height: 200.0, stock: 7, cost: DEFAULT_BIN_COST }
        );
        assert_eq!(
            BinSpec::from_str("100x200:7:3").unwrap(),
            BinSpec { width: 100.0, height: 200.0, stock: 7, cost: 3 }
        );
        // Floats and the upper-case separator are accepted
        assert_eq!(
            BinSpec::from_str("3200.5X1000.25:2:5").unwrap(),
            BinSpec { width: 3200.5, height: 1000.25, stock: 2, cost: 5 }
        );

        // Malformed specs
        for bad in ["", "100", "100x", "x200", "axb", "100x200:", "100x200:0", "100x200:1:2:3", "-1x200", "0x200"] {
            assert!(BinSpec::from_str(bad).is_err(), "'{bad}' should not parse as a BinSpec");
        }

        // ids are assigned 0.. in the given order
        let specs = ["10x10", "20x20:5", "30x30:5:2"].map(|s| BinSpec::from_str(s).unwrap());
        for (id, spec) in specs.iter().enumerate() {
            assert_eq!(spec.to_ext_bin(id).base.id, id as u64);
        }
    }

    /// An SPP instance without any `--bin` must be rejected with a helpful error.
    #[test]
    fn sp_instance_without_bins_errors() {
        let res = bpp_io::read_bpp_input(Path::new(INSTANCE_PATH), &[]);
        assert!(res.is_err(), "an SPP instance without --bin should be rejected");
    }

    /// (b) SP instance + bins → `ExtBPInstance` → `import_instance` succeeds and keeps all items.
    #[test]
    fn sp_instance_plus_bins_imports() -> Result<()> {
        let (sp_instance, _) = io::read_spp_input(Path::new(INSTANCE_PATH))?;
        let ext_instance = build_ext_bp_instance()?;

        assert_eq!(ext_instance.name, sp_instance.name);
        assert_eq!(ext_instance.items.len(), sp_instance.items.len());
        assert_eq!(ext_instance.bins.len(), 1);
        assert_eq!(ext_instance.bins[0].stock, 100);
        assert_eq!(ext_instance.bins[0].cost, 1);

        let instance = jagua_rs::probs::bpp::io::import_instance(&importer(), &ext_instance)?;
        assert_eq!(instance.bins.len(), 1);
        assert_eq!(
            instance.total_item_qty(),
            sp_instance.items.iter().map(|i| i.demand as usize).sum::<usize>()
        );
        Ok(())
    }

    /// (c) Round trip: LBF solution → `export` → `import_bp_solution` must reproduce it exactly.
    #[test]
    fn solution_round_trip() -> Result<()> {
        let instance = build_bp_instance()?;
        let rng = Xoshiro256PlusPlus::seed_from_u64(SEED);
        let builder = BPLBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG).construct()?;
        let original = builder.prob.save();

        assert!(original.layout_snapshots.len() >= 3, "expected >= 3 bins from LBF");

        let ext_solution = bpp_io::export_bp(&instance, &original);
        let reimported = bpp_io::import_bp_solution(&instance, &ext_solution)?;

        // Same cost and same number of layouts
        assert_eq!(reimported.cost(&instance), original.cost(&instance), "cost changed over the round trip");
        assert_eq!(reimported.layout_snapshots.len(), original.layout_snapshots.len(), "number of layouts changed");

        // Same overall density (up to float noise from the transformation round trip)
        let dens_diff = (reimported.density(&instance) - original.density(&instance)).abs();
        assert!(dens_diff < 1e-4, "density changed by {dens_diff} over the round trip");

        // Every re-imported layout holds as many items as the corresponding original one, and is feasible.
        let orig_counts: Vec<usize> = original.layout_snapshots.values().map(|ls| ls.placed_items.len()).collect();
        let reimp_counts: Vec<usize> = reimported.layout_snapshots.values().map(|ls| ls.placed_items.len()).collect();
        assert_eq!(reimp_counts, orig_counts, "per-layout item counts changed over the round trip");

        for (lkey, snapshot) in reimported.layout_snapshots.iter() {
            let layout = Layout::from_snapshot(snapshot);
            assert!(layout.is_feasible(), "re-imported layout {lkey:?} is not feasible");
        }

        // The bin ids (and thus the container types) are preserved, in order.
        let orig_bins: Vec<usize> = original.layout_snapshots.values().map(|ls| ls.container.id).collect();
        let reimp_bins: Vec<usize> = reimported.layout_snapshots.values().map(|ls| ls.container.id).collect();
        assert_eq!(reimp_bins, orig_bins, "bin ids changed over the round trip");

        println!("[TEST] round trip ok: {}", bpp_io::summarize(&reimported, &instance));
        Ok(())
    }

    /// An `ExtBPOutput` file (instance + solution) must be re-readable as a warm start.
    #[test]
    fn ext_bp_output_round_trip_through_file() -> Result<()> {
        let ext_instance = build_ext_bp_instance()?;
        let instance = jagua_rs::probs::bpp::io::import_instance(&importer(), &ext_instance)?;
        let rng = Xoshiro256PlusPlus::seed_from_u64(SEED);
        let solution = BPLBFBuilder::new(instance.clone(), rng, LBF_SAMPLE_CONFIG).construct()?.prob.save();

        let output = bpp_io::ExtBPOutput {
            instance: ext_instance,
            solution: bpp_io::export_bp(&instance, &solution),
        };

        let dir = std::env::temp_dir().join("sparrow_bpp_io_tests");
        std::fs::create_dir_all(&dir)?;
        let path = dir.join("final_swim_bpp.json");
        bpp_io::write_bp_json(&output, &path)?;

        // Reading it back must yield both the instance and the solution (warm start path)
        let (read_instance, read_solution) = bpp_io::read_bpp_input(&path, &[])?;
        assert_eq!(read_instance.bins.len(), 1);
        let read_solution = read_solution.expect("an ExtBPOutput file carries a solution");
        assert_eq!(read_solution.cost, solution.cost(&instance));

        let reimported = bpp_io::import_bp_solution(&instance, &read_solution)?;
        assert_eq!(reimported.cost(&instance), solution.cost(&instance));
        assert_eq!(reimported.layout_snapshots.len(), solution.layout_snapshots.len());

        std::fs::remove_file(&path)?;
        Ok(())
    }
}
