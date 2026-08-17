# Bin packing (BPP) support — design & work breakdown

Goal: next to the existing strip packing (SPP) pipeline, add a **bin packing** pipeline that packs all items into
copies of fixed-size bins (sheets) using `jagua_rs::probs::bpp` (`BPInstance`, `BPProblem`, `BPSolution`,
`Bin`, `LayKey`, `BPPlacement`, `BPLayoutType`) and **minimises the number (cost) of bins used**, with the
remainder consolidated in the last bin as a secondary goal.

Motivation and the full feasibility analysis (mapping tables, file:line references) live outside this repo
(MADisoCAD `docs/nesting/sparrow-fork-analysis.md`); this file is the self-contained spec for implementers.

## Non-negotiables

1. **The SPP pipeline must keep working unchanged**: `cargo test` (spp integration tests) and the `sparrow` binary
   must behave exactly as before. BPP is *added*, nothing in `src/optimizer/{separator,worker,explore,compress,lbf}.rs`,
   `src/sample/*`, `src/eval/*`, `src/quantify/*` changes semantics. Refactors that only extract shared helpers are OK
   if the SPP tests still pass.
2. **Reuse the container-agnostic core as-is**: `sample::search::search_placement(&Layout, …)`,
   `eval::sep_evaluator::SeparationEvaluator`, `eval::specialized_jaguars_pipeline`, `quantify::*`,
   `quantify::tracker::CollisionTracker` (one per layout). Do not fork these; call them.
3. **wasm32 must stay compilable** conceptually: no `std::time::Instant` (use `jagua_rs::Instant`), no `ctrlc`
   outside `cfg(not(target_arch = "wasm32"))`, rayon usage identical in spirit to the SPP path.
4. Determinism for a fixed seed + worker count + iteration count: worker results merged in worker-index order
   (`min_by_key` first-min), per-worker RNGs derived from the master RNG, no `HashMap` iteration order dependence
   (use `SlotMap`/`SecondaryMap`/`Vec`).
5. Rust stable 1.90, edition 2024, no new heavy dependencies. Enable jagua-rs feature `bpp` **in addition to** `spp`.
6. Everything documented with `///` where public; `debug_assert!`s mirroring the SPP ones (tracker matches layout,
   losses non-negative, feasibility on success).

## Module layout (file ownership)

```
src/optimizer/bpp/mod.rs        optimize_bpp(): orchestration (LBF -> explore -> compress), BPPhase configs
src/optimizer/bpp/separator.rs  BPSeparator: BPProblem + per-layout trackers + workers + thread pool
src/optimizer/bpp/worker.rs     BPSeparatorWorker: move_items over all layouts (intra-layout moves)
src/optimizer/bpp/lbf.rs        BPLBFBuilder: constructive first solution into bins
src/optimizer/bpp/explore.rs    bin-count reduction loop (close least-filled bin -> scatter -> separate), disruption
src/optimizer/bpp/compress.rs   remainder consolidation in the last bin (v1 may be a no-op that returns the input)
src/util/bpp_io.rs              read ExtBPInstance / build one from ExtSPInstance + --bin WxH[:stock[:cost]],
                                import_solution (hand-written), export, BPSvgExporter, BPSolutionListener
src/bpp_main.rs                 `sparrow-bpp` binary (clap), mirrors src/main.rs incl. -p parallel runs
src/config.rs                   + BPConfig / BPExplorationConfig / BPCompressionConfig + DEFAULT_BPP_CONFIG
tests/bpp_tests.rs              integration tests
docs/bpp.md                     user docs (CLI, JSON format, how the algorithm works)
```

`SPSolution`-typed things (`util::listener::SolutionListener`, `util::svg_exporter::SvgExporter`) are **not** changed;
BPP gets its own `BPSolutionListener` trait + `BPSvgExporter` in `bpp_io.rs`.

## Core data model

```rust
pub struct BPSeparator {
    pub instance: BPInstance,
    pub prob: BPProblem,
    /// One collision tracker per open layout (GLS weights live here, per layout)
    pub trackers: SecondaryMap<LayKey, CollisionTracker>,
    pub rng: Xoshiro256PlusPlus,
    pub workers: Vec<BPSeparatorWorker>,
    pub config: SeparatorConfig,          // reuse the SPP SeparatorConfig as-is
    pub thread_pool: Option<ThreadPool>,
}
pub struct BPSeparatorWorker { instance, prob: BPProblem, trackers: SecondaryMap<LayKey, CollisionTracker>, rng, sample_config }
pub type BPSnapshot = (BPSolution, SecondaryMap<LayKey, CTSnapshot>);
```

Rules:
* `BPProblem::restore(&sol) -> bool`: if `true` (layout keys changed) **rebuild all trackers** from the layouts;
  otherwise `restore_but_keep_weights` per layout (keys stable). Wrap this in `BPSeparator::rollback(&sol, Option<&trackers>)`.
* `BPProblem::place_item(BPLayoutType::Closed{bin_id})` does **not** check stock: always check
  `prob.bin_stock_qtys[bin_id] > 0` first.
* `remove_item` auto-closes empty layouts → after any removal, drop the tracker if the layout vanished.
* Total loss = Σ over trackers `get_total_loss()`; feasible ⇔ total loss == 0.
* `move_item(lkey, pk, dt) -> PItemKey`: remove + place into the **same** layout (`Open(lkey)`), then
  `trackers[lkey].register_item_move(&prob.layouts[lkey], old_pk, new_pk)`.

## Algorithms

### separate() — identical to SPP Algorithm 9
Same strike / no-improvement / GLS weight update logic as `optimizer/separator.rs::separate`, but over the sum
of all layouts' losses; `move_items_multi` runs the workers in parallel (each loads master solution + trackers,
runs `move_items`, master takes the worker with the lowest total **weighted** loss, `min_by_key` first-min).

### worker.move_items() — SPP Algorithm 5 per layout
Candidates = all `(lkey, pk)` with `trackers[lkey].get_loss(pk) > 0`, shuffled with the worker RNG.
For each still-colliding candidate: `SeparationEvaluator::new(&prob.layouts[lkey], item, pk, &trackers[lkey])`,
`search_placement(&prob.layouts[lkey], item, Some(pk), evaluator, sample_config, rng)`, then `move_item`.
v1: intra-layout moves only (cross-layout moves are a later extension; document the hook).

### BPLBFBuilder — constructive start
Items sorted like SPP LBF (convex-hull area × diameter, descending, expanded by demand). For each item, try the open
layouts in creation order with `LBFEvaluator` + `search_placement(&layout, item, None, …)`; first `Clear` sample wins
(`BPLayoutType::Open(lkey)`). If none fits, open a new bin: choose the bin type with stock > 0 and the lowest
`cost / container.area()` (v1: also fine to take the first with stock), `BPLayoutType::Closed{bin_id}`.
If no stock is left anywhere → error (`anyhow::bail!`) reported by the CLI.

### exploration_phase — bin-count reduction (replaces the strip-shrink loop)
```
best = current feasible solution (from LBF)
loop until terminator.kill() or strikes exhausted:
    pick target = open layout with the lowest density (ties: lowest LayKey order → deterministic)
    close_bin_and_scatter(target):        # analogue of change_strip_width
        collect its (item_id, d_transf), remove all its items (layout closes automatically, stock returns)
        for each item (largest first): choose a destination layout (v1: round-robin over remaining layouts,
            starting from the least dense), sample a random position with UniformBBoxSampler over the destination
            container bbox (feasible rotation), place with Open(lkey)   -> overlaps allowed
        rebuild trackers of the affected layouts (weights reset), reseed workers
    (sol, trackers) = separate()
    if total loss == 0:  best = sol (fewer bins!), reset strikes, report ExplFeas, continue with best
    else:                strikes += 1, report ExplInfeas, keep the infeasible solution in a small pool,
                         rollback to best (or to a pooled infeasible one + disrupt, like SPP explore.rs) and retry
                         (a different target layout / different scatter RNG)
```
`max_conseq_failed_attempts` (early termination) as in SPP. Optional v1.5: SPP-style `disrupt_solution` per layout
(swap two large items) — port `optimizer/explore.rs::disrupt_solution` with a layout parameter.

### compression_phase — remainder consolidation (secondary objective)
v1: return the best solution unchanged **but** implement the tie-break objective in the reporting: for the final
solution also compute and log per-bin density and the *used width* of the last bin. Stretch goal (separate task):
run the existing SPP separator on the least-dense bin's items with `Strip{fixed_height = bin bbox height, width =
bin bbox width}` to push its content to the left so the remainder is a single rectangle, then write the placements
back into that layout (offset by the container bbox origin) — only if the result is feasible.

## I/O and CLI

* Input JSON: `ExtBPInstance` (`jagua_rs::probs::bpp::io::ext_repr`), **or** an `ExtSPInstance` (items only) plus
  `--bin WxH[:stock[:cost]]` (repeatable) → build `ExtBPInstance` with rectangular bins (`ExtShape::Rectangle`,
  ids 0..). Default stock = large (e.g. `usize::MAX / 2` is not serialisable-friendly; use 1000), default cost = 1.
* Warm start: `ExtBPOutput { instance: ExtBPInstance, solution: ExtBPSolution }` — write our own
  `import_bp_solution` (jagua's is `unimplemented!()`): per `ExtLayout`, first item `Closed{bin_id = container_id}`,
  rest `Open(lkey)`, transformations via `jagua_rs::io::import::ext_to_int_transformation(&ext, &item.shape_orig.pre_transform)`
  (mirror `spp/io/import.rs::import_solution`).
* Output: `output/final_{name}.json` (`ExtBPOutput`), one SVG per bin `output/final_{name}_bin{idx}.svg`
  (`jagua_rs::io::svg::s_layout_to_svg(&snapshot, &instance, DRAW_OPTIONS, name)`), plus intermediate SVG dirs
  like SPP when not `only_final_svg`.
* CLI (`sparrow-bpp`): `-i`, `-t | -e -c`, `-x`, `-s`, `-p` (parallel runs, best = lowest `cost`, tie → highest
  density of the last bin), `--bin WxH[:stock[:cost]]`. Log the same style as SPP (`[BPLBF]`, `[BPEXPL]`, `[BPSEP]`).

## Tests (tests/bpp_tests.rs)

* `swim.json` items + `--bin`-style rectangular bins (pick W×H so that the LBF needs ≥ 3 bins, e.g. bin area ≈ 40 %
  of total item area), seed 0, 10 s + 5 s: final solution `layouts.iter().all(|l| l.is_feasible())`, all demand placed,
  `bin_cost` ≤ LBF's bin count.
* Round trip: export → import_bp_solution → same cost/density.
* Determinism smoke test: two runs with the same seed and an iteration-bounded terminator give the same cost.
* All under `cargo test` (debug assertions on).

## Phases / agents

1. **Foundation** — `bpp/{mod,separator,worker,lbf}.rs`, `config.rs` additions, `Cargo.toml` feature. Must compile,
   `cargo test` (spp) green, plus a unit test: LBF into 2000×1000 bins from swim items → feasible; scatter one bin →
   `separate()` reduces loss (ideally to 0) — proves the core.
2. **In parallel**: (a) `util/bpp_io.rs` + `src/bpp_main.rs` + Cargo `[[bin]]`; (b) `bpp/explore.rs`, `bpp/compress.rs`,
   `optimize_bpp()` orchestration.
3. **Integration** — tests, docs (`docs/bpp.md`, README section), clippy clean, measurement on swim/shirts with
   `--bin`, and on the MADisoCAD 112-part case if an input file is available.
