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

---

## Status (2026-08-18)

Phases 1–**4** are complete. `cargo test` is green (18 tests: 3 SPP integration + 15 BPP), `cargo
clippy --all-targets` is clean for all BPP files (the only two remaining warnings are pre-existing
`collapsible_if`s in the SPP `separator.rs`), and `cargo build --release` (also with `--features
only_final_svg`) succeeds.

### Phase 4 — cross-layout consolidation ("pack-down")

The problem phase 4 solved: on the real 112-part instance the exploration phase burned its entire
budget on 4 → 3 reductions that are *area-infeasible* (3 bins of 2000x1000 would need 94.3 % density),
and because every move was intra-layout, the slack stayed spread evenly over all four bins (65.9 /
66.1 / 75.0 / 75.9 %). The SPP pipeline on the same parts concentrates the remainder at the end of the
strip; the BPP pipeline had no way to.

| Item | Where |
| --- | --- |
| Area bound `required_density_for_reduction` + early return | `src/optimizer/bpp/explore.rs` |
| `max_reduction_density` (default 0.90) | `src/config.rs` (`BPExplorationConfig`) |
| Unused exploration time handed to compression | `src/optimizer/bpp/mod.rs` |
| `pack_down` — cross-layout item migration out of the sparsest bin | `src/optimizer/bpp/compress.rs` |
| `compression_phase` — pack-down ⇄ consolidation alternation with convergence check | `src/optimizer/bpp/compress.rs` |
| `BPSeparator::transfer_item` — the transactional cross-layout primitive phases 1–3 deferred | `src/optimizer/bpp/separator.rs` |
| `BPSeparator::swap_config` — cheap separator for the many short pack-down separations | `src/optimizer/bpp/separator.rs` |
| `pack_down`, `pack_down_time_ratio`, `pack_down_move_time_limit`, `pack_down_separator_config` | `src/config.rs` (`BPCompressionConfig`) |
| `-p` tie-break: lowest density of the least dense bin | `src/bpp_main.rs` (`min_bin_density`) |
| `-x` also lowers the pack-down budgets | `src/bpp_main.rs` |
| Tests: pack-down effect, area bound fires, area bound for a single bin | `tests/bpp_packdown_tests.rs` |

Measured effect (`-e 30 -c 20 -s 42`, both bin sizes; see `docs/bpp.md` for the full table):

| Instance | Least dense bin | Used width of that bin |
| --- | --- | --- |
| 112 parts, `--bin 2000x1000` | 65.9 % → **41.9 %** | 1985 → **1308** of 2000 |
| 112 parts, `--bin 3000x1500` | 54.6 % → **45.1 %** | 2124 → **1534** of 3000 |
| `swim`, `--bin 3200x3200` (`-e 10 -c 25 -s 0`) | 48.8 % → **29.9 %** | — |

Exploration now returns in < 0.1 s on both (instead of 30 s), and the freed budget goes to compression.

#### Bug found and fixed in phase 4

`BPSeparator::rollback` restored a tracker snapshot with `restore_but_keep_weights` whenever the
*layout keys* were unchanged. That copies losses into the live tracker's **pre-sized** pair matrix, so
it is only valid when the item *count* is also unchanged. Cross-layout moves change item counts
without changing keys, which made the rollback index out of bounds
(`quantify/pair_matrix.rs:43: assertion failed: row < size && col < size`). `rollback` now compares
sizes per layout and adopts the snapshot wholesale (weights included) when they differ. The bug was
latent in phases 1–3 — nothing there could produce a same-key, different-size rollback — but the guard
is general.

#### Phase 4 design decisions

* **Pack-down lives in the compression phase, not the exploration one.** It is a *secondary*-objective
  step (concentrate the slack), even though it can reduce the bin count as a side effect. Putting it in
  compression keeps the phase contract intact: exploration is where the bin count is *targeted*.
* **The pack-down / consolidation alternation.** A single pass wasted the tail of the budget (the
  consolidation converges in seconds once the bin is sparse). They now alternate until a round changes
  nothing, so a leftover budget keeps being useful, and consolidation freeing a contiguous block
  occasionally lets the next pack-down round evict one more item.
* **`pack_down_time_ratio` (0.6).** Without it pack-down ate the whole budget and the consolidation —
  the step that actually turns the emptied bin into *one* offcut — never ran.
* **The area bound uses the `n-1` **largest** containers.** That is the optimistic choice, so a value
  above the cap is a genuine "cannot", not an artefact of picking the wrong bins to keep. With
  homogeneous bins it makes no difference; with heterogeneous stock it avoids false negatives.
* **A freshly transferred item gets a `search_placement` pass before `separate()`.** Dropping it at the
  random sampled position and relying on the GLS loop alone wasted most of the (short) per-move budget
  climbing out of a bad start.
* **Both touched layouts' trackers are rebuilt on a transfer** (GLS weights reset for those two only).
  Migrating weights across a changed pair-matrix size is not meaningful, and the untouched layouts keep
  their long-term memory.

#### Phase 4 deviations from the task description

* **`pack_down` returns `(BPSolution, usize)`** rather than just the solution: the alternation loop
  needs to know whether anything moved to decide convergence.
* **`pack_down_time_ratio` was added** (not in the spec) — see above.
* **The compression phase alternates the two steps** instead of running pack-down once and then
  consolidating once.
* **The `-x` flag also lowers `pack_down_move_time_limit` and the pack-down separator limits**, as the
  spec suggested it "may".

### Implemented as specified

| Item | Where |
| --- | --- |
| `BPSeparator` + per-layout `CollisionTracker`s, `BPSnapshot` | `src/optimizer/bpp/separator.rs` |
| `BPSeparatorWorker`, intra-layout `move_items` (SPP Alg. 5) | `src/optimizer/bpp/worker.rs` |
| `BPLBFBuilder` — SPP item order, cheapest `cost/area` bin, `bail!` on no stock | `src/optimizer/bpp/lbf.rs` |
| `separate()` — SPP Alg. 9 over the summed loss, `move_items_multi` (SPP Alg. 10) | `src/optimizer/bpp/separator.rs` |
| `close_bin_and_scatter` — remove all, re-insert largest-first round-robin from the least dense | `src/optimizer/bpp/separator.rs` |
| `exploration_phase` — bin-count reduction, pool, `max_conseq_failed_attempts` | `src/optimizer/bpp/explore.rs` |
| `disrupt_solution` — SPP port, per layout, incl. `practically_contained_items` | `src/optimizer/bpp/explore.rs` |
| `compression_phase` — the **stretch goal** (SPP sub-optimization + guarded write-back), not the v1 no-op | `src/optimizer/bpp/compress.rs` |
| `--bin WxH[:stock[:cost]]`, 3 input formats, `import_bp_solution`, `BPSvgExporter` | `src/util/bpp_io.rs` |
| `sparrow-bpp` binary incl. `-p` parallel runs | `src/bpp_main.rs` |
| `BPConfig` / `BPExplorationConfig` / `BPCompressionConfig` / `DEFAULT_BPP_CONFIG` | `src/config.rs` |
| User docs | `docs/bpp.md`, README section |

All six non-negotiables hold: SPP files are semantically untouched, the container-agnostic core is
called rather than forked, `jagua_rs::Instant` is used throughout, worker merging is index-ordered with
RNGs derived from the master, no `HashMap` iteration, and `debug_assert!`s mirror the SPP ones.

### Deviations from the spec

* **Rollback target on failure.** The spec suggested rolling back to a *pooled infeasible* solution and
  disrupting it (as SPP does). BPP instead always rolls back to the **best feasible** solution. Pooled
  solutions have one bin fewer and are infeasible, so restarting from them compounds the infeasibility
  rather than exploring. The pool is still used, but only to decide *how strongly* to disrupt: the
  half-normal sample picks a pooled attempt and its rank becomes the number of swaps applied.
* **`import_bp_solution` returns `Result`.** The spec did not specify error handling. It validates bin
  ids, item ids, bin stock and remaining item demand, because `BPProblem::place_item` checks none of
  these and `register_included_item` decrements an unchecked `usize` (an over-placed item panicked with
  an arithmetic overflow instead of reporting the malformed file).
* **Warm start validation in the CLI.** `optimize_bpp` panics by contract if no initial solution can be
  built, so `bpp_main.rs` validates up front: it probes the LBF builder for a clean CLI error, and
  rejects a warm start that does not cover the full demand or contains a colliding layout
  (`BPProblem::restore` trusts the snapshot and cannot invent missing placements).
* **Live SVG output is a *directory*, not a path.** `BPSvgExporter` takes `live_dir` rather than a
  `live_path`, because a BPP solution needs one file per bin (`.live_solution_bin{k}.svg`).
* **`clamp_to_container` (new, BPP-only).** The unbounded coordinate descent in `search_placement` can
  walk an item outside a large, sparsely filled bin, where the quadtree cannot index it — the
  specialized collision pipeline then misses collisions against it. `worker::clamp_to_container` clamps
  the translation using the item's **rotated** bbox (`transform_from` with rotation only, which matches
  how `PlacedItem::new` builds the placed shape). This cannot occur in SPP, where the strip is always
  fitted tightly around the items, so SPP semantics stay untouched. When the item does not fit in the
  container in its current rotation, the transformation is left alone. Moves that were clamped skip the
  "weighted loss never increases" debug assertion, since the clamped transformation is not the one the
  evaluator scored.
* **`n_scatter_retries`** was added to `BPExplorationConfig` (not in the spec): consecutive failed
  attempts target the 1st, 2nd, … least dense bin, so retries are different subproblems.

### Deferred

* **Cross-layout moves *inside the GLS separation loop*.** ~~Requires a transactional remove/place
  helper~~ — that helper now exists (`BPSeparator::transfer_item`, phase 4) and is used by the
  pack-down step, but the *workers* still move items only within their own layout. The remaining
  obstacle is the merge: a worker holds a private problem copy and merges by key, so an auto-close
  inside a worker changes the master's key space. Hook documented in `worker::move_items`.
* **Cross-layout disruption swaps** — same reason.
* **Iteration-based terminator** for the determinism smoke test the spec asked for. Only a wall-clock
  terminator exists, so two same-seed runs complete a different number of iterations and a strict
  equality test would be flaky. The determinism *properties* (index-ordered merge, derived RNGs, no
  `HashMap`) are in place and documented in `docs/bpp.md`; the test is left out rather than made flaky.
* **Search over the bin type mix.** Bin types are chosen greedily by lowest `cost / area` at the moment
  a bin is opened.
* **Consolidating more than one bin**, and consolidating along both axes.

### Review findings fixed during phase 3

* `src/util/bpp_io.rs` — `import_bp_solution` panicked (`attempt to subtract with overflow` in
  jagua-rs `problem.rs:217`) on a warm start placing an item more often than demanded; now a clean
  `Err`. Regression test: `tests/bpp_io_tests.rs::over_placed_item_is_rejected`.
* `src/bpp_main.rs` — an incomplete warm start (fewer items placed than demanded) was silently
  optimized and written out with items missing; now rejected, together with collision-free validation
  of every warm-start layout.
