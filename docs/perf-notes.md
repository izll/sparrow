# Performance notes (2026-08-17)

Machine: AMD Ryzen 7 5800X (8C/16T, AVX2), Rust stable 1.90, jagua-rs 0.7.2.
Instance: `data/input/swim.json` (48 items), seed 0, 40 s exploration + 10 s compression, 3 workers.

## What was changed

| change | file(s) |
|---|---|
| Pole–pole overlap proxy in SoA layout, branch-free lane loop → LLVM auto-vectorizes on stable | `src/quantify/circles_soa.rs`, `src/quantify/overlap_proxy.rs`, `src/quantify/mod.rs` |
| Collector loads the moving shape's poles lazily (first quantification only) | `src/eval/specialized_jaguars_pipeline.rs` |
| Tracker uses the SoA path too; `CollisionTracker::new` computes item losses in parallel (rayon) | `src/quantify/tracker.rs` |
| `clone_from` instead of `clone` for per-iteration tracker sync (reuses allocations) | `src/optimizer/worker.rs`, `src/optimizer/separator.rs` |
| `target-cpu=native` for x86_64/aarch64, `codegen-units = 1`, `panic = "abort"` | `.cargo/config.toml`, `Cargo.toml` |
| `-p/--parallel-runs N`: N independent seeds in parallel, best result kept | `src/main.rs`, `src/util/io.rs`, `src/util/ctrlc_terminator.rs` |

## Measurements

Profile before (perf, self time): 58.7 % `SpecializedHazardCollector::loss` (scalar `sqrtss/divss` pole loop),
~27 % jagua-rs quadtree traversal, rest small. After: `loss` 20.8 % (8-wide `vsqrtps/vdivps`), quadtree ~50 %.

| build | evals/s (avg over separations) |
|---|---|
| original (rev 961ec31) | ~1000 K |
| SoA auto-vectorized, SSE2 baseline | ~1620 K |
| + `target-cpu=native` (AVX2) + profile tweaks | **~2000 K** |

Worker count (parallel threads, same wall time): 3 → 2.0 M evals/s, 8 → 4.8 M, 16 → 5.5 M aggregate,
but iterations/s and final density did not improve beyond 3 (single-seed runs). Extra cores are better spent on
independent runs: `-p 4` keeps ~77 % of the solo per-run throughput on 8C/16T (≈3.1× total search).

Workers on a **single core** (`taskset -c 0`, emulating a single-threaded build such as wasm):

| workers | seed 0 | seed 1 |
|---|---|---|
| 1 | 73.63 % | 74.46 % |
| **3** | **75.73 %** | **75.90 %** |
| 4 | 75.04 % | 75.44 % |
| 6 | 75.18 % | 75.45 % |

Best-of-3 orderings per iteration beats 3× more iterations with one ordering, even at equal CPU time.

## What the speed-up does and does not preserve

Both phases stop on **wall-clock time**, not on an iteration count, so a faster build does not
reproduce the slower build's run — it does *more* of it. Comparing this fork with upstream
`961ec31` on `swim -e 10 -c 5 -s 0` with 3 workers, an independent audit measured 132 shrink steps
(final exploration width 5949.413) upstream against 148 (5854.934) here, with the **first 132 steps
identical to the printed precision**. That is the correct claim to make: an *identical shrink-step
prefix for equal iteration counts*, not a bit-identical run. A wall-clock-limited run of a faster
build necessarily differs, and here it differs by being better.

Consequently:

* the same command with the same `-s` seed run twice will generally **not** give the same final
  width — three repeats of seed 0 gave 5837.700 / 5837.811 / 5834.733;
* the exploration phase's shrink-step count is itself a function of machine speed and load;
* `-p N` adds a second source of variation: the parallel runs compete for CPU, so which seed wins
  can change between invocations;
* to compare two builds or two configurations meaningfully, fix the seed **and** average over
  several runs, or compare the shrink-step prefixes rather than the final numbers.

Determinism for a fixed seed *and* a fixed iteration count holds (worker results merge in
worker-index order, per-worker RNGs derive from the master RNG, and no iteration order depends on a
`HashMap`), but the CLI exposes no iteration-based terminator, so it cannot be demonstrated
end-to-end today.

## Reproducing

```bash
cargo build --release --features only_final_svg
./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0          # baseline
./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0 -p 4     # 4 parallel runs
SPARROW_N_WORKERS=1 taskset -c 0 ./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0
```
`evals/s` is printed at the end of every separation (`[SEP] finished, evals/s: ...`).
