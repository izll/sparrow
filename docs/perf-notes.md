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

## Reproducing

```bash
cargo build --release --features only_final_svg
./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0          # baseline
./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0 -p 4     # 4 parallel runs
SPARROW_N_WORKERS=1 taskset -c 0 ./target/release/sparrow -i data/input/swim.json -e 40 -c 10 -s 0
```
`evals/s` is printed at the end of every separation (`[SEP] finished, evals/s: ...`).
