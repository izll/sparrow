# sparrow 🪶 
[![DOI](https://zenodo.org/badge/DOI/10.48550/arXiv.2509.13329.svg)](https://doi.org/10.48550/arXiv.2509.13329)
[![CI](https://github.com/JeroenGar/sparrow/actions/workflows/rust_ci.yml/badge.svg?branch=main)](https://github.com/JeroenGar/sparrow/actions/workflows/rust_ci.yml) 

<p>
    <img src="data/sparrow.jpeg" align="right" alt="logo" height=80>

> Sparrows are master weavers, crafting nests with intricate patterns. They utilize geometry and symmetry to ensure structural integrity and stability. By incorporating precise angles,  lengths, and weaving patterns, these birds achieve a balance between strength and efficiency.
[(read more)](https://www.mathnasium.com/math-centers/happyvalley/news/mathematical-marvels-bird-nest-construction-hv#)

</p>

### The state-of-the-art nesting heuristic for 2D irregular strip packing
`sparrow` can be used to solve 2D irregular strip packing problems, also commonly referred to as nesting problems.
This optimization algorithm builds on [`jagua-rs`](https://github.com/JeroenGar/jagua-rs): _a collision detection engine for 2D irregular cutting & packing problems_.

This repository accompanies the paper: ["_An open-source heuristic to reboot 2D nesting research_"](https://doi.org/10.48550/arXiv.2509.13329).

> [!TIP]
> Visit [**sparroWASM**](https://jeroengar.github.io/sparroWASM/) to see `sparrow` running in your browser!

## Nested by `sparrow`
<p align="center">
    <img src="data/records/final_best_trousers.svg" height=200/>
    <img src="data/records/final_best_mao.svg" height=200/>
</p>
<p align="center">
    <img src="data/records/final_best_swim.svg" height=270/>
    <img src="data/records/final_best_marques.svg" height=270/>
    <img src="data/records/final_best_dagli.svg" height=270/>
</p>
<p align="center">
    <img src="data/records/final_best_albano.svg" height=200/>
    <img src="data/records/final_best_shirts.svg" height=200/>
</p>
<p align="center">
    <img src="data/records/final_best_gardeyn1.svg" height=220/>
    <img src="data/records/final_best_gardeyn8.svg" height=220/>
</p>

## Requirements
- [Rust](https://www.rust-lang.org/tools/install) ≥ 1.86

## Usage

**General usage:**
```bash
cargo run --release  -- \
    -i [path to input JSON, or a solution JSON] \
    -t [timelimit in seconds (default is 600s)]
```
The optimization process contains two distinct phases: exploration & compression.
By default 80% of the timelimit is spent exploring and 20% is spent compressing.
Pressing 'Ctrl + C' immediately moves the algorithm to the next phase, or terminates it.

**All CLI options:**
```bash
-i, --input <INPUT>              Path to the input JSON file, or a solution JSON file for warm starting
-t, --global-time <GLOBAL_TIME>  Set a global time limit (in seconds)
-e, --exploration <EXPLORATION>  Set the exploration phase time limit (in seconds)
-c, --compression <COMPRESSION>  Set the compression phase time limit (in seconds)
-x, --early-termination          Enable early termination of the optimization process
-s, --rng-seed <RNG_SEED>        Fixed seed for the random number generator
    --min-sep <MM>               Minimum distance between items and between items and the container edge (mm)
-p, --parallel-runs <N>          Run N independent optimizations in parallel (seed, seed+1, ...) and keep the best (default: 1)
-h, --help                       Print help
```

`-p N` makes use of otherwise idle CPU cores: every run uses its own worker threads (3 by default), so on an 8-core machine
`-p 2`..`-p 4` gives 2-4 shots at the same time limit for a small per-run slowdown.

**Concrete example**:
```bash
cargo run --release -- \
    -i data/input/swim.json
```

## Bin packing (`sparrow-bpp`)

Next to the strip packing binary, this repo ships **`sparrow-bpp`**: the same heuristic, but packing all
items into copies of **fixed-size bins** (sheets/plates) while minimising **how many bins are used**,
with the leftover material consolidated into a single offcut as a secondary objective.

Any existing strip packing instance can be run as a bin packing problem by declaring the bin geometry
on the command line:

```bash
cargo run --release --bin sparrow-bpp -- \
    -i data/input/swim.json \
    --bin 3200x3200 \
    -t 600
```

`--bin WxH[:stock[:cost]]` is repeatable (`stock` defaults to 1000, `cost` to 1). Instances that already
declare their own `bins` are read as-is, and a previous `output/final_{name}.json` can be fed back in as
a warm start. Results are written to `output/final_{name}.json` plus one SVG per bin
(`output/final_{name}_bin{k}.svg`).

📖 **See [`docs/bpp.md`](docs/bpp.md)** for the full documentation: input formats, all CLI options, how
the algorithm works, determinism notes and known limitations.

## Multi-sheet strip packing (`--sheet-width`)

If the material comes as **fixed-size sheets** (e.g. 2000 x 1000 mm), neither of the two modes above
fits directly: cutting a strip solution at multiples of the sheet width slices through every part that
straddles a boundary, while `sparrow-bpp` minimises the bin count but leaves the leftover badly
distributed.

The **walled** mode solves both at once. It inserts a wall at every sheet boundary (as a `Hole`
hazard), so no part can ever straddle a boundary, while keeping the strip engine's full global
compaction — sheet 1 fills up, then sheet 2, and only the last sheet is left with an unused end band:

```bash
cargo run --release -- \
    -i data/input/swim.json \
    --sheet-width 2000 \
    -e 30 -c 20
```

The number of sheets is `ceil(width / (sheet-width + gap))`, so minimising the strip width minimises
the sheet count as a direct consequence. `--sheet-gap` sets the (virtual, material-free) wall
thickness; it defaults to `max(20, 2 * min-sep)` and must be **at least 1 mm** — a wall is a
rectangular collision hazard, and a zero-width one cannot be represented, so it would be dropped and
items would be free to straddle the sheet boundaries. The gap costs no material (the sheets are
separate physical objects), so there is never a reason to ask for less. Without `--sheet-width`
behaviour is unchanged.

Before the JSON is written, the solution is re-verified: the full demand must be placed, the layout
must be collision-free, and no item may straddle a sheet wall. A run that cannot satisfy all three
exits `1` and writes nothing rather than exporting a layout that cannot be cut.

A per-sheet report is logged, including the **reusable leftover band** of each sheet:

```
[SHEET] [FINAL] 4 sheet(s) of 2000 (+20 gap), last sheet used width 1160.6
[SHEET] sheet 0: 27 items, used 1999.3/2000 mm, dens 73.9%, leftover band 0.7 mm
...
[SHEET] leftover: 2342744 mm2 total (29.3%) = band (reusable) 843542 mm2 (10.5%) + internal gaps 1499203 mm2 (18.7%)
```

📖 **See [`docs/sheets.md`](docs/sheets.md)** for the geometry and coordinate mapping, min-sep semantics
at walls, measured results on real instances, and known limitations.

## Visualizer

This repo contains a simple visualizer to monitor the optimization process live.
Open [live_viewer.html](data/live/live_viewer.html) in a web browser,
and build `sparrow` with the `live_svg` feature enabled:

```bash
cargo run --release --features=live_svg -- \
    -i data/input/swim.json
```

![Demo of the live solution viewer](data/demo.gif)

## Input

This repository uses the same JSON format as [`jagua-rs`](https://github.com/JeroenGar/jagua-rs) to represent instances.
These are also available in Oscar Oliveira's [OR-Datasets repository](https://github.com/Oscar-Oliveira/OR-Datasets/tree/master/Cutting-and-Packing/2D-Irregular).

See [`jagua-rs` README](https://github.com/JeroenGar/jagua-rs?tab=readme-ov-file#input) for details on the input format.

## Output

Solutions are exported as SVG files in the `output` folder. 
The final SVG solution is saved as `output/final_{name}.svg`.

The SVG files serve both as a visual and exact representation of the solution.
All original shapes and their exact transformations applied to them are defined within the SVG:
```html
    ...
    <g id="items">
        <defs>
            <g id="item_0">...</g>
        </defs>
        <use transform="translate(1289.9116 1828.7717), rotate(-90)" xlink:href="#item_0">...</use>
    </g>
    ...
```
The [SVG spec](https://stackoverflow.com/questions/18582935/the-applying-order-of-svg-transforms) defines that the transformations are applied from right to left.
So here the item is always first rotated and then translated.

By default, a range of intermediate (and infeasible) solutions will be exported in `output/sols_{name}`.
To disable this and export only a single final solution, compile with the `only_final_svg` feature:
```bash
cargo run --release --features=only_final_svg -- \
    -i data/input/swim.json
```
The final solution is saved both in SVG and JSON format in `output/final_{name}.svg` and `output/final_{name}.json`, respectively.

## Determinism and reproducibility

Both phases stop on **wall-clock time**, not on an iteration count. That makes the *result* of a run
non-deterministic even with a fixed `-s` seed: a machine that is faster (or merely less loaded)
completes more iterations in the same budget and lands somewhere else. Three repeats of the identical
`swim -e 10 -c 5 -s 0` command gave final widths of 5837.700 / 5837.811 / 5834.733.

What *is* deterministic, for a fixed seed and worker count, is the search itself: worker results are
merged in worker-index order, per-worker RNGs are derived from the master RNG, and no iteration order
depends on a `HashMap`. Two builds of differing speed therefore produce an **identical shrink-step
prefix** — the same sequence of widths for as many steps as both manage — and diverge only because
the faster one takes more steps. Claims of bit-identical *runs* would require an iteration-based
terminator, which the CLI does not expose.

Practical consequences:

* to compare two builds or two settings, fix `-s` and average over several runs, or compare the
  shrink-step prefixes rather than the final widths;
* `-p N` adds a second source of variation — the parallel runs compete for CPU, so which seed wins
  can differ between invocations of the same command;
* `SPARROW_N_WORKERS` changes the search, not just its speed; results across different worker counts
  are not comparable.

## Targeting maximum performance

This crate is highly optimized and is floating-point heavy.
The hottest loop (pole-pole overlap proxy) is written in SoA layout so that it auto-vectorizes on stable Rust,
and `.cargo/config.toml` sets `target-cpu=native` for x86_64/aarch64 builds (AVX2 → 8-wide vector code).
Remove or override that flag (`RUSTFLAGS`) if the binary has to run on a different CPU than the build machine.

Optionally, the nightly-only `simd` feature uses [`std::simd`](https://doc.rust-lang.org/std/simd/index.html) explicitly:

```bash
  export RUSTUP_TOOLCHAIN=nightly
  cargo run --release --features=simd,only_final_svg -- \
      -i data/input/swim.json
```

## Testing
A suite of `debug_assert!()` checks are included throughout the codebase to verify the correctness of the heuristic.
These assertions are omitted in release builds to maximize performance, but are active in test builds.
Some basic integration tests are included that run the heuristic on a few example instances while all assertions are active:
```bash
  cargo test
```

Alternatively you can enable all `debug_assert!()` checks in release builds by running the tests with the `debug-release` profile:
```bash
cargo run --profile debug-release -- \
    -i data/input/swim.json
```

## Experiments
All solutions from the comparative experiments in the paper can be found at
[data/experiments](data/experiments).
The accompanying [README](data/experiments/README.md) details how to perform an exact reproduction of any benchmark run.

## Related Projects

- [`spyrrow`](https://github.com/PaulDL-RS/spyrrow): a Python wrapper of `sparrow`
- [`sparroWASM`](https://github.com/JeroenGar/sparroWASM): solve 2D nesting problems in the browser with WebAssembly
- [`sparrow-3d`](https://github.com/JonasTollenaere/sparrow-3d): a 3D adaptation of `sparrow`

## Development

This repo is meant to remain a faithful representation of the algorithm described in the paper.
However, I am open to pull requests containing bug fixes and speed/performance improvements as long as they do not alter the algorithm too significantly.

Feel free to fork the repository if you want to experiment with different heuristics or want to expand the functionality.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

This project began development at the CODeS research group of [NUMA - KU Leuven](https://numa.cs.kuleuven.be/) and was funded by [Research Foundation - Flanders (FWO)](https://www.fwo.be/en/) (grant number: 1S71222N).
<p>
<img src="https://upload.wikimedia.org/wikipedia/commons/4/49/KU_Leuven_logo.svg" height="50px" alt="KU Leuven logo">
&nbsp;
<img src="https://upload.wikimedia.org/wikipedia/commons/9/97/Fonds_Wetenschappelijk_Onderzoek_logo_2024.svg" height="50px" alt="FWO logo">
</p>
