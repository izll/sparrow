# Bin packing with `sparrow-bpp`

`sparrow-bpp` is the **bin packing (BPP)** variant of `sparrow`. Where the `sparrow` binary packs all
items into a *single strip of variable width* and minimises that width, `sparrow-bpp` packs them into
copies of **fixed-size bins** (sheets, plates, boards) and minimises **how many bins are used**.

It reuses the entire container-agnostic core of `sparrow` — sampling, collision evaluation, collision
quantification and the GLS separation loop — so the packing quality per bin is the same. Only the
objective and the surrounding search loop differ.

Built on [`jagua-rs`](https://github.com/JeroenGar/jagua-rs) (`jagua_rs::probs::bpp`).

---

## Objective

The optimizer pursues two objectives, strictly in this order:

1. **Primary — bin cost.** Minimise `Σ cost(bin)` over all opened bins. With the default cost of `1`
   per bin this is simply "use as few sheets as possible". This is what the *exploration phase* does.
2. **Secondary — remainder consolidation.** Once no further bin can be eliminated, concentrate the
   leftover material in **one** bin, so that what remains is a single rectangular offcut instead of a
   scattered set of unusable gaps spread over every bin. This is what the *compression phase* does, in
   two alternating steps: **pack-down** moves items *out of* the least dense bin into the others
   (cross-layout), and **strip consolidation** pushes what is left in it against one edge
   (intra-layout). Neither can make the solution infeasible; the bin count can only go *down* (a bin
   that pack-down empties completely auto-closes).

Both phases only ever accept solutions that are verified collision-free, so the returned solution is
always feasible with the full demand placed.

---

## Input formats

Three input shapes are accepted; the file is tried against each in order.

### 1. A bin packing instance (`ExtBPInstance`)

The native format. Same item representation as `jagua-rs`/`sparrow`, plus a `bins` array:

```jsonc
{
  "name": "my_instance",
  "items": [
    { "demand": 4, "shape": { /* … as in the sparrow/jagua-rs format … */ }, "allowed_orientations": [0, 180] }
  ],
  "bins": [
    {
      "id": 0,
      "shape": { "type": "rectangle", "x_min": 0.0, "y_min": 0.0, "width": 3200.0, "height": 1500.0 },
      "zones": [],
      "stock": 10,      // how many copies of this bin type are available
      "cost": 1         // cost of using one copy
    }
  ]
}
```

Multiple bin types may be declared. When a new bin has to be opened, the type with remaining stock
and the **lowest `cost / area` ratio** is chosen (ties resolve to the lowest bin id).

### 2. A strip packing instance + `--bin` on the command line

Any existing `sparrow` (SPP) instance can be run as a bin packing problem without editing the file —
just declare the bin geometry on the CLI:

```bash
sparrow-bpp -i data/input/swim.json --bin 3200x3200
```

The `--bin` flag is `WxH[:stock[:cost]]` and is **repeatable**; bins get ids `0..` in the order given.
`stock` defaults to `1000`, `cost` to `1`.

```bash
# two bin types: a large expensive one and a small cheap one
sparrow-bpp -i data/input/swim.json --bin 3200x3200:5:4 --bin 1600x1600:20:1
```

If the input file already defines `bins`, any `--bin` arguments are ignored (with a warning).

### 3. A previous solution file (`ExtBPOutput`) — warm start

`output/final_{name}.json` contains the instance **and** the solution in one flat object. Feeding it
back in resumes the optimization from that solution:

```bash
sparrow-bpp -i output/final_swim.json -t 600
```

The warm start is validated before it is used: it must cover the full item demand, every layout must
be collision-free, bin ids must exist and bin stock must suffice. Anything else is a hard CLI error
rather than a silently truncated run.

---

## Output

Written to the `output/` directory:

| File | Content |
| --- | --- |
| `output/final_{name}.json` | The instance + the final solution (`ExtBPOutput`). Re-usable as a warm start. |
| `output/final_{name}_bin{k}.svg` | One SVG per bin, `k = 0..n-1` in `LayKey` order. |
| `output/sols_{name}/` | Intermediate (and infeasible) solutions, unless built with `only_final_svg`. |
| `output/log.txt` | The full run log. |

As in `sparrow`, the SVG is both a picture and an exact representation: each item carries its original
shape and the exact `translate(...) rotate(...)` applied to it.

To export only the final SVGs:

```bash
cargo build --release --features only_final_svg
```

---

## CLI options

```
-i, --input <INPUT>              Path to the input JSON (BPP instance, SPP instance + --bin,
                                 or a BPP solution JSON for warm starting)
-t, --global-time <SECONDS>      Global time limit (split 80% exploration / 20% compression)
-e, --exploration <SECONDS>      Exploration phase time limit  (requires -c)
-c, --compression <SECONDS>      Compression phase time limit  (requires -e)
-x, --early-termination          Stop after N consecutive failed bin-removal attempts, and make the
                                 compression phase give up faster too (halved pack-down per-move
                                 budget, halved pack-down iteration/strike limits)
-s, --rng-seed <SEED>            Fixed seed for the random number generator
    --min-sep <MM>               Minimum distance between items and between items and the bin edge (mm).
                                 Overrides the SPARROW_MIN_SEP env var; default: none
-p, --parallel-runs <N>          Run N independent optimizations in parallel (seeds seed..seed+N-1)
                                 and keep the best (default: 1)
    --bin <WxH[:stock[:cost]]>   Declare a rectangular bin type. Repeatable
-h, --help                       Print help
```

**Minimum separation semantics** (`--min-sep s`, identical for `sparrow` and `sparrow-bpp`): every item is inflated
by `s/2` and every container (strip or bin) is deflated by `s/2`, i.e. items keep at least `s` from each other *and*
from the bin edge. A 992 mm item therefore needs a bin of at least 992 + 2·(s/2) + 2·(s/2) = 1002 mm (+ ε) when
`s = 5`. If a caller wants a different edge margin `m` than the item spacing `s`, it should shrink the bin by
`2·(m − s)` (only when `m > s`) — not by `2·(m − s/2)`. `tests/fit_parity_tests.rs` pins the equal-verdict guarantee.

`-t` is mutually exclusive with `-e`/`-c`, which must be given together. Pressing `Ctrl+C` moves the
algorithm to the next phase, or terminates it.

### Examples

```bash
# 30 s exploring, 20 s consolidating, fixed seed
sparrow-bpp -i my_instance.json --bin 2000x1000 -e 30 -c 20 -s 42

# 10 minutes total, 4 independent runs in parallel, keep the best
sparrow-bpp -i my_instance.json --bin 3000x1500 -t 600 -p 4 -s 42

# resume from a previous result
sparrow-bpp -i output/final_my_instance.json -t 300
```

`-p N` uses otherwise idle cores: each run has its own worker threads (3 by default), so on an 8-core
machine `-p 2`..`-p 4` gives several shots at the same time limit for a small per-run slowdown. The
best run is the one with the **lowest bin cost**, ties broken by the **lowest density of the least
dense bin** — i.e. at equal bin count the run whose leftover is most concentrated in a single bin (the
biggest usable offcut) wins, rather than the one that spreads the same slack evenly.

---

## How the algorithm works

```
                    ┌──────────────────────────────────────────────┐
   instance ──────► │ 1. Construction: LBF (bpp/lbf.rs) AND        │
                    │    shelf (bpp/shelf.rs) → start from the best│
                    └──────────────────┬───────────────────────────┘
                                       │ feasible start solution
                    ┌──────────────────▼───────────────────────────┐
                    │ 2. Exploration  (bpp/explore.rs)             │
                    │    repeat:                                   │
                    │      close least-dense bin & scatter items   │
                    │      separate (GLS)   (bpp/separator.rs)     │
                    │      area bound / stagnation? → stop,        │
                    │        hand the rest of the time to (3)      │
                    │      feasible?   → accept (one bin fewer)    │
                    │      else        → roll back to best+disrupt │
                    └──────────────────┬───────────────────────────┘
                                       │ minimal bin count
                    ┌──────────────────▼───────────────────────────┐
                    │ 3. Compression  (bpp/compress.rs)            │
                    │    repeat while time remains:                │
                    │      pack-down: move items out of the         │
                    │        least-dense bin into the others       │
                    │      consolidate the least-dense bin's rest  │
                    └──────────────────┬───────────────────────────┘
                                       ▼  final solution
```

### 1. Construction — two heuristics, better one wins (`bpp/lbf.rs`, `bpp/shelf.rs`)

Two *complementary* constructors are built and the better result seeds the pipeline (fewer bins;
tie → lower density of the least dense bin; tie → LBF). Both results are always logged:

```
[BPOPT] LBF start: 5 bin(s), min-bin dens 0.564% | shelf start: 5 bin(s), min-bin dens 36.511%
[BPOPT] starting from the LBF solution (5 bin(s))
```

Controlled by `BPConfig::constructive` (`Constructive::{Best, Lbf, Shelf}`, default `Best`). A warm
start (`initial_solution`) bypasses both.

#### 1b. Shelf/column construction (`bpp/shelf.rs`)

`BPShelfBuilder` ignores the contours and packs the items' **bounding boxes**. It works on
`item.shape_cd.bbox` (already inflated by `min_item_separation`) inside `container.outer_cd.bbox`
(already deflated by the same amount), so a placement that is collision-free in the bbox model is
collision-free in the real geometry *with the requested separation respected*.

It runs two first-fit-decreasing variants and keeps the better one:

* **column mode** — vertical stacks packed left → right; a new column starts when the item no longer
  fits on top of an open one,
* **shelf/row mode** — the transpose (horizontal shelves packed bottom → top).

Items are sorted by their along-axis extent (descending, ties: cross extent, then id) after choosing
per item the orientation with the largest along-axis extent that still fits — for a rectangle that
means "stand it up" in column mode. Candidate rotations come from `RotationRange::Discrete`; a
`Continuous` item is only tried at 0° and 90°, since a bbox model cannot exploit an arbitrary angle.

There is **no RNG anywhere** in this constructor: every ordering is a total order, so two builds
produce identical placements (tested).

Two details that matter:

* **`PLACEMENT_GAP` (1e-3).** jagua-rs treats *touching* shapes as colliding, and a perfect shelf
  packing produces exactly-touching bboxes. Every item is therefore inset by this much on both axes.
  It is orders of magnitude below the `min_item_separation` the geometry already carries.
* **Verification.** After materialising the packing, every layout is checked with
  `Layout::is_feasible()` and the demand is checked for completeness. A failure is a *bug in the bbox
  arithmetic*, never a property of the input, so it is an `Err` rather than a silent repair. Under
  `Constructive::Best` a failing shelf build is logged and the LBF result is used instead.

When it helps and when it does not: on rectangular part sets the bbox model *is* the geometry, so the
column structure it finds is what such an instance wants. On irregular parts it throws away
everything the contour would have saved — on `swim` it needs 7 bins where the LBF needs 5. That is
precisely why both are built and the better one wins.

### 1a. LBF construction (`bpp/lbf.rs`)

Items are sorted exactly as in the SPP builder — convex hull area × diameter, descending, expanded by
demand — and placed one at a time, left-bottom-first. Each item is first tried in the already open
layouts in creation order; the first collision-free (`Clear`) placement wins. If it fits in none of
them, a new bin is opened — the type with remaining stock and the lowest `cost / area`. The candidate
placement is searched in a *hypothetical* empty layout first, so a bin is never opened only to be
closed again. If an item fits nowhere and no stock is left, this is a hard error.

### 2. Exploration — bin-count reduction (`bpp/explore.rs`)

The discrete counterpart of the SPP strip-shrink loop.

**Area bound (checked before every attempt).** A reduction to `n - 1` bins can only exist if the placed
item area fits into the `n - 1` *largest* remaining containers:

```
required_density = Σ placed item area / Σ container area of the (n-1) largest layouts
```

If `required_density > expl_cfg.max_reduction_density` (default **0.90**; set it to `1.0` for a pure
area bound), the reduction is impossible — or hopelessly unlikely for irregular parts — and the phase
logs `[BPEXPL] reduction to {n-1} bins needs {x}% density > cap, skipping exploration` and **returns
immediately** instead of spinning until the timeout. The unused budget is handed to the compression
phase (`optimize_bpp` logs the handover), where the pack-down step always has more work to do.

Otherwise, each attempt:

1. **Pick a target bin.** The *n*-th least dense open layout, where *n* cycles through
   `0..n_scatter_retries` as attempts fail. Varying the target makes retries genuinely different
   subproblems instead of re-runs of the same one. Ties break by `LayKey` order, so it is deterministic.
2. **Close it and scatter** (`BPSeparator::close_bin_and_scatter`). All items of the target are
   removed — the layout auto-closes and its bin returns to stock — and re-inserted **largest first**,
   round-robin over the remaining layouts starting from the least dense one, each at a random position
   drawn uniformly over the destination container's bbox. Overlap is explicitly allowed here; this is
   what creates the subproblem.
3. **Separate** (`BPSeparator::separate`, SPP Algorithm 9). The GLS separation loop tries to resolve
   that overlap. It keeps **one collision tracker per open layout** (so GLS weights are per bin) and
   the "total loss" it minimises is the sum over all of them. Feasible ⟺ total loss is exactly 0.
   Internally `move_items_multi` runs *W* workers in parallel: each loads the master state, moves all
   currently-colliding items in its own random order, and the master adopts the worker with the lowest
   total *weighted* loss.

If the loss reaches zero, a strictly better solution — one bin fewer — has been found and becomes the
new best; the infeasible pool is cleared and the failure counter resets. If not, the attempt goes into
a pool sorted by loss, and the search **rolls back to the best (feasible) solution** and *disrupts* it
before retrying. Disruption swaps two 'large' items inside one randomly chosen layout and drags along
every item whose point of inaccessibility is contained by them. How many swaps are applied is drawn
from the pool: a half-normal sample picks a pooled attempt and its rank sets the disruption strength,
so better pooled attempts lead to gentler disruption.

The loop stops when the terminator fires, when `max_conseq_failed_attempts` consecutive attempts have
failed (`-x`), or when a single bin is left.

**Phase-wise stagnation stop.** The area bound catches reductions that are *area*-impossible. A
reduction can also be impossible for purely geometric reasons the area bound cannot see (the free
space exists but is fragmented). The symptom is an attempt series whose min loss oscillates around
the same value instead of trending down. `stagnation_limit` (default `Some(8)`, `-x` halves it)
stops exploration when that many consecutive attempts fail *and* none of them improved the best min
loss seen at the current bin count by at least `STAGNATION_MIN_IMPROVEMENT` (2 %). The remaining
budget is handed to the compression phase, which is the same mechanism the area bound already uses.

Each failed attempt now logs the state that decision is based on:

```
[BPEXPL] unable to reach feasibility with 11 bin(s) (dens: 62.323%, min loss: 223 K,
         best at this level: 161 K, 0.3s, strikes left: inf, stagnant: 1/8)
[BPEXPL] stagnated: 8 attempt(s) without a >2% improvement of the min loss (161 K) at 11 bin(s),
         handing the rest of the budget to compression
```

and the phase ends with a one-line reason: `finished (stagnation | area bound | strikes exhausted |
time limit | single bin | could not close a bin), best feasible solution: …`.

### 3. Compression — remainder consolidation (`bpp/compress.rs`)

The bin count is settled; now make the leftover material reusable. Two steps alternate until the
budget runs out or a full round changes nothing.

#### 3a. Pack-down — cross-layout (`pack_down`)

The step that actually makes the remainder *one* piece: it empties the least dense bin into the
others, item by item.

```
loop while time remains:
  L = least dense open layout                    (skip if only one layout)
  for every item of L, largest original area first (ties by PItemKey → deterministic):
    for every other layout M, most free area first (container.area - placed_item_area):
      snapshot = sep.save()                      (solution + all tracker snapshots)
      transfer the item L -> M at a random feasible-rotation position inside M's bbox
      search its lowest-loss position in M       (SeparationEvaluator + search_placement, as in the workers)
      separate() with a short budget             (the global "make room" step: it may move items
                                                  inside M — and anywhere else, the loss is global)
      total loss == 0 and feasible?  → accept, next item of L
      else                           → rollback(snapshot), try the next M
  stop when a full pass over L moved nothing
```

The cross-layout move itself is `BPSeparator::transfer_item`, the transactional primitive phase 1–3
deferred. It embraces jagua-rs' auto-close semantics rather than fighting them: removing the *last*
item of the source closes that layout and returns its bin to stock — which is exactly the outcome the
step hopes for, since the bin count then drops as a side effect. The destination is always another
open (non-empty) layout, so its `LayKey` survives the placement. Both touched layouts' trackers are
rebuilt (only those two lose their GLS weights) and the workers are reseeded.

Every accepted state passes the same guard the write-back uses: total loss `== 0` over **all**
layouts, `Layout::is_feasible()` per layout, and the full demand still placed. Anything else is rolled
back, so pack-down can never make the solution worse.

The step uses its own, deliberately cheap separator (`pack_down_separator_config`: 50
`iter_no_imprv_limit`, 2 strikes) because a pack-down attempt is a *local* repair and hundreds of them
are made, and each attempt is capped at `pack_down_move_time_limit` (2 s). Pack-down as a whole may
use `pack_down_time_ratio` (60 %) of the remaining compression budget; the rest is reserved for 3b.

**Per-item diagnostics.** Every source item produces exactly one info line, whatever the outcome:

```
[BPCMPR] item 6 (513x605 bbox) from bin LayKey(10v17): tried 11 bins,
         best residual loss 64.8 K (bin LayKey(11v19)) -> kept in place
```

`best residual loss` is the lowest collision loss any destination was left with after the short
`separate()`. A value far above zero for *every* destination is the signature of "the free area is
real but fragmented" — the transfer is being rejected legitimately, not for lack of budget.

**Block/column transfer: not implemented, deliberately.** The plan was to fall back to moving whole
columns when single-item transfers move nothing. Measurement says that would not help on the case
that motivated it. On `o90` with `--min-sep 5` the pack-down step genuinely cannot move anything,
because **12 bins is a proven lower bound** for that instance (see *Measured results*): the slack is
not merely fragmented, there is no 11-bin packing to find at all. A group transfer would spend the
budget re-deriving the same rejection with bigger objects. On the instances where free space *is*
merely fragmented (the 112-part case) single-item pack-down already moves 10 items and drops the
least dense bin from 65.9 % to 41.9 %. The hook stays open: `BPSeparator::transfer_item` is the
primitive a group version would loop over, and `report_stats` already exposes the per-bin free-width
information such a step needs.

#### 3b. Strip consolidation — intra-layout (`consolidate_layout`)

The **least dense** bin (recomputed: pack-down may have changed which one that is) is lifted into a
*strip packing subproblem*: its items become a fresh `SPInstance` whose strip has the bin's height and
starts at the width its content currently occupies, seeded with the current placements. The existing
SPP exploration phase is then asked to shrink that strip. If it succeeds, the narrower placements are
translated back and written into the BPP layout.

The write-back is fully guarded: the rebuilt layout is verified with jagua-rs' own CDE
(`Layout::is_feasible()` **and** total tracker loss `== 0`), and the item count and demand are checked.
On any failure the phase rolls back to the pre-consolidation solution, so in the worst case compression
is a no-op that only reports the per-bin statistics.

---

## Configuration knobs

All defaults live in `DEFAULT_BPP_CONFIG` (`src/config.rs`).

### `BPConfig`

| Field | Default | Meaning |
| --- | --- | --- |
| `constructive` | `Constructive::Best` | Which constructor seeds the pipeline. `Best` builds both (LBF and shelf) and starts from the better one; `Lbf` / `Shelf` force one. |

### `BPExplorationConfig`

| Field | Default | Meaning |
| --- | --- | --- |
| `time_limit` | 9 min (CLI overrides) | Wall-clock budget for the exploration phase. Unused time goes to compression. |
| `max_reduction_density` | `0.90` | **Area bound.** Skip (and return from) the exploration when reducing to `n-1` bins would need a higher density than this. `1.0` = pure area bound. |
| `max_conseq_failed_attempts` | `None` (`-x` → 10) | Give up after this many consecutive failed reductions. |
| `stagnation_limit` | `Some(8)` (`-x` → 4) | Stop when this many consecutive attempts fail **and** the best min loss at the current bin count has not improved by ≥ 2 %. |
| `n_scatter_retries` | `3` | Consecutive failed attempts target the 1st, 2nd, … least dense bin. |
| `separator_config` | 200 iters / 3 strikes | The GLS separation loop used for a reduction attempt. |
| `solution_pool_distribution_stddev` | `0.25` | Half-normal spread when picking a pooled attempt (sets the disruption strength). |
| `large_item_ch_area_cutoff_percentile` | `0.75` | Which items count as 'large' during disruption. |

### `BPCompressionConfig`

| Field | Default | Meaning |
| --- | --- | --- |
| `time_limit` | 60 s (CLI overrides) | Budget for the whole compression phase, **plus** whatever exploration left unused. |
| `pack_down` | `true` | Run the cross-layout pack-down step. |
| `pack_down_time_ratio` | `0.6` | Share of the *remaining* compression budget pack-down may use per round; the rest is reserved for the strip consolidation. |
| `pack_down_move_time_limit` | `2 s` | Budget for one pack-down attempt (one item into one destination bin). `-x` halves it. |
| `pack_down_separator_config` | 50 iters / 2 strikes | Deliberately cheap separator for the "make room" step. `-x` halves both limits. |
| `consolidate_remainder` | `true` | Run the strip consolidation of the least dense bin. |
| `consolidation_expl_cfg` | 30 s, shrink 0.005 | The SPP sub-optimization used for it; its `time_limit` caps a *single* consolidation attempt. |
| `separator_config` | 100 iters / 5 strikes | Separator the phase is constructed with (restored around each pack-down round). |

---

## Logging and diagnostics

**`debug!` does not exist in a release build.** `Cargo.toml` enables the `log` crate's
`release_max_level_info` feature, which compiles every `debug!`/`trace!` call *out at compile time*
for release profiles. So on a normal `cargo build --release` binary:

* `RUST_LOG=debug` can **never** show a `debug!` line — the call site is gone, not filtered,
* the same holds for `LOG_LEVEL_FILTER_DEBUG`; only `info!` and above survive.

To actually see `debug!` output you need a build with debug assertions
(`cargo build --profile debug-release` or a plain debug build), which is far slower — the
`debug_assert!`s in the separator and the trackers dominate the runtime. That is why every diagnostic
that has to be usable on a production run is emitted at **info** level and kept *bounded* (one line
per pack-down source item, one per exploration attempt, one summary per phase) rather than per
iteration.

The info-level diagnostics added for this:

| Prefix | Line | Where |
| --- | --- | --- |
| `[BPOPT]` | `LBF start: … \| shelf start: …` + which one is used | `bpp/mod.rs` |
| `[BPSHELF]` | which mode won, and the final bin count / density | `bpp/shelf.rs` |
| `[BPEXPL]` | per attempt: min loss, best-at-this-level, duration, strikes left, stagnation counter | `bpp/explore.rs` |
| `[BPEXPL]` | `finished (<reason>)` — time limit / strikes exhausted / stagnation / area bound / single bin | `bpp/explore.rs` |
| `[BPCMPR]` | per pack-down source item: bbox, bins tried, best residual loss, moved or kept | `bpp/compress.rs` |

## Determinism

For a **fixed seed and worker count**, worker results are merged in worker-index order (`min_by_key`
returns the first minimum), per-worker RNGs are derived from the master RNG, and no iteration order
depends on a `HashMap` — only `SlotMap`/`SecondaryMap`/`Vec`. Ties in "least dense layout" break by
`LayKey`.

However, the stopping criterion is **wall-clock time**, so two runs with the same seed will generally
*not* produce the same solution: they complete a different number of iterations. Determinism holds for
a fixed seed *and* a fixed iteration count. To compare configurations meaningfully, fix `-s` and either
average over several runs or replace the terminator with an iteration-based one (see below).

The worker count can be overridden with the `SPARROW_N_WORKERS` environment variable, which is useful
when comparing 1 / 3 / 8 / 16 workers on the same input.

---

## Measured results

Measured on a 112-part instance (a strip packing instance of `strip_height` 1000, run through `--bin`)
and on `swim.json`, all with `-e 30 -c 20 -s 42` unless noted. "**offcut**" is `bin width − used width`
of the least dense bin: the width of the contiguous rectangular remainder.

| Input | Bins | Least dense bin | Used width of that bin | Offcut |
| --- | --- | --- | --- | --- |
| 112 parts, `--bin 2000x1000` — *before* pack-down | 4 | 65.9 % | 1985 / 2000 | 15 |
| 112 parts, `--bin 2000x1000` — **with pack-down** | 4 | **41.9 %** | **1308** / 2000 | **692** |
| 112 parts, `--bin 3000x1500` — *before* pack-down | 2 | 54.6 % | 2124 / 3000 | 876 |
| 112 parts, `--bin 3000x1500` — **with pack-down** | 2 | **45.1 %** | **1534** / 3000 | **1466** |
| `swim.json`, `--bin 3200x3200`, `-e 10 -c 25 -s 0` — after exploration | 4 | 48.8 % | — | — |
| `swim.json`, `--bin 3200x3200`, `-e 10 -c 25 -s 0` — **with pack-down** | 4 | **29.9 %** | — | — |

The bin count is unchanged in all three cases — it is already area-optimal (3 bins of 2000x1000 would
need 94.3 % density, one bin of 3000x1500 would need 125.7 %) — and that is the point: the *total*
density cannot improve either, so the only thing left to optimise is **where** the slack sits. Pack-down
moves it out of three bins into one, turning the 112-part result from "four bins each with a sliver of
waste" into "three nearly full bins plus a 692 x 1000 clean offcut".

### The `o90` case — why 12 bins is the answer, not 10

`o90` is 7 rectangle types (all 600 wide, heights 992/540/102/96/102/892/508), demand 7 each = 49
items, all four orientations allowed, into 1990 x 995 bins with `--min-sep 5`.

| Metric | Before phase 5 | After phase 5 |
| --- | --- | --- |
| Bins | 12 (57.13 % density) | 12 (57.13 % density) |
| LBF / shelf start | 12 (LBF only) | 12 / 12 — tie, `Best` keeps LBF |
| Exploration | ran the full 15 s, min loss oscillating 160–270 K | **stops after 3.4 s** (`stagnation`) |
| Pack-down moves | 0 (no reason logged) | 0 (**one diagnostic line per item**, best residual loss 64.8–85.9 K) |
| Wall clock (`-e 15 -c 10`) | 18.0 s | **6.5 s** |

**12 is a proven lower bound for this instance**, so the pipeline was already optimal and the
"trivial shelf heuristic reaches 10" claim does not hold once `--min-sep 5` is taken into account:

* With `--min-sep 5` every item is inflated to 605 x (h+5) and the bin deflated to 1985 x 990. A
  first-fit-decreasing packer on that geometry needs **12** bins; the same packer on the *raw*
  600 x h / 1990 x 995 geometry needs **10**. The 10-bin figure silently drops the requested 5 mm
  separation — it is not a valid solution to the problem as posed.
* Call a piece "big" if it is ≥ 513 mm in one dimension: that is items 0, 5, 1, 6 = 28 pieces.
  Exhaustive placement search shows **no bin can hold 4 big pieces**, so ⌈28/3⌉ = 10 is a first bound.
* Item 0 (997 mm inflated) does **not** fit upright in a 990 mm bin, so it must lie down as
  997 x 605. Exhaustive search over all triples shows that **no bin containing item 0 can hold two
  other big pieces** — it admits at most one. The 7 copies of item 0 therefore occupy 7 bins and
  absorb at most 7 of the remaining 21 big pieces; the other 14 need ⌈14/3⌉ = 5 further bins.
  **7 + 5 = 12.**

So the remaining "gap" to the 6.86 → 7 area bound is **not** a nesting deficiency: it is forced by
the piece geometry plus the requested separation. The area bound ignores that a 997 mm piece cannot
stand in a 990 mm bin, and that is worth ~5 bins here. Answering the original question (4): after
this phase o90 is **12 bins, which is optimal**; pushing below 12 is impossible at `--min-sep 5`, and
reaching 10 requires giving up the separation (`--min-sep 0`).

Both instances also demonstrate the area bound: exploration now returns in **< 0.1 s** instead of
burning its full 30 s on provably impossible 4 → 3 (resp. 2 → 1) attempts, and `optimize_bpp` hands
that time to the compression phase (`[BPOPT] exploration returned 30.0s before its deadline, handing
that time to compression (20.0s -> 50.0s)`).

### Phase 5 regression check (the shelf seed must not hurt the irregular cases)

| Instance | Shelf start | LBF start | Chosen | Final |
| --- | --- | --- | --- | --- |
| `o90`, `--bin 1990x995:25:1 --min-sep 5`, `-e 15 -c 10 -s 42` | 12 bins | 12 bins | LBF (tie) | **12 bins**, 57.13 % |
| `swim.json`, `--bin 3200x3200`, `-e 30 -c 20 -s 42` | 7 bins | **5 bins** | LBF | **4 bins**, 62.12 % |
| 112 parts, `--bin 2000x1000`, `-e 30 -c 20 -s 42` | 5 bins | 5 bins (min-bin 0.56 %) | LBF (lower min-bin density) | **4 bins**, 70.72 %, least dense bin 41.9 % |

On the two irregular instances the shelf constructor is worse or equal and `Constructive::Best`
discards it — which is exactly the intended behaviour, and confirms the seed change cannot regress
them.

## Known limitations

These are deliberate v1 scope decisions, not bugs:

* **Intra-layout moves only *during separation*.** Inside the GLS separation loop an item is always
  re-placed in the bin it currently sits in. Items migrate between bins via `close_bin_and_scatter`
  (exploration) and `transfer_item` (the compression phase's pack-down step), never as part of a GLS
  move itself.
* **One bin type per opened layout, chosen greedily.** The bin type is picked by lowest `cost / area`
  among those with stock, at the moment the bin is opened. There is no search over the *mix* of bin
  types, so heterogeneous-bin instances are handled greedily rather than optimally.
* **Consolidation targets only the least dense bin.** Both compression steps work on one bin — the
  least dense one, where the largest offcut is. Other bins are only reported on (and receive the items
  pack-down evicts).
* **Pack-down is greedy and first-fit.** Items leave the sparsest bin largest-first and enter the first
  destination that accepts them; there is no look-ahead over *which* item should go *where*, and no
  backtracking over an accepted move. A move is also never undone once accepted, even if a later one
  would have been better.
* **The area bound is a heuristic cap, not a proof.** With the default `max_reduction_density = 0.90`
  the exploration also skips reductions that are merely *unlikely* (needing 90–100 % density). Set it
  to `1.0` for a pure "provably impossible" bound at the cost of spending the budget on long shots.
* **Consolidation only pushes along one axis.** The bin is lifted into a strip of the bin's *height*,
  so the content is compacted horizontally; the offcut is a full-height vertical strip on the right.
* **Coordinate-descent clamp.** The coordinate descent inside `search_placement` is unbounded, and in a
  large, sparsely filled bin it can walk an item far outside the container, where the quadtree cannot
  index it. `worker::clamp_to_container` therefore clamps the translation so the item's *rotated*
  bounding box stays inside the container bbox. In the SPP this cannot happen (the strip is always
  fitted tightly around the items), so the clamp lives on the BPP side only and SPP semantics are
  untouched. When an item is wider or taller than the container in its current rotation, nothing
  sensible can be clamped to and the placement is left as-is; the resulting `Exterior` collision is
  quantified normally.
* **Disruption is intra-layout.** A cross-layout swap would invalidate `PItemKey`s in both layouts and
  could auto-close a single-item layout mid-disruption — silently changing the very quantity being
  optimised. That needs a transactional helper (see below).

---

## How to extend

**Cross-layout moves inside the GLS loop.** The transactional primitive now exists —
`BPSeparator::transfer_item` — so this is no longer the blocker it was in v1. The remaining hook is
`BPSeparatorWorker::move_items` in `src/optimizer/bpp/worker.rs`, where the destination layout is still
fixed to the item's own (`lkey`) and would be chosen just before the `SeparationEvaluator` is
constructed. The care needed there is different from the pack-down case: a *worker* holds a private
copy of the problem and merges by key, so an auto-close inside a worker changes the master's key space
and forces a full tracker rebuild on merge.

**Iteration-based terminator.** `Terminator` (`src/util/terminator.rs`) is a trait; implementing an
iteration-counting variant next to `BasicTerminator` would make runs bit-for-bit reproducible for a
fixed seed and worker count, which is what the determinism note above asks for. Both phases already
take `&impl Terminator`, so nothing else has to change.

**A different secondary objective.** `compress::compression_phase` is where the tie-break lives.
`report_stats` already computes per-bin density and used width; consolidating *every* bin, or
optimising for a target offcut size, plugs in there.

---

## Source map

| File | Role |
| --- | --- |
| `src/optimizer/bpp/mod.rs` | `optimize_bpp()` — orchestration (LBF → explore → compress) |
| `src/optimizer/bpp/lbf.rs` | `BPLBFBuilder` — sampling-based constructive first solution |
| `src/optimizer/bpp/shelf.rs` | `BPShelfBuilder` — deterministic bbox column/shelf constructor, `Constructive` |
| `src/optimizer/bpp/separator.rs` | `BPSeparator` — separation loop, per-layout trackers, `close_bin_and_scatter`, `transfer_item` |
| `src/optimizer/bpp/worker.rs` | `BPSeparatorWorker` — parallel move workers, `clamp_to_container` |
| `src/optimizer/bpp/explore.rs` | Bin-count reduction loop, area bound (`required_density_for_reduction`) + disruption |
| `src/optimizer/bpp/compress.rs` | `pack_down` (cross-layout) + strip consolidation of the remainder |
| `src/util/bpp_io.rs` | CLI parsing, input formats, warm-start import, JSON/SVG export |
| `src/bpp_main.rs` | The `sparrow-bpp` binary |
| `src/config.rs` | `BPConfig`, `BPExplorationConfig`, `BPCompressionConfig`, `DEFAULT_BPP_CONFIG` |
| `docs/bpp-design.md` | Design spec and implementation status |
