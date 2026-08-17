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
2. **Secondary — remainder consolidation.** Once no further bin can be eliminated, push the content
   of the **least dense** bin towards one edge, so that the material left over in that bin is a single
   rectangular offcut instead of a scattered set of unusable gaps. This is what the *compression
   phase* does. It never changes the bin count and never makes the solution infeasible.

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
-x, --early-termination          Stop after N consecutive failed bin-removal attempts
-s, --rng-seed <SEED>            Fixed seed for the random number generator
-p, --parallel-runs <N>          Run N independent optimizations in parallel (seeds seed..seed+N-1)
                                 and keep the best (default: 1)
    --bin <WxH[:stock[:cost]]>   Declare a rectangular bin type. Repeatable
-h, --help                       Print help
```

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
best run is the one with the **lowest bin cost**, ties broken by the **higher density**.

---

## How the algorithm works

```
                    ┌──────────────────────────────────────────────┐
   instance ──────► │ 1. LBF construction        (bpp/lbf.rs)      │
                    └──────────────────┬───────────────────────────┘
                                       │ feasible start solution
                    ┌──────────────────▼───────────────────────────┐
                    │ 2. Exploration  (bpp/explore.rs)             │
                    │    repeat:                                   │
                    │      close least-dense bin & scatter items   │
                    │      separate (GLS)   (bpp/separator.rs)     │
                    │      feasible? → accept (one bin fewer)      │
                    │      else      → roll back to best + disrupt │
                    └──────────────────┬───────────────────────────┘
                                       │ minimal bin count
                    ┌──────────────────▼───────────────────────────┐
                    │ 3. Compression  (bpp/compress.rs)            │
                    │    consolidate the least-dense bin's content │
                    └──────────────────┬───────────────────────────┘
                                       ▼  final solution
```

### 1. LBF construction (`bpp/lbf.rs`)

Items are sorted exactly as in the SPP builder — convex hull area × diameter, descending, expanded by
demand — and placed one at a time, left-bottom-first. Each item is first tried in the already open
layouts in creation order; the first collision-free (`Clear`) placement wins. If it fits in none of
them, a new bin is opened — the type with remaining stock and the lowest `cost / area`. The candidate
placement is searched in a *hypothetical* empty layout first, so a bin is never opened only to be
closed again. If an item fits nowhere and no stock is left, this is a hard error.

### 2. Exploration — bin-count reduction (`bpp/explore.rs`)

The discrete counterpart of the SPP strip-shrink loop. Each attempt:

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

### 3. Compression — remainder consolidation (`bpp/compress.rs`)

The bin count is settled; now make the leftover material reusable. The **least dense** bin is lifted
into a *strip packing subproblem*: its items become a fresh `SPInstance` whose strip has the bin's
height and starts at the width its content currently occupies, seeded with the current placements.
The existing SPP exploration phase is then asked to shrink that strip. If it succeeds, the narrower
placements are translated back and written into the BPP layout.

The write-back is fully guarded: the rebuilt layout is verified with jagua-rs' own CDE
(`Layout::is_feasible()` **and** total tracker loss `== 0`), and the item count and demand are checked.
On any failure the phase rolls back to the exploration solution, so in the worst case compression is a
no-op that only reports the per-bin statistics.

---

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
and on `swim.json`:

| Input | Bins | Density | Note |
| --- | --- | --- | --- |
| 112 parts, `--bin 2000x1000 -e 30 -c 20 -s 42` | **4** | 70.7 % | 3 bins would need 94 % — infeasible |
| 112 parts, `--bin 3000x1500 -e 30 -c 20 -s 42` | **2** | 62.9 % | consolidation narrowed the last bin's used width **2124 → 1865** of 3000 |
| `swim.json`, `--bin 3200x3200` | **4** | — | 3 bins would need 82.8 % |

The consolidation result on the second row is the point of the compression phase: the same two bins
hold the same parts, but the leftover material in the last bin became a contiguous 1135-wide offcut
instead of 876 wide plus scattered gaps.

---

## Known limitations

These are deliberate v1 scope decisions, not bugs:

* **Intra-layout moves only.** During separation an item is always re-placed in the bin it currently
  sits in. Items migrate between bins only via `close_bin_and_scatter` and disruption, never as part of
  the GLS loop.
* **One bin type per opened layout, chosen greedily.** The bin type is picked by lowest `cost / area`
  among those with stock, at the moment the bin is opened. There is no search over the *mix* of bin
  types, so heterogeneous-bin instances are handled greedily rather than optimally.
* **Consolidation targets only the least dense bin.** The compression phase consolidates one bin — the
  least dense one, where the largest offcut is. Other bins are only reported on.
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

**Cross-layout moves.** The hook is `BPSeparatorWorker::move_items` in `src/optimizer/bpp/worker.rs`:
the destination layout is currently fixed to the item's own (`lkey`), and would be chosen just before
the `SeparationEvaluator` is constructed. `BPSeparatorWorker::move_item` would gain a destination
parameter. The care needed is on the bookkeeping side: `BPProblem::remove_item` auto-closes a layout
when its last item goes, which invalidates the `LayKey` and returns the bin to stock, so a
remove-then-place across layouts must be transactional (both trackers updated, bin count restored on
rollback).

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
| `src/optimizer/bpp/lbf.rs` | `BPLBFBuilder` — constructive first solution |
| `src/optimizer/bpp/separator.rs` | `BPSeparator` — separation loop, per-layout trackers, `close_bin_and_scatter` |
| `src/optimizer/bpp/worker.rs` | `BPSeparatorWorker` — parallel move workers, `clamp_to_container` |
| `src/optimizer/bpp/explore.rs` | Bin-count reduction loop + disruption |
| `src/optimizer/bpp/compress.rs` | Remainder consolidation |
| `src/util/bpp_io.rs` | CLI parsing, input formats, warm-start import, JSON/SVG export |
| `src/bpp_main.rs` | The `sparrow-bpp` binary |
| `src/config.rs` | `BPConfig`, `BPExplorationConfig`, `BPCompressionConfig`, `DEFAULT_BPP_CONFIG` |
| `docs/bpp-design.md` | Design spec and implementation status |
