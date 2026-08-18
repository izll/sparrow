# Multi-sheet ("walled") strip packing

*Phase 7. Implemented in `src/optimizer/sheets.rs`; enabled with `--sheet-width`.*

## The problem this solves

The strip packing engine (`sparrow`) minimises the width of one continuous strip. Real material is
often delivered as **fixed-size sheets** (MADisoCAD: 2000 x 1000 mm), and a strip solution cannot be
used on such material directly:

* **Cutting the strip** at multiples of the sheet width slices straight through every part that
  happens to straddle a boundary. Each straddling part has to be relocated, which in practice costs
  a whole extra sheet.
* **Running the bin packing engine** (`sparrow-bpp`) instead minimises the *bin count*, but it lacks
  the strip engine's global GLS compaction, so the leftover ends up badly distributed — e.g. iso7
  gives 3 bins at 65 / 65 / 13 %, and iso6 gives four bins at 30 % holding a single lonely part each.

The walled mode gives both properties at once: **no part can straddle a boundary**, and the strip's
full compaction is retained. Sheet 1 fills up, then sheet 2, and only the *last* sheet is left with
an unused end band. Since the sheet count is `ceil(width / (W + gap))`, minimising the strip width
minimises the number of sheets as a direct consequence.

## How it works

A vertical **wall** is inserted into the container at every sheet boundary, modelled as a
quality-0 `InferiorQualityZone` — i.e. a `HazardEntity::Hole`. Items may not overlap a hole, so the
collision detection engine enforces the sheet boundaries for free, and the separator's GLS loss
pushes straddling items off a wall exactly as it pushes two overlapping items apart.

No fork of jagua-rs is needed. `SPProblem` is kept as-is; after every jagua-level width change the
plain, wall-less container it installs is swapped for a walled one via `apply_sheet_walls`.

### Geometry and coordinate mapping

With sheet width `W` and gap `g` (pitch = `W + g`):

```
  sheet 0          wall 1     sheet 1          wall 2     sheet 2
|<----- W ----->|<-- g -->|<----- W ----->|<-- g -->|<-- ... -->|
0               W        W+g            2W+g      2W+2g
```

Wall `k` (for `k = 1, 2, ...`) covers the x-interval `[k*(W+g) - g, k*(W+g)]`. Only walls that
actually fall inside the strip are materialised, so a strip shorter than one sheet has none.

To map a strip coordinate `x` onto a physical sheet:

```
k       = floor(x / (W + g))      # which sheet
x_local = x - k * (W + g)         # coordinate on that sheet, in [0, W]
```

The y coordinate is unchanged — every sheet has the strip's fixed height.

### The gap is virtual

The sheets are *separate physical objects*, so the gap costs no material — nothing is lost by making
it generous. It defaults to `max(20 mm, 2 * min-sep)`.

Note that wall **thickness turned out not to matter** for search quality: gaps of 20, 60 and 150 mm
all produced the same result on iso7 (61.5 %). The wall's loss gradient is not the bottleneck; see
*Limitations* below.

### Minimum separation at walls

Walls are built with `ShapeModifyMode::Inflate` using the *same* `ShapeModifyConfig` as the strip, so
a wall is inflated by `min_sep / 2` exactly like a placed item is. Combined with the item's own
inflation this reproduces the full `min_sep` clearance between a part and a sheet edge — identical to
the clearance the deflated strip border already enforces.

### Hole quantification

Hole collisions are quantified by **bounding-box overlap** (`quantify_collision_poly_hole`), not by
the pole-based overlap proxy used for item pairs. Two reasons:

1. Hole shapes are container geometry and **never get a pole surrogate generated** — only items do —
   so the pole proxy is not even available for them. (Using it panics with `surrogate not generated`.)
2. A wall is a long, thin, axis-aligned rectangle. The exact bbox overlap is both cheaper and a
   *better* gradient than a pole approximation of such a sliver: it decreases strictly monotonically
   as the item is pushed off the wall, all the way to zero.

The function mirrors `quantify_collision_poly_container` (same shape penalty, same `sqrt` scaling),
so hole losses are directly comparable to container and pair losses and share the GLS weighting
unchanged.

## CLI

```
sparrow -i input.json --sheet-width 2000 [--sheet-gap 20] [--min-sep 5] -e 30 -c 20 -s 42
```

| Flag | Meaning |
| --- | --- |
| `--sheet-width <MM>` | Usable width of one physical sheet. **Enables the mode.** |
| `--sheet-gap <MM>` | Wall thickness. Defaults to `max(20, 2 * min-sep)`. |
| `--compact-sheets` | Per-sheet left-compaction post-pass. **Not yet implemented** (phase 8). |

Without `--sheet-width` the behaviour is the plain strip packing one, unchanged.

## Reporting

At the end of exploration, compression and the final solution, a per-sheet report is logged:

```
[SHEET] [FINAL] 4 sheet(s) of 2000 (+20 gap), last sheet used width 1160.6
[SHEET] sheet 0: 27 items, used 1999.3/2000 mm, dens 73.9%, leftover band 0.7 mm
[SHEET] sheet 1: 28 items, used 1998.5/2000 mm, dens 71.5%, leftover band 1.5 mm
[SHEET] sheet 2: 38 items, used 1997.9/2000 mm, dens 87.4%, leftover band 2.1 mm
[SHEET] sheet 3: 19 items, used 1160.6/2000 mm, dens 50.4%, leftover band 839.4 mm
[SHEET] leftover: 2342744 mm2 total (29.3%) = band (reusable) 843542 mm2 (10.5%) + internal gaps 1499203 mm2 (18.7%)
```

The **leftover band** is the secondary objective: the rectangular strip `[used, W] x [0, H]` at the
right edge of a sheet. Unlike the gaps *between* the parts, this band is one contiguous rectangle and
goes straight back into stock. The summary line splits the total waste into this reusable part and
the internal gaps.

## Results

30 s exploration + 20 s compression, seed 42. **All runs verified to have zero straddling items** and
to be feasible per jagua's own `is_feasible()`.

| Instance | Walled result | Sheets | Per-sheet used width (leftover band) | References |
| --- | --- | --- | --- | --- |
| **iso7** (33 parts, `-W 1995 --min-sep 5`) | 4721 mm @ 61.5 % | **3** | 1990.0 (5.0) / 1989.8 (5.2) / 686.1 (1308.9) | free strip 3466 mm @ 83.2 % → cut = 2 sheets; bpp 3 bins [65/65/13 %] |
| **iso6** (49 parts, `-W 1990 --min-sep 5`) | 17082 mm @ 50.9 % | **9** | 1977 / 1959 / 1905 / 1511 / 1510 / 997 / 997 / 997 / 997 | bpp 9 bins (**lower bound 9**) |
| **madisocad_iso** (112 parts, `-W 2000`) | 7221 mm @ 79.0 % | **4** | 1999.3 (0.7) / 1998.5 (1.5) / 1997.9 (2.1) / 1160.6 (839.4) | free strip 7086 mm @ 79.8 %; bpp 4 bins [41.9/72.5/83.8/84.7 %] |

### Reading the results

* **112 parts — the strongest case.** 4 sheets, matching the BPP's bin count, but with a far better
  *distribution*: 73.9 / 71.5 / 87.4 / 50.4 % versus the BPP's 41.9 / 72.5 / 83.8 / 84.7 %. Three
  sheets are packed essentially solid (leftover bands of 0.7, 1.5 and 2.1 mm) and **all** the slack is
  concentrated into one 839 mm reusable band on the last sheet. That is exactly the intended
  behaviour, and it is materially better than the BPP result for reuse. Density 79.0 % is within
  1 point of the free strip's 79.8 %, i.e. the walls cost almost nothing here.
* **iso6 — matches the proven optimum bin count.** 9 sheets equals the BPP lower bound, so the sheet
  count cannot be improved. The first three sheets are well packed; the tail sheets hold 1–2 parts
  each, which is forced by the part geometry (these are long parts that cannot share a sheet).
* **iso7 — the walls cost one sheet.** This is the one clearly negative result and it should not be
  glossed over. The free strip reaches 3463 mm @ 83.2 %, which *would* fit in 2 sheets (2 sheets need
  only 72.2 % density, and the area bound allows it). The walled run stalls at 3 sheets @ 61.5 %.

### Why iso7 stalls (diagnosed, not fixed)

The exploration log shows the search shrinking normally for 138 steps and then jamming completely at
width 4717.6 — just past the second wall — with 16 consecutive failures and a min loss that never
approaches zero. Sheet 2 holds only 8 parts in 686 mm.

The cause is **not** the wall's loss gradient: gaps of 20 / 60 / 150 mm all give 61.5 %. Nor is it
budget or seed: doubling the exploration budget to 60 s and trying seeds 0, 7 and 42 all give 3
sheets. The cause is structural — the 0.1 % shrink step is a *local* move, and getting below 3 sheets
requires a part to **jump across a wall** into a different sheet. The separator has no such move: it
can slide a part off a wall, but nothing relocates a part from sheet 2 into a gap on sheet 0. Once
the layout settles into a 3-sheet arrangement it is trapped.

A warm start from the good free-strip solution does not rescue it either (see below), which confirms
the diagnosis: the issue is the absence of a cross-sheet relocation move, not a bad starting point.

## Limitations and known issues

* **No cross-sheet relocation move.** This is the main limitation, and it is what costs iso7 a sheet.
  A "jump this part to the best gap on another sheet" operator — analogous to the BPP's `pack_down`
  — would address it. This is the natural next piece of work.
* **Warm starting from a wall-less solution.** An imported solution knows nothing about the walls, so
  items may straddle them. `widen_for_walls` widens the strip by one spare sheet first, so the
  separator has room to push those items off the walls; the exploration phase then shrinks the width
  back down. Without this the run reports an *infeasible* layout as its answer (items overflowing
  their sheet), because `exploration_phase` seeds its feasible-solution list with the unseparated
  start. Note this is a pre-existing sharp edge in `exploration_phase` that the walls merely expose —
  it is worked around at the call site rather than by changing shared SPP behaviour.
* **`--compact-sheets` is a documented no-op.** The flag and `SheetConfig::compact_sheets` are
  accepted and reported but do nothing yet; see phase 8 below.
* **Inferior quality zones (quality > 0) are unsupported** by the separator, and hit an explicit
  `unimplemented!`. Only holes (quality 0) are handled. Unlike holes, an inferior zone is forbidden
  only for *some* items, which the GLS tracker has no notion of.
* **Walls are always vertical and evenly spaced.** Mixed sheet sizes or a horizontal grid would need
  a generalised `wall_intervals`.

## Phase 8 hook: per-sheet compaction

`SheetConfig::compact_sheets` reserves the interface for a post-compression pass that re-compacts
each sheet (except the last) **to the left within its own sheet**, turning the many small gaps
between parts into one wide reusable band per sheet.

This is a purely *secondary* objective — it cannot reduce the sheet count, which the walled strip
width already minimises. It only redistributes slack inside a sheet. The intended implementation
mirrors the BPP's `consolidate_layout`: build an SPP sub-problem from one sheet's items with the
strip height fixed, run the separator on it for a share of a small budget, translate the result back,
and accept only if the whole layout stays feasible and no item crosses a wall — otherwise roll that
sheet back.

Note the measured numbers suggest the payoff is instance-dependent: on the 112-part instance the
first three sheets already have leftover bands under 2.1 mm, so there is essentially nothing left to
compact. The gain would come on instances like iso6, whose middle sheets carry 30–85 mm bands and
23.8 % internal gaps.

## Guarantees and tests

`tests/sheet_tests.rs` (9 tests) covers:

* walls materialise as `Hole` hazards in the CDE, and an item centred on a wall is detected as
  colliding while one inside a sheet is not;
* the tracker records hole losses, includes them in the total, and its GLS weights grow on collision
  and stay finite under the `GLS_WEIGHT_MAX` clamp;
* the walls survive `save`/`restore` round-trips in **both** directions (same width → cheap
  `Layout::restore`; different width → `Layout::from_snapshot`), which is what the container-id trick
  buys;
* a full pipeline run on `swim.json` stays feasible per jagua, keeps its walls, and has **zero**
  straddling items — with the tracker's debug assertions active, so hole losses are checked against
  jagua's own collision detection on every single move;
* a synthetic 3-rectangle instance whose optimum is obviously 2 sheets reaches ≤ 2;
* with `sheet = None` the tracker allocates **no** hole entries at all.

## SPP regression safety

With `sheet = None` the behaviour is unchanged:

* `CollisionTracker` allocates an *empty* `hole_collisions` vec (`size * 0`), and every hole loop is
  a no-op, so there is no extra work in the hot paths;
* the deterministic exploration phase on `swim.json -e 10 -c 5 -s 0` is **bit-identical** before and
  after the change, matching at every shrink step down to 5854.934 @ 75.555 %;
* the compression phase is wall-clock-paced and therefore *inherently* non-deterministic — three runs
  of the same unmodified binary with the same seed gave 5836.49 / 5840.47 / 5836.76. The before
  (5838.74) and after (5839.16) figures both sit inside that spread;
* all 36 tests across the whole suite pass, BPP included.
