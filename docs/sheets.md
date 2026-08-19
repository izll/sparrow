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
| `--sheet-gap <MM>` | Wall thickness. Defaults to `max(20, 2 * min-sep)`. **`0` is honoured** (with a warning), see "Gap semantics". |
| `--compact-sheets` | Per-sheet left-compaction post-pass (phase 8). Secondary objective only. |
| `--pack-down-sheets` | Cross-sheet pack-down in the compression phase (phase 8). **Off by default**, see below. |
| `--plain-first` | Explore without walls first, then install them (phase 8). **Off by default**, see below. |

Without `--sheet-width` the behaviour is the plain strip packing one, unchanged.

### Gap semantics

`--sheet-gap` is the thickness of the **virtual wall** between two consecutive sheets, so the pitch
(distance between two sheets' left edges) is `width + gap`.

* **Not given** → `max(20, 2 * min-sep)`. The default is generous on purpose: the gap costs nothing
  physically (the sheets are separate objects) and a thick wall gives the GLS separator a smooth loss
  gradient to push straddling items off a boundary.
* **`--sheet-gap 0`** → honoured, with a warning. The sheets butt up against each other, the wall
  intervals become degenerate (`x_min == x_max`) and are dropped entirely — the boundary is then
  enforced only by the strip width, so an item may sit *exactly* on a boundary and the cut has no
  kerf allowance. Previously this was silently replaced by the 20 mm default, laying the solution out
  on a pitch the caller never asked for.
* **A negative value** is rejected with a warning and the default is used instead.

### Items wider than a sheet

An item that does not fit inside a single sheet in **any** allowed rotation makes the walled mode
unsolvable: it would have to straddle a wall. This is now checked once at startup
(`sheets::items_too_wide_for_sheet`) and the run exits non-zero naming the offending items and their
minimum widths, e.g.

```
Error: --sheet-width 700 mm is too narrow for this instance: item 0 (1742.0 mm), item 2 (1941.4 mm),
       ... do(es) not fit inside a single sheet in any allowed rotation, so no walled solution can exist
```

Left undetected this surfaced either as an LBF "strip-width is running away" panic or — from a warm
start, where the LBF is bypassed — as an exported layout full of wall crossings.

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

### Straddling items are reported, never clamped

An item is assigned to the sheet its **right** edge (`x_max`) falls on — the edge that determines how
far into a sheet material is consumed. In a feasible walled solution both edges are on the same
sheet, so the choice is immaterial; in an infeasible one they need not be, and such an item is now
listed explicitly in `SheetStats::straddling_item_ids` and warned about:

```
[SHEET] [FINAL] 3 item(s) STRADDLE a sheet wall — this layout cannot be cut into sheets: 4@sheet7, 7@sheet7, 5@sheet8
```

The previous code derived the index from `x_min` and **clamped** it into range, so an uncuttable
layout was reported as a perfectly ordinary one — the only hint being a density above 100 % or a used
width larger than the sheet (e.g. "used 2611.4/700 mm, dens 178.2 %"). A straddling item means the
strip cannot be cut at all, which matters far more than any of the other numbers, so it is said out
loud.

## Results

*These are the **phase 7** numbers, kept for the before/after comparison. The current figures are in
"What phase 8 changed" further down.*

30 s exploration + 20 s compression, seed 42. **All runs verified to have zero straddling items** and
to be feasible per jagua's own `is_feasible()`.

| Instance | Walled result | Sheets | Per-sheet used width (leftover band) | References |
| --- | --- | --- | --- | --- |
| **iso7** (33 parts, `-W 1995 --min-sep 5`) | 4721 mm @ 61.5 % | **3** | 1990.0 (5.0) / 1989.8 (5.2) / 686.1 (1308.9) | free strip 3466 mm @ 83.2 % → cut = 2 sheets; bpp 3 bins [65/65/13 %] |
| **iso6** (49 parts, `-W 1990 --min-sep 5`) | 17082 mm @ 50.9 % | **9** | 1977 / 1959 / 1905 / 1511 / 1510 / 997 / 997 / 997 / 997 | bpp 9 bins (matches the bpp result; *not* a proven lower bound) |
| **madisocad_iso** (112 parts, `-W 2000`) | 7221 mm @ 79.0 % | **4** | 1999.3 (0.7) / 1998.5 (1.5) / 1997.9 (2.1) / 1160.6 (839.4) | free strip 7086 mm @ 79.8 %; bpp 4 bins [41.9/72.5/83.8/84.7 %] |

### Reading the results

* **112 parts — the strongest case.** 4 sheets, matching the BPP's bin count, but with a far better
  *distribution*: 73.9 / 71.5 / 87.4 / 50.4 % versus the BPP's 41.9 / 72.5 / 83.8 / 84.7 %. Three
  sheets are packed essentially solid (leftover bands of 0.7, 1.5 and 2.1 mm) and **all** the slack is
  concentrated into one 839 mm reusable band on the last sheet. That is exactly the intended
  behaviour, and it is materially better than the BPP result for reuse. Density 79.0 % is within
  1 point of the free strip's 79.8 %, i.e. the walls cost almost nothing here.
* **iso6 — matches the BPP's bin count.** 9 sheets equals what the BPP reaches. (Phase 7 called this
  a proven lower bound; it is not one — it is simply the best result either engine has found.) The first three sheets are well packed; the tail sheets hold 1–2 parts
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

* **Cross-sheet relocation exists but is not strong enough on every instance.** Phase 8 added it
  (see below); it fixes the synthetic cases and is a no-op risk elsewhere, but it does *not* rescue
  iso7. See "What phase 8 changed" for the measurements and the diagnosis.
* **Warm starting from a wall-less solution** — now *repaired and verified*, see below.
* **Inferior quality zones (quality > 0) are unsupported** by the separator, and hit an explicit
  `unimplemented!`. Only holes (quality 0) are handled. Unlike holes, an inferior zone is forbidden
  only for *some* items, which the GLS tracker has no notion of.
* **Walls are always vertical and evenly spaced.** Mixed sheet sizes or a horizontal grid would need
  a generalised `wall_intervals`.

## Warm starting a walled run (`-i previous_solution.json --sheet-width W`)

An imported solution knows nothing about the walls, so every part that happened to straddle a
boundary now overlaps one. The start is therefore prepared in three steps:

1. **`widen_for_walls`** grows the strip by one spare sheet, so the wall-crossers have somewhere to
   go and the separator has room to work.
2. **`repair_walled_warm_start`** (`src/optimizer/mod.rs`) then actually *performs* the repair: a
   `separate()` with the patient wall-repair settings (`WALL_REPAIR_STRIKE_LIMIT`,
   `WALL_REPAIR_ITER_NO_IMPRV_LIMIT`) on a bounded slice of the exploration budget
   (`WALL_REPAIR_BUDGET_RATIO`).
3. If the repair does **not** reach zero loss, the warm start is **discarded** in favour of a walled
   LBF construction, which is feasible by construction. The pre-pass's work is lost, but a correct
   answer from a worse start beats an infeasible one that gets exported as if it were fine.

The separator handed to `exploration_phase` is thus feasible in every case.

### Why step 2 is not optional

`widen_for_walls` only creates *room* for the repair — it does not do the repair, and until this fix
nothing downstream verified it either. `exploration_phase` seeds its feasible-solution list with
whatever start it is given, **without testing it**, so the still-overlapping layout was recorded as
feasible and came straight back out as the run's answer. In debug builds a `debug_assert!` caught
this; in **release** it is compiled out, which is exactly where the bug bit.

Measured (`swim.json` → 700 mm sheets): the exported JSON contained **85 wall crossings**, with
per-sheet reports showing densities above 100 % and used widths of 2611 mm on a 700 mm sheet. After
the fix that particular case is rejected up front (swim's parts are wider than 700 mm — see "Items
wider than a sheet"), and a solvable one (2500 mm sheets) repairs successfully and validates clean.

### Defence in depth: `exploration_phase`

The phase itself no longer trusts its start blindly. It still `debug_assert!`s feasibility, but in
release it now *checks* the start's loss and simply does **not** seed an infeasible layout into its
feasible-solution list — the phase has to earn its first feasible solution through `separate()` like
any other width. `best_width` starts at `+inf` in that case so the first solution found at any width
is recorded, and the phase may legitimately return an **empty** list if it never reaches feasibility;
`optimize` handles that.

For a feasible start this is bit-identical to the previous behaviour (the check passes, the start is
seeded, no RNG is touched), so plain SPP is unaffected.

## Phase 8: cross-sheet relocation

Phase 7's diagnosis was that the walled mode has no move that gets a part **across a wall**. The
0.1 % fine shrink narrows the strip's right-hand *end*, so the only sheet that ever feels compaction
pressure is the last one; sheets `0..n-2` are shielded by the walls and their holes get filled only
by lucky random samples. Phase 8 adds the missing operator in two places.

Both are built on one shared primitive, `sheets::scatter_and_shrink`:

> **Shrink the strip to a target width, and re-place every item that no longer fits at a uniformly
> random position inside a surviving sheet** (round-robin over the sheets, feasible rotation only,
> followed by a placement search restricted to that same sheet), **then separate.**

The width cut is not incidental — it is what makes the relocation *stick*. The separator's placement
search is global over the whole strip, so as long as the vacated tail is still part of the
container, the search happily puts every relocated item straight back into it. Measured on iso7,
single-item transfers without a width cut were undone within a fraction of a second, every time.
Removing the space in the same move is what prevents that.

Because the scatter is a global perturbation rather than a local touch-up, the repair separation
runs with a deliberately more patient configuration (`SCATTER_STRIKE_LIMIT = 8`,
`SCATTER_ITER_NO_IMPRV_LIMIT = 400`) than the fine shrink's.

### The sheet-drop move (exploration phase)

`sheets::try_drop_sheet` calls the primitive with `target_width_for(n-1)` — the strip is cut by a
**whole sheet** at once and the last sheet's contents are scattered over the others.

**When it fires.** Not after every feasible solution, but only once the *fine shrink has failed
`SHEET_DROP_AFTER_SHRINK_FAILURES = 2` times in a row*. The reason is measured: a drop from a loose
`n`-sheet layout has to relocate the contents of a nearly-full last sheet all at once and
essentially never succeeds, whereas after the shrink stalls, the last sheet holds as little as it
ever will — the smallest possible relocation. Attempting earlier also eats the budget the shrink
needs to get there. The attempt always starts from, and on failure rolls back to, the *last feasible
solution* (`sheets::rollback_to_width` handles the width change the plain `Separator::rollback`
refuses).

**Strike policy** (deterministic, no wall-clock input):

* at most `sheet_drop_strikes` (default **3**) failed attempts per sheet count; then the phase falls
  back to the fine shrink alone, so the last band still gets minimised;
* consecutive attempts differ only in the RNG state, i.e. a different random scatter;
* if the fine shrink *also* keeps failing, the strike budget is refilled after `SHEET_DROP_COOLDOWN
  = 6` fallback rounds. Without this, a long budget is spent entirely on a shrink that has
  provably stopped moving (iso7 at 300 s: strikes exhausted after 76 s, the remaining 224 s
  produced nothing);
* a successful drop, or any other change of the sheet count, resets the strikes: a new sheet count
  is a new subproblem.

**Area bound.** Before any attempt, `required_density_for(n-1)` = `Σ item area / ((n-1)·W·H)` is
compared against `max_reduction_density` (default **0.90**, the same notion and value as the BPP's).
Above it the reduction needs a nesting density that is out of reach for irregular parts, and the
attempt is skipped outright rather than burning seconds proving it.

Failed attempts are added to the exploration's infeasible-solution pool like any other failure.
Since they sit at a *different* (narrower) width, every pool rollback goes through
`rollback_to_width`; in plain strip-packing mode that is a plain `Separator::rollback`, so nothing
changes there.

### The pack-down (compression phase) — implemented, **off by default**

`sheets::pack_down_sheets` runs **before** the fine compression and gets at most
`pack_down_time_ratio` (default 0.3) of its budget. It is enabled with `--pack-down-sheets`;
see the measurement note at the end of this section for why it is not on by default. It attacks the *last sheet's band* with the same
primitive: cut `PACK_DOWN_BAND_STEP` (50 %) of the last sheet's **used width** away at once, scatter
whatever no longer fits into the earlier sheets, separate. On success the band is permanently
shorter and the step repeats; on failure the cut is halved, down to `PACK_DOWN_MIN_STEP` (5 %). It
never cuts past the last sheet's left edge — dropping a whole sheet is the exploration's job.

The result is always feasible and never wider than the input, so the fine compression that follows
can only improve on it.

**Why it is off by default.** On all four measured instances — iso6, iso7, madisocad_iso, swim —
*not a single band cut was ever accepted*, while the failed attempts cost 2–10 s of a 20 s
compression budget. On madisocad_iso that was enough to leave the final band ~56 mm worse than
without the step (763 mm vs 820 mm), because the fine compression is what actually shortens the band
on that instance and it lost the time. The step is kept (it is correct, cheap to reach, and the
mechanism is the right one for an instance whose last sheet *can* be emptied), but the operator that
pays off is the sheet-drop in the exploration phase.

An earlier, more obvious design — transfer one item, short `separate()`, keep if feasible — was
implemented and **measured to be useless**, for two reasons worth recording:

1. the separator moves the item back into the empty band (the reason the width cut is bundled in);
2. the acceptance test cannot be phrased on item keys, because a `restore` re-keys the layout, nor
   on the used width, because on iso7 all 8 items of the last sheet reach within 0.3 mm of the
   band's edge — removing one shortens the band by 0.3 mm, so a used-width criterion rejects every
   perfectly good transfer.

### `--compact-sheets`: per-sheet left-compaction

Now implemented (`sheets::compact_sheets_left`), and still a purely **secondary** objective: it
cannot change the strip width or the sheet count, it only redistributes slack *inside* a sheet.

For every sheet except the last, the items are visited left to right and each is slid as far left
inside **its own sheet** as it goes without colliding (binary search on the displacement, tolerance
`COMPACT_MIN_SHIFT = 0.5 mm`, every probe verified against the CDE and undone if it collides). The
many small gaps between the parts thus migrate into one contiguous, reusable right-hand band per
sheet.

**Key handling (was a crash).** `Separator::move_item` removes and re-places the item, so it mints a
**new `PItemKey` on every call — including the undo of a failed probe**. `try_shift` therefore
returns `(key, accepted)` and the caller must adopt the returned key in *both* branches; the key it
passed in is dangling on return. The original code dropped the key from the undo branch, so the
binary search's next probe reused a stale key and the run died with `invalid SlotMap key used`
(reproducible with `--sheet-width 2500 --compact-sheets -e 12 -c 8 -s 0` on swim).

Because the search can end on a *failed* probe — which restores the item to its original position —
the best accepted shift is **re-applied** afterwards if the item is no longer sitting at it.
Without that the item silently stayed put while `n_moved` claimed it had moved; `n_moved` is now
truthful.

This is a pure *translation* pass rather than the re-nesting phase 7 sketched (`consolidate_layout`
on an SPP sub-problem). The translation version cannot make the solution worse, needs no budget
management, and — unlike a sub-problem re-nest — cannot push an item into a wall. The payoff is
instance-dependent: on madisocad_iso the first three sheets already have bands under 5.5 mm, so
there is nothing to compact; the gain is on instances like iso6, whose middle sheets carry 30–85 mm
bands and 23.8 % internal gaps.

### The plain-first pipeline (`--plain-first`) — implemented, **off by default**

A third idea, and the one with the most appealing story: the plain strip engine reaches a far higher
density than a walled run of the same budget precisely *because* it is not fighting the walls
(iso7: 83.1 % at 3466 mm = 1.74 sheets, versus 61.5 % walled). So run the exploration wall-*less*
first, then cut the result into sheets and install the walls:

1. chop the plain strip into chunks of `W - slack` and translate every item right onto its own
   sheet, so each chunk moves *as a whole* and its internal neighbour relationships survive;
2. install the walls and `separate()`, which only has to rehome the items that were straddling a
   chunk boundary — a local repair rather than a 66 → 72 % global re-nest.

`sheets::install_walls_into_plain` does the cut; `optimizer::mod::plain_first_prepass` drives it.

**The repair is verified, and that verification is not optional.** `exploration_phase` seeds its
list of feasible solutions with whatever layout it is handed, *without testing it*, so an unrepaired
start is reported verbatim as the final answer. Before the check was added, iso6 proudly reported
"7 sheets" (2 better than phase 7!) for a layout with a total loss of 163 — i.e. overlapping parts —
and iso7 reported 2 sheets with a first sheet 2088 mm wide on a 1995 mm sheet. If the repair does not
reach zero loss, the cut is retried with more slack per sheet (`WALL_REPAIR_SLACK_STEP` = 10 % of `W`
per retry, `WALL_REPAIR_MAX_EXTRA_SHEETS` = 3 retries sharing one bounded budget), and if that still
fails the run falls back to a walled LBF start, which is feasible by construction.

**Measured verdict: it does not beat the walled-from-start pipeline**, which is why it is opt-in.
Head to head at 30 s + 20 s (seed 42, swim seed 0), last-sheet used width in mm:

| Instance | plain-first | walled-from-start |
| --- | --- | --- |
| iso7 | 3 sheets, 691.1 | 3 sheets, 691.1 |
| iso6 | 9 sheets, 1002.1 | 9 sheets, 1002.2 |
| madisocad_iso | 4 sheets, 1225.6 | 4 sheets, 1225.3 |
| swim | 2 sheets, 2964.7 | 2 sheets, **2888.4** |

Equal on three instances and worse on the fourth: the wall-installation repair costs real budget
(on iso7 the retries alone consume ~15 s of a 30 s exploration), and it buys nothing the walled
exploration does not reach on its own.

### What phase 8 changed (measured)

30 s exploration + 20 s compression, seed 42 (swim: seed 0), release build. **Every result below was
independently verified from the exported JSON**: sheet count, per-sheet used width, straddling items
and out-of-container items recomputed from the placements, and all items accounted for.

| Instance | | Sheets | Per-sheet used width (mm) | Last band | Density | Straddling |
| --- | --- | --- | --- | --- | --- | --- |
| **iso7** (33 parts, `-W 1995 --min-sep 5`) | phase 7 | 3 | 1989.0 / 1988.8 / 683.7 | 1311.3 | 61.54 % | 0 |
| | **phase 8** | **3** | 1989.9 / 1990.0 / 686.1 | 1308.9 | 61.52 % | 0 |
| **iso6** (49 parts, `-W 1990 --min-sep 5`) | phase 7 | 9 | 1977 / 1959 / 1905 / 1511 / 1510 / 997 x4 | 993.0 | 50.90 % | 0 |
| | **phase 8** | **9** | identical | 993.0 | 50.90 % | 0 |
| **madisocad_iso** (112 parts, `-W 2000`) | phase 7 | 4 | 2000.0 / 1999.2 / 1994.5 / 1180.2 | 819.8 | 78.79 % | 0 |
| | **phase 8** | **4** | 1994.5 / 1989.0 / 1998.8 / 1215.5 | 784.5 | 78.41 % | 0 |
| **swim** (48 parts, `-W 3000`, seed 0) | phase 7 | 2 | 2999.5 / 2939.1 | 60.9 | 74.48 % | 0 |
| | **phase 8** | **2** | 2999.1 / 2871.5 | 128.5 | 75.34 % | 0 |

Run time is the configured 30 s + 20 s in every case.

**Reading the table honestly:**

* **The sheet count did not improve on any of the four instances.** iso7 in particular — the
  instance phase 7 was diagnosed on, and the one this whole phase was aimed at — is still 3 sheets,
  not the targeted 2.
* **swim is the one measurable gain**: 75.34 % vs 74.48 %, and a 128.5 mm reusable band instead of
  60.9 mm. The sheet-drop *fires and succeeds* there (`3 -> 2 sheets` at 12 s of the 30 s budget,
  where phase 7 needed the whole 30 s of fine shrinking to arrive at 2 sheets), leaving more of the
  budget for the compression that follows.
* **madisocad_iso loses ~35 mm of band** to run-to-run variance in the compression phase; the
  per-seed spread there is ±100 mm (measured seeds 7 and 13: 1234 / 1262 mm walled), so this is
  noise, not a regression.
* **iso6 is unchanged**, which is the correct outcome — 9 sheets is what the geometry allows.

### Why iso7 still needs 3 sheets

Three independent operators were implemented and measured against it, and **all three fail in
exactly the same place**, which is itself the most informative result of this phase:

| Operator | What it does | Best loss reached |
| --- | --- | --- |
| sheet-drop (scatter 8 items + separate) | phase 8 exploration | ~160–180 K |
| pack-down band cut (50 % of the band) | phase 8 compression | ~170 K |
| plain-first wall installation (83 % layout, cut in two) | phase 8 pre-pass | ~161 K |

Zero would be needed. The agreement of three very different starting points on the same residual
says the obstruction is not the *move* — phase 7's diagnosis, that there is no cross-sheet
relocation operator, was correct but incomplete. **A 2-sheet walled iso7 needs 72.2 % density with
every part fully inside a 1995 mm sheet, and no local repair of a 3-sheet or a wall-less layout gets
there.** The free strip's 83.2 % is achieved by letting parts sit across the 1995 mm boundaries; the
parts that do so are not a few strays that can be nudged aside — moving them requires re-nesting
both sheets at once, which is a fresh global optimization and not a repair.

What would be needed is a genuine 2-sheet *search* (a walled run given the full budget at a fixed
2-sheet width, restarting from scratch rather than repairing), or the BPP engine with the strip
compaction folded in. That is a bigger piece of work than a move, and it is the honest next step.

## Guarantees and tests

`tests/sheet_tests.rs` (10 tests) covers:

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
* **(phase 8)** a synthetic 8-rectangle instance (900 x 450 in 2000 x 1000 sheets, 81 % of two
  sheets) run through the *whole* `optimize()` pipeline reaches 2 sheets, feasible, zero straddling —
  the regression test for the cross-sheet relocation operator;
* with `sheet = None` the tracker allocates **no** hole entries at all.

### Regression tests for the review findings

Added in `tests/sheet_tests.rs`. All of them assert on **returned data** (solutions, stats, counts)
rather than on debug assertions, so they are meaningful under `cargo test --release` too — which is
where the two critical bugs actually lived, `debug_assert!`s being compiled out there.

| Test | Guards |
| --- | --- |
| `walled_warm_start_never_returns_an_infeasible_layout` | CRITICAL 1. A wall-less solution is optimized, then imported as the warm start of a 2000 mm walled run (asserted to genuinely straddle). The separator `repair_walled_warm_start` produces must have **zero** loss, be wall-clear and still place all demand; the solution `optimize()` returns must be `is_feasible()` with zero straddling. Fails before the fix with a loss of ~4.8 M handed to `exploration_phase`. |
| `exploration_does_not_seed_an_infeasible_start_as_feasible` | CRITICAL 1, unit. An item is parked on a wall and the phase is run; every returned solution must be feasible and wall-clear. **`#[cfg(not(debug_assertions))]`** — the `debug_assert!` in `exploration_phase` would trip first under the test profile, and the bug is release-only, so the test runs where the bug lives. |
| `compact_sheets_completes_and_only_moves_items_left` | CRITICAL 2. `compact_sheets_left` on a ≥2-sheet swim layout must not panic, must stay feasible and wall-clear, must not lose items or change the sheet count, `n_moved` must be truthful (if it claims movement, some item really moved), and **no item may end further right**. Fails before the fix with `invalid SlotMap key used`. |
| `sheet_stats_reports_straddling_items` | `sheet_stats` must report an item parked on a wall in `straddling_item_ids` instead of clamping it into a sheet, while a clean layout reports none. |
| `items_too_wide_for_a_sheet_are_detected` | 700 mm sheets are reported as too narrow for swim (with the offending widths), 2500 mm ones are not. |
| `zero_sheet_gap_is_honoured` | `resolve_gap(Some(0.0), _) == 0.0`, explicit gaps pass through, the default still respects `2 * min_sep`, and a zero gap yields degenerate wall intervals that `apply_sheet_walls` filters out. |

## Release-mode testing

`cargo test` uses the **test profile, with `debug_assertions` ON**. Several of the walled/BPP
invariants are guarded by `debug_assert!`, which means a bug they cover is *masked* in `cargo test`
(the assertion fires, so the test "fails loudly" for the right reason) and *unmasked* in release
(the assertion is gone and the bad value flows on silently into the exported JSON).

Both criticals fixed here were of exactly that shape. So:

```bash
cargo test            # debug assertions ON  — catches invariant violations early
cargo test --release  # debug assertions OFF — catches what release users actually get
```

Run **both**. Tests that specifically exercise release semantics are marked
`#[cfg(not(debug_assertions))]` and only execute in the second.

## SPP regression safety

With `sheet = None` the behaviour is unchanged:

* `CollisionTracker` allocates an *empty* `hole_collisions` vec (`size * 0`), and every hole loop is
  a no-op, so there is no extra work in the hot paths;
* the deterministic exploration phase on `swim.json -e 10 -c 5 -s 0` is **bit-identical** before and
  after the change, matching at every shrink step down to 5854.934 @ 75.555 %;
* the compression phase is wall-clock-paced and therefore *inherently* non-deterministic — three runs
  of the same unmodified binary with the same seed gave 5836.49 / 5840.47 / 5836.76. The before
  (5838.74) and after (5839.16) figures both sit inside that spread;
* the deterministic plain-SPP exploration on `swim.json -e 10 -c 5 -s 0` is **bit-identical** across
  phase 7 and phase 8: 149 shrink steps, ending at the same 5854.934 → 5849.079;
* all 38 tests across the whole suite pass, BPP included.
