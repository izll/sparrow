#!/usr/bin/env python3
"""Independent (jagua-free) validator for sparrow / sparrow-bpp output JSON.

Checks, on the ORIGINAL contours (not the inflated collision shapes):
  * every pair of placed items keeps >= min_sep (default 0) distance (no overlap)
  * every item lies inside its container (bin rect, or the strip / sheet), with >= min_sep from the border
  * every item keeps >= min_sep from every hole (inner ring) of its container, and does not overlap one
  * every item's rotation is one of its `allowed_orientations` (mod 360); null/absent means
    continuous rotation, an EMPTY list means a fixed 0 degrees (jagua's RotationRange::None)
  * quality zones: an item with `min_quality` q may not touch (nor come within min_sep of) any zone
    whose quality is < q; a null `min_quality` demands top quality, i.e. every zone must be avoided
  * BPP: the number of layouts using a bin never exceeds that bin's `stock`
  * multi-sheet strip: no item crosses a wall [k(W+g)-g, k(W+g)], distance to walls >= min_sep
  * all demand placed

Overlap is judged by TWO independent criteria, because a single scalar tolerance cannot serve both
roles: a shared *edge* between two abutting parts has zero area but non-zero length, while a genuine
0.1 mm interpenetration over a long edge has a large area. So:
  * AREA:      intersection.area > --tol-area  (default 1.0 mm2)  -> OVERLAP
  * DEPTH:     p.buffer(-tol) meets q over a POSITIVE AREA                -> OVERLAP
                (penetration deeper than --tol, regardless of how little area it covers)
  * DEPTH, degenerate case: for shapes thinner than 2*tol the erosion above empties them and the
    depth test becomes vacuous, so a RELATIVE criterion takes over: intersection > 1% of the smaller
    shape's area.

usage: validate_solution.py <final.json> [--min-sep S] [--sheet-width W --sheet-gap G] [--tol T]
       validate_solution.py --self-test
"""
import json, math, sys, argparse
from shapely.geometry import LineString, Polygon, box
from shapely.strtree import STRtree
from shapely.affinity import affine_transform

ap = argparse.ArgumentParser()
ap.add_argument('file', nargs='?')
ap.add_argument('--min-sep', type=float, default=0.0,
                help='minimum separation the file was PRODUCED with; the validator cannot infer it')
ap.add_argument('--sheet-width', type=float)
ap.add_argument('--sheet-gap', '--gap', type=float, default=0.0, dest='sheet_gap')
ap.add_argument('--tol', type=float, default=0.05,
                help='geometric/penetration tolerance in mm (float32 + simplification)')
ap.add_argument('--tol-area', type=float, default=1.0,
                help='intersection area in mm2 above which an overlap is reported')
ap.add_argument('--self-test', action='store_true', help='run the built-in checks of the validator itself')
a = ap.parse_args()


def poly_of(shape):
    t, da = shape['type'], shape['data']
    if t == 'simple_polygon':
        return Polygon(da)
    if t == 'polygon':
        # Keep the inner rings (holes): an item or container with a hole is NOT solid, and treating
        # it as solid both hides real overlaps (another part legitimately nested in the hole reads
        # as an overlap) and misses fake ones.
        return Polygon(da['outer'], da.get('inner', []) or [])
    if t == 'rectangle':
        return box(da['x_min'], da['y_min'], da['x_min'] + da['width'], da['y_min'] + da['height'])
    raise SystemExit(f'unsupported shape {t}')


def place(item, tr):
    r = math.radians(tr['rotation']); tx, ty = tr['translation']   # ExtTransformation.rotation is in DEGREES
    c, s = math.cos(r), math.sin(r)
    P = poly_of(item['shape'])
    return affine_transform(P, [c, -s, s, c, tx, ty])   # p' = R p + t


ANGLE_TOL = 1e-3    # degrees; the exported rotations are float32-rounded, so exact equality is unusable


def orientation_ok(rotation, allowed):
    """True if `rotation` matches one of the `allowed` angles (degrees), modulo 360.

    The mapping is jagua-rs' (io/import.rs), not this script's invention:

        absent / null   -> RotationRange::Continuous  -- any angle
        []              -> RotationRange::None        -- FIXED at 0 degrees
        [0.0]           -> RotationRange::None        -- FIXED at 0 degrees
        [a, b, ...]     -> RotationRange::Discrete    -- one of those angles

    The empty list used to be read here as "no orientation is permitted at all", which rejects every
    possible placement of the item -- so a perfectly good solution came back as an error. It was not
    hypothetical: the engines themselves write `allowed_orientations: []` for a fixed-orientation
    item, so BOTH sparrow motors' legitimate output failed validation and nest_race.py exited 1 with
    no usable candidate. An item that may be placed at no angle whatsoever could never be packed, so
    "fixed at 0" is the only reading under which such an instance is solvable, and it is the one the
    engine implements.

    The comparison is modulo 360 because the engine exports the angle it happens to hold: a part
    with allowed_orientations [0, 180] is routinely written out as -180.0.
    """
    if allowed is None:
        return True
    if not allowed:                      # [] means a fixed 0-degree orientation, same as [0.0]
        allowed = [0.0]
    return any(min((rotation - ang) % 360.0, (ang - rotation) % 360.0) <= ANGLE_TOL for ang in allowed)


def zone_polys(container_spec):
    """The quality zones of a bin (or of the whole instance), as (quality, Polygon) pairs."""
    return [(z['quality'], poly_of(z['shape'])) for z in (container_spec.get('zones') or [])]


def overlaps(p, q, tol, tol_area):
    """(is_overlap, area, reason) for a pair of placed contours.

    Two independent criteria, either of which is conclusive:
      * the intersection covers more than `tol_area` mm2, or
      * one shape eroded by `tol` still meets the other over a POSITIVE AREA, i.e. they
        interpenetrate by more than `tol` somewhere. This is what catches a genuine 0.1 mm overlap
        along a long edge that an area-only test with a generous tolerance would wave through, and
        equally what stops a shared edge (zero-area, zero-depth) from being reported.
    """
    if not p.intersects(q):
        return False, 0.0, None
    inter = p.intersection(q)
    area = inter.area
    if area > tol_area:
        return True, area, f'area {area:.3f} mm2 > {tol_area}'
    # Depth test. Eroding ONE shape by `tol` and asking whether it still meets the other is the test
    # for "q penetrates p by more than tol". Eroding both would demand a penetration of 2*tol before
    # reporting anything, which is exactly how a real 0.1 mm overlap slips through a 0.05 mm
    # tolerance. Both directions are checked so the criterion is symmetric.
    pe, qe = p.buffer(-tol), q.buffer(-tol)
    if pe.is_empty or qe.is_empty:
        # Degenerate case: a shape thinner than 2*tol vanishes under the erosion, and an empty
        # geometry meets nothing, so the depth test above would silently pass ANY overlap of a thin
        # part -- two 0.04 mm tall slivers stacked exactly on top of each other read as clean. For
        # such shapes `tol` is not a meaningful absolute yardstick (it exceeds the part itself), so
        # switch to a relative one: an intersection covering more than 1% of the SMALLER shape is a
        # real overlap, while the sliver of area produced by a shared edge under float noise is
        # orders of magnitude below that.
        small = min(p.area, q.area)
        if small > 0 and area > 0.01 * small:
            return True, area, (f'overlap of {100 * area / small:.1f}% of the smaller shape '
                                f'(too thin for the {tol} mm depth test; area {area:.3f} mm2)')
        return False, area, None
    # POSITIVE-AREA intersection, not `intersects()`: a bare boundary touch counts as an
    # intersection in shapely, so an overlap of EXACTLY `tol` -- where the eroded shape's boundary
    # lands on the other's -- would be rejected even though `tol` is documented as the accepted
    # tolerance. Requiring real area makes the contract "deeper than tol", not "tol or more".
    # The epsilon (rather than > 0) absorbs the float noise of the buffer offsetting.
    if pe.intersection(q).area > 1e-9 or qe.intersection(p).area > 1e-9:
        return True, area, f'penetration deeper than {tol} mm (area {area:.3f} mm2)'
    return False, area, None


def validate(path, min_sep, sheet_width, sheet_gap, tol, tol_area, verbose=True):
    """Returns the number of errors found."""
    d = json.load(open(path))
    items = {it['id']: it for it in d['items']}

    sol = d['solution']
    errors = 0
    if 'layouts' in sol:            # BPP
        layouts = sol['layouts']
        bins = {b['id']: b for b in d['bins']}

        def container_of(lay):
            return poly_of(bins[lay['container_id']]['shape'])

        def zones_of(lay):
            return zone_polys(bins[lay['container_id']])

        # Stock check. A bin declaring `stock: n` may be cut n times at most; the solver emitting
        # more layouts for it than that describes a plan that cannot be executed, which is a
        # feasibility error just like an overlap -- and it is invisible to every per-layout check
        # below, since each individual layout is perfectly legal on its own.
        used = {}
        for lay in layouts:
            used[lay['container_id']] = used.get(lay['container_id'], 0) + 1
        for bid, n in used.items():
            stock = bins.get(bid, {}).get('stock')
            if stock is not None and n > stock:
                if verbose:
                    print(f'bin {bid}: {n} layouts used but stock is only {stock}')
                errors += 1
    else:                            # SPP
        layouts = [sol['layout']]
        H = d['strip_height']; W = sol['strip_width']

        def container_of(lay):
            return box(0, 0, W, H)

        def zones_of(lay):
            # The strip instance has no bin record, but a future/extended format may carry the
            # zones at the top level; read them if they are there rather than silently skipping.
            return zone_polys(d)

    placed = {}
    sep = min_sep - tol
    for li, lay in enumerate(layouts):
        cont = container_of(lay)
        zones = zones_of(lay)
        polys = []
        for pi in lay['placed_items']:
            it = items[pi['item_id']]
            polys.append((pi['item_id'], place(it, pi['transformation'])))
            placed[pi['item_id']] = placed.get(pi['item_id'], 0) + 1
            # An orientation outside the allowed set is not visible geometrically -- the placement
            # can be perfectly collision-free -- yet it is unmanufacturable whenever the material
            # has a grain/pattern direction, which is exactly why the item declares the list.
            rot = pi['transformation']['rotation']
            allowed = it.get('allowed_orientations')
            if not orientation_ok(rot, allowed):
                if verbose:
                    print(f'layout {li}: item {pi["item_id"]} rotation {rot} not in '
                          f'allowed_orientations {allowed}')
                errors += 1

        # pairwise
        tree = STRtree([p for _, p in polys])
        for i, (iid, p) in enumerate(polys):
            for j in tree.query(p.buffer(max(sep, 0) + tol)):
                if j <= i:
                    continue
                jid, q = polys[j]
                is_ov, area, why = overlaps(p, q, tol, tol_area)
                if is_ov:
                    if verbose:
                        print(f'layout {li}: items {iid} and {jid} OVERLAP: {why}')
                    errors += 1
                else:
                    dist = p.distance(q)
                    if dist < sep:
                        if verbose:
                            print(f'layout {li}: items {iid} and {jid} too close: {dist:.3f} < {min_sep}')
                        errors += 1

        # containment / border distance. Always checked, strip or bin: an item outside its container
        # is just as wrong in a strip solution, and this used to be skipped whenever --sheet-width
        # was absent.
        for iid, p in polys:
            if not cont.buffer(tol).contains(p):
                if verbose:
                    print(f'layout {li}: item {iid} outside container (bbox {p.bounds})')
                errors += 1
            elif cont.exterior.distance(p) < sep:
                if verbose:
                    print(f'layout {li}: item {iid} too close to border: {cont.exterior.distance(p):.3f}')
                errors += 1
            # Containers may have holes too (e.g. a defect zone): nothing may sit in one. The
            # separation requirement applies to a hole exactly as it does to the outer border -- a
            # part 1 mm from a defect under --min-sep 5 is as unusable as one 1 mm from the sheet
            # edge -- so the non-overlapping case still has to be measured.
            for ring in cont.interiors:
                hole = Polygon(ring)
                is_ov, area, why = overlaps(p, hole, tol, tol_area)
                if is_ov:
                    if verbose:
                        print(f'layout {li}: item {iid} overlaps a container hole: {why}')
                    errors += 1
                else:
                    dist = hole.distance(p)
                    if dist < sep:
                        if verbose:
                            print(f'layout {li}: item {iid} too close to a container hole: '
                                  f'{dist:.3f} < {min_sep}')
                        errors += 1

            # Quality zones. A zone is a region of degraded material: an item may only be cut from
            # it if the item tolerates that quality. `min_quality` null means the item demands the
            # best quality available, so EVERY declared zone is forbidden to it. A forbidden zone is
            # then no different from a hole -- the part cannot use that material -- so it gets the
            # same overlap + separation treatment.
            min_q = items[iid].get('min_quality')
            for qual, zone in zones:
                if min_q is not None and qual >= min_q:
                    continue        # the zone is good enough for this item
                is_ov, area, why = overlaps(p, zone, tol, tol_area)
                if is_ov:
                    if verbose:
                        print(f'layout {li}: item {iid} (min_quality {min_q}) overlaps a '
                              f'quality-{qual} zone: {why}')
                    errors += 1
                else:
                    dist = zone.distance(p)
                    if dist < sep:
                        if verbose:
                            print(f'layout {li}: item {iid} (min_quality {min_q}) too close to a '
                                  f'quality-{qual} zone: {dist:.3f} < {min_sep}')
                        errors += 1

        # walls
        if sheet_width:
            Wg = sheet_width + sheet_gap
            n_sheets = math.ceil(cont.bounds[2] / Wg)
            for k in range(1, n_sheets + 1):
                if sheet_gap > 0:
                    # A wall of real thickness: an item may neither overlap it nor come closer than
                    # min_sep to it.
                    wall = box(k * Wg - sheet_gap, cont.bounds[1] - 1, k * Wg, cont.bounds[3] + 1)
                    for iid, p in polys:
                        is_ov, area, why = overlaps(p, wall, tol, tol_area)
                        if is_ov:
                            if verbose:
                                print(f'item {iid} crosses wall {k} (bbox {p.bounds}): {why}')
                            errors += 1
                        else:
                            dist = p.distance(wall)
                            if dist < sep:
                                if verbose:
                                    print(f'item {iid} too close to wall {k}: {dist:.3f}')
                                errors += 1
                else:
                    # `--sheet-gap 0`: the wall has no thickness, so it is a LINE, not a box, and
                    # `box(x, .., x, ..)` is an empty geometry that intersects nothing. Skipping the
                    # check entirely — which is what this used to do — is why a solver export with
                    # two items straddling a boundary validated as OK.
                    #
                    # The cut still happens *somewhere*, so the boundary line itself is the
                    # constraint: an item whose interior the line passes through would be sliced in
                    # two. `crosses`/`intersection.length` on a LineString is the zero-width
                    # equivalent of the area test above. (The engine no longer accepts
                    # `--sheet-gap 0` at all, but a JSON produced by any other tool can still be
                    # handed to this validator, and it must not bless an uncuttable one.)
                    x = k * Wg
                    line = LineString([(x, cont.bounds[1] - 1), (x, cont.bounds[3] + 1)])
                    for iid, p in polys:
                        seg = p.intersection(line)
                        if not seg.is_empty and seg.length > tol:
                            if verbose:
                                print(f'item {iid} crosses sheet boundary {k} at x={x:.1f} '
                                      f'(bbox {p.bounds}): {seg.length:.3f} mm of the boundary line '
                                      f'runs through it')
                            errors += 1
                        elif sep > 0 and p.distance(line) < sep:
                            if verbose:
                                print(f'item {iid} too close to sheet boundary {k}: {p.distance(line):.3f}')
                            errors += 1

            # Per-sheet used width. This is an ERROR, not a report line: an item assigned to sheet k
            # whose right edge reaches past `k*Wg + W` does not fit on the physical sheet, whatever
            # the wall checks above concluded. The audited `--sheet-gap 0` export printed
            # `used 2085.7 / 1995` and `used 120.0 / 100.0` and still exited 0.
            for k in range(n_sheets):
                xs = [p.bounds[2] - k * Wg for _, p in polys
                      if k * Wg <= p.bounds[0] < k * Wg + sheet_width]
                if not xs:
                    continue
                used = max(xs)
                if verbose:
                    print(f'  sheet {k}: {len(xs)} items, used {used:.1f} / {sheet_width}')
                if used > sheet_width + tol:
                    if verbose:
                        print(f'sheet {k}: used width {used:.1f} exceeds the {sheet_width} mm sheet '
                              f'by {used - sheet_width:.1f} mm — the items do not fit on the sheet')
                    errors += 1
        if verbose:
            print(f'layout {li}: {len(polys)} items checked')

    for iid, it in items.items():
        if placed.get(iid, 0) != it['demand']:
            if verbose:
                print(f'item {iid}: placed {placed.get(iid,0)} != demand {it["demand"]}')
            errors += 1
    return errors


def _bpp_case(hole=False, zone=False, stock=10, n_layouts=1, demand=1,
              min_quality=None, tx=0.0, ty=0.0):
    """A minimal one-item BPP instance, for the self-tests that need a whole FILE rather than a
    pair of polygons (stock, quality zones and hole separation are properties of validate(), not of
    overlaps()). Kept as a dict so the self-test stays dependency-free: it is dumped to a temp file
    and fed straight back through the normal validate() entry point."""
    outer = [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]]
    shape = ({'type': 'polygon',
              'data': {'outer': outer,
                       'inner': [[[40.0, 40.0], [60.0, 40.0], [60.0, 60.0], [40.0, 60.0]]]}}
             if hole else
             {'type': 'rectangle', 'data': {'x_min': 0.0, 'y_min': 0.0, 'width': 100.0, 'height': 100.0}})
    zones = ([{'quality': 0,
               'shape': {'type': 'rectangle',
                         'data': {'x_min': 40.0, 'y_min': 40.0, 'width': 20.0, 'height': 20.0}}}]
             if zone else [])
    lay = {'container_id': 0, 'density': 0.0,
           'placed_items': [{'item_id': 0,
                             'transformation': {'rotation': 0.0, 'translation': [tx, ty]}}]}
    return {
        'name': 'selftest',
        'items': [{'id': 0, 'allowed_orientations': None, 'min_quality': min_quality,
                   'demand': demand,
                   'shape': {'type': 'simple_polygon',
                             'data': [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]]}}],
        'bins': [{'id': 0, 'shape': shape, 'zones': zones, 'stock': stock, 'cost': 1}],
        'solution': {'cost': 1, 'density': 0.0, 'run_time_sec': 0,
                     'layouts': [json.loads(json.dumps(lay)) for _ in range(n_layouts)]},
    }


def _run_case(inst, min_sep=0.0, tol=0.05, tol_area=1.0):
    """validate() the given in-memory instance via a throwaway file; returns the error count."""
    import tempfile, os
    fd, path = tempfile.mkstemp(suffix='.json', prefix='validate_selftest_')
    try:
        with os.fdopen(fd, 'w') as fh:
            json.dump(inst, fh)
        return validate(path, min_sep, None, 0.0, tol, tol_area, verbose=False)
    finally:
        os.unlink(path)


def self_test():
    """Checks the validator actually flags the failures it exists to catch."""
    ok = True

    # 1. Two squares overlapping by 0.1 mm along a 10 mm edge: 1.0 mm2 of area, 0.1 mm of
    #    penetration. Must be reported at the default tolerances.
    p = box(0, 0, 10, 10)
    q = box(9.9, 0, 19.9, 10)
    is_ov, area, why = overlaps(p, q, tol=0.05, tol_area=1.0)
    print(f'[self-test] 0.1 mm overlap: flagged={is_ov} area={area:.4f} ({why})')
    if not is_ov:
        print('[self-test] FAIL: a genuine 0.1 mm overlap was NOT flagged'); ok = False

    # 2. Two squares merely touching along an edge: zero area, zero penetration. Must NOT be
    #    reported, or every abutting pair in a tight nest becomes a false positive.
    p = box(0, 0, 10, 10)
    q = box(10, 0, 20, 10)
    is_ov, area, why = overlaps(p, q, tol=0.05, tol_area=1.0)
    print(f'[self-test] touching edges: flagged={is_ov} area={area:.4f}')
    if is_ov:
        print('[self-test] FAIL: two merely touching squares were flagged as overlapping'); ok = False

    # 3. Two squares 4.9 mm apart with --min-sep 5: a separation violation, not an overlap.
    p = box(0, 0, 10, 10)
    q = box(14.9, 0, 24.9, 10)
    tol, min_sep = 0.05, 5.0
    dist = p.distance(q)
    is_ov, _, _ = overlaps(p, q, tol, 1.0)
    violates = (not is_ov) and dist < (min_sep - tol)
    print(f'[self-test] 4.9 mm gap with --min-sep 5: dist={dist:.3f} flagged={violates}')
    if not violates:
        print('[self-test] FAIL: a 4.9 mm gap under --min-sep 5 was NOT flagged'); ok = False

    # 4. The same pair at 5.1 mm must pass.
    q = box(15.1, 0, 25.1, 10)
    dist = p.distance(q)
    violates = dist < (min_sep - tol)
    print(f'[self-test] 5.1 mm gap with --min-sep 5: dist={dist:.3f} flagged={violates}')
    if violates:
        print('[self-test] FAIL: a legal 5.1 mm gap was flagged'); ok = False

    # 5. A polygon with a hole keeps the hole: a part nested inside it must not read as an overlap.
    ring = Polygon([(0, 0), (30, 0), (30, 30), (0, 30)], [[(10, 10), (20, 10), (20, 20), (10, 20)]])
    inner = box(12, 12, 18, 18)
    is_ov, area, _ = overlaps(ring, inner, tol=0.05, tol_area=1.0)
    print(f'[self-test] part nested in a hole: flagged={is_ov} area={area:.4f}')
    if is_ov:
        print('[self-test] FAIL: inner rings are being dropped (hole treated as solid)'); ok = False

    # 6. Orientation matching. -180 must satisfy an allowed set of [0, 180] (the engine exports the
    #    angle it holds, and -180 is the same physical placement as 180), 45 must not, and a null
    #    list means continuous rotation so anything goes.
    #    An EMPTY list is jagua's `RotationRange::None`, i.e. a fixed 0 degrees -- exactly the same
    #    as [0.0], and emphatically NOT "nothing is allowed". Reading it the other way rejected the
    #    engines' own legitimate output; see orientation_ok().
    cases = [(-180.0, [0.0, 180.0], True), (45.0, [0.0], False), (360.0, [0.0], True),
             (0.0009, [0.0], True), (0.5, [0.0], False), (45.0, None, True),
             (0.0, [], True), (360.0, [], True), (-0.0005, [], True),
             (45.0, [], False), (90.0, [], False)]
    bad = [(r, al, exp) for r, al, exp in cases if orientation_ok(r, al) != exp]
    print(f'[self-test] orientation matching (mod 360 / continuous): {len(cases) - len(bad)}/{len(cases)} as expected')
    if bad:
        print(f'[self-test] FAIL: orientation_ok wrong for {bad}'); ok = False

    # 7. An item 1 mm from a container hole under --min-sep 5: not an overlap, but a separation
    #    violation all the same.
    n = _run_case(_bpp_case(hole=True, tx=29.0, ty=45.0), min_sep=5.0)
    print(f'[self-test] item 1 mm from a container hole, --min-sep 5: errors={n}')
    if n != 1:
        print('[self-test] FAIL: min-sep to a container hole is not checked'); ok = False
    n = _run_case(_bpp_case(hole=True, tx=29.0, ty=45.0), min_sep=0.0)
    if n != 0:
        print('[self-test] FAIL: a legal placement near a hole was flagged with --min-sep 0'); ok = False

    # 8. Two layouts on a bin whose stock is 1.
    n = _run_case(_bpp_case(stock=1, n_layouts=2, demand=2))
    print(f'[self-test] 2 layouts of a stock-1 bin: errors={n}')
    if n != 1:
        print('[self-test] FAIL: the BPP stock limit is not enforced'); ok = False
    n = _run_case(_bpp_case(stock=2, n_layouts=2, demand=2))
    if n != 0:
        print('[self-test] FAIL: 2 layouts within a stock of 2 were flagged'); ok = False

    # 9. An item with min_quality 1 sitting inside a quality-0 zone, and the same item merely close
    #    to it under --min-sep 5. A min_quality of 2 (<= zone quality is what matters) is not
    #    involved here; what must NOT fire is an item whose min_quality the zone satisfies.
    n = _run_case(_bpp_case(zone=True, min_quality=1, tx=45.0, ty=45.0), min_sep=5.0)
    print(f'[self-test] item (min_quality 1) inside a quality-0 zone: errors={n}')
    if n != 1:
        print('[self-test] FAIL: quality zones are not checked'); ok = False
    n = _run_case(_bpp_case(zone=True, min_quality=0, tx=45.0, ty=45.0))
    if n != 0:
        print('[self-test] FAIL: a zone the item tolerates was flagged'); ok = False
    n = _run_case(_bpp_case(zone=True, min_quality=None, tx=45.0, ty=45.0))
    if n != 1:
        print('[self-test] FAIL: a null min_quality must forbid every zone'); ok = False

    # 10. Two 20 x 0.04 slivers exactly on top of each other. Eroding by tol=0.05 empties both, so
    #     the absolute depth test is vacuous and only the relative criterion can catch this.
    thin = box(0, 0, 20, 0.04)
    is_ov, area, why = overlaps(thin, box(0, 0, 20, 0.04), tol=0.05, tol_area=1.0)
    print(f'[self-test] identical 20 x 0.04 slivers: flagged={is_ov} area={area:.4f} ({why})')
    if not is_ov:
        print('[self-test] FAIL: a fully overlapping thin item was NOT flagged'); ok = False
    #     ...while two thin parts that merely abut must still pass.
    is_ov, area, _ = overlaps(thin, box(20, 0, 40, 0.04), tol=0.05, tol_area=1.0)
    print(f'[self-test] abutting 20 x 0.04 slivers: flagged={is_ov} area={area:.4f}')
    if is_ov:
        print('[self-test] FAIL: two touching thin items were flagged as overlapping'); ok = False

    # 11. An overlap of EXACTLY tol must pass (tol is the documented tolerance, and the depth test
    #     used to reject it because the eroded boundary merely TOUCHES the other shape), while the
    #     0.1 mm overlap of case 1 stays flagged -- verified there.
    is_ov, area, why = overlaps(box(0, 0, 20, 10), box(19.95, 0, 39.95, 10), tol=0.05, tol_area=1.0)
    print(f'[self-test] overlap of exactly tol (0.05 mm): flagged={is_ov} area={area:.4f}')
    if is_ov:
        print(f'[self-test] FAIL: an overlap of exactly the tolerance was rejected ({why})'); ok = False
    #     Just past the tolerance must still be caught, so the fix did not simply disable the test.
    is_ov, _, _ = overlaps(box(0, 0, 20, 10), box(19.9, 0, 39.9, 10), tol=0.05, tol_area=1.0)
    print(f'[self-test] overlap of 0.1 mm (2x tol): flagged={is_ov}')
    if not is_ov:
        print('[self-test] FAIL: a 0.1 mm overlap slipped through the relaxed depth test'); ok = False

    # 12. Sheet mode with `--sheet-gap 0`. The zero-thickness wall used to be skipped outright
    #     ("nothing to cross"), so an export whose items sat across a boundary validated as OK — the
    #     audited reproduction printed `used 120.0 / 100.0` and still exited 0. Both the boundary
    #     LINE test and the per-sheet used-width test must now fire, and neither may fire on a
    #     layout that genuinely fits.
    def _sheet_case(tx):
        """One 60x45 item at x=tx in a 100 mm-wide sheet strip."""
        return {
            'name': 'selftest_sheet', 'strip_height': 100.0,
            'items': [{'id': 0, 'allowed_orientations': None, 'min_quality': None, 'demand': 1,
                       'shape': {'type': 'simple_polygon',
                                 'data': [[0.0, 0.0], [60.0, 0.0], [60.0, 45.0], [0.0, 45.0]]}}],
            'solution': {'strip_width': 200.0, 'density': 0.0, 'run_time_sec': 0,
                         'layout': {'container_id': 0, 'density': 0.0, 'placed_items': [
                             {'item_id': 0, 'transformation': {'rotation': 0.0, 'translation': [tx, 1.0]}}]}},
        }

    def _run_sheet(inst, sheet_width, sheet_gap):
        import tempfile, os
        fd, path = tempfile.mkstemp(suffix='.json', prefix='validate_selftest_sheet_')
        try:
            with os.fdopen(fd, 'w') as fh:
                json.dump(inst, fh)
            return validate(path, 0.0, sheet_width, sheet_gap, 0.05, 1.0, verbose=False)
        finally:
            os.unlink(path)

    #     An item spanning [70, 130] with W=100, gap=0: the boundary line at x=100 runs through it,
    #     and sheet 0's used width is 130 > 100. Two errors.
    n = _run_sheet(_sheet_case(70.0), 100.0, 0.0)
    print(f'[self-test] --sheet-gap 0, item across the x=100 boundary: errors={n}')
    if n < 1:
        print('[self-test] FAIL: a zero-gap boundary crossing was NOT flagged'); ok = False
    #     The same item wholly inside sheet 0 ([10, 70]) must pass.
    n = _run_sheet(_sheet_case(10.0), 100.0, 0.0)
    print(f'[self-test] --sheet-gap 0, item wholly inside a sheet: errors={n}')
    if n != 0:
        print('[self-test] FAIL: a legal zero-gap layout was flagged'); ok = False

    # 13. The per-sheet used-width check is independent of the gap: with a real 20 mm wall, an item
    #     reaching past its sheet's right edge is still an item that does not fit on the sheet.
    n = _run_sheet(_sheet_case(50.0), 100.0, 20.0)
    print(f'[self-test] sheet used width 110 > 100 (gap 20): errors={n}')
    if n < 1:
        print('[self-test] FAIL: a sheet-local used width beyond W was NOT flagged'); ok = False

    print('SELF-TEST:', 'OK' if ok else 'FAILED')
    return 0 if ok else 1


if a.self_test:
    sys.exit(self_test())

if not a.file:
    ap.error('a file is required unless --self-test is given')

# The minimum separation is a property of the RUN, not of the file: the exported JSON carries the
# original contours, with no record of the inflation the engine applied. There is therefore no way
# to detect a mismatch, and validating a --min-sep 5 solution without passing --min-sep 5 silently
# checks a weaker property than the one that was asked for. Say so, loudly, every time.
banner = (f'assuming --min-sep {a.min_sep} mm' if a.min_sep else
          'assuming --min-sep 0 mm (NO minimum separation)')
print(f'*** {banner} — this CANNOT be read from the file; pass --min-sep explicitly if the run used one ***')
if a.sheet_width:
    print(f'*** sheet mode: width {a.sheet_width} mm, gap {a.sheet_gap} mm ***')

errors = validate(a.file, a.min_sep, a.sheet_width, a.sheet_gap, a.tol, a.tol_area)
print('RESULT:', 'OK' if errors == 0 else f'{errors} ERROR(S)')
sys.exit(1 if errors else 0)
