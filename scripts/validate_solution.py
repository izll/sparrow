#!/usr/bin/env python3
"""Independent (jagua-free) validator for sparrow / sparrow-bpp output JSON.

Checks, on the ORIGINAL contours (not the inflated collision shapes):
  * every pair of placed items keeps >= min_sep (default 0) distance (no overlap)
  * every item lies inside its container (bin rect, or the strip / sheet), with >= min_sep from the border
  * multi-sheet strip: no item crosses a wall [k(W+g)-g, k(W+g)], distance to walls >= min_sep
  * all demand placed

Overlap is judged by TWO independent criteria, because a single scalar tolerance cannot serve both
roles: a shared *edge* between two abutting parts has zero area but non-zero length, while a genuine
0.1 mm interpenetration over a long edge has a large area. So:
  * AREA:      intersection.area > --tol-area  (default 1.0 mm2)  -> OVERLAP
  * DEPTH:     p.buffer(-tol) intersects q.buffer(-tol)           -> OVERLAP
                (penetration deeper than --tol, regardless of how little area it covers)

usage: validate_solution.py <final.json> [--min-sep S] [--sheet-width W --sheet-gap G] [--tol T]
       validate_solution.py --self-test
"""
import json, math, sys, argparse
from shapely.geometry import Polygon, box
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


def overlaps(p, q, tol, tol_area):
    """(is_overlap, area, reason) for a pair of placed contours.

    Two independent criteria, either of which is conclusive:
      * the intersection covers more than `tol_area` mm2, or
      * both shapes still intersect after being eroded by `tol`, i.e. they interpenetrate by more
        than `tol` somewhere. This is what catches a genuine 0.1 mm overlap along a long edge that
        an area-only test with a generous tolerance would wave through, and equally what stops a
        shared edge (zero-area, zero-depth) from being reported.
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
    if p.buffer(-tol).intersects(q) or q.buffer(-tol).intersects(p):
        return True, area, f'penetration deeper than {tol} mm (area {area:.3f} mm2)'
    return False, area, None


def validate(path, min_sep, sheet_width, sheet_gap, tol, tol_area, verbose=True):
    """Returns the number of errors found."""
    d = json.load(open(path))
    items = {it['id']: it for it in d['items']}

    sol = d['solution']
    if 'layouts' in sol:            # BPP
        layouts = sol['layouts']
        bins = {b['id']: b for b in d['bins']}

        def container_of(lay):
            return poly_of(bins[lay['container_id']]['shape'])
    else:                            # SPP
        layouts = [sol['layout']]
        H = d['strip_height']; W = sol['strip_width']

        def container_of(lay):
            return box(0, 0, W, H)

    errors = 0
    placed = {}
    sep = min_sep - tol
    for li, lay in enumerate(layouts):
        cont = container_of(lay)
        polys = []
        for pi in lay['placed_items']:
            it = items[pi['item_id']]
            polys.append((pi['item_id'], place(it, pi['transformation'])))
            placed[pi['item_id']] = placed.get(pi['item_id'], 0) + 1

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
            # Containers may have holes too (e.g. a defect zone): nothing may sit in one.
            for ring in cont.interiors:
                hole = Polygon(ring)
                is_ov, area, why = overlaps(p, hole, tol, tol_area)
                if is_ov:
                    if verbose:
                        print(f'layout {li}: item {iid} overlaps a container hole: {why}')
                    errors += 1

        # walls
        if sheet_width:
            Wg = sheet_width + sheet_gap
            n_sheets = math.ceil(cont.bounds[2] / Wg)
            for k in range(1, n_sheets + 1):
                if sheet_gap <= 0:
                    continue    # zero-width wall: nothing to cross
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
            # per-sheet report
            if verbose:
                for k in range(n_sheets):
                    xs = [p.bounds[2] - k * Wg for _, p in polys
                          if k * Wg <= p.bounds[0] < k * Wg + sheet_width]
                    if xs:
                        print(f'  sheet {k}: {len(xs)} items, used {max(xs):.1f} / {sheet_width}')
        if verbose:
            print(f'layout {li}: {len(polys)} items checked')

    for iid, it in items.items():
        if placed.get(iid, 0) != it['demand']:
            if verbose:
                print(f'item {iid}: placed {placed.get(iid,0)} != demand {it["demand"]}')
            errors += 1
    return errors


def self_test():
    """Checks the validator actually flags the two failures it exists to catch."""
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
