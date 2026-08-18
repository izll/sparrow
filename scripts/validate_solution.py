#!/usr/bin/env python3
"""Independent (jagua-free) validator for sparrow / sparrow-bpp output JSON.

Checks, on the ORIGINAL contours (not the inflated collision shapes):
  * every pair of placed items keeps >= min_sep (default 0) distance (no overlap)
  * every item lies inside its container (bin rect, or the strip / sheet), with >= min_sep from the border
  * multi-sheet strip: no item crosses a wall [k(W+g)-g, k(W+g)], distance to walls >= min_sep
  * all demand placed
usage: validate_solution.py <final.json> [--min-sep S] [--sheet-width W --sheet-gap G] [--tol T]
"""
import json, math, sys, argparse
from shapely.geometry import Polygon, box
from shapely.strtree import STRtree

ap = argparse.ArgumentParser()
ap.add_argument('file')
ap.add_argument('--min-sep', type=float, default=0.0)
ap.add_argument('--sheet-width', type=float)
ap.add_argument('--sheet-gap', type=float, default=0.0)
ap.add_argument('--tol', type=float, default=0.05, help='geometric tolerance in mm (float32 + simplification)')
a = ap.parse_args()

d = json.load(open(a.file))
items = {it['id']: it for it in d['items']}

def poly_of(shape):
    t, da = shape['type'], shape['data']
    if t == 'simple_polygon': return Polygon(da)
    if t == 'polygon': return Polygon(da['outer'], da.get('inner', []))
    if t == 'rectangle': return box(da['x_min'], da['y_min'], da['x_min']+da['width'], da['y_min']+da['height'])
    raise SystemExit(f'unsupported shape {t}')

def place(item, tr):
    r = math.radians(tr['rotation']); tx, ty = tr['translation']   # ExtTransformation.rotation is in DEGREES
    c, s = math.cos(r), math.sin(r)
    P = poly_of(item['shape'])
    from shapely.affinity import affine_transform
    return affine_transform(P, [c, -s, s, c, tx, ty])   # p' = R p + t

sol = d['solution']
if 'layouts' in sol:            # BPP
    layouts = sol['layouts']
    bins = {b['id']: b for b in d['bins']}
    def container_of(lay):
        b = bins[lay['container_id']]
        sh = b['shape']
        assert sh['type'] == 'rectangle', 'only rectangular bins validated'
        da = sh['data']; return box(da['x_min'], da['y_min'], da['x_min']+da['width'], da['y_min']+da['height'])
else:                            # SPP
    layouts = [sol['layout']]
    H = d['strip_height']; W = sol['strip_width']
    def container_of(lay): return box(0, 0, W, H)

errors = 0; placed = {}
sep = a.min_sep - a.tol
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
        for j in tree.query(p.buffer(max(sep, 0) + a.tol)):
            if j <= i: continue
            jid, q = polys[j]
            dist = p.distance(q)
            if p.intersects(q) and p.intersection(q).area > a.tol:
                print(f'layout {li}: items {iid} and {jid} OVERLAP, area {p.intersection(q).area:.2f}'); errors += 1
            elif dist < sep:
                print(f'layout {li}: items {iid} and {jid} too close: {dist:.3f} < {a.min_sep}'); errors += 1
    # containment / border distance
    for iid, p in polys:
        if not cont.buffer(a.tol).contains(p):
            print(f'layout {li}: item {iid} outside container (bbox {p.bounds})'); errors += 1
        elif cont.exterior.distance(p) < sep:
            print(f'layout {li}: item {iid} too close to border: {cont.exterior.distance(p):.3f}'); errors += 1
    # walls
    if a.sheet_width:
        Wg = a.sheet_width + a.sheet_gap
        n_sheets = math.ceil(cont.bounds[2] / Wg)
        for k in range(1, n_sheets + 1):
            wall = box(k*Wg - a.sheet_gap, cont.bounds[1] - 1, k*Wg, cont.bounds[3] + 1)
            for iid, p in polys:
                if p.intersects(wall) and p.intersection(wall).area > a.tol:
                    print(f'item {iid} crosses wall {k} (bbox {p.bounds})'); errors += 1
                elif p.distance(wall) < sep:
                    print(f'item {iid} too close to wall {k}: {p.distance(wall):.3f}'); errors += 1
        # per-sheet report
        for k in range(n_sheets):
            xs = [p.bounds[2] - k*Wg for _, p in polys if k*Wg <= p.bounds[0] < k*Wg + a.sheet_width]
            if xs: print(f'  sheet {k}: {len(xs)} items, used {max(xs):.1f} / {a.sheet_width}')
    print(f'layout {li}: {len(polys)} items checked')

for iid, it in items.items():
    if placed.get(iid, 0) != it['demand']:
        print(f'item {iid}: placed {placed.get(iid,0)} != demand {it["demand"]}'); errors += 1
print('RESULT:', 'OK' if errors == 0 else f'{errors} ERROR(S)')
sys.exit(1 if errors else 0)
