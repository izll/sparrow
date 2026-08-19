#!/usr/bin/env python3
"""nest_race.py — one command, best validated multi-sheet layout, no manual choices.

Runs several engines in parallel on the same strip-packing instance JSON (items + strip_height) for identical
sheet geometry and time budget, validates every result independently of the engines (scripts/validate_solution.py),
converts each to a common per-sheet representation, and picks the best by:
    1. fewer sheets
    2. larger contiguous leftover band on the last sheet (reusable rectangle)
    3. less internal gap area on the other sheets
    4. shorter run time
Engines:
    sheets   sparrow --sheet-width W --sheet-gap G          (walled multi-sheet strip; sheet k = floor(x/(W+G)))
    bpp      sparrow-bpp --bin WxH                          (bin packing + pack-down / consolidation)
    ps       packingsolver_irregular (optional, if --packingsolver <bin> is given)

Output: <out>/result.json  (see `write_result`), <out>/summary.json, and the raw engine outputs under <out>/<engine>/.
Exit code 0 if at least one engine produced a validated solution.

Example:
    scripts/nest_race.py -i job.json --sheet 1995x995 --min-sep 5 -t 50 -s 42 -o out/ \
        --sparrow target/release/sparrow --sparrow-bpp target/release/sparrow-bpp
"""
import argparse, json, math, os, subprocess, sys, time, shutil
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
VALIDATOR = os.path.join(HERE, 'validate_solution.py')

ap = argparse.ArgumentParser()
ap.add_argument('-i', '--input', required=True, help='sparrow strip instance JSON (items + strip_height)')
ap.add_argument('--sheet', required=True, help='usable sheet size WxH (mm), e.g. 1995x995')
ap.add_argument('--min-sep', type=float, default=0.0)
ap.add_argument('--sheet-gap', type=float, default=None, help='virtual wall thickness for the sheets engine (default max(20, 2*min_sep))')
ap.add_argument('-t', '--time', type=int, default=50, help='total time budget per engine (s), split 60/40 explore/compress')
ap.add_argument('-s', '--seed', type=int, default=42)
ap.add_argument('-p', '--parallel-runs', type=int, default=1, help='independent seeds per sparrow engine (-p)')
ap.add_argument('-o', '--out', required=True)
ap.add_argument('--engines', default='sheets,bpp', help='comma list of sheets,bpp,ps')
ap.add_argument('--sparrow', default=os.path.join(HERE, '..', 'target', 'release', 'sparrow'))
ap.add_argument('--sparrow-bpp', default=os.path.join(HERE, '..', 'target', 'release', 'sparrow-bpp'))
ap.add_argument('--packingsolver', default=None, help='path to packingsolver_irregular (enables engine ps)')
ap.add_argument('--tol', type=float, default=0.05)
ap.add_argument('--sequential', action='store_true', help='run engines one after another (each gets the whole CPU; total time = engines x t) instead of concurrently')
a = ap.parse_args()

W, H = (float(v) for v in a.sheet.lower().split('x'))
GAP = a.sheet_gap if a.sheet_gap is not None else max(20.0, 2 * a.min_sep)
engines = [e.strip() for e in a.engines.split(',') if e.strip()]
os.makedirs(a.out, exist_ok=True)
inst = json.load(open(a.input))
items = {it['id']: it for it in inst['items']}
name = inst.get('name', 'instance')
te = max(1, int(a.time * 0.6)); tc = max(1, a.time - te)

def log(*x): print('[RACE]', *x, flush=True)

# ---------------------------------------------------------------- geometry helpers (original contours)
def poly_pts(shape):
    t, da = shape['type'], shape['data']
    if t == 'simple_polygon': return da
    if t == 'polygon': return da['outer']
    if t == 'rectangle': x, y, w, h = da['x_min'], da['y_min'], da['width'], da['height']; return [(x, y), (x+w, y), (x+w, y+h), (x, y+h)]
    raise SystemExit(f'unsupported shape {t}')

def placed_bbox(item, rot_deg, tx, ty):
    r = math.radians(rot_deg); c, s = math.cos(r), math.sin(r)
    xs, ys = [], []
    for x, y in poly_pts(item['shape']):
        xs.append(c*x - s*y + tx); ys.append(s*x + c*y + ty)
    return min(xs), min(ys), max(xs), max(ys)

def item_area(item):
    P = poly_pts(item['shape']); n = len(P)
    return abs(sum(P[i][0]*P[(i+1) % n][1] - P[(i+1) % n][0]*P[i][1] for i in range(n))) / 2

# ---------------------------------------------------------------- engines
def run(cmd, cwd):
    t0 = time.time()
    with open(os.path.join(cwd, 'stdout.txt'), 'w') as f:
        rc = subprocess.call(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT)
    return rc, time.time() - t0

def engine_sheets():
    d = os.path.join(a.out, 'sheets'); os.makedirs(d, exist_ok=True)
    cmd = [os.path.abspath(a.sparrow), '-i', os.path.abspath(a.input), '--sheet-width', str(W), '--sheet-gap', str(GAP),
           '-e', str(te), '-c', str(tc), '-s', str(a.seed), '-p', str(a.parallel_runs)]
    if a.min_sep > 0: cmd += ['--min-sep', str(a.min_sep)]
    rc, dt = run(cmd, d)
    out = os.path.join(d, 'output', f'final_{name}.json')
    if rc != 0 or not os.path.exists(out): return {'engine': 'sheets', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    sol = json.load(open(out))
    sheets = {}
    for pi in sol['solution']['layout']['placed_items']:
        tr = pi['transformation']; it = items[pi['item_id']]
        x0, y0, x1, y1 = placed_bbox(it, tr['rotation'], *tr['translation'])
        k = int(math.floor(x0 / (W + GAP)))
        if x1 > k * (W + GAP) + W + a.tol: return {'engine': 'sheets', 'ok': False, 'error': f'item {pi["item_id"]} crosses a sheet boundary', 'time': dt}
        sheets.setdefault(k, []).append({'item_id': pi['item_id'], 'rotation_deg': tr['rotation'],
                                         'x': tr['translation'][0] - k * (W + GAP), 'y': tr['translation'][1],
                                         'bbox_local': (x0 - k*(W+GAP), y0, x1 - k*(W+GAP), y1)})
    return finish('sheets', out, sheets, dt, ['--min-sep', str(a.min_sep), '--sheet-width', str(W), '--sheet-gap', str(GAP)])

def engine_bpp():
    d = os.path.join(a.out, 'bpp'); os.makedirs(d, exist_ok=True)
    cmd = [os.path.abspath(a.sparrow_bpp), '-i', os.path.abspath(a.input), '--bin', f'{W}x{H}:1000:1',
           '-e', str(te), '-c', str(tc), '-s', str(a.seed), '-p', str(a.parallel_runs)]
    if a.min_sep > 0: cmd += ['--min-sep', str(a.min_sep)]
    rc, dt = run(cmd, d)
    out = os.path.join(d, 'output', f'final_{name}.json')
    if rc != 0 or not os.path.exists(out): return {'engine': 'bpp', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    sol = json.load(open(out))
    sheets = {}
    for k, lay in enumerate(sol['solution']['layouts']):
        for pi in lay['placed_items']:
            tr = pi['transformation']; it = items[pi['item_id']]
            sheets.setdefault(k, []).append({'item_id': pi['item_id'], 'rotation_deg': tr['rotation'],
                                             'x': tr['translation'][0], 'y': tr['translation'][1],
                                             'bbox_local': placed_bbox(it, tr['rotation'], *tr['translation'])})
    return finish('bpp', out, sheets, dt, ['--min-sep', str(a.min_sep)])

def engine_ps():
    if not a.packingsolver: return {'engine': 'ps', 'ok': False, 'error': 'no --packingsolver binary', 'time': 0}
    d = os.path.join(a.out, 'ps'); os.makedirs(d, exist_ok=True)
    inp = os.path.join(d, 'instance.json')
    ps_items = []
    for it in inst['items']:
        ao = it.get('allowed_orientations')
        o = {'type': 'polygon', 'copies': int(it.get('demand', 1)), 'vertices': [{'x': float(x), 'y': float(y)} for x, y in poly_pts(it['shape'])], 'holes': []}
        if ao is not None: o['allowed_rotations'] = [{'start': float(r), 'end': float(r), 'mirror': False} for r in (ao or [0.0])]
        ps_items.append(o)
    json.dump({'objective': 'bin-packing-with-leftovers', 'parameters': {'item_item_minimum_spacing': a.min_sep},
               'bin_types': [{'type': 'rectangle', 'width': W, 'height': H, 'copies': 1000, 'item_bin_minimum_spacing': a.min_sep}],
               'item_types': ps_items}, open(inp, 'w'))
    cert = os.path.join(d, 'solution.json')
    rc, dt = run([os.path.abspath(a.packingsolver), '--verbosity-level', '1', '--input', inp, '--time-limit', str(a.time), '--certificate', cert], d)
    if rc != 0 or not os.path.exists(cert): return {'engine': 'ps', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    sol = json.load(open(cert))
    # PackingSolver certificate: {"bins":[{"copies", "items":[{"id": item_type_id, "angle" (deg), "x","y", "mirror",
    #   "item_shapes":[{"shape":[LineSegment{xs,ys,xe,ye}...]}]}]}]}. The rotation centre is PS-internal, so recover the
    # translation for our convention (p' = R p + t on the original contour) from the transformed shape's bbox.
    sheets = {}; k = 0
    item_types = list(inst['items'])
    for b in sol.get('bins', []):
        for _ in range(int(b.get('copies', 1))):
            for pi in b.get('items', []):
                it = item_types[int(pi['id'])]; rot = float(pi.get('angle', 0.0))
                if pi.get('mirror'): return {'engine': 'ps', 'ok': False, 'error': 'mirrored placement not supported', 'time': dt}
                segs = [sg for shp in pi['item_shapes'] for sg in shp['shape']]
                txs = [sg['xs'] for sg in segs] + [sg['xe'] for sg in segs]; tys = [sg['ys'] for sg in segs] + [sg['ye'] for sg in segs]
                rx0, ry0, _, _ = placed_bbox(it, rot, 0.0, 0.0)
                tx, ty = min(txs) - rx0, min(tys) - ry0
                sheets.setdefault(k, []).append({'item_id': it['id'], 'rotation_deg': rot, 'x': tx, 'y': ty,
                                                 'bbox_local': placed_bbox(it, rot, tx, ty)})
            k += 1
    # write a sparrow-bpp style JSON so the same validator can check it
    bpp_like = {**{kk: v for kk, v in inst.items() if kk != 'strip_height'},
                'bins': [{'id': 0, 'shape': {'type': 'rectangle', 'data': {'x_min': 0.0, 'y_min': 0.0, 'width': W, 'height': H}}, 'zones': [], 'stock': 1000, 'cost': 1}],
                'solution': {'cost': len(sheets), 'density': 0.0, 'run_time_sec': int(dt), 'layouts': [
                    {'container_id': 0, 'density': 0.0, 'placed_items': [{'item_id': p['item_id'], 'transformation': {'rotation': p['rotation_deg'], 'translation': [p['x'], p['y']]}} for p in sheets[kk]]}
                    for kk in sorted(sheets)]}}
    out = os.path.join(d, 'as_bpp.json'); json.dump(bpp_like, open(out, 'w'))
    return finish('ps', out, sheets, dt, ['--min-sep', str(a.min_sep)])

def finish(engine, out_json, sheets, dt, val_args):
    # independent validation
    v = subprocess.run([sys.executable, VALIDATOR, out_json, '--tol', str(a.tol)] + val_args, capture_output=True, text=True)
    ok = v.returncode == 0
    # Order sheets so that the emptiest one is last (that is the sheet whose leftover band is the reusable rectangle);
    # for the walled strip this is already the natural order, for bins it makes the comparison engine-independent.
    keys = sorted(sheets, key=lambda k: (-max(b['bbox_local'][2] for b in sheets[k]), k))
    per = []
    for k in keys:
        pl = sheets[k]
        used = max(b['bbox_local'][2] for b in pl)
        area = sum(item_area(items[b['item_id']]) for b in pl)
        per.append({'sheet': k, 'n_items': len(pl), 'used_width': used, 'density': area / (W * H),
                    'leftover_band': W - used, 'internal_gaps': used * H - area})
    last = per[-1] if per else None
    return {'engine': engine, 'ok': ok, 'validator': v.stdout.strip().splitlines()[-1] if v.stdout else v.stderr[-200:],
            'time': dt, 'n_sheets': len(keys), 'sheets': per, 'placements': {k: sheets[k] for k in keys},
            'last_band': last['leftover_band'] if last else 0.0,
            'gaps_other': sum(p['internal_gaps'] for p in per[:-1]) if per else 0.0, 'raw': out_json}

# ---------------------------------------------------------------- run, select, write
funcs = {'sheets': engine_sheets, 'bpp': engine_bpp, 'ps': engine_ps}
log(f'{name}: {sum(it.get("demand",1) for it in inst["items"])} items, sheet {W}x{H}, min-sep {a.min_sep}, gap {GAP}, budget {a.time}s, engines {engines}')
with ThreadPoolExecutor(max_workers=1 if a.sequential else len(engines)) as ex:
    results = list(ex.map(lambda e: funcs[e](), engines))
for r in results:
    if r.get('ok'): log(f"{r['engine']:6s} OK  sheets={r['n_sheets']} last_band={r['last_band']:.1f}mm gaps_other={r['gaps_other']/1e6:.3f}m2 t={r['time']:.0f}s")
    else: log(f"{r['engine']:6s} FAILED: {r.get('error') or r.get('validator')} t={r.get('time',0):.0f}s")
valid = [r for r in results if r.get('ok')]
if not valid:
    json.dump({'ok': False, 'results': results}, open(os.path.join(a.out, 'summary.json'), 'w'), indent=1, default=str); sys.exit(1)
# leftover band compared in 5 mm steps so that sub-millimetre noise does not outrank internal gaps
best = sorted(valid, key=lambda r: (r['n_sheets'], -round(r['last_band'] / 5.0), r['gaps_other'], r['time']))[0]

def write_result(best):
    res = {'instance': name, 'sheet': {'width': W, 'height': H}, 'min_sep': a.min_sep, 'engine': best['engine'],
           'n_sheets': best['n_sheets'],
           'sheets': [{'index': p['sheet'], 'used_width': p['used_width'], 'density': p['density'], 'leftover_band': p['leftover_band'],
                       'placements': [{'item_id': q['item_id'], 'rotation_deg': q['rotation_deg'], 'x': q['x'], 'y': q['y']} for q in best['placements'][p['sheet']]]}
                      for p in best['sheets']]}
    json.dump(res, open(os.path.join(a.out, 'result.json'), 'w'), indent=1)
write_result(best)
json.dump({'ok': True, 'best': best['engine'], 'results': [{k: v for k, v in r.items() if k != 'placements'} for r in results]},
          open(os.path.join(a.out, 'summary.json'), 'w'), indent=1, default=str)
log(f"BEST: {best['engine']} — {best['n_sheets']} sheets, last band {best['last_band']:.1f} mm → {os.path.join(a.out, 'result.json')}")
