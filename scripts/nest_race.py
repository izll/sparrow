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

Trust model — the race never takes an engine at its word:
  * the per-engine working directory is WIPED before every run, so no output of a previous run can survive
    into this one, and the engine's output file must be NEWER than the moment the race started;
  * what gets validated is rebuilt from the AUTHORITATIVE input instance (-i) plus only the engine's
    placements, never the geometry the engine embedded in its own output — a stale or foreign placement
    set therefore fails validation instead of quietly winning;
  * the final result.json is re-validated (sheet-local coordinates re-expressed as a BPP-style JSON, one
    layout per sheet) before the race exits 0;
  * an engine that raises is recorded as a failed engine, it does not abort the race.

The race also keeps a CPU budget: each sparrow run uses ~3 worker threads, so `engines x -p x workers` can
easily exceed the machine. When it does, `-p` is reduced (never below 1) and the reduction is reported.

Output: <out>/result.json  (see `write_result`), <out>/summary.json, and the raw engine outputs under <out>/<engine>/.
Exit code 0 if at least one engine produced a validated solution AND the written result.json re-validates.

Example:
    scripts/nest_race.py -i job.json --sheet 1995x995 --min-sep 5 -t 50 -s 42 -o out/ \
        --sparrow target/release/sparrow --sparrow-bpp target/release/sparrow-bpp

    scripts/nest_race.py --self-test        # dependency-free checks of the anti-stale / metric fixes
"""
import argparse, json, math, os, subprocess, sys, time, shutil, tempfile, textwrap, traceback
from concurrent.futures import ThreadPoolExecutor
from shapely.geometry import Polygon, box

HERE = os.path.dirname(os.path.abspath(__file__))
VALIDATOR = os.path.join(HERE, 'validate_solution.py')

# sparrow's default worker count per independent run (src/config.rs: n_workers = 3), overridable by the
# same env var the engine itself reads, so the race's CPU estimate matches what will actually be spawned.
SPARROW_WORKERS_PER_RUN = 3

ap = argparse.ArgumentParser()
# -i/--sheet/-o are NOT argparse-required: --self-test must be usable on its own. They are validated by
# hand below, so the error message for a real run is unchanged in substance.
ap.add_argument('-i', '--input', help='sparrow strip instance JSON (items + strip_height)')
ap.add_argument('--sheet', help='usable sheet size WxH (mm), e.g. 1995x995')
ap.add_argument('--min-sep', type=float, default=0.0)
ap.add_argument('--sheet-gap', type=float, default=None, help='virtual wall thickness for the sheets engine (default max(20, 2*min_sep))')
ap.add_argument('-t', '--time', type=int, default=50, help='total time budget per engine (s), split 60/40 explore/compress')
ap.add_argument('-s', '--seed', type=int, default=42)
ap.add_argument('-p', '--parallel-runs', type=int, default=1, help='independent seeds per sparrow engine (-p)')
ap.add_argument('-o', '--out')
ap.add_argument('--engines', default='sheets,bpp', help='comma list of sheets,bpp,ps')
ap.add_argument('--sparrow', default=os.path.join(HERE, '..', 'target', 'release', 'sparrow'))
ap.add_argument('--sparrow-bpp', default=os.path.join(HERE, '..', 'target', 'release', 'sparrow-bpp'))
ap.add_argument('--packingsolver', default=None, help='path to packingsolver_irregular (enables engine ps)')
ap.add_argument('--tol', type=float, default=0.05)
ap.add_argument('--sequential', action='store_true', help='run engines one after another (each gets the whole CPU; total time = engines x t) instead of concurrently')
ap.add_argument('--self-test', action='store_true', help='run the built-in checks (fake engines, no binaries needed) and exit')
a = ap.parse_args()

SELF_TEST = a.self_test
if not SELF_TEST:
    missing = [n for n, v in (('-i/--input', a.input), ('--sheet', a.sheet), ('-o/--out', a.out)) if not v]
    if missing: ap.error('the following arguments are required: ' + ', '.join(missing))

# For --self-test these globals are placeholders; the self-test rebinds them per fixture.
W = H = 0.0
GAP = 0.0
engines = [e.strip() for e in a.engines.split(',') if e.strip()]
inst = {}
items = {}
name = 'instance'
te = tc = 1
# Every engine's output must be newer than this. Set once, just before the engines are launched.
RACE_T0 = 0.0
# Slack on the freshness comparison. time.time() and the filesystem's mtime are not the same clock and do
# not have the same resolution (ext4 stores ns, but coarse timer ticks and NFS/FAT granularity are real),
# so a file written milliseconds after the race started can report an mtime a hair BEFORE RACE_T0. A stale
# file is stale by minutes or hours, so two seconds of slack costs nothing and stops false rejections.
MTIME_SLACK = 2.0

if not SELF_TEST:
    W, H = (float(v) for v in a.sheet.lower().split('x'))
    GAP = a.sheet_gap if a.sheet_gap is not None else max(20.0, 2 * a.min_sep)
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

def shape_poly(shape):
    """The shape as a Shapely polygon, WITH its inner rings — same reading as validate_solution.poly_of.

    poly_pts() deliberately returns only the outer ring (it is used for bbox / rotation maths, where the
    holes are irrelevant), but an area computed from it counts every hole as solid material. That is what
    made the per-sheet density / internal_gaps figures wrong for holed parts, and — because gaps_other is
    a selection key — could hand the race to the worse engine on the time tie-break.
    """
    t, da = shape['type'], shape['data']
    if t == 'simple_polygon': return Polygon(da)
    if t == 'polygon': return Polygon(da['outer'], da.get('inner', []) or [])
    if t == 'rectangle': return box(da['x_min'], da['y_min'], da['x_min'] + da['width'], da['y_min'] + da['height'])
    raise SystemExit(f'unsupported shape {t}')

def placed_bbox(item, rot_deg, tx, ty):
    r = math.radians(rot_deg); c, s = math.cos(r), math.sin(r)
    xs, ys = [], []
    for x, y in poly_pts(item['shape']):
        xs.append(c*x - s*y + tx); ys.append(s*x + c*y + ty)
    return min(xs), min(ys), max(xs), max(ys)

def item_area(item):
    # Holes subtracted: see shape_poly(). Shapely's .area is the signed-corrected net area.
    return shape_poly(item['shape']).area

# ---------------------------------------------------------------- engines
def fresh_dir(sub):
    """Per-engine working directory, WIPED first.

    The engine writes output/final_<name>.json under its cwd. If a previous race left one there and this
    run's engine dies (or exits 0 without writing), the stale file would be read as this run's result —
    with the previous run's geometry, item sizes and all. Removing the whole tree makes that impossible.
    """
    d = os.path.join(a.out, sub)
    shutil.rmtree(d, ignore_errors=True)
    os.makedirs(d, exist_ok=True)
    return d

def run(cmd, cwd):
    t0 = time.time()
    with open(os.path.join(cwd, 'stdout.txt'), 'w') as f:
        rc = subprocess.call(cmd, cwd=cwd, stdout=f, stderr=subprocess.STDOUT)
    return rc, time.time() - t0

def fresh_output(path):
    """(ok, error) — the file exists AND was written by THIS run.

    Second line of defence behind fresh_dir(): a directory wipe cannot protect against an engine that is
    handed a pre-populated -o, or against a file the engine left behind from an earlier internal attempt.
    A mtime older than the race start means the file is not this run's answer, whatever the exit code said.
    """
    if not os.path.exists(path): return False, 'engine wrote no output file'
    mt = os.path.getmtime(path)
    if mt < RACE_T0 - MTIME_SLACK:
        return False, f'output file is stale ({RACE_T0 - mt:.1f}s older than the race start)'
    return True, None

def rebuild_for_validation(placed_layouts, kind, strip_width=None):
    """Build the JSON the validator will actually check, from the AUTHORITATIVE input.

    `placed_layouts` is a list of placed_items lists (one per layout) taken from the engine's output — the
    ONLY thing we borrow from it. items / strip_height / bins come from `inst`, i.e. from the -i file the
    user asked us to nest. Validating the engine's own JSON instead would validate the engine's private
    copy of the geometry, so a solution for entirely different parts (a stale file, a file from another
    job that happens to share the instance name and item ids) passes with flying colours. With the
    rebuild, foreign placements are checked against the real parts and overlap / containment fails.
    """
    base = {'name': name, 'items': inst['items']}
    if kind == 'spp':
        # Single walled strip: one layout, container is the strip. The strip width is NOT free to invent:
        # the validator measures every item's distance to the container border, so a strip trimmed to the
        # rightmost item puts that item 0 mm from the edge and manufactures a --min-sep violation that the
        # engine never committed. Take the engine's declared strip_width when it has one, and otherwise
        # round the extent up to a whole number of sheet pitches, which is the strip the walls imply.
        pis = placed_layouts[0] if placed_layouts else []
        extent = max([placed_bbox(items[p['item_id']], p['transformation']['rotation'], *p['transformation']['translation'])[2]
                      for p in pis] or [0.0])
        if strip_width is None:
            pitch = W + GAP
            strip_width = math.ceil(extent / pitch) * pitch if pitch > 0 else extent
        base['strip_height'] = inst['strip_height']
        base['solution'] = {'strip_width': max(strip_width, extent), 'density': 0.0, 'run_time_sec': 0,
                            'layout': {'container_id': 0, 'density': 0.0, 'placed_items': pis}}
    else:
        base['bins'] = [{'id': 0, 'shape': {'type': 'rectangle', 'data': {'x_min': 0.0, 'y_min': 0.0, 'width': W, 'height': H}},
                         'zones': [], 'stock': 1000, 'cost': 1}]
        base['solution'] = {'cost': len(placed_layouts), 'density': 0.0, 'run_time_sec': 0,
                            'layouts': [{'container_id': 0, 'density': 0.0, 'placed_items': pis} for pis in placed_layouts]}
    return base

def validate_json(obj, val_args, tag):
    """Write `obj` to a temp file and run the independent validator on it. Returns (ok, last_line)."""
    fd, path = tempfile.mkstemp(prefix=f'nest_race_{tag}_', suffix='.json'); os.close(fd)
    try:
        with open(path, 'w') as f: json.dump(obj, f)
        v = subprocess.run([sys.executable, VALIDATOR, path, '--tol', str(a.tol)] + val_args,
                           capture_output=True, text=True)
        out = v.stdout.strip().splitlines()[-1] if v.stdout.strip() else (v.stderr or '')[-200:]
        return v.returncode == 0, out
    finally:
        try: os.unlink(path)
        except OSError: pass

def engine_sheets():
    d = fresh_dir('sheets')
    cmd = [os.path.abspath(a.sparrow), '-i', os.path.abspath(a.input), '--sheet-width', str(W), '--sheet-gap', str(GAP),
           '-e', str(te), '-c', str(tc), '-s', str(a.seed), '-p', str(a.parallel_runs)]
    if a.min_sep > 0: cmd += ['--min-sep', str(a.min_sep)]
    rc, dt = run(cmd, d)
    out = os.path.join(d, 'output', f'final_{name}.json')
    if rc != 0: return {'engine': 'sheets', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    fresh, why = fresh_output(out)
    if not fresh: return {'engine': 'sheets', 'ok': False, 'error': why, 'time': dt}
    sol = json.load(open(out))
    pis = sol['solution']['layout']['placed_items']
    sheets = {}
    for pi in pis:
        tr = pi['transformation']; it = items[pi['item_id']]
        x0, y0, x1, y1 = placed_bbox(it, tr['rotation'], *tr['translation'])
        k = int(math.floor(x0 / (W + GAP)))
        if x1 > k * (W + GAP) + W + a.tol: return {'engine': 'sheets', 'ok': False, 'error': f'item {pi["item_id"]} crosses a sheet boundary', 'time': dt}
        sheets.setdefault(k, []).append({'item_id': pi['item_id'], 'rotation_deg': tr['rotation'],
                                         'x': tr['translation'][0] - k * (W + GAP), 'y': tr['translation'][1],
                                         'bbox_local': (x0 - k*(W+GAP), y0, x1 - k*(W+GAP), y1)})
    # Only the strip WIDTH is taken from the engine's file (a scalar container dimension, cross-checked
    # against the placements below); the item geometry still comes from the authoritative input.
    return finish('sheets', out, sheets, dt, ['--min-sep', str(a.min_sep), '--sheet-width', str(W), '--sheet-gap', str(GAP)],
                  rebuild_for_validation([pis], 'spp', strip_width=sol['solution'].get('strip_width')))

def engine_bpp():
    d = fresh_dir('bpp')
    cmd = [os.path.abspath(a.sparrow_bpp), '-i', os.path.abspath(a.input), '--bin', f'{W}x{H}:1000:1',
           '-e', str(te), '-c', str(tc), '-s', str(a.seed), '-p', str(a.parallel_runs)]
    if a.min_sep > 0: cmd += ['--min-sep', str(a.min_sep)]
    rc, dt = run(cmd, d)
    out = os.path.join(d, 'output', f'final_{name}.json')
    if rc != 0: return {'engine': 'bpp', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    fresh, why = fresh_output(out)
    if not fresh: return {'engine': 'bpp', 'ok': False, 'error': why, 'time': dt}
    sol = json.load(open(out))
    layouts = sol['solution']['layouts']
    sheets = {}
    for k, lay in enumerate(layouts):
        for pi in lay['placed_items']:
            tr = pi['transformation']; it = items[pi['item_id']]
            sheets.setdefault(k, []).append({'item_id': pi['item_id'], 'rotation_deg': tr['rotation'],
                                             'x': tr['translation'][0], 'y': tr['translation'][1],
                                             'bbox_local': placed_bbox(it, tr['rotation'], *tr['translation'])})
    return finish('bpp', out, sheets, dt, ['--min-sep', str(a.min_sep)],
                  rebuild_for_validation([lay['placed_items'] for lay in layouts], 'bpp'))

def engine_ps():
    if not a.packingsolver: return {'engine': 'ps', 'ok': False, 'error': 'no --packingsolver binary', 'time': 0}
    d = fresh_dir('ps')
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
    if rc != 0: return {'engine': 'ps', 'ok': False, 'error': f'exit {rc}', 'time': dt}
    fresh, why = fresh_output(cert)
    if not fresh: return {'engine': 'ps', 'ok': False, 'error': why, 'time': dt}
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
    bpp_like = rebuild_for_validation([[{'item_id': p['item_id'], 'transformation': {'rotation': p['rotation_deg'], 'translation': [p['x'], p['y']]}}
                                        for p in sheets[kk]] for kk in sorted(sheets)], 'bpp')
    out = os.path.join(d, 'as_bpp.json'); json.dump(bpp_like, open(out, 'w'))
    return finish('ps', out, sheets, dt, ['--min-sep', str(a.min_sep)], bpp_like)

def finish(engine, out_json, sheets, dt, val_args, to_validate):
    # Independent validation of the REBUILT json (authoritative input + this engine's placements only),
    # never of the engine's own file: see rebuild_for_validation().
    ok, vline = validate_json(to_validate, val_args, engine)
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
    return {'engine': engine, 'ok': ok, 'validator': vline,
            'time': dt, 'n_sheets': len(keys), 'sheets': per, 'placements': {k: sheets[k] for k in keys},
            'last_band': last['leftover_band'] if last else 0.0,
            'gaps_other': sum(p['internal_gaps'] for p in per[:-1]) if per else 0.0, 'raw': out_json}

def safe_engine(e):
    """One engine's failure is that engine's failure only.

    A corrupt output JSON (JSONDecodeError), an unexpected key, a missing item id — any of these used to
    propagate out of ThreadPoolExecutor.map and kill the whole race, throwing away the other engine's
    perfectly good result. Now it is just another {ok: False} row.
    """
    try:
        return funcs[e]()
    except Exception as exc:
        return {'engine': e, 'ok': False, 'error': f'{type(exc).__name__}: {exc}', 'time': 0,
                'traceback': traceback.format_exc(limit=3)}

# ---------------------------------------------------------------- CPU budget
def cpu_budget(n_engines_concurrent, parallel_runs, cpus=None, workers=None):
    """(parallel_runs, warning) — keep engines x runs x workers within the CPU count.

    Each sparrow -p run spawns its own separator workers (3 by default, SPARROW_N_WORKERS overrides), so a
    16-CPU box asked for `-p 16` with two concurrent engines really gets 2*16*3 = 96 runnable threads.
    They do not go faster; they thrash, and the wall-clock time budget each engine was given is then spent
    on context switching. Reduce -p until the product fits, never below 1 (one run per engine is the
    minimum unit of work; if even that overcommits, there is nothing left to cut).
    """
    cpus = cpus or os.cpu_count() or 1
    if workers is None:
        workers = SPARROW_WORKERS_PER_RUN
        env = os.environ.get('SPARROW_N_WORKERS')
        if env:
            try:
                v = int(env)
                if v > 0: workers = v
            except ValueError:
                pass
    total = n_engines_concurrent * parallel_runs * workers
    if total <= cpus: return parallel_runs, None
    allowed = max(1, cpus // max(1, n_engines_concurrent * workers))
    if allowed >= parallel_runs: return parallel_runs, None
    warn = (f'CPU budget: {n_engines_concurrent} concurrent engine(s) x -p {parallel_runs} x {workers} workers/run '
            f'= {total} threads > {cpus} CPUs; reducing -p to {allowed} '
            f'({n_engines_concurrent * allowed * workers} threads). Pass --sequential to give each engine the whole CPU.')
    return allowed, warn

# ---------------------------------------------------------------- final result re-validation
def result_as_bpp(res):
    """result.json (SHEET-LOCAL coordinates) re-expressed as a BPP-style JSON the validator can read.

    result.json is the artefact the caller will actually cut from, and it is NOT the engine's file: the
    coordinates have been translated per sheet and the sheets re-ordered. A bug anywhere in that
    conversion would be invisible if we only validated the engine output, so the written file is checked
    once more on its own terms — every sheet becomes one layout in a WxH bin, using exactly the stored
    local x/y/rotation.
    """
    return {'name': name, 'items': inst['items'],
            'bins': [{'id': 0, 'shape': {'type': 'rectangle', 'data': {'x_min': 0.0, 'y_min': 0.0, 'width': res['sheet']['width'], 'height': res['sheet']['height']}},
                      'zones': [], 'stock': 1000, 'cost': 1}],
            'solution': {'cost': len(res['sheets']), 'density': 0.0, 'run_time_sec': 0,
                         'layouts': [{'container_id': 0, 'density': 0.0,
                                      'placed_items': [{'item_id': q['item_id'],
                                                        'transformation': {'rotation': q['rotation_deg'], 'translation': [q['x'], q['y']]}}
                                                       for q in s['placements']]}
                                     for s in res['sheets']]}}

# ---------------------------------------------------------------- run, select, write
funcs = {'sheets': engine_sheets, 'bpp': engine_bpp, 'ps': engine_ps}

def write_result(best):
    res = {'instance': name, 'sheet': {'width': W, 'height': H}, 'min_sep': a.min_sep, 'engine': best['engine'],
           'n_sheets': best['n_sheets'],
           'sheets': [{'index': p['sheet'], 'used_width': p['used_width'], 'density': p['density'], 'leftover_band': p['leftover_band'],
                       'placements': [{'item_id': q['item_id'], 'rotation_deg': q['rotation_deg'], 'x': q['x'], 'y': q['y']} for q in best['placements'][p['sheet']]]}
                      for p in best['sheets']]}
    json.dump(res, open(os.path.join(a.out, 'result.json'), 'w'), indent=1)
    return res

def main():
    global RACE_T0
    log(f'{name}: {sum(it.get("demand",1) for it in inst["items"])} items, sheet {W}x{H}, min-sep {a.min_sep}, gap {GAP}, budget {a.time}s, engines {engines}')
    # --sequential runs one engine at a time, so the budget only has to hold for a single engine.
    n_conc = 1 if a.sequential else len([e for e in engines if e in ('sheets', 'bpp')]) or 1
    p, warn = cpu_budget(n_conc, a.parallel_runs)
    if warn: log('WARNING: ' + warn)
    a.parallel_runs = p
    RACE_T0 = time.time()
    with ThreadPoolExecutor(max_workers=1 if a.sequential else len(engines)) as ex:
        results = list(ex.map(safe_engine, engines))
    for r in results:
        if r.get('ok'): log(f"{r['engine']:6s} OK  sheets={r['n_sheets']} last_band={r['last_band']:.1f}mm gaps_other={r['gaps_other']/1e6:.3f}m2 t={r['time']:.0f}s")
        else: log(f"{r['engine']:6s} FAILED: {r.get('error') or r.get('validator')} t={r.get('time',0):.0f}s")
    valid = [r for r in results if r.get('ok')]
    if not valid:
        json.dump({'ok': False, 'results': results}, open(os.path.join(a.out, 'summary.json'), 'w'), indent=1, default=str); return 1
    # leftover band compared in 5 mm steps so that sub-millimetre noise does not outrank internal gaps
    best = sorted(valid, key=lambda r: (r['n_sheets'], -round(r['last_band'] / 5.0), r['gaps_other'], r['time']))[0]
    res = write_result(best)
    # Last gate: the file we are about to declare good must itself validate.
    ok, vline = validate_json(result_as_bpp(res), ['--min-sep', str(a.min_sep)], 'result')
    json.dump({'ok': ok, 'best': best['engine'], 'result_validated': ok, 'result_validator': vline,
               'results': [{k: v for k, v in r.items() if k != 'placements'} for r in results]},
              open(os.path.join(a.out, 'summary.json'), 'w'), indent=1, default=str)
    if not ok:
        log(f'FATAL: the written result.json does NOT validate ({vline}) — refusing to report success'); return 1
    log(f"BEST: {best['engine']} — {best['n_sheets']} sheets, last band {best['last_band']:.1f} mm → {os.path.join(a.out, 'result.json')}")
    return 0

# ---------------------------------------------------------------- self-test
FAKE_ENGINE = textwrap.dedent('''\
    #!{py}
    """Fake sparrow/sparrow-bpp: writes whatever PAYLOAD says into output/final_<NAME>.json under cwd.

    It ignores every flag the race passes it and exits 0 — exactly the shape of the failure mode the
    anti-stale checks exist for: an engine that reports success without producing this run's answer.
    """
    import json, os, sys
    PAYLOAD = {payload}   # None -> write nothing; "corrupt" -> write a broken file; else the JSON dict
    NAME = {iname!r}
    if PAYLOAD is not None:
        os.makedirs('output', exist_ok=True)
        with open(os.path.join('output', 'final_%s.json' % NAME), 'w') as f:
            f.write('{{broken' if PAYLOAD == 'corrupt' else json.dumps(PAYLOAD))
    sys.exit(0)
    ''')

def _fake_engine(path, payload, iname):
    """Write an executable fake engine script; returns its path."""
    with open(path, 'w') as f:
        f.write(FAKE_ENGINE.format(py=sys.executable, payload=repr(payload), iname=iname))
    os.chmod(path, 0o755)
    return path

def _spp_out(iname, inst_obj, placements, width):
    return {'name': iname, 'items': inst_obj['items'], 'strip_height': inst_obj['strip_height'],
            'solution': {'strip_width': width, 'density': 0.9, 'run_time_sec': 1,
                         'layout': {'container_id': 0, 'density': 0.9,
                                    'placed_items': [{'item_id': i, 'transformation': {'rotation': 0.0, 'translation': [x, y]}}
                                                     for i, x, y in placements]}}}

def _bpp_out(iname, inst_obj, layouts, w, h):
    return {'name': iname, 'items': inst_obj['items'],
            'bins': [{'id': 0, 'shape': {'type': 'rectangle', 'data': {'x_min': 0.0, 'y_min': 0.0, 'width': w, 'height': h}}, 'zones': [], 'stock': 1000, 'cost': 1}],
            'solution': {'cost': len(layouts), 'density': 0.9, 'run_time_sec': 1,
                         'layouts': [{'container_id': 0, 'density': 0.9,
                                      'placed_items': [{'item_id': i, 'transformation': {'rotation': 0.0, 'translation': [x, y]}}
                                                       for i, x, y in lay]} for lay in layouts]}}

def _square_inst(iname, side, demand, sh=100.0):
    return {'name': iname, 'strip_height': sh,
            'items': [{'id': 0, 'allowed_orientations': [0.0], 'demand': demand, 'min_quality': None,
                       'shape': {'type': 'simple_polygon',
                                 'data': [[0.0, 0.0], [side, 0.0], [side, side], [0.0, side]]}}]}

def _configure(tmp, inst_obj, sheet_w, sheet_h, out_sub, engine_list, sparrow=None, sparrow_bpp=None):
    """Rebind the module globals the engine functions read, as a real run would."""
    global inst, items, name, W, H, GAP, te, tc, engines
    inst = inst_obj; items = {it['id']: it for it in inst_obj['items']}; name = inst_obj['name']
    W, H = float(sheet_w), float(sheet_h); GAP = 20.0; te = tc = 1
    engines = list(engine_list)
    a.input = os.path.join(tmp, f'{name}_in.json')
    with open(a.input, 'w') as f: json.dump(inst_obj, f)
    a.out = os.path.join(tmp, out_sub); os.makedirs(a.out, exist_ok=True)
    a.min_sep = 0.0; a.parallel_runs = 1; a.sequential = True; a.time = 2
    if sparrow: a.sparrow = sparrow
    if sparrow_bpp: a.sparrow_bpp = sparrow_bpp

def self_test():
    checks = []
    def chk(label, cond, detail=''):
        checks.append(cond)
        print(f'[self-test] {"PASS" if cond else "FAIL"}  {label}' + (f'  ({detail})' if detail else ''))

    global RACE_T0
    tmp = tempfile.mkdtemp(prefix='nest_race_selftest_')
    try:
        # ---- 1. stale result: a previous run left output/final_tiny.json; this run's engine writes nothing.
        real = _square_inst('tiny', 40.0, 2)
        _configure(tmp, real, 100.0, 100.0, 'stale', ['sheets'])
        a.sparrow = _fake_engine(os.path.join(tmp, 'stale_engine'), None, 'tiny')   # exits 0, writes NOTHING
        # Plant the stale file exactly where the engine would have written it, with an ancient mtime —
        # this is the file the old code happily read as "the result".
        d = os.path.join(a.out, 'sheets', 'output'); os.makedirs(d, exist_ok=True)
        planted = os.path.join(d, 'final_tiny.json')
        with open(planted, 'w') as f:
            json.dump(_spp_out('tiny', real, [(0, 0.0, 0.0), (0, 45.0, 0.0)], 85.0), f)
        os.utime(planted, (1e9, 1e9))
        RACE_T0 = time.time()
        r = safe_engine('sheets')
        err = r.get('error') or ''
        chk('stale engine output is rejected, not selected',
            (not r['ok']) and ('stale' in err or 'no output' in err), err)
        chk('the per-engine directory wipe removed the planted stale file', not os.path.exists(planted))

        # ---- 1b. the freshness window must not reject a file the engine really did just write. mtime and
        #          time.time() are different clocks, so a legitimate file can land a few ms "before" T0.
        probe = os.path.join(tmp, 'probe.json'); open(probe, 'w').write('{}')
        RACE_T0 = time.time()
        os.utime(probe, (RACE_T0 - 0.5, RACE_T0 - 0.5))          # just-written, marginally early
        chk('a just-written file inside the clock-slack window is accepted', fresh_output(probe)[0])
        os.utime(probe, (RACE_T0 - 3600, RACE_T0 - 3600))        # an hour old: unambiguously stale
        ok_f, why_f = fresh_output(probe)
        chk('an hour-old file is still rejected as stale', not ok_f, why_f)

        # ---- 2. foreign geometry: the engine returns a placement set that is only feasible for SMALLER
        #        items. Validated against the authoritative 40x40 input it must overlap.
        foreign = _square_inst('tiny', 40.0, 2)
        _configure(tmp, foreign, 100.0, 100.0, 'foreign', ['sheets'])
        # placements legal for 10x10 parts (12 mm apart), fatally overlapping for the real 40x40 ones
        a.sparrow = _fake_engine(os.path.join(tmp, 'foreign_engine'),
                                 _spp_out('tiny', _square_inst('tiny', 10.0, 2), [(0, 0.0, 0.0), (0, 12.0, 0.0)], 22.0), 'tiny')
        RACE_T0 = time.time()
        r = safe_engine('sheets')
        chk('foreign placement set fails validation (rebuild uses the authoritative input)',
            not r['ok'], r.get('error') or r.get('validator'))

        # ---- 2b. the engine cannot buy itself a pass with a generous strip_width: the width is only a
        #          container dimension, the parts are still the authoritative ones and still overlap.
        _configure(tmp, _square_inst('tiny', 40.0, 2), 100.0, 100.0, 'widelie', ['sheets'])
        wide = _spp_out('tiny', _square_inst('tiny', 10.0, 2), [(0, 0.0, 0.0), (0, 12.0, 0.0)], 100000.0)
        a.sparrow = _fake_engine(os.path.join(tmp, 'wide_engine'), wide, 'tiny')
        RACE_T0 = time.time()
        r = safe_engine('sheets')
        chk('an inflated strip_width does not rescue overlapping placements',
            not r['ok'], r.get('error') or r.get('validator'))

        # ---- 3. corrupt JSON from one engine, valid output from the other: only the corrupt one fails.
        two = _square_inst('tiny', 40.0, 2)
        _configure(tmp, two, 100.0, 100.0, 'mixed', ['sheets', 'bpp'])
        a.sparrow = _fake_engine(os.path.join(tmp, 'corrupt_engine'), 'corrupt', 'tiny')
        a.sparrow_bpp = _fake_engine(os.path.join(tmp, 'good_engine'),
                                     _bpp_out('tiny', two, [[(0, 0.0, 0.0), (0, 50.0, 0.0)]], 100.0, 100.0), 'tiny')
        RACE_T0 = time.time()
        rs = [safe_engine(e) for e in ('sheets', 'bpp')]
        by = {r['engine']: r for r in rs}
        chk('a corrupt engine JSON yields ok=false for that engine only',
            (not by['sheets']['ok']) and 'JSONDecodeError' in (by['sheets'].get('error') or ''),
            by['sheets'].get('error'))
        chk('the other engine still produces a valid result', by['bpp']['ok'],
            by['bpp'].get('error') or by['bpp'].get('validator'))

        # ---- 4. hole-aware area: a 100x100 square with a 50x50 hole is 7500 mm2, not 10000.
        holed = {'shape': {'type': 'polygon', 'data': {
            'outer': [[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]],
            'inner': [[[25.0, 25.0], [75.0, 25.0], [75.0, 75.0], [25.0, 75.0]]]}}}
        area = item_area(holed)
        chk('item_area subtracts holes', abs(area - 7500.0) < 1e-6, f'area={area}')
        # and the outer-only reading (the bug) really is different, so the check is not vacuous
        chk('the hole-free reading would have been wrong', abs(10000.0 - area) > 1.0, 'outer=10000')
        # rectangle / simple_polygon still work
        chk('rectangle area still works',
            abs(item_area({'shape': {'type': 'rectangle', 'data': {'x_min': 1.0, 'y_min': 2.0, 'width': 3.0, 'height': 4.0}}}) - 12.0) < 1e-9)
        chk('simple_polygon area still works',
            abs(item_area({'shape': {'type': 'simple_polygon', 'data': [[0, 0], [5, 0], [5, 4], [0, 4]]}}) - 20.0) < 1e-9)

        # ---- 5. CPU budget: 16 CPUs, 2 engines, -p 16, 3 workers -> must come down to 2.
        p, warn = cpu_budget(2, 16, cpus=16, workers=3)
        chk('cpu budget reduces -p when overcommitted', p == 2 and warn is not None, f'-p {p}')
        p, warn = cpu_budget(1, 16, cpus=16, workers=3)
        chk('--sequential (1 concurrent engine) budget is per-engine', p == 5 and warn is not None, f'-p {p}')
        p, warn = cpu_budget(2, 2, cpus=16, workers=3)
        chk('a fitting request is left alone', p == 2 and warn is None)
        p, warn = cpu_budget(8, 4, cpus=2, workers=3)
        chk('-p never drops below 1', p == 1)

        # ---- 6. the final result.json re-validation catches a broken conversion.
        _configure(tmp, _square_inst('tiny', 40.0, 2), 100.0, 100.0, 'finalv', ['bpp'])
        good = {'instance': 'tiny', 'sheet': {'width': 100.0, 'height': 100.0}, 'n_sheets': 1,
                'sheets': [{'index': 0, 'placements': [{'item_id': 0, 'rotation_deg': 0.0, 'x': 0.0, 'y': 0.0},
                                                       {'item_id': 0, 'rotation_deg': 0.0, 'x': 50.0, 'y': 0.0}]}]}
        ok, _ = validate_json(result_as_bpp(good), ['--min-sep', '0.0'], 'st')
        chk('a correct result.json re-validates', ok)
        bad = json.loads(json.dumps(good)); bad['sheets'][0]['placements'][1]['x'] = 10.0   # 30 mm overlap
        ok, line = validate_json(result_as_bpp(bad), ['--min-sep', '0.0'], 'st')
        chk('an overlapping result.json is rejected', not ok, line)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    good = all(checks)
    print(f'SELF-TEST: {"OK" if good else "FAILED"} ({sum(checks)}/{len(checks)})')
    return 0 if good else 1


if __name__ == '__main__':
    sys.exit(self_test() if SELF_TEST else main())
