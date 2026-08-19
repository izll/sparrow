# `feat/bin-packing` második független audit

**Vizsgált ág:** `feat/bin-packing`  
**Vizsgált HEAD:** `2c484601a928ace435d64a925a6d1e917d665cdd`  
**Dátum:** 2026-08-19  
**Módszer:** read-only kódvizsgálat, release build, célzott adverzális fixture-ök, ügyféladat-smoke, validátor- és race-önellenőrzés.  

Ez az audit a `949ec94` állapoton készült első audit javításait ellenőrzi újra. A korábbi kritikus hibák többsége javult, de maradt egy release-ben hibás gyártási eredményt exportáló út, valamint több robusztussági és tesztintegritási hiba.

## Összefoglaló

| Súlyosság | Finding | Állapot |
|---|---|---|
| CRITICAL | BPP warm start tiltott forgatással is exportálható | CONFIRMED |
| HIGH | `allowed_orientations: []` esetet a Python-validátor hibásan elutasítja | CONFIRMED |
| HIGH | A demand-limit `u64` túlcsordulással megkerülhető | CONFIRMED |
| HIGH | A 24/16 lépéses forgatásrács megoldható inputot utasíthat el | CONFIRMED |
| HIGH | Elutasított layout után hibás `final_*.svg` marad | CONFIRMED |
| MEDIUM | Sikertelen race után a korábbi top-level `result.json` megmarad | CONFIRMED |
| MEDIUM | A release E2E tesztek kihagyhatók vagy ellenőrzés nélkül zöldek lehetnek | CONFIRMED |
| MEDIUM | Az iso6 pack-down release teszt wall-clock miatt flaky | CONFIRMED |
| MEDIUM | PackingSolver mellett a race CPU-budgetje hiányos | PLAUSIBLE |
| LOW | Warm start nélküli BPP kétszer futtatja a konstruktorokat | CONFIRMED |

---

## CRITICAL

### BPP warm start tiltott forgatással is exportálható — CONFIRMED

**Érintett fájlok:**

- `src/bpp_main.rs:141-150`
- `src/util/verify.rs:127-144`
- `src/optimizer/bpp/mod.rs:79-104`

Az elem csak 0°-ban helyezhető el, a warm start viszont 45°-os transzformációt tartalmaz. A layout geometriailag ütközésmentes, ezért a Rust-oldali warm-start és final export gate elfogadja. A program exit `0` mellett JSON-t és SVG-t ír.

A Python-validátor a már javított `[0]` ellenőrzéssel helyesen elutasítja ugyanezt az outputot.

Repró:

```bash
mkdir -p /tmp/sparrow-audit-bpp-rotation
cd /tmp/sparrow-audit-bpp-rotation

/path/to/sparrow-bpp \
  -i /tmp/bpp-warm-rotation.json \
  -e 0 -c 0 -s 42

python3 /path/to/sparrow/scripts/validate_solution.py \
  output/final_audit_bpp_warm_rotation.json
```

Input:

```json
{
  "name": "audit_bpp_warm_rotation",
  "items": [{
    "id": 0,
    "demand": 1,
    "allowed_orientations": [0.0],
    "shape": {
      "type": "rectangle",
      "data": {"x_min": 0, "y_min": 0, "width": 10, "height": 5}
    }
  }],
  "bins": [{
    "id": 0,
    "shape": {
      "type": "rectangle",
      "data": {"x_min": 0, "y_min": 0, "width": 30, "height": 30}
    },
    "zones": [],
    "stock": 1,
    "cost": 1
  }],
  "solution": {
    "cost": 1,
    "density": 0.0555556,
    "run_time_sec": 0,
    "layouts": [{
      "container_id": 0,
      "density": 0.0555556,
      "placed_items": [{
        "item_id": 0,
        "transformation": {"rotation": 45.0, "translation": [10.0, 10.0]}
      }]
    }]
  }
}
```

Mért eredmény:

```text
sparrow-bpp exit: 0
exportált rotation: 45.0
validate_solution.py: RESULT: 1 ERROR(S)
```

**Javasolt javítás:** a Rust `verify_spp_solution()` és `verify_bpp_solution()` ellenőrizze minden placement forgatását az item `RotationRange` értékével, moduló 360° és kis fok-tolerancia mellett. Ugyanez fusson warm-start import előtt és a final exportkapuban.

---

## HIGH

### A Python-validátor az üres orientációlistát hibásan elutasítja — CONFIRMED

**Érintett fájl:** `scripts/validate_solution.py:71-81,185-194`

A jagua-rs importere szerint:

```text
allowed_orientations: null / hiányzik -> folyamatos forgatás
allowed_orientations: []             -> RotationRange::None, fix 0°
allowed_orientations: [0, 180]       -> diszkrét lista
```

A validátor az üres listát jelenleg úgy kezeli, mintha egyetlen szög sem lenne megengedett.

Repró:

```bash
python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/cases/no_rot_spp/output/final_audit_no_rotation.json

python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/cases/no_rot_bpp/output/final_audit_no_rotation.json
```

Mindkettő:

```text
item 0 rotation 0.0 not in allowed_orientations []
RESULT: 1 ERROR(S)
```

A teljes `nest_race.py` futásban emiatt mind a sheets, mind a BPP motor legitim eredménye kiesik, a race exit `1`-gyel zár.

**Javasolt javítás:** az üres lista legyen ekvivalens `[0.0]`-val; a self-test jelenlegi „empty means no permitted orientation” elvárását is módosítani kell.

### A demand-limit `u64` túlcsordulással megkerülhető — CONFIRMED

**Érintett fájlok:**

- `src/main.rs:125-130`
- `src/bpp_main.rs:118-123`

A kapu ezt használja:

```rust
let total_demand: u64 = ext_instance.items.iter().map(|it| it.demand).sum();
```

Release-ben `u64::MAX + 1` 0-ra fordul, így a `MAX_TOTAL_DEMAND` ellenőrzés kimarad.

Repró input:

```json
{
  "name": "audit_demand_overflow",
  "items": [
    {
      "id": 0,
      "demand": 18446744073709551615,
      "allowed_orientations": [0.0],
      "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}
    },
    {
      "id": 1,
      "demand": 1,
      "allowed_orientations": [0.0],
      "shape": {"type": "simple_polygon", "data": [[0,0],[1,0],[1,1],[0,1]]}
    }
  ],
  "strip_height": 100.0
}
```

SPP:

```bash
ulimit -v 262144
sparrow -i /tmp/demand-overflow.json -e 0 -c 0 -s 42
```

```text
loaded instance audit_demand_overflow with #0 items
thread 'main' panicked ... no pole found
exit 134
```

BPP:

```bash
ulimit -v 262144
sparrow-bpp -i /tmp/demand-overflow.json --bin 100x100 -e 0 -c 0 -s 42
```

```text
capacity overflow
exit 134
```

**Javasolt javítás:** `try_fold()` + `checked_add()`, overflow esetén tiszta inputhiba; ezen felül minden egyedi demand kapjon explicit felső korlátot. Legyen `MAX_TOTAL_DEMAND + 1` és aritmetikai overflow E2E teszt mindkét binárisra.

### A 24/16 lépéses forgatásrács megoldható inputot utasít el — CONFIRMED

**Érintett fájlok:**

- `src/util/packability.rs:31-43`
- `src/optimizer/sheets.rs:112-122,512-519`
- `src/sample/uniform_sampler.rs:13,34-40`

Az új packability- és sheet-precheck 24 egyenletes szöget vizsgál. A tényleges `UniformBBoxSampler` 16 szöget használ. A két rács nem ekvivalens.

Egy 22,5°-kal előforgatott, 100×10-es téglalap 10,2 mm magas stripbe a 16-os rács `-22,5°` szögén pontosan belefér. A 24-es precheck legközelebbi mintája 7,5°-kal eltér, ezért 23 mm minimális magasságot számol és tévesen elutasítja.

Input:

```json
{
  "name": "audit_rotation_grid",
  "items": [{
    "id": 0,
    "demand": 1,
    "shape": {
      "type": "simple_polygon",
      "data": [
        [0.0, 0.0],
        [92.387953, 38.268343],
        [88.561119, 47.507138],
        [-3.826834, 9.238795]
      ]
    }
  }],
  "strip_height": 10.2
}
```

HEAD `2c48460`:

```bash
sparrow -i /tmp/rotation-grid.json -e 0 -c 0 -s 42
```

```text
Error: the strip is only 10.2 mm high, but item 0 (23.0 mm) does not fit
exit 1
```

A precheck előtti `949ec94` ugyanezzel az inputtal:

```text
final_audit_rotation_grid.json
exit 0
```

**Javasolt javítás:** egyetlen közös `rotations_of()` implementáció legyen, amely pontosan a tényleges sampler által használt rácsot adja. A precheck ne tartson fenn másolatban saját mintaszámot.

### Elutasított layout után hibás `final_*.svg` marad — CONFIRMED

**Érintett fájlok:**

- `src/main.rs:194-214,266-280`
- `src/optimizer/mod.rs:150-166`
- `src/util/svg_exporter.rs:59-63`
- analóg BPP út: `src/bpp_main.rs:255-268`

Az optimizer listener a final SVG-t a Rust exportkapu előtt készíti el. Átfedő warm startnál a program helyesen exit `1`-et ad és JSON-t nem ír, de a hibás layoutot tartalmazó `final_audit_one.svg` már ott marad.

Repró:

```bash
mkdir -p /tmp/sparrow-svg-gate
cd /tmp/sparrow-svg-gate

/path/to/sparrow \
  -i /tmp/claude-1000/sparrow-audit/spp_overlap.json \
  -e 0 -c 0 -s 42

echo $?
find output -maxdepth 1 -type f -printf '%f\n'
```

Eredmény:

```text
Error: refusing to export
exit 1
final_audit_one.svg
log.txt
```

**Javasolt javítás:** a listener csak temp/intermediate fájlt készítsen; a végleges SVG és JSON kizárólag a teljes exportkapu után, atomikusan kerüljön a végleges névre.

---

## MEDIUM

### Sikertelen race után a korábbi top-level `result.json` megmarad — CONFIRMED

**Érintett fájl:** `scripts/nest_race.py:135-160,387-422`

Az engine alkönyvtárak most már futásonként törlődnek, ezért a korábbi motor-output nem kerül kiválasztásra. A top-level `result.json` azonban nincs eltávolítva.

Forgatókönyv:

1. sikeres race ugyanabba az output könyvtárba;
2. a `result.json` hashének mentése;
3. második race más inputtal, olyan fake engine-nel, amely exit 0 mellett nem ír outputot;
4. a második futás exit `1`, `summary.ok=false`, de a korábbi `result.json` hash-e változatlan.

Kapcsolódó hiba: `write_result()` a végső validáció előtt írja ki a fájlt. Ha a final gate bukik, az elutasított result ugyancsak ott marad.

**Javasolt javítás:** induláskor a régi `result.json` és `summary.json` törlése; memóriabeli validáció; csak siker esetén temp fájl + `os.replace()`. Minden failure ágon result unlink.

### A release E2E tesztkapu kihagyható vagy hamisan zöld lehet — CONFIRMED

**Érintett fájlok:**

- `tests/audit_regression_tests.rs:548-600`
- `docs/sheets.md:514-529`

A 10 valódi binárisszintű regressziós teszt `#[ignore]`, ezért a dokumentált:

```bash
cargo test --release
```

nem futtatja őket.

További tesztintegritási probléma:

```rust
PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/release/sparrow")
```

A teszt nem a Cargo által az aktuális tesztfutáshoz épített binárist használja. Alternatív `CARGO_TARGET_DIR` mellett egy régi repo-local binaryt tesztelhet. Ha a bináris hiányzik, az `Option` és `else { return }` miatt a teszt tényleges ellenőrzés nélkül zöld.

Külön futtatva:

```bash
cargo test --release --locked \
  --test audit_regression_tests -- --ignored --nocapture
```

Eredmény: `10/10 PASS`, de a fenti binárisútvonal-korláttal.

**Javasolt javítás:** Cargo által biztosított `CARGO_BIN_EXE_sparrow`; hiánykor hard fail; explicit CI-job az ignored suite-ra; a dokumentációban szerepeljen a teljes parancs. Az exporttilalmi tesztek JSON mellett SVG-t is ellenőrizzenek.

### Az iso6 pack-down release teszt wall-clock miatt flaky — CONFIRMED

**Érintett fájl:** `tests/bpp_packdown_tests.rs:290-353`

Első clean release suite:

```text
pack_down_uses_every_bin_as_source_on_iso6 FAILED
no cross-bin move happened
```

Azonnali izolált újrafuttatás:

```text
PASS
```

A density-vector azonos volt, csak a wall-clock alatt nem történt elfogadott cross-bin move. Ez ellentmond annak, hogy a teszt determinisztikus release gate-ként használható.

**Javasolt javítás:** fix prestate/iteráció alapú teszt; közvetlenül azt ellenőrizze, hogy minden bin source-ként sorra kerül, ne azt, hogy a heurisztika adott időn belül biztosan elfogad egy move-ot.

### PackingSolver mellett a race CPU-budgetje hiányos — PLAUSIBLE

**Érintett fájl:** `scripts/nest_race.py:399-403`

A budget csak a `sheets` és `bpp` engine-eket számolja:

```python
len([e for e in engines if e in ('sheets', 'bpp')])
```

A velük párhuzamos PackingSolver CPU-használata nincs lefoglalva vagy beleszámítva. Két Sparrow motor például 12/16 becsült workerre korlátozható, miközben a PS még további CPU-kat használhat.

**Javasolt javítás:** a PS tényleges thread-limitjét is átadni és beleszámítani; ha nem korlátozható, automatikus sequential mód vagy egyértelmű figyelmeztetés.

---

## LOW

### Warm start nélküli BPP kétszer futtatja a konstruktorokat — CONFIRMED

**Érintett fájlok:**

- `src/bpp_main.rs:163-178`
- `src/optimizer/bpp/mod.rs:176-238`

A CLI először külön LBF- és shelf-probe-ot épít azért, hogy a konstrukciós hibából ne legyen panic. Az `optimize_bpp()` ezután ismét felépíti ugyanazokat a konstruktorokat.

Iso7, `-e 0 -c 0` logban:

```text
[MAIN] LBF probe ...
[MAIN] shelf probe ...
[BPSHELF] ...
[BPOPT] LBF start ... | shelf start ...
```

A teljes startup ezen a kis instance-on 0,12 s volt; összetettebb instance-nál a fölösleges konstrukció drágább lehet.

**Javasolt javítás:** a sikeres probe eredményének átadása tényleges initial solutionként, vagy `optimize_bpp()` térjen vissza `Result`-tal, külön probe nélkül.

---

## Mérési ellenőrzés

### A második audit aktuális HEAD-en futtatott mérései

| Ellenőrzés | Paraméter | Eredmény | Validáció |
|---|---|---:|---|
| iso7 falas sheets | `--sheet-width 1995 --min-sep 5 -e 20 -c 10 -s 42` | 30,05 s; 3 sheet; 33/33; használt szélesség 1989,8 / 1988,8 / 683,7 mm | OK |
| iso6 BPP smoke | `--bin 1990x995:25:1 --min-sep 5 -e 5 -c 5 -s 42` | 8,24 s; 9 bin; 48,192%; 49/49 | OK |
| SPP drift `949ec94` vs `2c48460` | `swim -e 10 -c 5 -s 0`, azonos worker | mindkettő 148 sikeres shrink; exploration 5854,934; közös prefix exact | új export OK |
| Compression végszélesség | azonos fenti futás | 5837,909 vs 5836,791 | dokumentált wall-clock szórás |
| Release ignored E2E | `--test audit_regression_tests -- --ignored` | 10/10 PASS | hardcoded binary caveat |
| Validator self-test | `scripts/validate_solution.py --self-test` | PASS | — |
| Race self-test | `scripts/nest_race.py --self-test` | 18/18 PASS | — |

A korábbi 40+10 s teljesítménymérést ebben a második körben nem futtattuk újra; a `2c48460` nem módosította a SoA hot pathot. Az első audit pontos mérése `971K -> 1910K eval/s`, kb. `1,97×` gyorsulás volt.

Az SPP determinizmus-dokumentáció most helyes:

- az azonos iterációs/shrink-prefix determinisztikusnak látszik;
- a teljes wall-clock futás nem determinisztikus;
- CLI iteration terminator továbbra sincs, ezért teljes end-to-end fixed-iteration determinizmus csak PLAUSIBLE.

---

## Átnézve és rendben

A következő korábbi findingok javítását reprodukáltuk:

- `--sheet-gap 0` már tiszta CLI-hiba, nem csendes fal nélküli export;
- hiányos, duplikált, ismeretlen ID-s és átfedő SPP warm start JSON-exportja blokkolva;
- nem véges CLI számok tiszta hibával leállnak;
- túl magas/széles elem és normál nagy `--min-sep` kezelése javult;
- BPP bin-stock validálása működik;
- quality-zone validálása működik;
- lyukak körüli minimum távolság ellenőrzése működik;
- vékony átfedés felismerése működik;
- a toleranciahatáron levő, valójában feasible eset átmegy;
- a race az autoritatív input geometriájából építi újra a jelölteket;
- stale engine-output nem kerül kiválasztásra;
- sérült engine JSON nem dönti le a másik motort;
- a lyukas polygon nettó területe helyesen számolódik;
- a lokalizált `result.json` visszavalidálása működik;
- BPP-nél a legüresebb tábla kerül utolsóra;
- két Sparrow motor mellett a `-p 16` automatikusan csökkentett;
- a normál iso7 és iso6 ügyfél-smoke helyes;
- az SPP shrink-prefix nem driftelt;
- a wall-clock nondeterminizmus dokumentációja technikailag helyes.

## Release-verdict

A `2c48460` lényegesen jobb a korábban auditált állapotnál, de a **BPP tiltott warm-start forgatásának exit 0 melletti exportja release-blocker**. A demand-overflow abort, a forgatásrács hamis elutasítása és az invalid/stale végleges fájlok miatt a javítások után újabb release-audit és valódi E2E kapu szükséges.


---

## Status after fixes

**Javítási HEAD:** `2c48460` + a jelen munkafa változtatásai (nem commitolt).
**Dátum:** 2026-08-19.

| Súlyosság | Finding | Állapot | Hol |
|---|---|---|---|
| CRITICAL | BPP warm start tiltott forgatással is exportálható | **fixed** | `src/util/rotations.rs:97,125`; gate: `src/util/verify.rs:148,158,199,223`; warm start: `src/util/bpp_io.rs:319`, `src/util/io.rs:302` |
| HIGH 1 | `allowed_orientations: []` hibás elutasítása | **fixed** | `scripts/validate_solution.py:72-95` + self-test 11/11 |
| HIGH 2 | Demand-limit `u64` túlcsordulással megkerülhető | **fixed** | `src/util/demand.rs:43` (`checked_add`, per-item + total cap), hívók: `src/main.rs:117`, `src/bpp_main.rs:114` |
| HIGH 3 | 24/16 lépéses forgatásrács megoldható inputot utasít el | **fixed** | `src/util/rotations.rs:31,42` — egy rács; használók: `src/sample/uniform_sampler.rs:33`, `src/util/packability.rs:50`, `src/optimizer/sheets.rs:112,501` |
| HIGH 4 | Elutasított layout után hibás `final_*.svg` marad | **fixed** | `Final` report a kapu mögé: `src/optimizer/mod.rs:166`, `src/optimizer/bpp/mod.rs:162`; export: `src/main.rs:298-310`, `src/bpp_main.rs:291-305` |
| MEDIUM 5 | Sikertelen race után a `result.json` megmarad | **fixed** | `scripts/nest_race.py`: `clear_previous_outputs()`, `build_result()`, `write_result_atomically()` (`os.replace`), `unlink_result()`; PS thread-budget: `ps_thread_limit()` + `cpu_budget(reserved=)` |
| MEDIUM 6 | Release E2E tesztek kihagyhatók / hamisan zöldek | **fixed** | `CARGO_BIN_EXE_*` (`tests/audit_regression_tests.rs`, `tests/audit2_regression_tests.rs`), `#[ignore]` levéve, `scripts/ci.sh`, `docs/sheets.md` |
| MEDIUM 7 | iso6 pack-down teszt wall-clock miatt flaky | **fixed** | `PackDownStats::sources_visited` (`src/optimizer/bpp/compress.rs:227-243`); determinisztikus teszt + `#[ignore]`-olt long-budget variáns (`tests/bpp_packdown_tests.rs`) |
| LOW 8 | Warm start nélküli BPP kétszer futtatja a konstruktorokat | **fixed** | `optimize_bpp` → `Result` (`src/optimizer/bpp/mod.rs:79`), a probe törölve (`src/bpp_main.rs:193`) |

### A findingonkénti javítás egy mondatban

- **CRITICAL.** Minden placement forgatását ellenőrizzük az item `RotationRange`-éhez képest, 360° moduló és 1e-3° tolerancia mellett, mindkét bináris exportkapujában **és** a warm-start importban. Az audit BPP JSON-ja (`allowed_orientations: [0.0]`, rotation 45) most exit `1`, JSON és SVG nélkül.
- **HIGH 1.** Az üres lista jagua szemantikája szerint fix 0° (`RotationRange::None`), nem „semmi sem megengedett”; a self-test elvárása is javítva.
- **HIGH 2.** A `sum()` helyett `try_fold` + `checked_add`, külön per-item és összesített korláttal — a `u64::MAX` fixture és a `MAX_TOTAL_DEMAND + 1` is tiszta hibaüzenet, mindkét binárison.
- **HIGH 3.** Egyetlen `candidate_rotations()` (16 lépés) — a precheckek ugyanazt a rácsot kérdezik, mint a sampler; ráadásul *continuous* itemet a precheck sosem utasít el, mert a rács csak minta. Az audit 22,5°-os téglalapja 10,2 mm-es stripben újra megoldódik.
- **HIGH 4.** A `Final` report kikerült az optimizerből; a végleges SVG/JSON csak a kapu után íródik, és a kapu előtt a korábbi futás `final_*` maradványait is töröljük — elutasított futás után `output/`-ban nincs `final_*.svg` és nincs `.json`.
- **MEDIUM 5.** A `result.json`/`summary.json` induláskor törlődik, a validáció memóriában fut, írás csak sikernél temp + `os.replace`-szel, minden hibaágon unlink. A PackingSolver thread-limitje a `--help`-ből derül ki, beleszámít a budgetbe, és ha nem korlátozható, a race figyelmeztet és sequential módra vált.
- **MEDIUM 6.** A binárisok `env!("CARGO_BIN_EXE_sparrow")`/`..._sparrow-bpp` alapján jönnek (hiányzó bináris már nem lehet „zöld”), az E2E esetek `-e 0 -c 0`-val alapból futnak, és a teljes kapu a `scripts/ci.sh`-ban van dokumentálva.
- **MEDIUM 7.** A teszt már nem azt méri, hogy a heurisztika időn belül elfogad-e egy move-ot, hanem a determinisztikus tulajdonságot: minden bin sorra kerül source-ként (`sources_visited`). A wall-clockos fele `#[ignore]`-olt, hosszabb budgettel.
- **LOW 8.** `optimize_bpp()` `Result`-ot ad, így a külön LBF/shelf probe elhagyható — a konstruktorok futásonként egyszer futnak.

### Ellenőrzés

| Ellenőrzés | Eredmény |
|---|---|
| `cargo test` | PASS (0 failed) |
| `cargo test --release` | PASS (0 failed) |
| `cargo test --release -- --ignored` | PASS (2 wall-clock teszt) |
| `scripts/validate_solution.py --self-test` | PASS (orientáció 11/11) |
| `scripts/nest_race.py --self-test` | PASS (31/31, korábban 18/18) |
| `cargo clippy --all-targets` | 0 warning |
| `cargo build --release --features only_final_svg` | PASS |
| `scripts/ci.sh` (teljes kapu) | PASS |
| swim seed-0 shrink-prefix vs `2c48460` | **azonos** (302/302 sor, byte-identikus) |

Fail-before/pass-after az audit saját reprodukcióival, `2c48460` release buildjéhez mérve:

| Repró | `2c48460` | javítva |
|---|---|---|
| BPP warm start, rotation 45, `allowed_orientations: [0.0]` | exit `0`, 2 db `final_*` fájl | exit `1`, 0 fájl |
| 22,5°-os téglalap, 10,2 mm strip | exit `1` (téves elutasítás) | exit `0`, exportál |
| `demand: u64::MAX` + 1 (SPP) | exit `134` (abort) | exit `1`, hibaüzenet |
| `demand: u64::MAX` + 1 (BPP) | exit `134` (abort) | exit `1`, hibaüzenet |

Új tesztfájl: `tests/audit2_regression_tests.rs` (28 teszt), plusz `src/util/demand.rs` unit tesztjei (4) és az átírt `tests/bpp_packdown_tests.rs` pack-down property teszt.
