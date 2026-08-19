# `feat/bin-packing` független helyességi és teljesítmény-audit

**Vizsgált ág:** `feat/bin-packing`  
**Vizsgált commit:** `949ec9454549c987b2226917f205b0f3f136f1db`  
**Upstream alap:** `961ec31`  
**Vizsgálat dátuma:** 2026-08-19  
**Környezet:** Rust stable 1.90, release build, jagua-rs 0.7.2

## Összefoglaló

A branch jelen állapotában nem release-ready. Négy reprodukált kritikus helyességi hiba van; ebből kettő közvetlenül rossz layoutot exportálhat, egy a versenyeztetőn keresztül engedhet ki idegen vagy korábbi geometriát, egy pedig a nulla lemezhézag kezelését érinti.

A numerikus gyorsítás és a normál BPP/falas algoritmus eredményei meggyőzőek. A kb. `1000K → 2000K` teljesítményállítás reprodukálható, a nyolc normál referenciafutás a projekt validátorán átment. Az SPP warm start, a nulla gap, a race autoritatív inputkezelése és a validátor hiányosságai azonban javítás nélkül kizárják a biztonságos release-t.

## CRITICAL

### CONFIRMED — `--sheet-gap 0` vághatatlan layoutot exportál

Hely:

- `src/config.rs:149-160`
- `src/optimizer/sheets.rs:160-165`
- `src/main.rs:216-225`
- `scripts/validate_solution.py:155-174`

A nulla szélességű falakat a solver eldobja. Az iso7 futás 10 lemezhatárt keresztező elemet jelez, mégis exit `0` és JSON export történik. A validátor is hibásan elfogadja: `used 2085.7/1995`, illetve `2166.6/1995`.

Repró:

```bash
cd /tmp/claude-1000/sparrow-audit/cases/iso7_gap0
/tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/-home-izll-NetBeansProjects-sparrow/27632e09-68b7-4baf-92e6-eccba6e8cede/scratchpad/iso7.json \
  --sheet-width 1995 --sheet-gap 0 --min-sep 5 \
  -e 0 -c 0 -s 42
```

Javasolt javítás: a `gap == 0` módot egyelőre elutasítani, vagy valódi nulla vastagságú vágási korlátot implementálni; export előtt kötelező straddling-kapu.

### CONFIRMED — az SPP warm start hibás darabszámmal is exportálható

Hely:

- `src/main.rs:108-115`
- `src/main.rs:170-172`
- `src/main.rs:185-201`

Reprodukált esetek:

- demand `1`, placement `0` → exit `0`;
- demand `1`, placement `2` → exit `0`;
- `-p 2` mellett a hiányos, de geometriailag „feasible” üres layout szintén nyerhet;
- falas módban ugyanez az exact-demand kapu hiányzik.

```bash
cd /tmp/claude-1000/sparrow-audit/cases/spp_warm_missing
/tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit/spp_missing.json \
  -e 0 -c 0 -s 42
```

Javasolt javítás: importkor, párhuzamos kiválasztáskor és exportkor item-ID-nként pontos demand-egyezés szükséges.

### CONFIRMED — geometriailag infeasible SPP warm start single-runban kijuthat

Hely:

- `src/main.rs:108-115`
- `src/optimizer/mod.rs:114-124`
- `src/main.rs:170-172`

A nulla idejű szeparálás után az optimizer a „possibly infeasible” állapotot használja tovább. A reprodukált output átfedési területe `1 185 179.5 mm²`, mégis exit `0`.

```bash
cd /tmp/claude-1000/sparrow-audit/cases/spp_warm_overlap
/tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit/spp_overlap.json \
  -e 0 -c 0 -s 42
```

A `-p 2` ág ezt a konkrét geometriai hibát helyesen kizárja, de a single-run nem.

Javasolt javítás: a „possibly infeasible” fallbacket eltávolítani; minden végső output előtt teljes feasibility-kapu kell.

### CONFIRMED — a `nest_race.py` stale vagy más bemenethez tartozó eredményt választhat

Hely:

- `scripts/nest_race.py:84-111`
- `scripts/nest_race.py:164-167`
- `scripts/nest_race.py:198-204`

A motor könyvtára újrahasznált, a régi `final_<name>.json` nincs törölve, és a validátor a motor JSON-jába ágyazott geometriát ellenőrzi, nem az aktuális `-i` bemenetet.

Reprodukálva:

1. első futás 10×10 elem;
2. második bemenet ugyanazzal a névvel és ID-val 50×50 elem;
3. a második motor exit `0`, de nem ír fájlt;
4. a race a régi eredményt választja, `leftover=-20`, `density=2.78`, mégis exit `0`;
5. az aktuális geometriával újraépített outputot a validátor már elutasítja.

```bash
python3 scripts/nest_race.py \
  -i /tmp/claude-1000/sparrow-audit/race_stale/large.json \
  --sheet 30x30 --engines bpp -t 1 \
  -o /tmp/claude-1000/sparrow-audit/race_stale/out \
  --sparrow-bpp /tmp/claude-1000/sparrow-audit/race_stale/write_nothing.py
```

Javasolt javítás: futásonként üres, egyedi könyvtár; az autoritatív inputból és kizárólag a placementekből újraépített JSON validálása; a végleges `result.json` ismételt validálása.

## HIGH

### CONFIRMED — hibás SPP warm start tiszta hiba helyett abortál

Hely: `src/main.rs:108-115`

Ismeretlen `item_id=999` és negatív `strip_width` is `panic=abort`, exit `134`.

```bash
/tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit/spp_foreign.json \
  -e 0 -c 0 -s 42
```

Javasolt javítás: jagua import előtt strip width, ID-k, darabszámok és transzformációk validálása, `Result`-tal.

### CONFIRMED — NaN/Infinity elfogadható CLI-floatként

Hely:

- `src/util/io.rs:38-54`
- `src/util/io.rs:91-97`

Reprodukált eredmények:

- `--sheet-width NaN` → abort, exit `134`;
- `--sheet-width inf` → exit `0`, NaN/Infinity metrikás output;
- `--sheet-gap inf` → exit `0`;
- `--min-sep NaN` → csendben kikapcsolja a távolságot SPP-ben és BPP-ben.

```bash
/tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit/tiny.json \
  --sheet-width NaN -e 0 -c 0 -s 42
```

Javasolt javítás: közös Clap-parser, `is_finite()`, width `>0`, gap/min-sep `>=0`, hibánál exit `2`.

### CONFIRMED — legitim, de be nem férő vagy nagy min-sep-es input abortál

Hely:

- `src/optimizer/lbf.rs:83-95`
- `jagua-rs-0.7.2/src/probs/spp/entities/strip.rs:46-54`

Két külön eset:

- stripmagasságnál magasabb elem: rekurzív szélesítés után `strip-width is running away`, exit `134`;
- 10×10 elem, 100 magas strip, `--min-sep 20`: a kezdeti konténer deflációja üres polygont ad, exit `134`, bár megfelelő szélességgel a feladat kezelhető lenne.

```bash
/tmp/claude-1000/sparrow-audit-root/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit-root/cases/square.json \
  --min-sep 20 -e 0 -c 0 -s 42
```

Javasolt javítás: packability és defláció előellenőrzés; megfelelő kezdeti strip-szélesség vagy tiszta inputhiba, assert/unwrap nélkül.

### CONFIRMED — a validátor nem ellenőrzi az engedélyezett orientációkat

Hely: `scripts/validate_solution.py:110-113`

`allowed_orientations:[0]` mellett egy 45°-os placement `RESULT: OK`, és a race ezt ki is exportálja.

```bash
python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/validator/disallowed_rotation.json
```

Javasolt javítás: normalizált modulo 360 szögillesztés toleranciával; folytonos forgatás csak explicit continuous esetben.

### CONFIRMED — a validátor nem tartja a min-sep-et konténerlyukaknál

Hely: `scripts/validate_solution.py:137-153`

Egy elem 1 mm-re van a belső gyűrűtől `--min-sep 5` mellett, mégis `RESULT: OK`.

```bash
python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/validator/hole_sep.json \
  --min-sep 5
```

Javasolt javítás: minden `cont.interiors` gyűrűre átfedés- és `distance < min_sep-tol` ellenőrzés.

### CONFIRMED — a validátor nem ellenőrzi a BPP stockot

Hely: `scripts/validate_solution.py:91-107`, `scripts/validate_solution.py:185-189`

`stock:1` mellett két layout használja ugyanazt a bin típust, és átmegy.

```bash
python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/validator/stock_exceeded.json
```

Javasolt javítás: `container_id` szerinti layout-darabszámot összevetni a stockkal.

## MEDIUM

### CONFIRMED — nagy demand kontrollálatlan allokációval abortál

Hely:

- `src/optimizer/lbf.rs:55-69`
- `src/optimizer/bpp/lbf.rs:43-58`

A demand teljesen kibontódik egy `Vec`-be. A 100 milliós fixture 800 MB foglalásnál abortál.

```bash
bash -c 'ulimit -v 262144; exec /tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/sparrow-audit/huge.json -e 0 -c 0 -s 42'
```

Javasolt javítás: számlálós iteráció vagy checked cap/`try_reserve`, tiszta inputhibával.

### CONFIRMED — a sheet pack-down jelentősen túllépi a határidőt

Hely:

- `src/optimizer/separator.rs:112-124`
- `src/optimizer/sheets.rs:630-665`
- `src/optimizer/sheets.rs:774-775`

Az iso7 `-e 1 -c 1` futásban a 0.3 s pack-down keret kb. 2.5 s lett; a teljes futás kb. 3.8 s.

```bash
SPARROW_N_WORKERS=3 /tmp/claude-1000/sparrow-audit/target/release/sparrow \
  -i /tmp/claude-1000/-home-izll-NetBeansProjects-sparrow/27632e09-68b7-4baf-92e6-eccba6e8cede/scratchpad/iso7.json \
  --sheet-width 1995 --min-sep 5 \
  --compact-sheets --pack-down-sheets -e 1 -c 1 -s 42
```

Javasolt javítás: deadline-check a separator belső ciklusában és a scatter/search alatt; timeoutkor rollback.

### CONFIRMED — quality zone-ok kimaradnak a validálásból

Hely: `scripts/validate_solution.py:91-96`, `scripts/validate_solution.py:134-153`

`min_quality:1` elem quality-0 zónában is átmegy.

```bash
python3 scripts/validate_solution.py \
  /tmp/claude-1000/sparrow-audit/validator/quality_zone.json --min-sep 5
```

Javasolt javítás: az item `min_quality` szerint tiltott zónákat konténerlyukként ellenőrizni.

### CONFIRMED — lyukas polygonoknál hibás a race belsőhézag-metrikája

Hely:

- `scripts/nest_race.py:58-74`
- `scripts/nest_race.py:172-182`
- `scripts/nest_race.py:195-196`

A területszámítás csak az outer gyűrűt használja. A tesztben a valós `gaps_other` 996.04 vs 900 volt, a script mindkettőre 900-at számolt, majd idő alapján a rosszabb motort választotta.

```bash
python3 scripts/nest_race.py \
  -i /tmp/claude-1000/sparrow-audit/race_metric/input.json \
  --sheet 100x100 --sheet-gap 20 -t 1 \
  -o /tmp/claude-1000/sparrow-audit/race_metric/out \
  --sparrow /tmp/claude-1000/sparrow-audit/race_metric/fake_sheets.py \
  --sparrow-bpp /tmp/claude-1000/sparrow-audit/race_metric/fake_bpp.py
```

Javasolt javítás: outer terület mínusz inner gyűrűk, vagy Shapely polygon area.

### CONFIRMED — egy sérült motor-JSON az egész race-t leállítja

Hely:

- `scripts/nest_race.py:91`
- `scripts/nest_race.py:111`
- `scripts/nest_race.py:187-188`

Egy valid sheets eredmény mellett a BPP `{broken` JSON-ja `JSONDecodeError`-ral leállítja az egész race-t.

Javasolt javítás: engine-feladatonként `try/except`, a hibát `{ok:false}` eredménnyé alakítani.

### CONFIRMED — a race túlfoglalja a CPU-t

Hely:

- `scripts/nest_race.py:85-106`
- `scripts/nest_race.py:187-188`

16 logikai CPU-n `-p16`, két Sparrow motor mellett mindkét motor 48 workert indít: legalább 96 worker egyszerre. A mért idő így a motorok egymás elleni CPU-versenyét is tartalmazza.

Javasolt javítás: race-szintű teljes thread-budget vagy automatikus `-p` korlátozás; a `--sequential` jelenleg helyes manuális kerülőút.

### CONFIRMED — az SPP bitazonossági dokumentáció túl erős

Hely: `docs/sheets.md:536-542`

Azonos `swim -e 10 -c 5 -s 0`, 3 worker:

- upstream: 132 shrink, exploration `5949.413`;
- fork: 148 shrink, exploration `5854.934`;
- az első 132 shrink bitre és a kiírt pontosságra azonos.

A gyorsabb fork ugyanannyi wall-clock alatt több iterációt végez, ezért a teljes időlimitált futás nem bitazonos.

Javasolt javítás: „azonos közös iterációs prefix”; teljes bitazonosságot csak fix iterációszámú terminátor mellett állítani.

## LOW

### CONFIRMED — nagyon vékony, teljesen egymáson fekvő elemek átmehetnek

Hely: `scripts/validate_solution.py:70-82`

Két azonos 20×0.04 elem metszetterülete 0.8 mm², az erodált alakok üresek; `RESULT: OK`.

Javasolt javítás: üres erózió esetén relatív metszetterület vagy adaptív tolerancia.

### CONFIRMED — pontosan 0.05 mm átfedést a tolerancia ellenére elutasít

Hely: `scripts/validate_solution.py:76-81`

Az `intersects()` a határérintést is „deeper than tol” esetnek veszi. Ez a validátor saját toleranciaszerződése szerinti false reject; valóban átfedésmentes layout téves elutasítását nem találtam.

Javasolt javítás: pozitív területű belső metszést vizsgálni, nem boundary `intersects()`-et.

## C) Saját mérések

HEAD release, seed 42, 30+20 s, kivéve a dokumentált swim seed 0. Minden export külön mentve, majd a kért paraméterekkel validálva: `RESULT: OK`.

| Mód / input | Saját eredmény | Dokumentációhoz viszonyítva |
|---|---:|---|
| sheets iso7, 1995×995, sep5 | 51.38 s; 3 sheet; 61.513%; used 1990.0 / 1989.0 / 686.4 | reprodukálva, max. 1 mm eltérés |
| sheets iso6, 1990×995, sep5 | 50.16 s; 9 sheet; 50.904%; utolsó négy kb. 997 mm | dokumentált eredmény reprodukálva |
| sheets madisocad, 2000×1000 | 50.52 s; 4 sheet; 78.169%; 1997.2 / 1996.7 / 1995.1 / 1237.3 | −0.24 százalékpont, +21.8 mm; dokumentált szóráson belül |
| sheets swim, width3000 | 50.91 s; 2 sheet; 75.134%; 2999.8 / 2887.7 | −0.21 pont, +16.2 mm; sheet-drop reprodukálva |
| BPP iso7, 1995×995, sep5 | 50.74 s; 3 bin; 48.114%; 12.9/65.0/66.4% | reprodukálva |
| BPP iso6, 1990×995, sep5 | 35.21 s; 9 bin; 48.192%; 8 cross-bin move | dokumentált számok pontosan reprodukálva |
| BPP madisocad, 2000×1000 | 44.06 s; 4 bin; 70.716%; 10 move | pontosan reprodukálva |
| BPP swim, 3200×3200 | 50.50 s; 4 bin; 62.122% | pontosan reprodukálva |

A validátor ismert hiányosságai miatt az `OK` nem tekinthető általános bizonyításnak, de a nyolc normál referenciafutásnál új solver-regressziót nem találtam.

### Teljesítményállítás

`swim -e 40 -c 10 -s 0`, 3 worker:

| Build | eval/s | Végső width / density |
|---|---:|---:|
| upstream `961ec31` | átlag 971K, medián 988K | 5826.768 / 75.920% |
| fork `949ec94` | átlag 1910K, medián 1916K | 5800.932 / 76.258% |

A kb. `1000K → 2000K` állítás CONFIRMED, mért gyorsulás kb. `1.97×`. A 8/16 worker és `-p4` throughput-részállításokat nem mértem újra.

## F) SPP drift

Három seed, `swim -e 10 -c 5`, 3 worker:

| Seed | upstream final | fork final |
|---:|---:|---:|
| 0 | 5932.357 | 5837.700 |
| 1 | 5959.321 | 5838.314 |
| 2 | 5920.744 | 5856.930 |

A fork mindhárom időlimitált futásban jobb lett, mert gyorsabban több iterációt végez. A seed0 első 132 shrinkje pontosan egyezik; geometriai vagy algoritmikus driftre nem találtam bizonyítékot. A három upstream/fork export közös outputútja miatt csak az utolsó upstream fájl maradt külön visszavalidálható; az `OK` lett.

## G) Determinizmus

Azonos fork, seed0, 3 worker, 10+5 s háromszor:

- exploration: mindháromban 148 shrink és `5854.934`;
- final: `5837.700`, `5837.811`, `5834.733`.

Következtetések:

- wall-clock terminátorral nem determinisztikus — CONFIRMED;
- a `docs/bpp.md:550-560` ezt helyesen jelzi;
- fix iteráció + seed + workerszám determinisztikussága PLAUSIBLE, de CLI-s iteration terminator nincs, ezért end-to-end nem volt igazolható;
- `-p` eredménye is időalapúan változhat a futások CPU-versenye miatt;
- a workereredmények indexsorrendű merge-je és az RNG-szétosztás kód szerint stabil;
- az SPP/README mellett a wall-clock nondeterminizmust külön is dokumentálni kellene.

## Átnézve és rendben

- `cargo build --release --locked --bins`: zöld.
- `cargo test --release --locked`: minden tesztcél zöld.
- BPP warm start: missing/overplaced/overlap/unknown item/unknown bin mind tiszta exit `1`.
- Normál gap-es `--compact-sheets`: iso7, 33/33 elem, valid.
- Sheet-drop normál útja reprodukálva.
- Sikertelen pack-down rollback után valid layout maradt.
- `BPProblem::restore()` mindkét bool ága kezelt; tracker rebuild/size-check megvan.
- `remove_item` auto-close: source tracker, destination rebuild és sampler reseed megvan.
- LayKey/PItemKey újrahasználat spread pack-down alatt valid maradt.
- Tracker különböző méretű `clone_from` útja átméretez.
- Hole collision tracker falas width-váltás után újraépül; normál gap-es eredmények validak.
- `validate_solution.py --self-test`: 5/5 OK.
- Fokkonvenció és `p'=R·p+t`: helyes.
- Sheet → lokális koordináta visszahelyettesítése valid.
- BPP legnagyobb jobb oldali offcutú tábla kerül utolsónak.
- Race kiválasztási kulcs sorrendje helyes: lemezszám → 5 mm-es last-band → belső hézag → idő.
- Nulla demand, önmetsző kontúr, kevés stock és hibás CLI-kombinációk tiszta hibával állnak le.
- Duplikált csúcsok normalizálódnak.
- `-p 1..16` kis szabályos inputon helyes outputot adott.
- BPP concentrate és spread referenciafutások helyesek.


---

## Status after fixes (commit pending)

Minden CONFIRMED finding javítva, regressziós teszttel. A javítások a `feat/bin-packing` ágon,
a working tree-ben állnak (nincs commit). Ellenőrzés: `cargo test` és `cargo test --release` zöld
(68 teszt + 33 új audit-teszt), `cargo clippy --all-targets` tiszta, `cargo build --release
--features only_final_svg` sikeres, `scripts/validate_solution.py --self-test` és
`scripts/nest_race.py --self-test` (18/18) zöld. A sima SPP exploration prefix változatlan:
`swim -s 0`, 3 worker, mind a 149 shrink lépés bitre egyezik a javítás előtti binárissal.

### CRITICAL

| # | Finding | Állapot | Megjegyzés |
|---|---|---|---|
| 1 | `--sheet-gap 0` vághatatlan layoutot exportál | **fixed** | A gap < 1 mm elutasítva (exit 2, `SheetConfig::resolve_gap` + clap parser); `apply_sheet_walls` assertel; kötelező straddling-kapu export előtt (`verify_spp_solution`); a validátor gap 0 mellett is ellenőrzi a határvonalat (LineString-metszés) és hibaként jelzi a `used > W` lemezt. |
| 2 | SPP warm start hibás darabszámmal exportálható | **fixed** | Item-ID-nkénti pontos demand-egyezés importkor (`validate_spp_warm_start`), a `-p` kiválasztásnál és exportkor (`verify_spp_solution`); a falas warm start útja is ezen a kapun megy át. |
| 3 | Geometriailag infeasible SPP warm start single-runban kijut | **fixed** | A „possibly infeasible” fallback törölve (`optimizer/mod.rs`), helyette javító szeparálás; export előtt `Layout::from_snapshot(..).is_feasible()` + teljes demand. Közös `util::verify` helper mindkét binárisnak. |
| 4 | `nest_race.py` stale vagy idegen eredményt választhat | **fixed** | Futásonként törölt motorkönyvtár, mtime > futás kezdete követelmény, az autoritatív inputból + kizárólag a placementekből újraépített JSON validálása, a végleges `result.json` újravalidálása exit 0 előtt, motoronkénti try/except → `{ok:false}`, Shapely-alapú (lyukakat levonó) területszámítás, race-szintű thread-budget. |

### HIGH

| # | Finding | Állapot | Megjegyzés |
|---|---|---|---|
| 5 | Hibás SPP warm start abortál (exit 134) | **fixed** | `validate_spp_warm_start` a jagua import ELŐTT: strip width, ismeretlen ID, nem-véges transzformáció, darabszám. Exit 1, tiszta üzenet. |
| 6 | NaN/Infinity elfogadható CLI-floatként | **fixed** | Közös clap value parserek (`parse_finite_f32` / `parse_positive_f32` / `parse_non_negative_f32` / `parse_sheet_gap`) mindkét binárison, exit 2; `resolve_min_item_separation` `Result`-ot ad, a nem-véges érték hiba (nem néma kikapcsolás). |
| 7 | Be nem férő vagy nagy min-sep-es input abortál | **fixed** | `util::packability`: stripmagasságnál magasabb elem → tiszta hiba (exit 1); a defláció miatt üres konténer esetén a kezdeti strip szélesítése — a `10x10 / --min-sep 20` eset most **sikeresen lefut** (exit 0, validátor OK) abort helyett. |
| 8 | Validátor: orientáció, lyuk-min-sep, BPP stock, quality zone, vékony elem, tolerancia | **fixed** | Mind a hat ellenőrzés bekerült (mod 360, 1e-3° tűrés, continuous csak `null` esetén; `cont.interiors` távolság; `container_id`-nkénti layout-szám vs stock; `min_quality` vs zóna; üres erózió esetén relatív 1%-os fallback; pozitív területű belső metszés a boundary `intersects()` helyett). 13 self-test csoport. |

### MEDIUM

| # | Finding | Állapot | Megjegyzés |
|---|---|---|---|
| 9 | Nagy demand kontrollálatlan allokációval abortál | **fixed** | `MAX_TOTAL_DEMAND = 1_000_000` mindkét binárisban, tiszta hibaüzenettel (exit 1) a 800 MB-os foglalás helyett. |
| 10 | Sheet pack-down túllépi a határidőt | **fixed** | Terminátor-ellenőrzés a separator belső ciklusában (eddig csak strike-onként), a scatter relokációs ciklusában és a `compact_sheets_left` sweepben (utóbbi saját 500 ms-os kerettel, mert a fázis kerete ekkorra már lejárt). Mért: pack-down 1.5 s → 0.3 s, teljes futás 2.60 s → 2.08 s a 2 s-os kereten. |
| 11 | Az SPP bitazonossági dokumentáció túl erős | **fixed** | `docs/sheets.md`, `docs/perf-notes.md`, `README.md`: „azonos shrink-lépés prefix azonos iterációszám mellett; a wall-clock-limitált futások eltérnek, mert a fork gyorsabb”. A wall-clock nondeterminizmus külön szakaszt kapott a README-ben és a perf-notes-ban (a `docs/bpp.md` már korrekt volt). |
| — | Sérült motor-JSON leállítja a race-t | **fixed** | 4. findinggel együtt (`try/except` motoronként). |
| — | Race lyukas polygonoknál rossz metrika | **fixed** | 4. findinggel együtt (Shapely, lyukak levonva). |
| — | Race túlfoglalja a CPU-t | **fixed** | 4. findinggel együtt (thread-budget, `-p` automatikus csökkentése figyelmeztetéssel). |

### LOW

| # | Finding | Állapot | Megjegyzés |
|---|---|---|---|
| 12a | Vékony, egymáson fekvő elemek átmehetnek | **fixed** | Üres erózió esetén relatív metszetterület: a kisebb alak területének 1%-a felett átfedés. |
| 12b | Pontosan 0.05 mm átfedést elutasít | **fixed** | A mélységteszt pozitív területű belső metszést vizsgál (`intersection.area > 1e-9`) a boundary `intersects()` helyett; a 0.1 mm-es valódi átfedés továbbra is jelzett. |

### Új tesztek

* `tests/audit_regression_tests.rs` — 23 unit + 10 end-to-end teszt (utóbbiak `#[ignore]`,
  `cargo test --release -- --ignored`), findingenként, az audit reprodukcióival a doc commentekben.
* `tests/sheet_tests.rs::zero_sheet_gap_is_rejected` — a korábbi `zero_sheet_gap_is_honoured`
  helyén, ami épp a hibás viselkedést rögzítette elvárásként.
* `tests/fit_parity_tests.rs` — a `resolve_min_item_separation` NaN-ága.
* `scripts/validate_solution.py --self-test` — 13 ellenőrzési csoport (5 eredeti + 8 új).
* `scripts/nest_race.py --self-test` — 18 ellenőrzés fake engine scriptekkel, pytest nélkül.

### Megjegyzés a C) szakasz méréseihez

A referenciafutás megismételve (iso7, 1995×995, sep 5, 20+10 s): 3 sheet, utolsó lemez 689.0 mm,
`RESULT: OK` a megszigorított validátorral — a dokumentált eredmény a javítások után is áll.
