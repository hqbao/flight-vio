# VL53L1X Downward Rangefinder — Bench HIL Protocol & Calibration

**Scope:** Hardware-only validation + calibration of the VL53L1X short-range validity
fix (Tier T3, FC height reference). The host SIL suite
(`lidar/tests/lidar_mock_selftest.py`) already proves the *software* logic — gate mask,
calibration store guards, offset/xtalk packing, fail-closed read, bench-routine guards.
This document covers everything the host CANNOT prove: real I2C register acceptance,
**SHORT-mode cadence** (the MEDIUM-confidence macro-period constants + the new
`0x004B=0x14`), the **VALID% regression jump** on a real surface, the physical
offset/xtalk solves, and the **end-to-end FC state machine** on live dblink.

> **You (the operator) must complete this on the real Pi + VL53L1X.** Final QA sign-off
> is GATED on every PASS box below being checked. A FAIL on any step is a BLOCK — stop
> and follow the FAIL action.

---

## 0. Equipment & preconditions

| Item | Requirement |
|---|---|
| Compute | Flight Raspberry Pi (Pi 5, py3.13, `.venv` with `smbus2`) |
| Sensor | VL53L1X downward breakout on `/dev/i2c-1` @ `0x29` (factory) |
| Targets | **17% grey reflectance card** (ST cal standard); a matte white/grey wall |
| Ruler | Calibrated steel rule / caliper readable to ±1 mm |
| Mount | The bracket the sensor flies on (skid standoff measurable) |
| Cover | The **final flight cover glass / window** (if the sensor flies behind one) |
| Dark | A room you can darken to near-zero ambient IR for the xtalk step |
| FC link | Pi ↔ FC dblink wired + powered; a GCS / `fctool` to read FC state |

**Sensor id:** pick a stable id for THIS physical sensor (e.g. its serial, or `dn0`).
Use the **same** `--sensor-id <id>` in every command below and in `lidar.main` at flight
time. The calibration is keyed by it; a mismatch silently runs uncalibrated.

```bash
# On the Pi, from the repo root:
cd ~/skydev/flight-vio
SID=dn0                      # <-- YOUR sensor id; keep it consistent everywhere
PY=.venv/bin/python
```

**Sanity: the chip answers on the bus** (do this first; everything else depends on it):
```bash
i2cdetect -y 1               # expect 0x29 present
```
- **PASS:** `29` appears in the grid.
- **FAIL:** no `29` → wiring / address / power problem. STOP. Fix the bus before any
  software step (do NOT proceed — every step below will fail-closed to `valid=0`).

---

## 1. Deploy & launch with `--sensor-id`

Deploy the current branch to the Pi (your normal rsync/scp/git pull flow), then confirm
the lidar process opens the real sensor and starts SHORT-mode ranging.

```bash
# Smoke: open the device, publish ~100 readings, then exit.
$PY -m lidar.main --sensor-id "$SID" --max-reads 100
```
- **PASS:** log shows
  `VL53L1X (smbus2) ranging on i2c bus=1 addr=0x29 (mode=1, 50000us, sensor_id='dn0')`
  and `shutdown complete (published 100 readings, N valid)`.
  Mode MUST read `mode=1` (SHORT). A first run with no cal on disk MUST also print
  exactly ONE loud `NO calibration for sensor_id='dn0' ... running UNCALIBRATED` warning
  — that is correct (honest valid readings are safe; the FC gates on `valid`).
- **FAIL:** `could NOT open the rangefinder` / `no VL53L1X at 0x29 (model id ...)` →
  back to §0 bus sanity. A `RuntimeError`/traceback that is NOT a clean
  "could not open" is a software regression → BLOCK, report to tester.

☐ **STEP 1 PASS**

---

## 2. SHORT-mode cadence read-back  *(validates the MEDIUM-confidence constants)*

This is the empirical check on the SHORT macro-period pair
(`0x005E=01AE / 0x0061=01E8` for SHORT-50 ms) **and** the newly-added
`0x004B=0x14` (PHASECAL_CONFIG__TIMEOUT_MACROP) — all flagged MEDIUM-confidence in
`PLAN.md`. If those constants are wrong, the sensor either stays in LONG timing or
produces frames at the wrong rate.

The reader is configured for SHORT @ **50 ms** inter-measurement → a fresh frame should
be available roughly every 50 ms, i.e. an effective **~20 Hz** measurement cadence.
With the read loop polling at its default rate you should see new frames land at
~20 Hz (≈33 Hz if you later switch to the SHORT-33 ms constants).

**Read it two ways:**

**(a) On the FC side — `fctool lidar view`, RX RATE field.** With the Pi lidar running
and dblink connected, the FC sender bundles the freshest range; the GCS RX RATE for the
downward range should read **~20 Hz** (band **20–33 Hz** acceptable).

**(b) On the Pi directly — measure produced-frame cadence:**
```bash
# Count distinct fresh frames over 10 s. Distinct = the chip advanced a measurement;
# a frozen distance for many reads = NOT a fresh frame (would indicate stuck timing).
timeout 11 $PY -m lidar.tools.characterize --seconds 10 --rate 60 --sensor-id "$SID" \
  | tee /tmp/lidar_cadence.txt
# Eyeball the distance column: it should UPDATE ~20x/sec, not step at 60/sec (read rate)
# and not freeze. The summary's valid count over 10 s should be ~200 for ~20 Hz.
```
- **PASS:** measured fresh-frame cadence is **20–33 Hz** (RX RATE ~20 Hz on the GCS;
  ~200 fresh frames over 10 s on the Pi). Distance updates smoothly, never freezes.
- **FAIL (rate far off, e.g. ~9 Hz or ~5 Hz, or frozen frames):** the SHORT-50 ms
  macro-periods and/or `0x004B` are wrong for this part.
  1. First try the **SHORT-33 ms** constants: launch with `timing_budget_us=33000`
     (edit the `lidar.main` reader construction or add the flag) — `_MACROP` already
     carries `(SHORT, 33_000): (0x00D6, 0x006E)`. Re-run; expect ~30 Hz.
  2. If still wrong, **escalate to `researcher`** — the macro-period derivation needs
     re-checking against the ST ULD for this silicon rev. Do NOT fly on a wrong cadence
     (it changes the integration time → noise/sigma the FC's disarm margin assumes).

☐ **STEP 2 PASS** — cadence: ________ Hz (record it)

---

## 3. Mask-fix proof — the headline regression  *(VALID% must jump)*

This proves the `& 0x1F` + accept-only-code-9 fix on real hardware. The user's
**baseline was ~38% valid**; the masked gate must recover the frames that high status
bits were rejecting.

Point the sensor at a **matte surface at ~10–20 cm**, perpendicular, steady (a wall, a
grey card, a book — NOT a mirror, NOT open air, NOT closer than ~10 cm).

```bash
$PY -m lidar.tools.characterize --seconds 10 --rate 20 --sensor-id "$SID" --quiet
```
- **PASS:** summary reads `readings: N/M valid` with **valid fraction > 90%**
  (e.g. `185/200 valid`). This is the headline proof the mask fix landed.
- **FAIL (still ~38% or low):**
  - Confirm you are ≥ ~10 cm and on a real matte surface (sub-4 cm or open air or a
    glossy/black surface legitimately rejects).
  - If valid% is low on a known-good surface at a known-good distance, the mask /
    status-code logic did not take effect on device → BLOCK, report to tester with the
    `--quiet`-off per-frame log (`status=` column) so the rejecting codes are visible.

☐ **STEP 3 PASS** — VALID%: ________ (baseline was ~38%)

---

## 4. Offset calibration  *(run FIRST — ST order)*

Solves this part's part-to-part range bias. **17% grey card, EXACTLY 140 mm,
perpendicular, flat.** Measure the 140 mm from the **sensor cover-glass face** to the
card with the rule; do not eyeball it (the solve is `offset_mm = 140 − mean_measured`,
so a sloppy standoff injects a constant bias into every future reading).

```bash
$PY -m lidar.tools.characterize --calibrate --sensor-id "$SID"
# Prompt [1/2] OFFSET: place the 17% grey target flat at 140mm, press Enter.
```
- **PASS:** prints `-> offset_mm = <value>` with a **plausible** magnitude
  (typically within roughly ±50 mm; the store hard-rejects |offset| > 2000 mm). Record it.
- **FAIL:**
  - `calibration aborted (... data-ready timeout ...)` → the sensor stopped producing
    frames (check the card is actually in view at 140 mm, not too close/dark). Re-seat,
    retry.
  - `offset_mm` absurd (hundreds of mm) → wrong standoff or wrong target. Re-measure the
    140 mm and retry — do NOT save a bad offset.

☐ **STEP 4 PASS** — offset_mm: ________

*(The tool continues to the xtalk prompt — see §5 before pressing Enter.)*

---

## 5. Crosstalk calibration  *(after offset, IN THE DARK — cover-glass critical)*

> **CRITICAL (schematic-reviewer):** crosstalk is the reflection off the **cover glass /
> window** in front of the sensor. The cal is only meaningful if it is taken with the
> **FINAL flight cover MOUNTED over the sensor exactly as it flies.** If you calibrate
> xtalk bare and mount the cover afterward, the cal is a near-no-op AND is invalidated by
> the cover you just added. **Mount the cover NOW, before this step.**
>
> **If the sensor flies BARE (no cover glass):** crosstalk ≈ 0. You may skip this step —
> press Enter at the prompt only if you've set the card up, or abort the calibrate run
> (`Ctrl-C`) and re-run §4 alone is not possible (the tool does offset+xtalk together);
> instead complete the xtalk step against the card and expect `xtalk_raw ≈ 0`. Document
> that this sensor flies bare and xtalk is ~0.

Conditions for the xtalk solve:
- **Final cover glass/window MOUNTED** (unless flying bare — see above).
- **DARK room** — no ambient IR, no other reflectors in the cone. (Ambient IR corrupts
  the SignalRate the solve uses.)
- **17% grey card at ~600 mm.**

```bash
# (continuing the same --calibrate run from §4)
# Prompt [2/2] XTALK: place the 17% grey target at ~600mm in the DARK, press Enter.
```
- **PASS:** prints `-> xtalk_raw = <value>` (a uint16, `0..65535`), then
  `saved calibration for sensor_id='dn0' -> .../.cache/lidar_calib.json`. Record xtalk.
  A bare sensor in the dark should read **near 0**; a cover glass adds a positive value.
- **FAIL:**
  - `xtalk cal: zero effective SPADs (sensor dark / no valid frames)` (RuntimeError) →
    the sensor saw NOTHING (too dark on the *target* too, or card not at 600 mm, or out
    of view). This guard is correct — it refuses to persist a bogus "calibrated" 0.
    Re-place the card so the chip gets a return, retry.
  - Any other abort → nothing is saved; the prior cal (if any) is untouched. Retry.

☐ **STEP 5 PASS** — xtalk_raw: ________   (cover mounted: ☐ yes  ☐ flies bare)

**Confirm persistence + that the live reader will apply it:**
```bash
cat .cache/lidar_calib.json          # entry under "dn0" with xtalk/offset_mm/...
$PY -m lidar.main --sensor-id "$SID" --max-reads 20
# Log MUST now show: applied calibration sensor_id='dn0' (offset=..mm, xtalk_raw=..)
# and MUST NOT show the UNCALIBRATED warning.
```
- **PASS:** `applied calibration sensor_id='dn0' (...)` appears; no UNCALIBRATED warning.
- **FAIL:** still warns UNCALIBRATED → the `--sensor-id` used here differs from the one
  saved, or the entry was rejected by a magnitude guard (offset > 2000 / xtalk out of
  uint16). Check `.cache/lidar_calib.json` and the id.

☐ **STEP 5b PASS** — cal applied on live start

---

## 6. Post-calibration accuracy

With the cal applied, verify reported distance tracks truth at the band ends and stays
valid. Use the rule to set the card at a known distance from the cover-glass face.

```bash
# 100 mm:
$PY -m lidar.tools.characterize --seconds 5 --rate 20 --sensor-id "$SID" --quiet
#   place card at EXACTLY 100 mm first
# 1000 mm:
#   then 1000 mm, re-run the same command
```
- **PASS:**
  - At **100 mm**: median range ≈ **0.100 m ± 0.020 m** (within ~20 mm), VALID% > 90%.
  - At **1000 mm**: median range ≈ **1.000 m ± 0.050 m** (within ~50 mm), VALID% > 90%.
    *(SHORT mode tops out ~1.3 m; 1000 mm is in-band. If you cannot make 1000 mm, use the
    farthest in-band distance you can measure and check the reading matches the rule.)*
- **FAIL (bias outside tolerance):** the offset solve was off — re-run §4 with a carefully
  measured 140 mm standoff, re-save, re-test. Persistent large error after a clean offset
  cal → BLOCK, report to tester.

☐ **STEP 6 PASS** — @100 mm: ______ m  ·  @1000 mm: ______ m

---

## 7. Mounting & FC threshold constraints  *(safety-reviewer)*

These are physics + FC-config gates. They do NOT need a script, but they MUST be
checked — a wrong standoff or a default `disarm_range` makes the height reference unusable
or the touchdown auto-disarm inert.

1. **Lidar standoff to ground ≥ 40 mm.** The VL53L1X has a **hard 4 cm optical floor**.
   A 3 cm standoff WILL read `valid=0` at rest — *by physics*, not a bug. Mount the sensor
   so that, with the gear on the ground, the cover-glass-to-ground distance is **≥ 40 mm**
   (ideally ≥ 50 mm for margin).
   - **CHECK:** with the vehicle on the ground, measure standoff with the rule → **≥ 40 mm**.
     Then run `characterize` on the ground; ground-floor median must read VALID and ≥ 40 mm.
   - **FAIL:** standoff < 40 mm → raise the mount / gear. Do not proceed; the height
     reference is unusable below the floor.

2. **Set `PARAM_ID_DISARM_RANGE` ≥ floor + margin.** Default is **10 mm** with a **strict
   `<`** compare (`flight_state.c`: `g_downward_range < g_disarm_range`). Since a valid
   reading is always ≥ 40 mm, `< 10` is **unsatisfiable** → the **touchdown auto-disarm
   never fires** until you raise this. Set it to roughly **(measured ground floor + margin)**,
   i.e. **~60–80 mm** (`characterize` prints a recommended value = floor + 0.10 m margin).
   - **CHECK:** read back `PARAM_ID_DISARM_RANGE` on the FC (GCS / `tuning_board.py` param
     id 86) → **≥ 40 mm**, and ≥ (ground-floor + a few mm). Record the value set.
   - **FAIL / left at 10:** touchdown auto-disarm stays inert (documented, user-accepted at
     bring-up, but the range-disarm will not function). Set it before relying on auto-disarm.

3. **Range-independent disarm fallbacks remain available.** Confirm the two disarms that do
   NOT depend on the rangefinder still work, so the vehicle is **never stuck armed** even if
   the range is untrusted:
   - **RC-disarm** (`flight_state.c` `RC_STATE_DISARMED`, range-independent): flip the arm
     switch to disarmed → vehicle disarms regardless of range. **CHECK on the bench
     (props OFF):** arm, then RC-disarm → state leaves armed.
   - **Angle-disarm** (range-independent, `> g_disarm_angle`, default 60°): documented
     fallback; do not bench-trigger by tilting an armed vehicle with props on. Confirm the
     param is set (id for disarm_angle) and that the code path is range-independent (it is).
   - **PASS:** RC-disarm verified on the bench; angle-disarm param confirmed present.

☐ **STEP 7 PASS** — standoff: ____ mm · DISARM_RANGE: ____ mm · RC-disarm verified ☐

---

## 8. End-to-end: lidar → fc/dblink → FC flight_state

Prove the full chain: a **valid + fresh** range lets **READY → TAKING_OFF** fire (mode 0),
and a **sub-40 mm / no-target** reading (`valid=0`) **HOLDS** — no false takeoff, no false
disarm. **PROPS OFF for all of this** (bench, no spin-up).

The FC trusts the range only when it is BOTH valid (`range_valid` bit3 in `pack_vio_pose`)
AND fresh (`range_is_trusted()` = valid AND age ≤ ~120 ms). READY→TAKING_OFF (mode 0)
requires throttle above the takeoff threshold AND `range_is_trusted()` (mode 2 bypasses
sensors — keep OUT of mode 2 for this test so the range gate is actually exercised).

**8a — Valid+fresh range permits takeoff (mode 0):**
1. Lidar running, cal applied (§5b), dblink connected. Place a target so the sensor reads
   a **valid in-band** distance (e.g. card at ~200 mm → VALID, range_m ~0.200).
2. On the GCS confirm the FC sees `range_valid=1` and the downward range ~0.200 m, RX
   fresh (RX RATE ~20 Hz).
3. In **mode 0**, with the vehicle in **READY**, raise throttle past `TAKEOFF_THROTTLE`.
   - **PASS:** flight_state transitions **READY → TAKING_OFF**. (Props off — you're
     watching the state field on the GCS, not the motors.)
   - **FAIL:** stays in READY with a valid+fresh range and throttle up → the range is not
     being trusted (check `range_valid` bit3 actually set, age < 120 ms). BLOCK if the
     valid+fresh range is confirmed at the FC yet takeoff is gated off.

**8b — Invalid / no-target range HOLDS (no false takeoff):**
1. Remove the target (sensor sees open air / no return) OR present a sub-40 mm surface →
   the lidar publishes `valid=0`, FC `range_valid=0`.
2. Confirm on the GCS `range_valid=0`.
3. In **mode 0**, READY, raise throttle past `TAKEOFF_THROTTLE`.
   - **PASS:** flight_state **HOLDS in READY** — does NOT enter TAKING_OFF on an untrusted
     range. (This is the safe state: an untrusted range must not be read as ground contact
     nor permit a blind takeoff.)
   - **FAIL:** enters TAKING_OFF with `range_valid=0` → the trust gate is broken → BLOCK.

**8c — Range loss does not trigger a false disarm:**
1. While the FC believes it is FLYING/TAKING_OFF (you can stage this on the bench by the
   state you're in), **stop the lidar process** (`Ctrl-C` / kill) so the range stream dies.
2. Within ~120 ms the FC's `range_is_trusted()` decays to false (the topic stops arriving).
   - **PASS:** the FC does NOT execute a range-based disarm on the stale/absent range — the
     on-ground/touchdown disarms are gated on `range_is_trusted()` and HOLD when it is
     false (RC-disarm + angle-disarm remain available as the operator fallback).
   - **FAIL:** a range-loss causes an uncommanded disarm or a "near ground" misread → BLOCK.

☐ **STEP 8a PASS** (valid → TAKING_OFF)  ☐ **STEP 8b PASS** (invalid → HOLD)
☐ **STEP 8c PASS** (range-loss → no false disarm)

---

## Sign-off

| Step | What it proves | PASS |
|---|---|---|
| 1 | Real sensor opens, SHORT mode, sensor-id keyed | ☐ |
| 2 | SHORT-mode cadence (MEDIUM-conf macro-periods + 0x004B) | ☐ |
| 3 | **Mask fix** — VALID% > 90% (headline regression) | ☐ |
| 4 | Offset calibration solved + plausible | ☐ |
| 5 | Xtalk calibration (cover mounted / bare documented) | ☐ |
| 5b | Cal persisted + auto-applied on live start | ☐ |
| 6 | Post-cal accuracy @100 mm & @1000 mm | ☐ |
| 7 | Standoff ≥ 40 mm, DISARM_RANGE set, RC/angle disarm | ☐ |
| 8 | End-to-end FC: valid→TAKING_OFF, invalid→HOLD, loss→no-disarm | ☐ |

**All boxes checked → the VL53L1X fix is hardware-validated. Report results back to
`tester` for final sign-off (the host SIL is already APPROVED).**
**Any FAIL → BLOCK: follow that step's FAIL action; do not fly.**
