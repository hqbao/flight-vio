# Camera-mount calibration (R_body_cam / `--fc-mount`)

How to make the VIO heading **roll-invariant** for any physical camera mount
(forward / backward / tilted-down / straight-down). Get this wrong and the drone
**WILL crash** — read the symptom + the gate before flying.

---

## 1. The problem (and the symptom)

flight-vio sends the FC a quaternion the FC treats as **body→NED**. But the VIO
actually computes the **camera** pose (`T_world_cam`, see `sky/fc/fc_earth_pose.py`).
The conversion applies an EXTRA mount rotation `R_body_cam` (camera-OpenCV → FRD
airframe body); the launcher passes it via **`--fc-mount`**. Its default is
**identity = nominal forward-facing camera**.

If the camera is mounted at any non-forward angle and `--fc-mount` is wrong/identity,
the FC reads the **camera** heading as the **body** heading. A correct heading (yaw
about gravity) is **invariant to roll/pitch**; the camera's is not. So:

> **body roll/pitch LEAKS into heading → the VIO yaw-anchor `psi0` corrupts →
> the fused NED position rotates → slow position drift → crash.**

**Symptom (bench, no flight needed):** open fctool → **VIO nav**, hold the drone and
**roll it**. If `YAW / VIO HEADING` jumps, the mount is wrong. (Measured on the
broken rig: **−0.67°/° to −2.1°/° of roll** — e.g. a 30° roll swung heading ~20°.)
A correctly-calibrated mount holds heading flat (**< ~0.05°/°**).

---

## 2. Why guessing / shortcuts fail (lessons paid for in blood)

- **Presets ("forward-down-45") are usually wrong.** A real mount is a full 3-DOF
  rotation, rarely a clean cardinal angle. (Our "45°" rig measured ~70° pitch + a
  flipped roll in the raw frame.)
- **Gravity from ONE level pose is not enough.** It gives the mount's tilt+roll but
  **NOT the azimuth** (rotation about gravity is unobservable from a single pose).
  A tilt+roll-only fix **still leaks roll into heading**.
- **A single hand-held roll can FALSE-PASS verification.** A sloppy roll that also
  pitches/yaws can show a coincidentally-small leak. **Verify on a CLEAN continuous
  roll sweep**, not one point.
- **Synthetic / self-consistent verification proves nothing about the real rig.**
  Always verify on **real hardware data**.

The correct method below avoids all four.

---

## 3. The method: gravity-pair Kabsch + roll-sweep verify

Gravity is the truth. Capture the gravity direction in **two frames** at several
**different tilts**:
- **body frame** — from the FC accelerometer (`LOG_CLASS_IMU_ACCEL_CALIB 0x0A`,
  mapped chip→body by the board's `SENSOR_ORIENT`; h7v1 = ID 8 = `(-cy,-cx,-cz)`).
- **camera frame** — from the VIO pose (`LOG_CLASS_VIO_RX 0x34`, `rx_quat`, run RAW
  = no `--fc-mount`).

`R_body_cam` is the rotation mapping camera-gravity → body-gravity at every pose.
With **≥2 non-parallel gravity directions** this is fully determined (azimuth
included) by **Kabsch/Wahba** (SVD). Math: `R_ned = M·R_opt·P·R_body_cam.T` (the
SSOT in `sky/fc/fc_earth_pose.py`); the recovered heading is `ZYX-yaw(R_ned)` and
must be roll-invariant.

Tool: **`flight-controller/tools/camera_mount_calib.py`** (it reads the FC dblink,
so it lives in the flight-controller repo; the output is flight-vio's `--fc-mount`).

---

## 4. Procedure (step by step)

### 4.1 Run the VIO RAW (no mount) for calibration
On the Pi, start the stack with your usual flight recipe **but omit `--fc-mount`**, so
`rx_quat` is the raw camera pose:
```
./deploy/pi-run.sh [your usual flight args]      # e.g. --vl53l9cx --direct — but NO --fc-mount
```
(`pi-run.sh` defaults to `--no-ui` and forwards the FC sender from `run.sh`; the VIO
flies as `--no-ba --no-slam` pure odometry. The flag we care about here is the absence
of `--fc-mount`.) Close fctool (the dblink port allows only one reader).

### 4.2 Run the calibration tool (on the Mac)
```
python3 flight-controller/tools/camera_mount_calib.py [/dev/cu.usbmodem2101]
```
It guides you through **4 poses** — hold STILL, press Enter at each:
1. **LEVEL** (flat surface). Sanity: it checks `g_body ≈ [0,0,1]` and that the
   camera gravity ≠ body gravity (else a mount is already applied → restart RAW).
2. **NOSE-DOWN ~40°**, 3. **ROLL-LEFT ~40°**, 4. **ROLL-RIGHT ~40°** (any well-spread
   tilts work; >20° apart).

It solves `R_body_cam` (Kabsch) and prints the **fit residual** (want < ~4°).

### 4.3 Verify on a clean roll sweep
The tool then asks you to **roll slowly, nose-fixed, left↔right for ~12 s**. It
checks the recovered heading is flat:
- **PASS** = leak `< 0.05°/°` and spread `< 8°` over the sweep → it prints the
  `--fc-mount` 9-value matrix.
- **FAIL** = redo with cleaner/more-spread poses. **Do not fly.**

### 4.4 Apply + restart
```
./deploy/pi-stop.sh
./deploy/pi-run.sh [your usual flight args] --fc-mount R11,R12,...,R33   # 9 values from the tool
```
(Launcher flag is **`--fc-mount`**, 9 comma-separated row-major values, forwarded to
`fc.main --mount`. The launcher also accepts presets / `azimuth,tilt[,roll]` and a
`--mount` alias — but the measured 9-value matrix is what you trust.)

### 4.5 FINAL GATE (operator confirms before flight — every time)
In fctool → **VIO nav**:
1. Drone **level** → `ROLL` and `PITCH` read **~0°** (not the raw tilted values).
2. **Roll the drone** through a big angle → `YAW / VIO HEADING` stays **flat**
   (±1–2°). If it still jumps → **do not fly**, re-calibrate.

---

## 5. Worked example (2026-06-29, the forward-tilted rig)

| | heading vs roll |
|---|---|
| broken (`--fc-mount` identity / wrong) | leak **+0.67°/°**, spread **57°** over a 67° roll |
| after calibration | leak **−0.013°/°**, spread **1.4°** — **51× better, flat** |

Verified result: `--fc-mount 0.90499,0.02170,0.42487,0.01815,-0.99976,0.01242,0.42503,-0.00352,-0.90517`
(this value is specific to that physical mount — **re-calibrate for each mount**).

---

## 6. Gotchas / notes

- **Per-mount.** The matrix is specific to the physical camera orientation.
  Re-run the tool for each of the 5 mounts (forward, backward, fwd-down, back-down,
  down) and keep a labelled value per mount.
- **`--fc-mount`, not `--mount`.** On the launcher (`launcher.main`) the flag is
  `--fc-mount`; it forwards to `fc.main --mount`. `--mount` on the launcher only
  works with the newer code (alias added 2026-06-29).
- **Calibrate RAW.** `rx_quat` must be the raw camera pose during calibration (VIO
  run without `--fc-mount`), or the tool aborts at the level-pose sanity check.
- **chip→body orient** in the tool is hard-coded for h7v1 (`SENSOR_ORIENT` ID 8).
  Change `chip_to_body()` for a board with a different IMU mount.
- **Cross-session transfer.** The mount is physical (constant), and the VIO world is
  gravity-aligned, so the value should persist across VIO restarts — but the FINAL
  GATE (§4.5) re-confirms every session; trust the gate, not the assumption.
- **Down-facing mount (tilt≈90°).** Azimuth aliases into the image-roll DOF; the
  gravity-pair method still resolves the full rotation, but pick distinct tilts that
  aren't all near-vertical.
- **This only fixes heading roll-leak.** Residual hover wobble / VIO drift are
  separate (see the FC's `mode0-wobble` investigation); re-assess them on a balanced,
  correctly-mounted rig.

## 7. References
- `sky/fc/fc_earth_pose.py` — the SSOT pose conversion + `R_body_cam_from_angles` +
  `MOUNT_PRESETS` (preset → matrix, math-reviewer-verified).
- `fc/main.py` `_parse_mount`, `launcher/main.py` `--fc-mount` / `build_fc_args`.
- `flight-controller/tools/camera_mount_calib.py` — the calibration tool.
- flight-controller `tools/_dblink.py` LOG_CLASS_VIO_RX 0x34 / IMU_ACCEL_CALIB 0x0A.
