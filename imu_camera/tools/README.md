# `imu_camera/tools/` — pre-flight diagnostics (calibration owner)

`imu_camera` owns and publishes the calibration contract (`calib.bundle`), so the
calibration sanity gate lives here. These tools are **standalone and additive** —
they only *import* the project's own loader (`imu_camera.io.reader`); they never
modify a runtime path, the comms contract, or any recorded session on disk.

## Files

| File | Purpose |
|---|---|
| `calib_check.py` | Pre-flight / CI gate that validates a session's parsed `StereoCalib` (intrinsics, stereo extrinsic, IMU↔cam extrinsic, recorded-data consistency) against physical sanity bands and flags malformed / implausible values **before** a run. |

## `calib_check.py`

Validates the **parsed** calib (the exact object the live pipeline consumes via
`StereoCalib.from_json`) — so what it checks is byte-identical to what VIO/depth
will use. In particular the loader converts the `T_left_right` translation from
**centimetres to metres**, so the tool validates the metres value (OAK-D baseline
≈ 0.075 m) and specifically catches a skipped or doubled cm→m conversion.

```sh
cd /Users/bao/skydev/oak-d

# Validate a recorded session (primary — also checks recorded-frame consistency)
.venv/bin/python -m imu_camera.tools.calib_check --session sessions/gold/lab_loop_30s

# Validate a bare calib.json (secondary — no recorded-data checks)
.venv/bin/python -m imu_camera.tools.calib_check --calib path/to/calib.json

# Strict CI gate: treat WARN as failure for the exit code
.venv/bin/python -m imu_camera.tools.calib_check --session <dir> --strict
```

**Output** is an aligned `CHECK | MEASURED | EXPECTED | STATUS` table (status ∈
`{PASS, WARN, FAIL, INFO}`), a one-line explanation per non-`PASS` row, and a
`N pass / M warn / K fail / J info` summary.

**Exit code** — `0` when no `FAIL` (`WARN` allowed); nonzero on any `FAIL`, or
(under `--strict`) any `WARN`. Suitable as a pre-run / CI gate.

### Checks

* **Intrinsics** (left + right): `fx,fy>0`; pixel aspect `|fx-fy|/fx`; principal
  point inside image + near centre; `K` consistent with `fx,fy,cx,cy`; image size
  `>0` and equal L/R; horizontal FOV in a sane band; distortion finite, a known
  model length, sane magnitude.
* **Stereo extrinsic** `T_left_right` (metres): rotation ∈ SO(3) (`‖RRᵀ−I‖`,
  `det≈+1`); inter-camera angle small (parallel rig); baseline in `0.02–0.30 m`
  (with a cm→m hint when out of band); baseline dominantly along camera-X.
* **IMU↔camera**: if `T_imu_left` present, rotation ∈ SO(3) + small lever-arm;
  otherwise **INFO** "no IMU extrinsics → gyro prior disabled" (a valid state, not
  a failure). `imu_noise` densities finite/positive when present, else INFO.
* **Recorded-data consistency** (`--session` only): calib resolution == recorded
  frame shape; median recorded depth in an indoor band (skips stereo warm-up
  frames that pin a handful of pixels at the far disparity rail).

## Self-test

```sh
.venv/bin/python -m imu_camera.tests.calib_check_selftest
```

Asserts a real gold calib produces **no FAIL** (exit 0, and `--strict`-clean),
and that five injected faults — non-orthonormal stereo R, absurd baseline, L/R
size mismatch, K/scalars desync, NaN distortion — each FAIL with a nonzero exit.
The broken cases are built **in memory** (deep-copied `StereoCalib`); no session
on disk is touched.
