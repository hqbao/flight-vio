# Conventions (FC ↔ VIO), Gold tests & UI validation

> Document verifying the correctness of the VIO → dblink → FC ESKF chain. Every claim
> cites file:line from the real code (no guessing). 2026-06-19.

---

## A. AXIS & SIGN CONVENTIONS (sign conventions) — whole chain

### A.0 Conclusion first
**The wire (dblink) matches at both ends, and the VIO ALREADY converts to true NED before sending.** The VIO's
native "optical-world" frame (gravity-aligned, arbitrary yaw because there is no compass) is converted to
NED at a SINGLE place (SSOT) `sky/fc/fc_earth_pose.py` — the UI and sender share it, so
they cannot drift apart. Heading is RELATIVE (no compass) → the FC handles it itself via an anchor.

### A.1 Earth frame = NED (North, East, Down)
| | Convention | Evidence |
|---|---|---|
| FC | NED, gravity **+Z (down)**, `a_earth[2]+=g`, g=+9.80665 | `robotkit/fusion6.c:313`, `fusion6.h:27` |
| VIO (after convert) | NED, fwd→N, right→E, down→D | `sky/fc/fc_earth_pose.py:62-64` `_M_OPT_TO_NED=[[0,0,1],[1,0,0],[0,1,0]]` |
| Accel reading at rest | ~(0,0,−g) (specific force opposes gravity) | `fusion6.c:206-213` |

### A.2 Body frame = FRD (Forward, Right, Down)
| | Convention | Evidence |
|---|---|---|
| FC | X=forward, Y=right, Z=down; Euler ZYX (yaw-pitch-roll) | `state_estimation/earth2body.h:12-16`, `quat.c:205-222` |
| NED→body | `fwd= cψ·vN+sψ·vE; right=−sψ·vN+cψ·vE; down=vD` | `earth2body.c:31-36` |

### A.3 Quaternion = Hamilton, (w,x,y,z), **body→earth/NED**
| | Evidence |
|---|---|
| FC: stores (w,x,y,z), body→earth, Hamilton, right-mult error `q←q⊗Exp(δθ)` | `quat.h:7-10`, `fusion6.h:27`, `fusion6.c:567-570` |
| VIO: (w,x,y,z), body→world, Hamilton (same quat→rot formula) | `sky/math/quat.py:10-28` |
| Wire: `q_w,q_x,q_y,q_z` = **body→NED**, w-first | `messages.h:597`, `sky/fc/dblink.py:35-39` |

### A.4 VIO native frame and the conversion to NED (the EASIEST place to get WRONG — verified)
- VIO native = **gravity-aligned OPTICAL world** (cam optical: X=right, Y=down, Z=forward),
  yaw = camera heading at init (NO compass → relative "North"). `sky/imu/imu.py:257-291` (`gravity_aligned_R0`).
- Convert to NED (SSOT, shared by UI + sender): `sky/fc/fc_earth_pose.py:99-106`
  ```
  pos_ned = M @ pos_opt                      # fwd→N, right→E, down→D
  R_ned   = M @ R_opt @ P @ R_body_cam.T     # P = opencv-cam → FRD; R_body_cam = mount offset (default I)
  q_ned   = rot_to_quat(R_ned)
  ```
  Called at `fc/main.py:388` BEFORE `pack_vision_pose` → **on the wire it is true NED + quat body→NED.**
- **RELATIVE heading** (no mag): the FC re-anchors with `ψ0 = yaw_FC − yaw_VIO`, fuses
  `fused = Rz(ψ0)·(vio_pos − anchor) + offset`, D goes straight through. `vision_pose_rx/vision_pose_math.h:43-69`.
  - **SE1**: fuses POSITION only (heading is owned by the FC compass) → relative-North is harmless.
  - **SE2**: uses the position DERIVATIVE (velocity) → origin-invariant, no anchor needed; rotates NED→body via the fusion3 yaw (has mag). `vio_body_vel_math.h:45-51`.

### A.5 IMU axis map — PER BOARD (needs physical validation)
| Board | raw→body | Evidence |
|---|---|---|
| **h7v1** (board under HIL) | body=(−raw_y, −raw_x, −raw_z) | `h7v1/modules/icm42688p/icm42688p.c:10-11,32-35,48-50` |
| h7v2 | body=(raw_x, raw_y, raw_z) | `h7v2/.../icm42688p.c:10-11,24-27,36-39` |

⚠️ **The two boards map DIFFERENTLY** (different PCB mount). Not a bug, but **you must validate
the tilt-test for the exact board you fly** (section C). The "sensor X=Right…" comment is the same in both files
but the map differs → don't trust the comment, trust the tilt-test.

### A.6 OAK-D Lite BMI270 — extrinsic EEPROM WRONG (fix exists)
The EEPROM returns a wrong `Rx(90°)` → flips roll ~180°. Fixed by a per-device calibration wizard
(Kabsch/Wahba) `sky/sensors/imu_cam_extrinsic.py`. **You must make sure the calibration is applied** before flying the Lite.

### A.7 Z-sign at the SE2 publish boundary (already annotated, CRITICAL)
fusion5_z runs POSITIVE-UP internally → **negate** at the publish boundary to emit NED-down:
`state_estimation2.c:335,338` `pos_body.z=-g_pos_z.pos_final`. Forget this = positive-feedback altitude-hold.

---

## B. GOLD TESTS — how far is correctness established?

### B.1 The ESKF optimization I just did → HAS a gold test, TIGHT (machine precision)
- `robotkit/test/fusion6_equiv.c` — 200k random cases, optimized == naive to ~1e-14.
- `robotkit/test/fusion6_traj.c` — 200k steps, parity vs git-reference (P-trace 3e-14).
→ **Proves conclusively: the optimization does NOT change the algorithm.** (This is "regression-correctness".)

### B.2 PER-COMPONENT correctness vs physical truth → YES (strong)
| Test | Proves | File |
|---|---|---|
| IMU dead-reckon + ZUPT | integration yields the correct displacement; no drift at rest | `vio/tests/imu_propagate_selftest.py` |
| Wahba extrinsic IMU→cam | recovers a known R to 1e-9, det=+1, noise-robust | `imu_camera/tests/imu_cam_extrinsic_selftest.py` |
| Gravity 6-face | accel calib tracks \|g\|=9.81 | `imu_camera/tests/gravity_sphere_selftest.py` |
| Preint covariance | analytic Σ == Monte-Carlo | `vio/tests/imu_preint_cov_selftest.py` |
| **fc_earth_pose SSOT** (optical→NED) | **known axes + pitch-90 → correct** | `verification/fc_earth_pose_selftest.py` ✅ |
| anchor + body-vel (FC) | yaw, transform, gate correct (hand-checked) | `tools/vision_pose_rx/test_vision_pose_math.c`, `test_vio_body_vel.c` |

### B.3 Regression gold (matches the frozen reference) → YES
- gap=0 oracle byte-parity vs Basalt: `verification/oracle_replay_selftest.py` (TOL 1e-6 mm).
- 12 gold sessions `sessions/gold/` (lab_static/straight/loop, push, shake, yaw…).

### B.4 GAP: no END-TO-END gold test vs ground-truth yet
There is NO test for: "a known REAL motion (measured by tape measure/mocap) → VIO → FC ESKF → compare the estimate
against the truth". gap=0 only proves "matches Basalt" (regression), not "physically correct".
→ **How to close the gap NOW (manual, ground-truth = tape measure):** section C below. An automated
e2e harness can be built later (replay a known session → compare the FC estimate).

---

## C. UI VALIDATION STEPS (physical, doable right now)

> Goal: visually confirm the SIGN of each axis is correct, end-to-end. Requires the FC powered + AP `/dev/cu.usbmodem2101`,
> and (for VIO) the Pi running the VIO stack (320x200, no --tight/BA/SLAM).

### C.1 Validate IMU→body (FC, board h7v1) — `state_estimation_view.py` / `attitude_control_view.py`
```bash
cd /Users/bao/skydev/flight-controller && python3 tools/state_estimation_view.py
```
Hold the FC, perform each motion, confirm the SIGN:
| Physical motion | Expected | Meaning |
|---|---|---|
| Pitch nose DOWN | **pitch +** (or per your chosen convention) | Y/forward axis correct |
| Roll right | **roll +** | X/right axis correct |
| Yaw nose to the right (viewed from above) | **yaw +** | Z/down axis correct |
| Hold level at rest | roll≈pitch≈0, gravity loads the Z axis | accel Z-down correct |
→ Any wrong sign = the board's IMU axis map (A.5) needs fixing. **This is the most important gate.**

### C.2 Validate VIO→NED (Pi VIO, 3D UI) — `ui/main.py`
```bash
cd /Users/bao/skydev/flight-vio && ./run-ui-remote.sh    # or ./run.sh ... then open the UI
```
⚠️ **VIO has NO compass → the VIO's "North" = the camera heading AT INIT** (gravity-aligned, code:
`_M_OPT_TO_NED` ⇒ NED-North = init-forward). So you must test in 3 groups:

**(a) Down — HEADING-FREE (gravity), test this first, the most reliable:**
| Move | Expected |
|---|---|
| Lower the rig DOWN ~0.5 m | **pos_d +0.5** (always correct, no need to know heading) |

**(b) N/E — DEPENDS on the init heading → you must NOT ROTATE the rig after init:**
| Move (keep heading, pure translation) | Expected |
|---|---|
| Along the exact direction the camera faces (= init-forward) | **pos_n +**, pos_e ≈ 0 |
| To the right of init-forward | **pos_e +**, pos_n ≈ 0 |

**(c) HEADING-FREE invariants (check the axis structure WITHOUT knowing North):**
- Move PURELY vertically → only `pos_d` changes; PURELY horizontally → only `pos_n/pos_e` changes (`pos_d≈0`).
- Move out and BACK to the same spot → pos returns to ~0 (small drift).
- Move 1 m horizontally in any direction → `√(pos_n²+pos_e²) ≈ 1 m` (checks SCALE, no heading needed).

→ A discrepancy in (a)/(c) = the optical→NED conversion or the cam-IMU extrinsic (A.6) is WRONG. (b) is only correct when
not rotating; if you rotate, N/E decompose along the init-fixed axes (mathematically correct, not a bug).
**ABSOLUTE North: VIO cannot provide it — by design, the FC compass owns the absolute yaw.**

### C.3 Validate the VIO→FC link (0x34) — `tools/vision_pose_rx/vision_pose_rx_view.py`
```bash
cd /Users/bao/skydev/flight-controller && python3 tools/vision_pose_rx/vision_pose_rx_view.py
```
The Pi sends the real VIO over `--fc /dev/ttyAMA0`. Move the rig → watch `rx (VIO world)` N/E/D
move in the CORRECT direction + SIGN as in C.2. Confirm the FC receives exactly what the VIO sends.

### C.4 End-to-end cross-check (ground-truth tape measure)
- Place the rig at marker 0, **no rotation**, move a MEASURED distance along init-forward (e.g. tape: 2.00 m).
- Watch `state_estimation_position_earth.py` (FC fused) **and** the 3D UI (VIO): both should read ~+2.00 m
  North, same sign, within a few cm. (Or check HEADING-FREE: horizontal magnitude `√(N²+E²)≈2.00 m`
  — no need to move exactly along North, just measure the distance correctly.)
```bash
python3 tools/state_estimation_position_earth.py    # FC fused NED position
```
- This IS the **manual end-to-end gold test** (closes the B.4 gap): known motion →
  estimate matches to within tape-measure error.

### C.5 Relative heading (reminder)
VIO yaw is NOT absolute (no mag) → it will drift slowly; **the FC compass owns the absolute yaw**. Don't expect
VIO-North = true North. When GPS/mag are healthy the FC prefers them; VIO compensates when GPS is lost.

---

## D. One-line summary
The conventions at both ends are CONSISTENT (NED + FRD + Hamilton body→NED), VIO converts to NED at a self-tested
SSOT, and relative heading is handled by an anchor. Correctness: components + regression are TIGHTLY covered;
still missing is an end-to-end ground-truth gold test → use C.4 (tape measure) to lock it down before flying.
