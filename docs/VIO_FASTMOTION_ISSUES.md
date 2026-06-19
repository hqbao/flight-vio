# Fast-motion `--tight` VIO — issue tracker

Behaviour of the live `pose.odom` ("VIO") line under fast arc / push sweeps.
**Issue #1 (the snap-back) is FIXED**; #2–#4 remain open. Fix one at a
time; escalate a tier the moment a change touches the flight-critical path.

Repro session: `sessions/arc_fast_15s` (OAK-D W, 640x400, 399 frames, fast arc).
Deterministic offline tools (kept under `verification/`, run them CLEAN — no
concurrent numba):
- `_arc_live_driver.py` — drives the REAL `OdometryModule` single-process, no
  frame drops, seq-aligned. Prints jitter (path / max-dist), max single-frame step.
- `_arc_vo_vs_odom.py` — `pose.vo` (pure vision) vs `pose.odom` (IMU-fused); the
  decomposition that pinned #1.

> NOTE: the recorded `basalt/` reference for this session DIVERGED to 77 m, so
> Sim3-scale / ATE-vs-Basalt are meaningless here — judge our trajectory on its
> own (jitter = path / max-dist; clean ~1–3, spiky ≫). Do NOT trust the offline
> bench either (it over-diverges ~10× vs live); the live driver above is the metric.

## Architecture (which line is which)
```
KLT track -> estimate_motion (RGB-D PnP frame-to-frame + gyro fusion) -> vo.pose
          -> propagate_imu (IMU dead-reckon + complementary correction TOWARD vo.pose
                            + SLAM loop.correction when on)            -> pose.odom
window BA (run_ba) -> pose.refined ("VIO-BA")  --- INDEPENDENT, does NOT feed pose.odom
SLAM pose-graph    -> loop.correction          --- only on REVISIT
```
The ONLY external feedback into `pose.odom` is `loop.correction` (gated on
`loop_correct`, `--tight` live only). `pose.refined` is output-only.

## Issues

### #1 — snap-back on fast `--tight` · ✅ FIXED (2026-06-17, confirmed live)
Root cause = the per-frame complementary VISION correction yanking the IMU-tracked
arc back to the frame-to-frame PnP, which under-estimates arc translation under
fast rotation. Fix = overlap gate + re-anchor offset in `propagate_imu` (inlier
gate removed). **Full write-up: `docs/TIGHT_COUPLED_PLAN.md` Phase 4(k); commit
`ba53f15`.** Driver: jitter 14.5 → 5.8, arc preserved 3.94 m. User: "reacts
exactly like BasaltVIO".

### #2 — Frontend VO holds back (translation under-estimate) · OPEN (compensated)
Under fast rotation the frame-to-frame PnP under-estimates TRANSLATION (optical
flow is rotation-dominated): `pose.vo` reached only 0.83 m on the arc while the
real motion was larger. Smooth but too small. The #1 re-anchor now COMPENSATES for
this on `pose.odom` (the IMU carries the true arc), so it is no longer user-visible
on the live line — but the underlying VO scale is still short, which matters for
#4 (BA fed by the same correspondences) and for any pure-vision fallback. Not
urgent while the IMU is healthy.

### #3 — IMU centripetal phantom translation · OPEN (deferred, ~negligible)
One ~49 cm single-frame spike at the hardest fast-rotation frames: `predict_state`
double-integrates the centripetal / lever-arm accel as PHANTOM translation. Mostly
invisible now (it also appears in the locally-valid Basalt steps at the same seqs →
largely REAL motion). Fix direction if pursued: rotation-gated translation in the
IMU PREDICT — mirror odometry's `rot_damp_gate_deg` (when the gyro rate is high the
true translation ≈ 0, so freeze the accel double-integral through the burst). Verify
on `_arc_live_driver.py`; gap stays 0 (`propagate_imu` is `--tight` live only).

### #4 — Window BA ("VIO-BA" / `pose.refined`) diverges under fast motion · OPEN
The windowed-BA line goes haywire under fast sweeps. Independent of `pose.odom`
(output-only) but it is the line that will feed the FC's refined pose, so it must
be trustworthy before the FC link consumes it. Two known causes:
- (a) **KLT track SLIP** — wrong correspondences (not loss) survive `huber_px=2.0`
  + `min_ba_views=2` and poison the solve. Fix: reject slipped tracks before BA
  (RANSAC / IMU-consistency).
- (b) **velocity-gauge unobservability** → round-off chaos (see
  `docs/TIGHT_COUPLED_PLAN.md` Phase 4(e), velocity-divergence stabilisation). Fix:
  absolute-velocity prior + Tikhonov nav floor.

### #5 — Dynamic object drags the pose at rest (hand-wave) · ✅ FIXED (ZUPT gate)
A moving object in view of a STATIC camera (e.g. a hand waved in front) makes the
frame-to-frame PnP report a spurious translation (it assumes a static scene); the
per-frame vision correction then dragged `pose.odom` along with it. The predict-side
ZUPT already froze the IMU translation at rest but did NOT gate the vision pull.
Fix (`propagate_imu`, LIVE + `--tight`, gap=0): when `zupt` holds, zero the vision
TRANSLATION pull (`k_pos`/`k_vel` → 0) and freeze the re-anchor offset; ROTATION is
still corrected (gyro-anchored yaw, robust to the dynamic object). Env
`OAKD_ZUPT_FREEZE_TRANS=0` disables. Selftest contrast: 0 cm (gate ON) vs 22.45 cm
(OFF); confirmed live ("waving a hand no longer drags it along"). SCOPE: only fires at genuine
rest (ZUPT hysteresis) — the camera-MOVING + dynamic-object case still needs a
general IMU↔vision disagreement gate (the coast-gate that was reverted for fast
motion; see below), kept out of the fast path on purpose.

## Tried + reverted (did NOT work — do not retry blindly)
IMU-aided KLT seed · nav-state clamp · disagreement coast-gate · fast-rotation
translation-damp gate + velocity clamp · higher/lower correction gain. All
implemented, gap=0-safe, and either null or worse on the live driver; reverted.
