# Basalt VIO — source study & rebuild plan

> Goal: reach Basalt-level VIO accuracy (EuRoC ~2–6 cm ATE, no forward-push
> undershoot) by rebuilding its architecture block-by-block in this codebase.
> Every fact below is taken from the **actual Basalt source** (read 2026‑06‑06):
> - frontend: `basalt/optical_flow/frame_to_frame_optical_flow.h`
> - IMU: `basalt-headers/imu/preintegration.h`, `imu/imu_types.h`
> - backend: `basalt/vi_estimator/sqrt_keypoint_vio.cpp`
> - config: `basalt/utils/vio_config.cpp`
>
> This document is the authoritative plan; `OURS_VS_BASALT.md` is the high-level
> gap summary and `SKYSLAM_ROADMAP.md` the schedule. Read those for context.

---

## 0. Why this matters for the "đẩy nhanh rồi ì lại" bug

Measured on our gold suite (offline, `--depth ours`, post VO‑prior fix):

* frame‑to‑frame PnP (`ours`) keeps Sim3 scale **0.90–0.98** on a forward push;
* windowed BA collapsed to 0.30–0.39 **before** the VO‑prior fix; after it,
  the offline live‑replay tracks **0.95–0.97** with no frame drops.

So on recorded data our path is already good.

### Why `ours` moves full but `ours-ba/slam` stalls (verified from source, 2026‑06‑06)
Both live sources run the **same KLT frontend** on the same device: `ours`
(`FlowPoseSource` → `OdometryFlow` → `RGBDVisualOdometry`) uses the default
`FrontendConfig()` = win21/lvl3/400; `ours-ba` (`OakOursVioSource`) uses
`res.frontend(numba=HAVE_NUMBA)` which on a Numba‑capable device is **the exact
same** win21/lvl3/400. So KLT is **not** what separates them. The difference is
entirely in the **display/translation path**:

* `ours` paints the **raw `pose.odom`** (frame‑to‑frame VO translation) — instant,
  unfiltered, uncapped.
* `ours-ba` paints `filt_p` from `InertialTranslationFilter` (an EMA on velocity,
  `vision_trust=0.8`) **and then** crawls toward an **async, rate‑limited BA
  correction** `C_applied`, capped at `_CORR_MAX_STEP_T=0.015 m/frame = 0.3 m/s`.

On a super‑fast push the EMA softens the onset and the **0.3 m/s correction cap**
is the smoking gun: when the displayed position lags the true motion, BA wants a
forward correction but the marker can only *crawl* there at 0.3 m/s — exactly
"đi 50%, **ì lại**, rồi **đi từ từ** 30%". The crawl speed is the rate cap, not a
tracking failure. (Earlier note blamed KLT; that was wrong — KLT is identical on
both branches. Corrected here.)

This is a *display* artefact of loose coupling, and it is **offline‑verifiable**:
`ours/tools/live_replay.py` already models the filter + rate‑limit + async BA
latency. A recorded super‑fast‑push session will let us A/B (raw‑tip vs filtered
vs rate‑limited‑corrected) and confirm the mechanism before any change.

Basalt does not have this failure because the **IMU is inside the estimator**:
during the fast push the preintegrated accelerometer *predicts* the translation
(`predictState`) and vision only *refines* it — one consistent state, no separate
"filter then crawl a correction" stage. That is the fundamental fix, and it is the
reason this rebuild is the real answer, not another loose‑coupling tuning pass.

> See `docs/TIGHT_COUPLED_TASKS.md` for the smallest‑possible, each‑step‑visualised
> task breakdown of the tight‑coupled rebuild.

---

## 1. Basalt at a glance (the 5 blocks)

| # | Block | Basalt file | Our current analogue | Status |
|---|-------|-------------|----------------------|--------|
| 1 | Optical‑flow frontend (patch KLT + grid FAST + stereo epipolar) | `frame_to_frame_optical_flow.h` | `ours/lib/frontend` (KLT + Shi‑Tomasi) | partial |
| 2 | IMU preintegration (Forster, midpoint, bias Jacobian, sqrt cov) | `imu/preintegration.h` | `ours/lib/imu` (gyro preint only) | partial |
| 3 | Sliding‑window sqrt VIO (joint pose+vel+bias+landmarks) | `sqrt_keypoint_vio.cpp::optimize` | `ours/lib/backend/vio_window.py` (immature) | weak |
| 4 | Keyframe management + triangulation (anchored inverse‑depth) | `sqrt_keypoint_vio.cpp::measure` | `WindowedBAMap` (XYZ landmarks) | different |
| 5 | Square‑root marginalization (QR, FEJ prior) | `sqrt_keypoint_vio.cpp::marginalize` + `marg_helper` | `ours/lib/backend/marginalize.py` (opt‑in, plain Schur) | weak |

### Basalt default config (verbatim from `vio_config.cpp`)
```
optical_flow_type            = "frame_to_frame"
optical_flow_detection_grid_size = 50      # detect 1 FAST corner per 50x50 cell
optical_flow_pattern         = 51          # the residual pattern (Rosten 52→51 pts)
optical_flow_max_iterations  = 5           # inverse-compositional GN per level
optical_flow_levels          = 3           # pyramid levels
optical_flow_max_recovered_dist2 = 0.09    # fwd-bwd check (px^2) = 0.3 px
optical_flow_epipolar_error  = 0.005       # stereo essential-matrix gate

vio_linearization_type       = ABS_QR      # square-root, absolute (not nullspace)
vio_sqrt_marg                = true
vio_max_states               = 3           # # of FULL pose-vel-bias states (recent)
vio_max_kfs                  = 7           # # of pose-only keyframes (older)
vio_min_frames_after_kf      = 5
vio_new_kf_keypoints_thresh  = 0.7         # new KF when <70% of obs are connected
vio_obs_std_dev              = 0.5         # px; reprojection sigma
vio_obs_huber_thresh         = 1.0         # px
vio_min_triangulation_dist   = 0.05        # m; min baseline to triangulate
vio_max_iterations           = 7           # LM iters per frame
vio_init_pose_weight         = 1e8         # prior on initial position + yaw
vio_init_ba_weight           = 1e1         # accel-bias prior
vio_init_bg_weight           = 1e2         # gyro-bias prior
vio_marg_lost_landmarks      = true
vio_kf_marg_feature_ratio    = 0.1         # marg a KF when <10% of its lms tracked
```

---

## 2. Block-by-block spec (exact algorithms) + rebuild tasks

### Block 1 — Optical-flow frontend
**Basalt (`FrameToFrameOpticalFlow`):**
1. Build a `uint16` image pyramid (`optical_flow_levels`=3) per camera.
2. `trackPoints(old_pyr, pyr)`: for every existing patch, `trackPoint`
   coarse→fine. Each level runs `trackPointAtLevel`: an **inverse‑compositional
   Gauss–Newton** over an **SE2 affine** patch warp —
   `res = patch.residual(img, warp·pattern)`, `inc = −H_se2_inv_Jᵀ·res`,
   `transform *= SE2::exp(inc)`, ≤5 iters; patch invalid if it leaves bounds.
   The patch `pattern2` has 51 samples (`optical_flow_pattern`).
3. **Forward‑backward check:** track 1→2 then 2→1; reject if recovered point
   moved > `optical_flow_max_recovered_dist2` (0.09 px² ≈ 0.3 px).
4. `addPoints`: `detectKeypoints` on a **grid** (`grid_size`=50 → ~one FAST
   corner per cell), skipping cells that already hold a point; new points are
   also tracked into cam1 to seed stereo.
5. `filterPoints` (stereo): unproject cam0/cam1 points, drop any pair whose
   **epipolar error** `|p0ᵀ·E·p1|` > `optical_flow_epipolar_error` (0.005).

**What we have:** pyramidal LK (`klt.py`, win=21/lvl=3 full, 13/2 live) +
Shi‑Tomasi (`corners.py`) + forward‑backward (`fb_threshold`=1.0 px). **Missing:**
the per‑patch **SE2 affine** warp (we track pure translation), the **grid‑uniform**
detector (we use global Shi‑Tomasi with min‑distance), and the **stereo
epipolar** filter (we drive depth from the chip/SGM, not a tracked stereo pair).

**Rebuild tasks**
* 1a. Switch the detector to a **grid** (cell = 50 px at 640×400 → 12×8 cells);
  one strongest corner per empty cell. Gives uniform coverage → fewer dropouts
  on fast motion (the undershoot lever). *Gate:* `live_replay --klt-*` KLT‑fail
  on `push_*` must drop vs the current preset.
* 1b. Add an **affine (SE2) patch warp** to the KLT core (`klt.py`): 2‑DoF
  translation → 4‑DoF (rotation+scale+translation) per patch via the same
  inverse‑compositional GN. Longer tracks under rotation/scale → less undershoot.
* 1c. (later, when we own stereo) add the epipolar stereo filter. For now SGM
  depth replaces this role — keep it.

### Block 2 — IMU preintegration (the missing metric anchor)
**Basalt (`IntegratedImuMeasurement`):**
* State `PoseVelState` = (SO3 `T_w_i.so3`, `vel_w_i`, `trans`), 9‑D.
* `propagateState` (midpoint): `R_mid = R·exp(0.5·dt·ω)`, `a_w = R_mid·a`,
  `vel += a_w·dt`, `trans += vel·dt + 0.5·a_w·dt²`, `R = R·exp(dt·ω)`. Returns
  full Jacobians `F` (d next/d curr), `A` (d/d accel), `G` (d/d gyro) using
  `rightJacobianSO3`.
* `integrate`: subtract the **linearization‑point bias**, propagate, update
  covariance `cov = F·cov·Fᵀ + A·Σa·Aᵀ + G·Σg·Gᵀ`, and accumulate the **bias
  Jacobians** `d_state_d_ba = −A + F·d_state_d_ba`, `d_state_d_bg = −G + F·…`.
* `predictState(state0, g)`: propagate a *full* state through the delta given
  gravity — **this is what predicts translation through a fast push**.
* `residual(state0, g, state1, bg, ba)`: 9‑D [trans, rot, vel] residual with a
  **first‑order bias correction** (`bg_diff = d_state_d_bg·(bg−bg_lin)`), plus
  analytic Jacobians. `sqrt_cov_inv` (LDLT) whitens it.

**What we have:** gyro‑only preintegration for the rotation prior
(`ours/lib/imu`), accelerometer used *only* to level attitude. **Missing:** the
accel inside an estimated‑bias velocity/translation state.

**Rebuild tasks**
* 2a. Port `IntegratedImuMeasurement` to NumPy: `PoseVelState`, `propagateState`
  (midpoint + F/A/G), `integrate` (cov + bias Jacobians), `predictState`,
  `residual` (+ Jacobians), `sqrt_cov_inv` via `scipy`/LDLT. Pure‑Python, vectorised
  where possible. *Gate:* a `imu_preint_selftest` that checks `p0.applyInc(diff)`
  round‑trips and Jacobians vs finite differences (like our `_vt_jac_check`).
* 2b. Feed it real accel/gyro covariance from `calib` (noise std²).

### Block 3 — Sliding-window sqrt VIO solve
**Basalt (`optimize`):**
* Order states by time into an `AbsOrderMap`: `max_states`=3 **full**
  pose‑vel‑bias states (15‑D each) for the most recent frames, `max_kfs`=7
  **pose‑only** states (6‑D) for older keyframes.
* Build the linearization (`ABS_QR`): per‑landmark blocks, `performQR()`
  marginalizes the landmark out *in place* (Givens QR on each landmark block →
  the square‑root reduced camera system). `get_dense_H_b` forms the reduced
  normal equations over the camera/state block only.
* LM loop (≤`vio_max_iterations`=7): `H.diag += λ·diag`, `LDLT.solve`,
  `backSubstitute` to recover the landmark increments, apply to all
  pose/vel/bias states, recompute vision+IMU+bias+marg‑prior error, accept iff
  cost decreased; Nielsen λ update (`λ *= max(1/3, 1−(2ρ−1)³)`), converge when
  `f_diff<1e‑6` or `step_∞<1e‑4`.
* IMU error term ties consecutive full states via Block 2's `residual`; bias
  random‑walk priors via `gyro/accel_bias_sqrt_weight`.

**What we have:** `vio_window.py` (dense finite‑difference IMU window) — works
but immature and slow, regresses vs `ba` on healthy motion. `bundle.py` does the
vision sqrt‑Schur per landmark already (our `optimize`).

**Rebuild tasks**
* 3a. Generalise `bundle.optimize` state to **pose+vel+bias** for the recent
  `max_states`, **pose‑only** for older keyframes (mirror `AbsOrderMap`).
* 3b. Add the **IMU residual** (Block 2) between consecutive full states with the
  analytic Jacobians (replace `vio_window.py`'s finite differences → ~100× faster,
  the reason it's currently too slow to keep corrections fresh).
* 3c. Add **bias random‑walk priors**. Use the same LM accept/reject + Nielsen λ
  we already have. *Gate:* `vio_run --backend vio` must beat `ba` on the gold
  motion suite (currently it regresses).

### Block 4 — Keyframe management + anchored inverse-depth landmarks
**Basalt (`measure`):**
* Every frame: `predictState` (IMU) → seed the new state; add observations to
  existing landmarks; count `connected/(connected+unconnected)` in cam0.
* **New keyframe** when that ratio < `vio_new_kf_keypoints_thresh` (0.7) **and**
  `frames_after_kf > vio_min_frames_after_kf` (5).
* On a KF: **triangulate** each unconnected track from the observation pair with
  the largest baseline (≥`vio_min_triangulation_dist`=0.05 m), store it as an
  **anchored inverse‑depth** landmark: `host_kf_id`, `direction` =
  `StereographicParam::project`, `inv_dist` (must be finite, 0<inv_dist<3).
* Landmarks live in `lmdb` keyed by host keyframe; observations are (frame,cam).

**What we have:** `WindowedBAMap` inserts a KF every `kf_every` frames (time‑based,
no parallax/connection gate) and stores **world‑XYZ** landmarks back‑projected
from SGM depth. **Missing:** the connection‑ratio KF gate and anchored
inverse‑depth (XYZ is what makes our forward scale ill‑conditioned — inverse
depth at a host frame conditions the low‑parallax direction far better).

**Rebuild tasks**
* 4a. Replace the time‑based KF trigger with Basalt's **connection‑ratio + min
  frames** gate. *Gate:* fewer, better‑spread KFs at equal ATE on gold.
* 4b. Migrate landmarks to **anchored inverse‑depth** (host KF + bearing +
  `inv_dist`). Keep the SGM depth as the *initial* `inv_dist` (our metric prior)
  instead of a per‑view residual — this is the principled version of the
  `depth_host_coeff` experiment that failed as a global tweak.

### Block 5 — Square-root marginalization (carry the past forward)
**Basalt (`marginalize`):**
* Trigger when `frame_poses > max_kfs` or `frame_states ≥ max_states`.
* **Keyframe to drop:** DSO‑style score — drop an old KF whose tracked‑landmark
  ratio < `vio_kf_marg_feature_ratio` (0.1); else the KF with the smallest
  "distance‑to‑others / distance‑to‑latest" score (keeps a well‑spread set).
* Older full states are demoted: **keep the pose (6‑D), marginalize vel+bias**
  (`states_to_marg_vel_bias`); the very oldest are fully removed.
* Marginalize in **square‑root** form: linearize (QR), then
  `MargHelper::marginalizeHelperSqrtToSqrt(Q2Jp, Q2r, idx_to_keep, idx_to_marg)`
  → a new `sqrt H, b` prior. **FEJ**: the marginalized state is fixed at its
  linearization point (`setLinTrue`), and the prior is converted to a
  delta‑independent form `b -= H·delta`.
* Init prior: `marg_data.H.diag` = √(`vio_init_pose_weight`=1e8) on position+yaw,
  √(`init_ba_weight`/`init_bg_weight`) on biases — i.e. a strong pose+yaw anchor
  at startup, weak bias anchors.

**What we have:** `marginalize.py` does a plain Schur marg prior, opt‑in
(`--marg`), and our BA otherwise **plain‑drops** the oldest KF (loses info).
**Missing:** square‑root form + FEJ + the demote‑to‑pose‑only step.

**Rebuild tasks**
* 5a. Make the marg prior **always on** for the VIO path, in square‑root form,
  with FEJ (freeze the linearization point, `b -= H·delta`).
* 5b. Implement the **demote pose‑vel‑bias → pose** transition so old KFs keep a
  pose constraint without the (stale) vel/bias DoF.
* 5c. Port the DSO‑style KF‑drop score.

---

## 3. Init & gravity alignment (small but load-bearing)
Basalt `initialize`: gravity from the first accel
(`T_w_i = FromTwoVectors(accel, +Z)`), `vel=0`, biases from args, and a **strong
pose+yaw prior 1e8** baked straight into `marg_data.H`. We already gravity‑level
from accel; add the explicit strong startup prior so early frames don't wander.

---

## 4. Phased plan with offline gates

All gates run **offline** on the gold suite via `vio_run.py` / `live_replay.py`
(no device). Target = Basalt EuRoC‑class: motion sessions ATE < 1% path, scale in
[0.95,1.05], no forward‑push collapse, still sessions drift small.

* **Phase A — frontend coverage (Block 1a).** Grid detector + restore live
  pyramid depth. *Gate:* KLT‑fail on `push_*` ≤ full‑preset level; no ATE
  regression. *Lowest risk, directly attacks the fast‑push undershoot.*
* **Phase B — IMU preintegration core (Block 2).** Port + unit‑test Jacobians.
  No estimator change yet. *Gate:* preint selftest green.
* **Phase C — tight‑coupled solve (Blocks 3+4).** Pose+vel+bias states, IMU
  residual with analytic Jacobians, connection‑ratio KFs, anchored inverse‑depth.
  *Gate:* `vio_run --backend vio` ≥ `--backend ba` on every gold motion session,
  and `push_*` scale ≥ 0.95 **without** the VO‑prior crutch.
* **Phase D — sqrt marginalization (Block 5).** Always‑on FEJ sqrt prior +
  demote‑to‑pose. *Gate:* corridor/loop ATE improves vs Phase C; no nullspace
  drift (replicate Basalt's `checkMargNullspace`).
* **Phase E — affine patches + stereo (Blocks 1b/1c).** Last, once the estimator
  is the bottleneck.

Each phase is independently shippable and offline‑measurable. Phase A alone
should noticeably help the bench fast‑push; Phase C is the one that makes
ours‑vio match Basalt and retires the loosely‑coupled `ours‑ba/slam` undershoot
for good.

---

## 5. Mapping table (Basalt → our files)

| Basalt symbol | Our target file |
|---|---|
| `FrameToFrameOpticalFlow`, `detectKeypoints(grid)` | `ours/lib/frontend/frontend.py`, `corners.py`, `klt.py` |
| `IntegratedImuMeasurement` | new `ours/lib/imu/preintegration.py` |
| `SqrtKeypointVioEstimator::optimize` | `ours/lib/backend/vio_window.py` (rewrite over `bundle.py`) |
| `measure` KF gate + `triangulate` + `lmdb` (inv‑depth) | `ours/lib/backend/vio_window.py`, `windowed.py` |
| `marginalize` + `MargHelper` (sqrt, FEJ) | `ours/lib/backend/marginalize.py` |
| `VioConfig` | `BAConfig`/`VioConfig` dataclasses |

---

*Source read 2026‑06‑06 from gitlab.com/VladyslavUsenko/basalt(+-headers). Config
numbers are the repo defaults; they are starting points to re‑validate on our
OAK‑D + SGM depth, not constants to copy blindly.*
