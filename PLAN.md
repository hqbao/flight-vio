# PLAN — Tight-VIO solve optimisation chain (Schur + gauge reg + divergence guard + njit)

## Task
Optimise + harden the `--tight` LM window solve (`sky/vio/window.py`,
`sky/vio/imu_factor_numba.py`, `vio/engine/steps.py`, `vio/modules/backend.py`),
in order: (1) Schur complement on the dense solve, (2) absolute-velocity gauge
regularisation (the real fix for the long-standing "tight explodes on shake"),
(3) divergence guard, (4) njit IMU-Jacobian kernel coupled to the guard, and
(5) thread a `vio_degraded` health signal end-to-end to the published pose.

## Tier: T3 (touches the estimation core / flight-relevant tight VIO)

## Status: COMPLETE + VALIDATED (2026-06-16). Not committed.

Opt-in/defaults: `--tight` is opt-in (loose is default; the oracle's loose
entries stay `gap=0`). Within `--tight`, Schur + regularisation + guard + njit are
all ON by default.

### Done (all five links)
- [x] **Schur complement** — `_schur_solve`/`_schur_partition`/`_schur_reduce_dense`,
      per-landmark scatter; marginalises the block-diagonal 3×3 landmark Hessian →
      reduced nav-only solve (`ndim ~993 → ~114`). `use_schur` gated on IMU factors +
      `M>0`. ALGEBRAICALLY EXACT. Closes the planned-Schur TODO in the window.py docstring.
- [x] **Gauge regularisation** — `VioConfig.vel_abs_prior` (default ON, `sigma_vabs=1.0`,
      toward the IMU forward-prediction not zero), `tau_nav=1e-3` (absolute nav-block
      Tikhonov floor), `min_lambda` 1e-9→1e-6. ROOT CAUSE: IMU vel residual is a pure
      DIFFERENCE operator → rank-3 abs-velocity null space → relative damping amplifies
      round-off ~1.9e6×. math-reviewer APPROVED (legitimate gauge anchor, invents no
      info; ~400× weaker than the IMU tie → drags a real manoeuvre only ~2 mm/s).
- [x] **Divergence guard** — `WindowedVIOConfig.divergence_guard` (default ON) +
      `max_reproj_px=20`, `max_window_jump_m=1.0`, `jump_reproj_floor_px=15`,
      `max_deadreckon_speed_mps=5`. Detect (reproj primary, reproj-gated jump
      secondary) → reject window mutation → bounded fallback (IMU dead-reckon if it
      agrees with the frontend seed, else frontend seed) → gate `self.vo.pose`
      writeback (no frontend poisoning) → raise `vio_degraded`. safety-reviewer APPROVE
      (flagged: document as an always-on flight invariant; not FC-pose-trustworthy
      until the FC consumes `vio_degraded`).
- [x] **njit IMU-Jacobian kernel** — `imu_factor_numba.py` `@njit(cache=True,
      fastmath=False)`, DEFAULT ON, COUPLED to the guard. `njit_guard_ok` gate
      force-disables it if the guard is off (even with `SKY_VIO_IMU_NJIT=1`);
      `SKY_VIO_IMU_NJIT=0` always forces off; `HAVE_NUMBA` fallback to pure-Python.
- [x] **`vio_degraded` health signal** — `vio_step` returns `(T_cw, health)`;
      `run_ba` merges `{vio_degraded, vio_reproj_px, vio_window_jump_m}` into
      `pose.refined` `PoseMsg.info` (alongside `refined`/`pos_sigma_m`). InProcess +
      Subprocess engines. Loose path info unchanged (key absent).

### Validation (measured, verbatim)
- Oracle `gap=0` (loose byte-frozen; the 2 tight `backend="vio"` baseline entries
  RE-BASELINED against the new exact/intended tight solve).
- `schur_equiv_selftest`: worst `max|δ_scatter − δ_dense| ≈ 1.7e-14` (observed up
  to 2.4e-14 over 35 inner solves / 4 tight configs), far below the 1e-9 gate. PASS.
- `imu_factor_njit_ate`: njit==pure full-session ATE **0.0 mm** incl. shake (the
  guard makes shake deterministic). PASS.
- `imu_factor_njit_equiv`: H/b relative ~7e-11.
- Gauge rank: nav block lam_min **−6.45e-9 (rank-3 null) → +7.59e-3 (full rank)**.
- Divergence guard: shake bounded **836 → 83 cm** (no runaway), ZERO false
  positives on well-conditioned sessions. `divergence_guard_selftest` PASS
  (rejects+flags a diverged KF without poisoning the frontend; bit-for-bit no-op on
  a healthy solve).
- `vio_ba_selftest` + `tight_smoke_selftest` + `tight_live_regression_selftest`
  PASS; pyflakes 0.
- Pi `--tight` fps: improved but noisy; does NOT reach 20 fps @ 320 (~6–9 fps was
  the design estimate; the real Pi measurement is load-noisy). Loose stays the Pi
  flight path.

### Reviewer verdicts
- math-reviewer: APPROVE (gauge anchor legitimate, invents no information).
- safety-reviewer: APPROVE (divergence_guard = documented always-on flight
  invariant; `--tight` bounded but NOT FC-pose-trustworthy until the FC consumes
  `vio_degraded`).

### Known gap (PENDING — separate, unbuilt item)
- The FC consumer of `pose.refined.info['vio_degraded']` (hook
  `launcher/main.py::_start_pose_logger._on_pose`) is NOT wired. The FC link itself
  is a separate unbuilt item. Until then a sustained `vio_degraded` should drive FC
  `pos_sigma_m` inflation / loiter-RTH once the FC link exists.

### Docs (this change)
- `docs/TIGHT_COUPLED_PLAN.md` §4(g–j) — Schur, gauge reg, divergence guard, njit
  kernel + the run_ba Mermaid state diagram; Phase status updated.
- `docs/ALGORITHMS.md` §3.3 — tight sibling note extended to the chain.
- `docs/RPI5_DEPLOY.md` — `--tight` Pi profile (chain ON, guard invariant, njit
  override, still slower than loose / not 20 fps@320).
- `docs/PROC4_ARCHITECTURE.md` §9 invariant 18 — guard always-on + `vio_degraded`
  flow + pending FC gap (SAFETY).
- `README.md` — `--tight` recipe bullet.
- DRIFT fixed: `sky/vio/imu_factor_numba.py` module docstring (was "SHIPPED
  DISABLED / default-OFF", superseded by the divergence-guard default-ON gate).
