# PLAN — Complete tight-coupling: backend → live feed-forward · Tier T3

> Supersedes the FC-link plan (PAUSED: packer committed `be9d511`). User decision:
> build the COMPLETE tight VIO on the Mac mini now; the lightweight Pi config
> (`--no-ba`/`--no-slam`) comes AFTER.

## Goal
Feed the backend's optimized state back into the live IMU propagation so `pose.odom`
becomes the proper VINS/Basalt forward-propagation from the backend truth — HEALTH-
GATED so a diverging BA never corrupts the live pose.

## Design REVIEWED by architecture + math (both REQUEST_CHANGES → resolved below)
Both converged on: **adopt BIAS first (the real, unambiguous win); DEFER the
pose/velocity re-base.** Key corrections folded in:
- **Source is the WORKER SUBPROCESS.** Optimized `v/bg/ba` live only in the child
  (`sky/vio/window.py` run_ba); `vio_step` returns only `(T_cw, health)`. → P1 must
  FIRST extend the worker return to carry `(kf.seq, bg, ba)`.
- **`BACKEND_STATE` is LOCAL-BUS-ONLY.** OdometryModule + BackendModule share the same
  in-process `local` bus (`vio/main.py:225/245/267`) → the topic NEVER crosses IPC.
  Add the id to **`vio/comms/topics.py` ONLY** — NOT the wire/converters, NOT the
  other 6 comms copies (there are 7 total). Adding it to the codec is what would risk gap=0.
- **`seq` = `kf.seq`** (the frame seq the keyframe was emitted under), not the window index.
- **NO velocity re-base.** The abs-velocity gauge anchor is ~1111–10000× weaker than the
  IMU tie → not absolute-trustworthy; re-basing v risks copying a poorly-observed gauge.
- **Bias = bounded low-pass (τ≈0.5s), not a hard set** (BIBO-stable either way, but a
  raw 0→0.1 m/s² `ba` jump kinks velocity; low-pass also rejects a noisy solve). `bg` may go faster.
- **Health-gate ≠ just `vio_degraded`:** add (a) hysteresis (N consecutive healthy kfs
  before adopting; 1 degraded → stop immediately — fast to distrust, slow to trust),
  (b) staleness drop (drop a `BACKEND_STATE` whose `seq` ≤ last-adopted), (c) the bias low-pass clamp.

## Phases (each tight+LIVE only → gap=0; independently testable)
- **P1 — worker return + local-bus publish.** Extend `vio_step` →
  `(T_cw, health, backend_state)` with `backend_state = (kf.seq, bg, ba)` (plain
  float64; mirror `_vio_health`'s scalar discipline; crosses the existing `out_q`).
  Publish `BACKEND_STATE` from `run_ba`/`process_kf` (parent) on the LOCAL bus; topic
  id in `vio/comms/topics.py` ONLY.
- **P2 — bias-only feed, health-gated.** New `BackendStateInbox` (clone
  `loop_inbox.py`: bounded deque + lock + drain-in-order). `propagate_imu` drains on
  the odometry thread; adopt `bg/ba` via a per-keyframe low-pass `_K_BIAS` ONLY when
  `not vio_degraded` AND N-healthy hysteresis AND `seq` fresh. NO pose/velocity re-base.
  Allocate the inbox+subscription only under `retain_imu and tight` (same gate as
  `loop_inbox`). → loose/oracle byte-identical.
- **P2b — pose re-base (CONDITIONAL, only if SIL shows residual position drift P2
  doesn't close).** Re-base POSE only, as a STATIONARY SE(3) delta vs a stored
  per-`seq` pre-correction anchor (reuse `kf_pose_pre`), and **lift the backend pose by
  `loop_applied` first** (mirror `propagate_imu.py:296-310`) so it doesn't re-inject
  loop-removed drift. Blend with the bounded geodesic gain (never snap). Re-run
  architecture+math review on P2b specifically.
- **P3 — #4a KLT-slip rejection** before BA → widens the healthy window → more feed
  coverage. Enhancement, not a blocker.

## Invariants (every phase)
- Tight + LIVE only (`retain_imu`) → loose/oracle byte-identical (gap=0).
- `BACKEND_STATE` local-bus-only — never IPC wire/converters/other copies.
- Health-gated (`vio_degraded` + hysteresis + staleness) → diverging BA never fed.
- Bias adopted via bounded low-pass; backend never touches `nav` directly (only the
  odometry thread mutates `nav`, via the inbox drain — mirror `_drain_loop_inbox`).
- Opt-out `--no-backend-feedback` (also the default-off-on-Pi lever).

## Reviewer verdicts
- **architecture-reviewer: REQUEST_CHANGES** → resolved. BLOCKERS: P1 source is the
  subprocess (extend worker return); `BACKEND_STATE` local-bus-only (topics.py only, 7
  copies not 6). MAJOR: loop-frame composition; bias-only first; hysteresis+staleness.
- **math-reviewer: REQUEST_CHANGES** → resolved. Re-base must be a stationary SE(3)
  delta (not blend-toward-absolute → 250mm lag proof); lift by `loop_applied` (else
  re-inject 50cm loop drift); pose-only no velocity (gauge 1111–10000× too weak);
  bias low-pass τ≈0.5s (BIBO-stable, kink-smoothing). Bias-only P2 = clean consistent update.

## Go-list for developer (P1 → P2, bias-only)
1. `vio_step` → `(T_cw, health, (kf.seq, bg, ba))`, plain float64 across `out_q`.
2. Publish `BACKEND_STATE` parent-side on the local bus; id in `vio/comms/topics.py` only.
3. `BackendStateInbox` (clone `loop_inbox`); alloc under `retain_imu and tight`.
4. `propagate_imu`: drain on odometry thread; gate = `not vio_degraded` AND N-healthy
   hysteresis AND `seq` fresh; adopt `bg/ba` via `_K_BIAS` low-pass (τ≈0.5s), never hard set.
5. SIL gate: dead-reckon accuracy improves on a gold covered-camera interval; degraded
   session falls back byte-equal; gap=0; loose untouched. Then safety-reviewer on the gate.
