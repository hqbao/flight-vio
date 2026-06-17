# PLAN — FC link (VIO → custom FC ESKF) · Tier T3

> Supersedes the prior PLAN.md (tight-solve optimisation chain, COMPLETE — that
> record lives in `docs/TIGHT_COUPLED_PLAN.md` §4(g–j) + git history). This is the
> "Known gap (PENDING)" that prior plan pointed to: the FC link itself.

## Task
Stream the live VIO pose (`pose.odom`, `T_world_cam`) to the user's **custom**
flight controller's ESKF as MAVLink `VISION_POSITION_ESTIMATE` (#102) over UART,
with covariance from `pos_sigma_m` and a loop-jump `reset_counter`. Insertion
point = the existing `# === FC OUTPUT HOOK` in
`launcher/main.py::_start_pose_logger._on_pose` (a read-only `pose.odom` consumer).

## Decisions (delegated by the user: "Custom FC" + "you decide the lib")
- **Self-owned, dependency-free MAVLink v2 packer** for the ONE message (#102) —
  NO pymavlink in the flight runtime (keeps the lean Pi image + the self-owned
  ethos; maps to the roadmap's future C `fc_link_mavlink.c`). pymavlink is used
  ONLY as a dev-time GOLD cross-check in the selftest (never a flight dep).
- **Opt-in** `--fc-out <port>` (+ `--fc-baud`, default 921600): OFF by default, so
  the flight stack / oracle / `gap=0` are byte-unaffected when disabled.
- Custom FC ⇒ no PX4/ArduPilot constraint; WE define the contract, the FC parses it.

## Phases (each independently testable, opt-in, off-path when disabled)
- **A — MVP (in progress).** Self-owned #102 packer (`sky/fc/mavlink_vpe.py`,
  stdlib-only, leaf) + byte-correctness selftest [DONE]. Then: optical→NED
  transform + wire into `_on_pose` behind `--fc-out` over pyserial.
- **B.** Loop-closure → bump `reset_counter` + re-anchor (NEVER fuse the jump — a
  fused discontinuity injects phantom velocity into the FC ESKF).
- **C.** Velocity (`ODOMETRY` msg) where the FC supports it (VIO drifts → prefer
  velocity); `vio_degraded` → inflate R / signal loiter-RTH; extend `pos_sigma_m`
  to `--tight`/loose (today it is `--direct`-only).

## FRAME TRANSFORM — RESOLVED (math-reviewer; REQUEST_CHANGES → design locked)
`pose.odom` world = **gravity-aligned OPTICAL** (proven from `gravity_aligned_R0`,
`sky/imu/imu.py:257`, NOT the docstrings): +X = camera-start right, **+Y = DOWN
along real gravity (the ONLY absolute axis)**, +Z = camera-start forward. Yaw is
**vision-relative, NOT North-referenced** (zero magnetometer in the tree); roll/
pitch ARE gravity-locked/absolute. `frames.py`'s `World=NED` / `R_BODY_CAM=I` is
DEAD, misleading doc — never applied in the pipeline; do NOT wire from it.
- **Position → NED:** `R_world→NED = [[0,0,1],[1,0,0],[0,1,0]]` (det +1; Down=+Y
  gravity correct). BUT North/East are a FICTITIOUS gauge (vision-relative) → send
  as a VISION / local-FRD frame (`MAV_FRAME_VISION_NED`), NOT true-North NED.
- **Attitude → FRD:** FULL similarity `R_ned_frd = R_world→NED · R_cw · R_FRD_OPTᵀ`,
  `R_FRD_OPT=[[0,0,1],[1,0,0],[0,1,0]]` (optical→FRD); then `quat_to_rpy` (ZYX) as-is.
  Naive world-only rotation = WRONG (verified `(70,0,125)` vs correct `(0,−20,35)`).
- **Covariance:** `Σ_ned = R Σ Rᵀ` is a provable NO-OP while σ is isotropic scalar
  (`σ²I → σ²I`) — send σ² on the diagonal directly; the matmul becomes load-bearing
  only at the anisotropic (inv-Hessian) upgrade. Mark that in code.
- **SAFETY (the crash-risk finding):** VPE yaw is gauge-free → the FC must NOT fuse
  it as absolute heading. ⇒ **MVP DECISION = POSITION-ONLY**: send position + the
  Down-correct vision frame, attitude block marked UNKNOWN (NOT a small variance),
  the FC keeps its own gyro/mag heading. Attitude (relative-yaw) is a later,
  explicitly-flagged step.

## FC-side contract (what the user's custom FC must parse)
Standard MAVLink v2 frame, `STX=0xFD`, msgid 102, `CRC_EXTRA=158`. Payload (MAVLink
size-descending order, trailing zeros truncated per v2 → zero-pad to 117 on
receive): `uint64 usec | float x,y,z,roll,pitch,yaw | float covariance[21] |
uint8 reset_counter`. `covariance` = upper-triangular 6×6 pose cov; `cov[0]=NaN` ⇒
"unknown". Position variance = σ² from `pos_sigma_m` at cov indices 0/6/11.

## Tier-T3 gate sequence
1. math-reviewer — frame/covariance transform. ✅ DONE (REQUEST_CHANGES → resolved
   into the position-only MVP above; frame proven from `gravity_aligned_R0`).
2. developer — wire the position-only sender (transform + pyserial + `--fc-out`). ⏳ NEXT
3. architecture-reviewer — `sky/fc` leaf placement + opt-in wiring + off-path proof.
4. safety-reviewer + security-reviewer — FC-link behavior (relative-yaw / reset_counter /
   degraded; the link's flight-safety + integrity posture).
5. tester — loopback SIL (sender ↔ parser) + a documented HIL/UART protocol.
6. docs-writer — `docs/` FC-link section + the FC-side contract; FIX the misleading
   `frames.py` docstring (`World=NED`/`R_BODY_CAM=I` is dead/false).

## Verdicts / blockers
- **Packer** (`sky/fc/mavlink_vpe.py`) + selftest: BUILT, self-verified (CRC anchor
  `MCRF4XX("123456789")==0x6F91` + independent self-parse round-trip + truncation).
  pymavlink GOLD cross-check SKIPPED (pymavlink not installed — `pip install
  pymavlink` as dev-only to run it). gap=0 unaffected (new leaf; nothing imports it yet).
- **math-reviewer:** REQUEST_CHANGES — frame is gravity-aligned optical with a
  GAUGE-FREE yaw, not NED. Resolved → MVP is POSITION-ONLY in a vision frame; FC
  must not fuse VPE yaw as heading. Attitude needs the FULL similarity transform
  when added. `frames.py` docstring is dead/misleading (developer to fix).
- **NOT committed** (FC work is mid-Phase-A; user commits when ready).
