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
- **safety-reviewer: BLOCK → RESOLVED.** Caught a REAL bug: `_backend_bias` read
  `vio_map.bg` (non-existent attr) → the feed-forward was DEAD CODE (never published),
  masked by the unit test (it pushed dicts straight into the inbox). FIXED: read
  `keyframes[-1]["bg"]/["ba"]`. Also folded: integration test (`tight_smoke` asserts
  `n_bias>0` — the real source path), DECAY of the held bias on sustained degrade (FMEA),
  env CLAMPS (`_K_BIAS≤0.6`, `HOLD≥2`). FMEA/HIL note: the Pi flight build should default
  `OAKD_BACKEND_FEEDBACK=0` until HIL evidence.

## Status: P1 + P2 DONE + verified (2026-06-17)
- **P1** (`8f7c7cd`, committed) — backend publishes `backend.state` (was INERT due to the
  `_backend_bias` bug above; the fix lands in the P2 commit).
- **P2** (uncommitted) — the `_backend_bias` FIX + the live bias adoption (low-pass,
  health-gated, hysteresis, staleness, sustained-degrade decay, clamps).
- Verify: gap=0 PASS; `tight_smoke` `n_bias>0` PASS (real source path live); imu_propagate
  5 sub-checks PASS (adopt / degraded-gate / hysteresis / stale-drop / decay);
  tight_live_regression PASS; live `--tight --worker` clean; pyflakes 0.
- NEXT (deferred): tester HIL/SIL protocol (the safety-reviewer's 6 cases); P3 (#4a slip).

## Go-list for developer (P1 → P2, bias-only)
1. `vio_step` → `(T_cw, health, (kf.seq, bg, ba))`, plain float64 across `out_q`.
2. Publish `BACKEND_STATE` parent-side on the local bus; id in `vio/comms/topics.py` only.
3. `BackendStateInbox` (clone `loop_inbox`); alloc under `retain_imu and tight`.
4. `propagate_imu`: drain on odometry thread; gate = `not vio_degraded` AND N-healthy
   hysteresis AND `seq` fresh; adopt `bg/ba` via `_K_BIAS` low-pass (τ≈0.5s), never hard set.
5. SIL gate: dead-reckon accuracy improves on a gold covered-camera interval; degraded
   session falls back byte-equal; gap=0; loose untouched. Then safety-reviewer on the gate.

---

## Sensor-dropout safety guard (2026-06-17) — DONE (bench), FC plumbing DEFERRED

**Root cause of the Pi "sometimes freezes, sometimes runs / jumps dangerously far":** the OAK
USB-crashes + re-enumerates mid-run (95 crashdumps; `get_throttled=0x0` → NOT power;
depthai 3.6.1 still crashes under full flight load — recovers vs 3.7.x loop). Each crash
= seconds of NO camera AND NO IMU. The live `--tight` propagator (`propagate_imu`) prepends
the stale `prev_tail` across the blackout → `predict_state` integrates `v·dt+½a·dt²` over
the whole gap in one step → **238 cm jump on reconnect** (measured).

**FIX (uncommitted):** `_SENSOR_GAP_S=0.25s` guard in `vio/modules/propagate_imu.py` — on a
boundary gap > threshold: don't prepend, integrate only the fresh block, zero velocity,
re-seed the re-anchor, flag `info["sensor_gap_s"]`+`inertial_dr`. Test
`vio/tests/sensor_gap_guard_selftest.py`: OFF=238cm/no-flag → ON=0.2cm/flagged (995×).
gap=0 byte-identical; tight_live_regression LOCKED; pyflakes 0. Also: `--loop-search-radius`
made `nargs='?' const=5.0` (bare flag = 5m).

**Reviews:** math-reviewer APPROVE_WITH_NITS (zeroing v correct; dropping prepend loses no
info — gap has 0 samples; keeping R correct; applied the `anchor_dt=0` nit).
safety-reviewer **REQUEST_CHANGES** (NOT a BLOCK — safe in-process; FC unwired so nothing
fuses it today). **DEFER rationale (§5):** the required changes are at the FC seam, which is
not wired into the live pose path (confirmed: no `mavlink_vpe`/`reset_counter` refs in
vio/launcher/ui). Recorded as a HARD GATE on the paused FC-link work in
[[oakd-fc-position-noise]] rule 5: before any FC fusion the dropout frame MUST bump
`reset_counter`, carry a do-not-fuse marker (`Pose.tracking_ok=False`/NaN-cov), + a persistent
"VIO degraded" latch after K dropouts. **The crashing camera is the hard flight blocker.**

---

## TASK: Extract VIO windowed-BA into a 6th project `ba/` (2026-06-18) — DESIGN APPROVED (corrected)

**Goal:** architectural cleanliness (independent lifecycle, fault isolation, clean
`libsky*` port boundary), NOT performance (`--worker` already runs BA GIL-free).
Tier **T2** (touches the tight feed-forward → flight-safety pose; gate accordingly).

**architecture-reviewer verdict: REQUEST_CHANGES → corrected design below is APPROVED.**
Key finding (falsified my premise): the offline ATE oracle `verification/oracle_replay.py`
drives `sky.backend.windowed`/`sky.vio.window` DIRECTLY (loop :163-223) — it NEVER imports
`vio.engine`/`steps`/`run_ba`/`Keyframe`. So the live backend is decoupled from the frozen
path → the engine moves out WHOLESALE and the oracle stays gap=0.

**Corrected design (rule-ins):**
- Extract BOTH backends (loose `WindowedBAMap`/`ba_step` + tight `WindowedVIOMap`/`vio_step`)
  — one `BackendWorker` selected by `--tight`. `--tight` becomes a `ba/` flag.
- Move `vio/engine/*` → `ba/engine/*` and `run_ba`/`process_kf`/`BackendWorker` → `ba/modules/`.
  `Keyframe` STAYS in shared comms (vio's `emit_keyframe` produces it on the odom thread;
  it already crosses IPC to slam). ba/ reuses `sky/*` leaf verbatim. No vio↔ba cycle.
- ba/ is LIVE-ONLY; uses InProcessEngine internally (it IS the process → SubprocessEngine
  retired for BA; `--worker` stays plumbed to slam only). Offline path untouched (decoupled).
- Tight feedback: `BACKEND_STATE` (local) → new IPC POD topic `ba.state` (seq,bg,ba,degraded)
  published by ba/; vio opens a read-only client on `ba_endpoint` → EXISTING `BackendStateInbox`
  → `propagate_imu` (mirror of slam `loop.correction`). seq MUST survive to the wire (the
  `backend_bias_seq` staleness gate — built for `--worker` async — makes the IPC hop tolerable).
- `pose.refined`: ba/ publishes on its endpoint AND vio keeps a thin pass-through that
  re-publishes it onto the vio endpoint (mirror the loop.correction re-hydrate) → **UI byte-
  unchanged** (no 4th UI endpoint). Move `pose.refined` VIO_POD→BA_POD in netbridge.
- `--no-ba` becomes a launcher SPAWN gate (mirror `--no-slam`), not a vio forward.

**Importer inventory (re-point to ba.engine/ba.modules):** vio/modules/pipeline.py:63,
vio/main.py:57, vio/tests/tight_smoke_selftest.py, verification/{vio_degraded_e2e_check,
stage_profile,phase4_*,loose_vs_tight_bench}.py (~14 files). **netbridge multi-site:**
receive.py hardcodes ("capture","vio","slam") at :174,:190,:248,:322-328 + forward/receive
need `--ba-endpoint` + 4th EndpointServer. **comms gate:** add `ba/comms` (7th copy,
diff-identical to imu_camera/comms) to verification/ipc_comms_selftest.py:51 COPIES.

**ORDERED BUILD PLAN (each step independently gated; oracle gap=0 + tight_live_regression
green throughout):**
1. Pre-flight inventory recorded (this section) — the change-surface contract.
2. Additive `WireBackendState` (seq,bg,ba,degraded) + codec converter, vendored into all
   copies + ipc_comms_selftest sha256 vectors. Gate: ipc_comms_selftest (still 6 copies).
3. Scaffold `ba/`: COPY `comms/` (7th, diff vs imu_camera) + add to COPIES; COPY
   `vio/engine/*`→`ba/engine`, `run_ba`/`process_kf`/`BackendWorker`→`ba/modules`; write
   `ba/main.py run_ba_proc(vio_endpoint,endpoint,tight,...)` (calib barrier on vio endpoint,
   attach vio kf_* rings, InProcessEngine, publish pose.refined + ba.state, retain calib).
   ba/ NOT yet spawned (inert). Gate: ipc_comms_selftest (7) + `python -m ba.main --help` +
   oracle gap=0 (vio still has its own backend → unchanged).
4. Re-point vio: delete in-vio BackendModule + BACKEND_STATE producer + `--no-ba`/`--worker`
   forward; add vio read-only `ba_endpoint` client→BackendStateInbox; re-point the ~5
   verification/test importers to ba.engine. Gate: **oracle gap=0** (proves offline decoupled)
   + vio_degraded_e2e_check.
5. pose.refined UI routing: vio thin pass-through re-publish. Gate: UI replay launch +
   offscreen Qt selftest shows the VIO-BA line (tester MUST launch — "tester must verify UI").
6. Launcher: spawn ba.main after vio/before slam; `--no-ba` spawn gate; wire ba_endpoint→vio.
   Gate: build_ba_args unit test + full ./run.sh replay + **tight_live_regression** (proves
   ba.state→vio bias survives the IPC hop + seq staleness gate).
7. netbridge: BA_POD (move pose.refined out of VIO_POD), "ba" role, --ba-endpoint + 4th
   EndpointServer + the 4 hardcoded role tuples. Gate: netbridge_loopback_selftest.
8. Docs: ADR (6th project) + Mermaid (capture→vio→ba→slam, ba.state edge) + CLAUDE.md roster.

**Sequencing caution (mine):** steps 4↔6 can leave the live tight path backend-less between
"remove in-vio backend" and "spawn ba/". Do step 3 as COPY (vio keeps its engine), then land
step 4+6 together (or keep a vio re-export shim) so tight_live_regression never goes red.

**PROGRESS (developer):**
- Steps 2+3 DONE (additive, vio untouched). Step 2: new IPC POD topic `ba.state`
  (`BackendState` seq:int64/bg:f64[3]/ba:f64[3]/degraded:bool + `WireBackendState` + converter,
  modelled on `loop.correction`) added to ALL 7 comms copies; `BACKEND_STATE` kept as the lone
  vio-only exception. ipc_comms_selftest extended: `ba` added to COPIES (now 7), `ba.state` sha256
  vector added (20 vectors), and `test_source_parity` now tolerates the ONE documented
  `BACKEND_STATE`-only vio/topics.py delta (it was silently red on HEAD; now green + still fails
  on any OTHER divergence). Step 3: scaffolded inert `ba/` (`__init__`, `comms` 7th diff-identical
  copy, `engine` = COPY of vio/engine with cross-refs re-pointed + transient-dup note, `modules`
  = run_ba/process_kf/BackendWorker/publish_refined/publish_ba_window, `main.py run_ba_proc`).
  `process_kf` publishes `ba.state` over IPC (was the in-VIO local-bus `backend.state`). ba uses
  InProcessEngine (worker=False); `--worker` accepted as a logged no-op. Launcher NOT touched;
  NOTHING removed from vio. GATES ALL GREEN: ipc_comms_selftest(7) PASS, `ba.main --help` clean,
  oracle gap=0 byte-identical (vio backend entries 0.000e+00), pyflakes 0.
- NEXT: step 4 (re-point vio: delete in-VIO BackendModule/BACKEND_STATE producer, add vio
  read-only ba_endpoint client -> BackendStateInbox; re-point importers) + step 6 (launcher spawn)
  landed TOGETHER so tight_live_regression never goes red (see sequencing caution above).

### Chunk 1 (steps 2-3) DONE + verified (2026-06-18) — additive, all gates green
- `ba.state` IPC POD topic (`BackendState` seq/bg/ba/degraded, mirrors loop.correction)
  added byte-identically to ALL 7 comms copies + codec sha256 vectors.
- `ba/` scaffolded: 7th comms copy (diff-identical) + COPY of `vio/engine/*` + `ba/modules/`
  (run_ba/process_kf/BackendWorker/publish_*) + `ba/main.py run_ba_proc`. INERT (launcher does
  NOT spawn it yet; vio keeps its backend → oracle + live untouched).
- Gates REPRODUCED by main session: ipc_comms_selftest PASS (7 copies); oracle gap=0
  (gap=0.000e+00 — proves vio untouched); `ba.main --help` ok; pyflakes 0.
- **FINDING (pre-existing, now fixed):** commit `8f7c7cd` (P1/P2 backend-bias) left
  `ipc_comms_selftest` **source-parity RED on main** — it added `BACKEND_STATE` to vio/comms
  ONLY, but the gate did a strict `diff -r ... empty` with no tolerance (verified on a clean
  HEAD worktree: "[FAIL] source parity", main() returns 1). The developer added a tightly-
  scoped tolerance (`_diff_is_only_backend_state`: accepts ONLY the BACKEND_STATE topics.py
  comment+assignment delta in vio, fails loudly on anything else). Gate now green + still
  blocking.
- **STEP-4 CLEANUP (when re-pointing vio):** the in-vio local `backend.state` producer is
  retired (consumer switches to IPC `ba.state`). DELETE `BACKEND_STATE` from vio/comms/topics.py
  then → source parity becomes CLEAN → REMOVE the `_diff_is_only_backend_state` tolerance (it is
  TRANSITIONAL, only needed while BACKEND_STATE still lives in vio during the additive chunk).
### Chunk 2 (steps 4+5+6 + step-4 comms cleanup) DONE + verified (2026-06-18) — the cut
ba/ is now the LIVE backend; vio stops running it (landed together so --tight is never
backend-less). Pass-through design keeps the UI + netbridge UNCHANGED.
- **vio (main + modules):** deleted the in-vio BackendModule construction + start/drain;
  stripped the backend half of `vio/modules/pipeline.py` (process_kf / BackendWorker / the
  make_*_engine + run_ba imports) and the back-end publishers (`publish_refined` /
  `publish_ba_window`) + `run_ba` from `vio/modules/backend.py` (KEPT `emit_keyframe` — vio
  still produces KEYFRAME for ba+slam). Added `--ba-endpoint`: a read-only client (mirror of
  the slam loop_client) bridging `pose.refined` (re-emitted on the VIO endpoint via the
  existing IPCPublisher → UI byte-unchanged) + `ba.state` (→ BackendStateInbox →
  propagate_imu) back onto vio's local bus. `_adopt_backend_bias` reconciled dict→`BackendState`
  DATACLASS (`.seq/.bg/.ba/.degraded`); seq staleness gate + health gate + low-pass intact.
  OdometryWorker now subscribes `BA_STATE` (filtered to the dataclass), not BACKEND_STATE.
- **--no-ba reconcile:** REMOVED `--no-ba`/`no_ba` from vio.main (backend left vio); it is now
  a LAUNCHER SPAWN gate. The other uncommitted lean-config (`--no-slam` / `--loop-search-radius`)
  + the propagate_imu sensor-gap guard are PRESERVED.
- **ba:** `run_ba_proc` already end-to-end from chunk 1 (calib barrier on vio ep, attach vio kf
  rings, consume KEYFRAME, publish pose.refined + (tight) ba.state). PROVEN to emit pose.refined
  over IPC by the new `ba/tests/ba_refined_functional_selftest.py` (38 refined poses on a replay,
  loose + tight).
- **launcher:** added `ba_ep = f"oak.ba{suffix}"` + `build_ba_args` (--vio-endpoint/--endpoint
  [+--tight]; --worker NOT forwarded). Spawns `ba.main` AFTER vio, BEFORE slam, gated on NOT
  --no-ba; passes `--ba-endpoint ba_ep` into vio ONLY when ba spawned. Removed the old
  --no-ba→vio forward. Refactored the spawn/drain to track procs by ROLE (a `named` dict) so the
  --no-ui drain is not a fragile procs[] index (ba sits between vio and slam — no IndexError).
  numba budget + orphan-cleanup role map extended for "ba".
- **comms cleanup (step 4):** DELETED `BACKEND_STATE` from vio/comms/topics.py → source parity
  CLEAN (vio/comms == anchor) → REMOVED `_diff_is_only_backend_state` + `_BACKEND_STATE_EXCEPTION`
  from ipc_comms_selftest. The now-stale "Distinct from vio's intra-process backend.state"
  comment was rewritten IDENTICALLY in all 7 topics.py copies (parity held). `ba.state` kept in
  all 7.
- **deleted the transient dup + re-pointed importers:** DELETED `vio/engine/`. Re-pointed every
  live importer to ba.engine/ba.modules: `vio/tests/tight_smoke_selftest.py`,
  `verification/imu_factor_njit_ate.py`, `verification/vio_degraded_e2e_check.py` (all 3 of
  InProcessEngine / steps.ba_step+vio_step / modules.run_ba). stage_profile / loose_vs_tight_bench
  / phase4_* drive `sky.*` directly — they never imported vio.engine (verified by grep).
- **GATES (all green, real output in the handoff):** (1) oracle_replay_selftest gap=0 — offline
  decoupled, byte-identical; (2) ipc_comms_selftest PASS, source parity CLEAN (no exception), 7
  copies; (3) tight_live_regression PASS (push/covered/ZUPT/shake + closed-loop) + imu_propagate
  5 sub-checks PASS with the `BackendState` DATACLASS (proves the dataclass-vs-dict reconcile +
  seq staleness gate survive the IPC hop); (4) no_ba_no_slam + stabilize/depth_icp/direct/ba_window/
  frontend_viz/netbridge forward selftests PASS; (5) ba_refined_functional PASS (≥1 pose.refined
  over IPC, loose+tight) + probe confirmed pose.refined ALSO re-emitted on the VIO endpoint (UI
  contract); (6) pyflakes vio/ ba/ launcher/ + ipc_comms = 0; (7) headless `./run.sh --tight
  --no-ui --session ... --max-frames 150` — capture→vio→ba→slam up, pose.odom flows, clean
  Ctrl-C/natural teardown (all os._exit(0)), no traceback.
- **DEFERRED (follow-ups, NOT this chunk):** netbridge BA_POD (pose.refined stays VIO_POD —
  pass-through makes it unnecessary now); docs/ADR/Mermaid; the opt-in `--ba-window` visualiser
  (BA_WINDOW) wiring INTO ba (the DEFAULT no-ba-window UI path works). The tight backend knobs
  `--stabilize-velocity`/`--depth-icp`/`--ba-window`/`--worker`/`--backend-window`/`--backend-iters`
  are kept on vio.main + still forwarded by build_vio_args (gate 4's stabilize/depth_icp/ba_window
  forward selftests assert the vio forward), but are now INERT in vio (backend left) and NOT yet
  routed to ba — a follow-up must thread them into build_ba_args/ba.main to restore their effect.
- NOT committed.

### Chunk 2 (steps 4-6) DONE + verified (2026-06-18) — ba/ is the live backend
- vio backend removed; ba/ spawned by launcher (capture→vio→ba→slam→ui); pass-through
  re-emits pose.refined on the VIO endpoint (UI byte-unchanged) + ba.state→BackendStateInbox.
  `--no-ba` is now a launcher spawn gate. BACKEND_STATE deleted from vio/comms → parity CLEAN
  (transitional tolerance removed). vio/engine deleted; importers re-pointed.
- Gates REPRODUCED by main session: oracle gap=0 (8 exact-zero); ipc_comms PASS (7 copies,
  clean parity); tight_live_regression PASS; full ./run.sh WITH Qt UI (offscreen) brings up
  all 5 procs clean, no traceback, "BA pass-through ENABLED" logged.
- **UI breakage FOUND + FIXED** (the thing the user warned about): ui_dataflow_selftest spawned
  the OLD 3-proc stack (no ba) → pose.refined trail empty → [FAIL]. The dev's gates MISSED it
  (never ran ui_dataflow). Fixed: test now spawns the 4-proc stack (ba + vio --ba-endpoint);
  default max-frames 20→80 (the cap→vio→ba→vio-passthrough chain needs more keyframes to
  observe than the old in-vio backend). FULL ui_dataflow (Qt + menus) now PASS. netbridge
  loopback PASS (UI renders).

### Chunk 3 (remaining — so NO flag silently breaks) — IN PROGRESS (developer)
1. **Backend knobs INERT** (dev kept them on vio.main + forwarded, but the backend left vio):
   `--stabilize-velocity` / `--depth-icp` / `--backend-window` / `--backend-iters` must route to
   ba.main + run_ba_proc→backend cfg (build_ba_args), be REMOVED from vio.main + build_vio_args,
   and the forward selftests updated to assert them in the BA argv. (--stabilize-velocity is the
   shipped shake fix the user was told to use — currently a no-op end-to-end.)
2. **--ba-window visualizer BROKEN** (_ba_window_png FAIL: 0 ba.window): wire the BA-window
   capture (ba/engine/ba_capture.py) into ba/ so --ba-window publishes ba.window; pass it
   through vio like pose.refined (UI + netbridge unchanged) so _ba_window_png passes.
3. **Docs (step 8):** ADR for the 6th project + Mermaid (capture→vio→ba→slam→ui, ba.state edge,
   pose.refined pass-through) + CLAUDE.md roster note. (docs-writer.)

**Chunk 3 TASK 1+2 DONE + verified (2026-06-18) — no backend flag is inert any more.**
- Pre-split wiring CONFIRMED (git `8f7c7cd:vio/main.py`): old in-vio backend was
  `BackendModule(local, K, window=backend_window, iters=backend_iters, latest_only=False,
  worker=worker, tight=tight, stabilize_velocity=.., depth_icp=.., capture_window=ba_window)`
  + `ba_window_on = ba_window and not tight` gated appending `BA_WINDOW` to the output
  publisher's pose topics. `BackendWorker.__init__` in ba/ ALREADY accepted all these params;
  ba/main.py just wasn't passing them through.
- **TASK 1 (knobs route to ba):** `run_ba_proc` + ba.main argparse gained `stabilize_velocity/
  depth_icp/backend_window/backend_iters` + `ba_window`; threaded into `BackendWorker(...,
  window=backend_window, iters=backend_iters, stabilize_velocity=.., depth_icp=..,
  capture_window=ba_window_on)`. `build_ba_args` forwards them (stabilize/depth_icp tight-gated
  same as the old vio forward; backend_window/iters only when non-default). REMOVED the 4 knobs
  + `--ba-window` from vio.main (argparse + run_vio signature + call site) + from build_vio_args.
  vio's `--worker` KEPT (inert in vio but still live for slam; not in the TASK 1 removal list).
  **FINDING:** `--backend-window`/`--backend-iters` were NEVER launcher-forwarded pre-split (only
  vio.main argparse defaults the launcher never set → dead end-to-end); ADDED them to the launcher
  argparse + build_ba_args so they now reach ba (operator-reachable for the first time).
- **TASK 2 (--ba-window restored):** ba.main gained `--ba-window`; `ba_window_on = ba_window and
  not tight` → `capture_window=ba_window_on` + append `BA_WINDOW` to ba's out_topics. vio
  pass-through `_BA_FEEDBACK_TOPICS` + `_OUTPUT_TOPICS` + `_pose_topics` gained `BA_WINDOW`
  (bridged from ba, re-emitted on the VIO endpoint — UI source reads it there, unchanged; BA_WINDOW
  is pure POD, converter already in all comms copies). `build_ba_args` forwards `--ba-window`
  (resolve_ba_window stays the launcher gate). `_ba_window_png` retargeted to the 3-proc stack
  (vio --ba-endpoint + ba --ba-window + capture).
- **GATES (all green, real output in handoff):** (1) oracle gap=0 (vio + ba backend entries
  exact-0 / ~1e-8mm pre-existing tol); (2) ipc_comms PASS 7 copies, source parity CLEAN; (3)
  tight_live_regression PASS; (4) all 9 launcher forward selftests PASS — stabilize/depth_icp now
  assert the BA argv + NOT the vio argv, ba_window asserts build_ba_args + backend_window/iters;
  (5) ui_dataflow PASS (4-proc + all menus); (6) _ba_window_png PASS (12 ba.window snapshots
  through the ba→vio pass-through, PNG non-blank); (7) netbridge_loopback PASS (unchanged); (8)
  headless `./run.sh --tight --stabilize-velocity --no-ui` — ba log shows `ba: tight
  velocity-stabilize ON (CV prior + gated ZUPT)` (the shipped shake fix is LIVE end-to-end again);
  (9) pyflakes vio/ ba/ launcher/ = 0. ba_refined_functional loose+tight PASS.
- NOT committed. REMAINING in Chunk 3: step 3 docs (ADR + Mermaid + CLAUDE.md roster) — docs-writer.

---

## TASK: `fc/` project — UART output to the FC (2026-06-18) — PROTOCOL = dblink (user-locked)

**Goal:** consumer-only sibling `fc/` that streams the VIO earth-frame pose to the in-house
drone FC over UART. Tier **T3** (flight-safety). Staged: BENCH/UI-verify first, then flight-harden.
architecture-reviewer: **APPROVE_WITH_NITS** (structure). `fc/` already exists with the approved
structure (latest-wins, UART-off-callback, safety floors, reset_counter) — only the PROTOCOL swaps.

**PROTOCOL DECISION (user, 2026-06-18): dblink, NOT MAVLink.** The FC speaks its own `dblink`
(sibling repo `../flight-controller`). MAVLink VPE (`sky/fc/mavlink_vpe.py`) is SUPERSEDED — remove it.
- **Wire frame** (host→FC, verbatim `build_db_frame`, [tools/dblink_test.py]): `'d''b'` + CMD(1B) +
  CLASS(1B,=0x00) + LEN(2B LE) + payload + checksum(2B LE). Checksum = `(cmd+class+len_lo+len_hi+
  sum(payload)) & 0xFFFF`, LE. FC routes by the CMD byte (`data[0]`); FC does NOT verify the DB
  checksum (only UBX) but the frame must be well-formed. FC has **no vision receiver yet** → a
  matching FC-side module + EKF wiring is SEPARATE work in `../flight-controller` (user owns).
- **CMD = `DB_CMD_VIO_POSE` = 0x0C** (proposed; FC header owns the final value).
- **Payload (38B, little-endian, `struct '<8f I 2B'`):** `pos_n,pos_e,pos_d` (f32 NED m);
  `q_w,q_x,q_y,q_z` (f32 quaternion body→NED, Hamilton, unit — **FC extracts heading itself**, no
  Euler imposed, gimbal-lock-free); `pos_sigma_m` (f32 1-σ → √R; inflated when degraded);
  `age_us` (u32); `reset_counter` (u8); `flags` (u8: bit0 pos_valid, bit1 att_valid, bit2 degraded).
- **TIME SYNC = "Level 1 / age" (user-locked, simplest, TX-only, NO clock sync).** Send `age`;
  FC computes `validity_fc = fc_rx_time − age − C`. age is a *duration* → FC anchors it to its OWN
  clock; the module's absolute clock is never needed.
  **HONEST age property (do NOT call it "conservative/errs old" — it is biased YOUNGER):** because
  `O_est = running-min(recv − ts_device) = O + min(capture→fc pipeline latency)`, the reported
  `age = send − ts − O_est` UNDER-reports the true capture→send age by ≈ that minimum pipeline-latency
  floor. That floor is NOT sub-ms — it includes the VIO compute floor (tens of ms) + IPC hop + sender
  queue wait. So `age` carries only the VARIABLE latency ABOVE the floor (fc queue wait + pipeline
  jitter); the only hard guarantees are age ≥ 0 (floored, never negative) and this constant under-report.
  Therefore **C must absorb the floor: `C = UART_transport + pipeline_latency_floor`** (NOT just the
  ~4ms UART) — FC-calibrated once via `DB_CMD_ECHO` + the observed pipeline floor. With C set that way,
  `fc_rx_time − age − C` lands on the true capture instant.
  **age clock detail:** `pose.ts_ns` is DEVICE clock (`imu_camera/io/synced.py`), populated live
  (`publishers.py:192` `PoseMsg(seq, frame.ts_ns, …)`). fc/ estimates `O` by a running-min of
  `(recv_host_s − ts_ns·1e-9)` (relax-up ~1e-4/s for drift; a candidate >0.5s below the min is rejected
  as a corrupt/future ts so one bad sample can't latch `O_est` low forever), then
  `age_us = clamp((send_host_s − ts_ns·1e-9 − O)·1e6, 0, …)`. A hard `age > 1s` ceiling drops the frame.
  Fallback ts_ns==0 → age from recv time only. Future **"Level 2"** = `imu_camera` host-capture-stamp (or
  passive regression on FC's `t_ms` heartbeat) → age becomes the FULL absolute capture→send age and
  C reduces to UART transport only (swap `age_us`→`t_fc_ms`, u32, same size, wire unchanged).

**Key corrections (baked into the plan):**
- **reset_counter trigger** (MAJOR): NOT `loop.correction` (tight-only + blended, invisible on
  the loose/`--direct` default). Key it off `info["sensor_gap_s"]` (re-lock after dropout) +
  a fc-local position-JUMP detector (delta vs `pos_sigma_m`+dt). Rising-edge debounce (bump once).
- **reset_counter owned in `fc/`** (sender state), NOT `sky/`. `sky/fc/fc_earth_pose.py` stays
  PURE (pose→earth-pose, no time/IO/counters) → testable + the SSOT shared by ui/ + fc/.
- **UART off the IPC callback** (REQUIRED): callback only stores latest (1-slot); a daemon UART
  thread loops fixed cadence (default 30Hz, clamp [10,50]), drops stale (>250ms), `write_timeout`,
  non-fatal on error (never back-pressure flight / never crash the run).
- **Bench FLOOR (non-negotiable from send #1):** the latest-wins thread + **never send an
  over-confident cov** (cov from `pos_sigma_m` when present; NaN-cov "unknown" when absent OR
  `vio_degraded`/`sensor_gap_s` set). Full reset/jump-detector edge logic = hardening pass.
- `fc/` = FULL vendored `comms/` (9th copy, add `"fc"` to ipc_comms_selftest COPIES); it just
  doesn't instantiate the server/publisher. Shared conversion in `sky/fc/fc_earth_pose.py`
  (NOT the frozen `comms/lib/misc/frames.py`). Heading RELATIVE (no mag); `R_body_cam` mount
  extrinsic = operator config, default I; rpy derived from the quaternion (singularity-safe).

**Build plan (dblink swap — structure already in place, DONE items marked):**
- ✅ `sky/fc/fc_earth_pose.py` SSOT extracted; ui's call sites refactored (UI byte-unchanged).
- ✅ `fc/` package (vendored comms 9th copy; `"fc"` in ipc_comms COPIES); latest-wins UART thread,
  staleness floor, reset_counter edge logic, calib barrier — all present (MAVLink-flavored).
- ✅ launcher `--fc PORT[:BAUD]` + `--fc-rate` + `--fc-mount`, `parse_fc_port`, `build_fc_args`,
  gated spawn after slam — wired (docstrings still say MAVLink → update).
- **(1) NEW `sky/fc/dblink.py`:** `DB_CMD_VIO_POSE=0x0C`, `build_db_frame` (verbatim checksum),
  `pack_vio_pose(pos_ned, q_wxyz, pos_sigma_m, age_us, reset_counter, flags) -> frame bytes`.
- **(2) `fc/main.py` swap:** import dblink (not mavlink_vpe); `send_once` packs the dblink frame
  (now carries the FULL quaternion + age_us + flags); add the age/offset computation above; degraded
  → inflate `pos_sigma_m` + set flag bit2 (NOT a NaN on the wire). Keep latest-wins/staleness/
  reset_counter UNTOUCHED. Update docstrings MAVLink→dblink.
- **(3) Remove `sky/fc/mavlink_vpe.py`** + drop the MAVLink dep refs (confirm nothing else imports it).
- **(4) Launcher cosmetics:** `build_fc_args`/`--fc-rate` help text MAVLink VPE → dblink.
- **(5) UI:** fix the misleading `heading == yaw` comment in `ui/qt/panels.py` (value already = heading).
- **(6) tester:** ipc_comms 9-copy, NEW `sky/fc/dblink` selftest (golden frame + `parse_db_stream`
  roundtrip + checksum parity), rewrite `fc/tests/fc_sil_selftest.py` for dblink (frame bytes + age
  + reset edge + latest-wins-under-load + degraded floor), `./run.sh` replay `--fc <pty>` + offscreen UI.
- **(7) T3 fan-out before in-flight send:** safety + security + math reviewers on the diff. (8) docs.
- **OPEN (user):** final `DB_CMD_VIO_POSE` value (FC header); who writes the FC-side receiver+EKF fusion.

---

# PLAN — Stage 4b (Pi side): downward-lidar read + BUNDLE into VIO pose · Tier T3

Task: VL53L1X downward rangefinder over I2C on the Pi, published as `lidar.range`,
BUNDLED into the EXISTING `db_vio_pose_t` (38B -> 42B) on the fc/ UART sender — NOT a
separate dblink message. The FC side (flight-controller repo) is DONE and expects the
range INSIDE the VIO message. Supersedes the leftover SEPARATE-channel lidar attempt
(`DB_CMD_LIDAR_RANGE=0x0E`, `pack_lidar_range`) — DELETED here.

LOCKED WIRE (from the FC, matched byte-for-byte): `db_vio_pose_t` = `'<8fIBBf'` = 42 B.
8×f32 [pos_n,pos_e,pos_d,q_w,q_x,q_y,q_z,pos_sigma_m], u32 age_us, u8 reset_counter,
u8 flags, f32 range_m @ offset 38. CMD = 0x0C. VIO_FLAG_RANGE_VALID = 0x08.

## Steps / status
1. `sky/fc/dblink.py`: 38->42B `pack_vio_pose` (+`range_m`,+`range_valid` -> 0x08 fold);
   DELETE `DB_CMD_LIDAR_RANGE`/`pack_lidar_range`/`_LIDAR_STRUCT`/`LIDAR_LEN`. [DONE]
2. comms contract: add `LIDAR_RANGE` topic + `WireRange` POD + registry entry to ALL
   copies (anchor `imu_camera` + depth/vio/slam/ui/launcher/netbridge/ba/fc) and the
   NEW `lidar/comms` (`cp -r imu_camera/comms`). diff -r stays EMPTY. [DONE]
3. `lidar/` project: `io/vl53l1x_reader.py` (real I2C via a pure-`smbus2`
   register-level driver for a bare VL53L1X + MOCK), `main.py` (read->publish
   `lidar.range`, bridge to `oak.lidar`), `requirements.txt`, `tools/characterize.py`
   (I2C, prints FC `disarm_range`). [DONE]
4. `fc/main.py`: `LatestRange` 1-slot holder + `lidar.range` client + bundle range into
   `UartSender.send_once()`; freshness-gated. NO second frame. [DONE]
5. `launcher/main.py`: `build_lidar_args` + `--no-lidar` spawn gate + `oak.lidar`
   endpoint + spawn lidar (after slam, before fc). [DONE]
6. `requirements-flight.txt`: + `smbus2` (after pyserial) — the bare-VL53L1X register
   driver needs no other sensor dep. [DONE]
7. selftests: `fc_dblink_selftest` + `fc_sil_selftest` -> 42B + 0x08; lidar mock
   selftest; add `"lidar"` to `ipc_comms_selftest` COPIES + a `WireRange` vector. [DONE]

## Gates (all run GREEN 2026-06-21)
- fc_dblink_selftest PASS @42B + 0x08 set/clear ........... [GREEN]
- fc_sil_selftest PASS @42B (range fold; stale/absent->0) . [GREEN]
- ipc_comms_selftest (10 copies byte-identical + WireRange) [GREEN]
- lidar mock selftest (gate status!=0->0; publish RT) ..... [GREEN]
- grep DB_CMD_LIDAR_RANGE/pack_lidar_range/_LIDAR_STRUCT=0 . [GREEN]
- grep cv2/opencv in lidar/ = 0 (+cv2-blocked import OK) .. [GREEN]
- oracle_replay_selftest (gap=0, unaffected) ............. [GREEN]
- launcher build_lidar_args / --no-lidar gate ............ [GREEN]
- FC wire byte-match: Pi pack <-> FC '<8fIBBf' (CMD 0x0C,
  42B, range@38, flag 0x08) ............................. [GREEN]
- real lidar.main subprocess -> fc-style client reads
  lidar.range (valid+invalid gated) ..................... [GREEN]
- depth proc smoke (comms copies still functional) ....... [GREEN]

## Driver: FINAL + verified on-device (2026-06-22)
- bare VL53L1X @ 0x29, pure-`smbus2` register driver (`VL53L1XReader`): writes the
  91-byte ST/Adafruit default config block, long mode @ 50 ms; gate = status 0x09 +
  30..4000 mm. Verified on-device (model id EA CC 10, RESULT__RANGE_STATUS 0x09, live
  distance). No `pimoroni-vl53l1x` (404 on PyPI) / no C-ext (no aarch64-py3.13 wheel) /
  no Blinka -- `smbus2` is the whole driver. Reader is still SWAPPABLE + has a MOCK;
  I2C errors -> valid=0, never crash. `--characterize` prints the FC `disarm_range`.
- Remaining: run `--characterize` on the ground for `disarm_range`; full-rig HIL
  (lidar -> fc -> dblink on the assembled drone).

## Blockers: none.
