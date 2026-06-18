# `vio/` — the visual-inertial odometry project (Phase 3 of the split)

The **third** of the split projects (`imu_camera`, `depth`, `vio`, `ba`, `slam`,
`ui`), built by replicating the **proven `imu_camera` template**. `vio` subscribes
to the capture process over IPC, runs the RGB-D visual odometry (+ gyro prior) and
the live IMU dead-reckon, emits a `keyframe` stream, and republishes its results on
its own IPC endpoint for `ba` / `slam` / UI / tools.

The **windowed bundle adjustment is NO LONGER in `vio`** — it was extracted into the
[`ba/`](../ba/README.md) process (ADR
[0001](../docs/adr/0001-extract-windowed-ba-into-ba-process.md), commit `611dc45`):
`vio` now PRODUCES `keyframe` (both `ba` and `slam` consume it) and is a pure
*consumer* of `ba`'s `pose.refined` over a read-only `--ba-endpoint` client, which
it re-emits on its own endpoint so the UI keeps a single endpoint (see
[`vio/main.py`](#viomainpy--the-vio-process) below).

```
imu_camera.main ─(oak.capture)─▶ vio.main ─(oak.vio)─┬─▶ ba.main   ─(oak.ba)─┐
   capture proc       IPC         VIO proc     IPC    └─▶ slam.main ─(oak.slam)┴─▶ ui / tools
                                              ba.pose.refined re-emitted on oak.vio ┘
```

It was ported **VERBATIM** from the reference oracle (`ours/`): only import roots
were re-rooted and Flow/Task/Bus classes were renamed. **No algorithm changed**,
so the numerical output is byte-identical to the oracle — proved by
`vio.tests.vio_ba_selftest` (its numbers match `ours/tools/vio_ba_selftest.py`
line-for-line).

## Layers

| Package | Role | Source it was ported from |
|---------|------|---------------------------|
| `vio/comms/` | the **FROZEN** vendored comms contract | copied **bit-identically** from `imu_camera/comms` |
| `vio/resolution_build.py`, `vio/warmup.py` | the math-coupled config builders + JIT warmup VIO owns at the project root | `ResolutionProfile.{frontend,odometry,ba_huber_px}` + `ours/lib` warmup |
| `vio/modules/` | the odometry pipeline + keyframe emission + live IMU dead-reckon (**procedural** step functions + one plain worker thread) | `ours/flows/odometry` |
| `vio/main.py` | the VIO process (incl. the `--ba-endpoint` pass-through) | `ours/proc/vio.py` |
| `vio/tests/` | regression self-tests | `ours/tools/{klt,vio_ba}_selftest.py` |

> **No `vio/engine/`.** The in-process engine that drove the heavy keyframe solve
> moved to [`ba/engine/`](../ba/README.md) when the windowed BA was extracted into
> the `ba` process; `vio/engine/` was **deleted**. `vio` no longer runs any
> keyframe solve — it only emits the `keyframe` that `ba` (and `slam`) consume.

### `vio/comms/` — byte-identical, do not hand-edit

`vio/comms` is **copied bit-identically** from `imu_camera/comms`. A CI gate runs
`diff -r vio/comms imu_camera/comms` and it must be empty (build caches —
`__pycache__`, `*.pyc`, `*.nbc/.nbi` — are git-ignored and excluded). All its
internal imports are RELATIVE, so the copy works as `vio.comms` unchanged. **Never
hand-edit it** — change `imu_camera/comms` and re-vendor.

Public API the VIO process uses: `LocalPubSub`, `IPCPubSub(role="server"|"client")`,
`IPCPublisher`, `IPCSubscriber`, `RingRegistry`, `topics`, `encode`/`decode`, and
`wire.WireCalibBundle`. (The reactive `Module` / pub-sub `Step` classes are still
**defined** in the vendored comms — other processes use them — but `vio/modules/`
no longer imports them: its pipeline is plain procedural Python, see below.
`ModuleContext` is the one comms type still used, as a plain `(bus, name, state)`
state holder for the odometry worker.)

### What VIO owns now — execution glue, no solve engine

After the `sky.*` consolidation the VIO algorithm itself (frontend KLT, RGB-D
odometry, the IMU/SO(3) helpers) lives in the shared `sky` leaf library; the
misnamed grab-bag `vio/mathlib/` has been **dissolved by concern**. The heavy
keyframe solvers (windowed BA `WindowedBAMap`, tight VIO window `WindowedVIOMap`)
and the in-process engine that drove them now live in [`ba/`](../ba/README.md) —
`vio/engine/` was **deleted**. What VIO still owns is the *execution* glue around
the front-end:

- the **odometry worker** (`vio/modules/pipeline.py`) — KLT track → RGB-D PnP +
  gyro fusion + the live IMU dead-reckon, emitting `pose.odom` / `pose.vo` and the
  `keyframe` stream that `ba` and `slam` consume. There is **no keyframe solve in
  VIO** any more.
- the **`--ba-endpoint` pass-through** (`vio/main.py`) — a read-only client on the
  `ba` endpoint that re-emits `ba`'s `pose.refined` / `ba.window` on the VIO endpoint
  and feeds `ba.state` (the `--tight` optimised bias) into `propagate_imu`.

**ARCHITECTURE RULE.** The math-coupled config builders and the JIT warmup live at
the **project root**, **not** in the generic, bit-identical `vio/comms/`:

- `vio/resolution_build.py` — `frontend_config(res, *, numba)`,
  `odometry_config(res, **guards)`, `ba_huber_px(res)` (ported verbatim from the
  pre-split `ResolutionProfile.frontend` / `.odometry` / `.ba_huber_px`). They
  import VIO's own (now `sky`) math; the profile in `vio.comms.lib.config.resolution`
  stays data-only and headless.
- `vio/warmup.py` — `warmup_klt(klt_cfg=None)` warms **only** the KLT numba
  kernel. VIO consumes `frame.depth` from capture, so it does **not** run SGM (that
  is `imu_camera`, which warms its own SGM kernel).

### `vio/modules/` — the procedural pipeline (no reactive `Module` / `Step`)

The class-heavy Step/Module reactive framework was flattened to plain procedural
Python: every step is a function with explicit args (no `ctx.state` lookups), and
each reactive module became a plain `threading.Thread` worker that owns its inbox,
coalescing, END handling, and downstream-END forward explicitly.

The files are grouped by **role in the data flow** (the package `__init__.py` carries
the full module-map): `pipeline.py` (read first — the odometry worker that
orchestrates), `carriers.py` (the per-frame dataclass records), `frontend.py`
(sparse visual VO), `imu_prior.py` (IMU prior + gravity + tilt), `backend.py`
(keyframe **emission** — `emit_keyframe`, on the odometry thread; the windowed BA it
used to feed moved to the `ba` process), `publishers.py` (emit results on topics),
plus `propagate_imu.py` (the `--tight` live nav), `direct_odometry.py` (the
`--direct` alternative front-end), and `loop_inbox.py` (the SLAM `loop.correction`
**and** `ba.state` bias feedback inboxes). Flow: `frame → frontend → imu_prior →
backend → publishers`.

`OdometryWorker` joins `imucam.sample` (IMU prior, `process_imucam`) +
`frame.depth` (KLT track → RGB-D PnP → gyro fusion → pose, `process_frame`) and
publishes `pose.odom`, `keyframe`, `frame.tracks`, `frame.inliers` (+ `pose.vo`
when the live builder enables it). It owns the **2-input multi-END join**
explicitly: one inbox carries `(topic, msg)` tuples, the loop routes each by topic
to the right step chain, and END is forwarded downstream + `done` set only once
**both** inputs have ENDed (`expected_ends == 2`) — the load-bearing concurrency
the old `Module` gave for free. `OdometryModule` is kept as a public alias for the
worker (vio.main + the vio/verification selftests import it). The internal carriers
(`Step` / `Primed` / `Tracked`, in `carriers.py`) thread one frame's state through
the chain; they never go on the bus.

> **The backend worker is gone from VIO.** `BackendWorker` / `BackendModule` /
> `process_kf` / `run_ba` now live in [`ba.modules.pipeline`](../ba/README.md); `vio`
> no longer constructs a backend worker — it is a pure producer of `keyframe`.

The odometry worker holds a `ModuleContext` (a plain `(bus, name, state)` holder,
NOT the reactive substrate) so the per-run state the step functions thread through
(`vo` / `priors` / `imu_segs` / the live `live_nav` / `loop_inbox` …) lives in one
place — and the selftests that reach into `odom.ctx.state` keep working unchanged.

> Naming note: the carrier dataclass is named `Step` (`vio/modules/carriers.py`) — a
> real per-frame data record (`estimate_motion` → downstream), **kept as a
> dataclass**. The framework `Step` base class (the old per-step superclass) is
> gone from `vio/modules/` now that the steps are plain functions, so the old
> `Step` / `StepBase` import collision is gone too.

#### TIGHT live pose — `propagate_imu` (`--tight` only)

On `--tight` (`retain_imu=True`) the live `pose.odom` is **IMU forward-propagated**
between vision solves (Basalt-like `predictState`), so it reacts instantly to motion
and keeps moving through a covered camera / textureless wall instead of freezing. The
step (`vio/modules/propagate_imu.py`) owns a body→world nav-state `(R, p, v)` and on
every frame:

1. **Gap-free integration.** The retained per-frame IMU block is integrated forward
   under gravity (`imu.predict_state`). The previous block's last sample is prepended
   so the interval is exactly `(prev_block_last_ts, this_block_last_ts]` with **no
   dropped boundary segment** — a fast push registers at full magnitude (the naive
   per-block cut `(prev_ts, ts]` shares no sample and silently drops ~1-of-N
   inter-sample segments → only ~60 % of the displacement).
2. **Velocity-gated ZUPT.** A Zero-Velocity Update freezes translation only when
   *genuinely* at rest = accel ≈ g **and** gyro ≈ 0 **and** `|v|` small (sustained, with
   hysteresis). Accel+gyro alone cannot tell rest from a constant-velocity cruise (both
   read `|accel|≈g`, `|gyro|≈0`), so the velocity gate is what stops the old mid-push
   *pause*; at-rest drift is still held to ~0.
3. **Smooth complementary vision correction.** Every frame whose PnP solve is valid
   (`step.info["ok"]` + enough inliers), the nav-state is nudged a **bounded fraction**
   toward the fresh vision pose (`imu.complementary_correct`: position + velocity +
   attitude error-state feedback), replacing the old hard `p = p_vis` re-anchor +
   `v = displacement/dt` injection — so vision pulls the drift back *continuously* with
   no snap and no overshoot. On a failed/covered frame the correction is skipped and the
   pose pure-dead-reckons.

LOOSE path (`retain_imu=False`) is a **pure pass-through no-op** — `pose.odom` stays the
vision-only odometry pose, so the byte-parity oracle is untouched. Gates:
`vio.tests.imu_push_response_selftest` (the push-profile gate), `imu_propagate_selftest`,
`tight_live_pose_selftest`.

#### DIRECT odometry mode — `process_frame_direct` (`--direct` only)

`--direct` selects a **third** odometry mode (alongside loose-default and `--tight`):
dense **direct** RGB-D visual odometry, for the 54×42 VL53-class ToF target where the
sparse corner/KLT front-end scale-collapses (Sim3 scale 0.23–0.63) from feature
starvation. It is opt-in and **byte-identical-off**: with no flag the loose/tight
path is unchanged and the byte-parity oracle stays gap=0 (the oracle never passes
`--direct` and runs its own in-process harness, not this worker).

On `--direct` the `frame.depth` edge routes to `process_frame_direct` (not the sparse
`process_frame`), which drives `DirectOdometryEngine` (`vio/modules/direct_odometry.py`)
— the live port of the offline-proven loop in `verification/direct_vo_bench.py`. Per
frame:

1. **Dense direct frame-to-keyframe alignment.** `sky.front.direct.estimate_pose_direct`
   (the LEAF estimator, reused verbatim) aligns every gradient pixel by photometric
   Gauss-Newton against the current keyframe, reading metric scale straight from the
   accurate per-pixel ToF depth (geometric point-to-plane term OFF by default — the
   ablation showed it redundant at 54×42; available via `DirectConfig.geo_weight`).
2. **Live IMU 6-DoF seed (reused, not rebuilt).** The GN `init_T` is the keyframe→cur
   relative pose from an IMU dead-reckon nav-state propagated with the SAME
   `sky.vio.imu.predict_state` the live tight path runs, gravity-levelled once with
   `gravity_aligned_R0` (seeded from the **bundle's** `accel_align` startup reference —
   the per-frame prior is empty on the no-IMU startup frames), and pulled toward each
   accepted fix with `complementary_correct` (same gains as the tight path). The
   per-frame raw-IMU block comes from the SAME `preintegrate_prior` retention the tight
   path uses (`--direct` forces it on, independent of `retain_imu`).
3. **Divergence guard.** A frame's VO pose is rejected — replaced by the IMU
   dead-reckon, which is also what the dead-reckoner is then corrected toward (so the
   seed velocity is not poisoned) — when the estimator flags `diverged` OR the VO
   keyframe-relative step ≫ the IMU-predicted step (ratio gate with a floor). This is
   the lever that kills the fast-motion divergence.

Keyframes are emitted on a **natural** cadence (trans ≥ 0.1 m / rot ≥ 6° / overlap
drop / divergence), not the fixed `kf_every` count. The published topics are the SAME
as the other modes (`pose.odom`, `pose.vo`, `keyframe`, and empty-but-ticking
`frame.tracks` / `frame.inliers`), so the UI + SLAM + comms are untouched (no new IPC
topic). `--direct` is independent of `--tight` and is meant to pair with `--vl53l9cx`:
the live recipe is `./run.sh --vl53l9cx --direct`. Gate: `vio.tests.direct_smoke_selftest`
(live worker smoke + Sim3-scale sanity vs Basalt); launcher forwarding:
`launcher.tests.direct_forward_selftest`.

### `vio/main.py` — the VIO process

Two-client startup against the capture endpoint (a **calib client** that blocks on
the retained `calib.bundle`, then a **data client** for `imucam.sample` +
`frame.depth`), builds the local **odometry** graph with the **live** config
(`level_tilt=True`, `OdometryConfig(gyro_fuse=use_gyro)`, `publish_vo=True`), and
mirrors its outputs onto its own `IPCPubSub` server with an `IPCPublisher`,
re-broadcasting the retained `calib.bundle` as a readiness barrier. There is **no
keyframe solve in the VIO process** — VIO emits `keyframe`; the windowed BA runs in
the `ba` process. Same SIGTERM / drain / `os._exit` lifecycle as the template.

**Backend pass-through (`--ba-endpoint`).** The `ba` process publishes `pose.refined`
(the windowed-BA line) on ITS OWN endpoint. To keep the UI + netbridge UNCHANGED (no
4th UI endpoint), VIO — when given `--ba-endpoint` — opens a **read-only** client on
the `ba` endpoint and bridges two topics back onto its local bus (the mirror of the
slam `loop.correction` re-hydrate):

- `pose.refined` (and, under `--ba-window`, `ba.window`) is re-published so VIO's
  existing `IPCPublisher` re-emits it on the **VIO** endpoint — the UI's trajectory +
  "BA Window" sources read it off the single VIO endpoint, unchanged across the split.
- `ba.state` (the `--tight` optimised-bias feed-forward) is handed to `propagate_imu`
  via `vio.modules.loop_inbox.BackendStateInbox`. It is consumed VIO-local only and
  is **never** re-emitted on the VIO endpoint (off-bridge).

When `--ba-endpoint` is unset (the launcher omits it under `--no-ba`, which spawns no
`ba` process), VIO simply never opens the client — no `pose.refined`, inert bias feed.

## Run

```bash
# capture (replay) serves oak.capture; vio subscribes + serves oak.vio; ba
# consumes vio's keyframe + publishes pose.refined; vio re-emits it via --ba-endpoint.
python -m imu_camera.main --session sessions/gold/lab_loop_30s &
python -m vio.main --capture-endpoint oak.capture --endpoint oak.vio \
                   --ba-endpoint oak.ba &
python -m ba.main  --vio-endpoint oak.vio --endpoint oak.ba
```

> `vio` alone (no `ba`, no `--ba-endpoint`) still runs fully — it just emits no
> `pose.refined` (that line now comes from `ba`). The whole stack is normally
> launched by `launcher.main` / `run.sh`, which spawns `ba` between `vio` and `slam`.

## Verify

```bash
cd /Users/bao/skydev/flight-vio

# 1. comms byte-identical (build caches excluded; they are git-ignored)
diff -r -x '__pycache__' -x '*.pyc' -x '*.nbc' -x '*.nbi' \
     vio/comms imu_camera/comms && echo "COMMS BYTE-IDENTICAL"

# 2. import smoke
.venv/bin/python -c "import vio.main, vio.modules.pipeline; print('VIO IMPORT OK')"

# 3. math byte-parity vs the oracle + KLT correctness
.venv/bin/python -m vio.tests.vio_ba_selftest      # PASS (== ours numbers)
.venv/bin/python -m vio.tests.odometry_selftest    # PASS

# 4. PAIR smoke: capture (replay) + vio over IPC on a gold session
#    expect ~60 pose.odom (dense), ~12 keyframe, clean exit. (pose.refined now
#    comes from the ba process — add `ba.main` + `--ba-endpoint` to see it.)
```
