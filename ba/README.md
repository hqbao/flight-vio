# `ba/` — the windowed bundle-adjustment project

The windowed BA, extracted out of `vio` into its **own process** (ADR
[0001](../docs/adr/0001-extract-windowed-ba-into-ba-process.md), commit `611dc45`),
built by replicating the **proven `imu_camera` / `vio` / `slam` template**. `ba`
subscribes to the VIO process over IPC, runs the sliding-window bundle adjustment
over the `keyframe` stream, and republishes its refined pose on its own IPC endpoint
for the UI / VIO.

```
imu_camera.main ─(oak.capture)─▶ vio.main ─(oak.vio)─▶ ba.main ─(oak.ba)─▶ vio (re-emit) / ui
   capture proc       IPC         VIO proc     IPC       BA proc     IPC
```

It owns the **windowed-BA map** (a sliding window of keyframe poses + landmarks).
The SLAM map (ORB index + pose graph) lives in the [`slam`](../slam/README.md)
process; the two maps are **independent by design** — they consume different things
and serve different views. `ba` is a pure **consumer** of VIO's keyframe output:
`emit_keyframe` stays in `vio` (it rides VIO's odometry thread); `ba` only ingests
the resulting `keyframe`.

Goal of the split = **architectural cleanliness** (fault-isolate the bursty
~48 ms/keyframe solve from the live odometry; keep the `libsky*` port boundary
clean), **not** performance — the pre-split in-VIO backend already ran the solve
GIL-free behind an opt-in worker-child engine. Now that `ba` is its own process, it
runs the solve **in-process** and the worker-child engine is gone (see
[`ba/engine/`](#baengine--the-in-process-solve-runner) below).

The LOOSE default path is **byte-identical** to the in-VIO backend it replaced — it
runs the SAME frozen `run_ba` solve — so the byte-parity oracle (`gap = 0`) is
unchanged.

## Layers

| Package | Role | Source it was ported from |
|---------|------|---------------------------|
| `ba/comms/` | the **FROZEN** vendored comms contract | copied **bit-identically** from `imu_camera/comms` |
| `ba/engine/` | the in-process runner for the heavy keyframe solve (no worker child); the BA / tight-VIO math comes from `sky.backend` / `sky.vio` | `ours/lib/engine` (it is now the SINGLE home — `vio/engine/` was deleted) |
| `ba/modules/` | the windowed-BA pipeline (**procedural** functions + a plain worker thread) | the back-end half of `vio.modules.pipeline` |
| `ba/main.py` | the BA process | new (mirrors `slam/main.py`) |
| `ba/tests/` | functional self-test | new |

### `ba/comms/` — byte-identical, do not hand-edit

`ba/comms` is **copied bit-identically** from `imu_camera/comms`. The
`verification.ipc_comms_selftest` gate dir-diffs it against the anchor (`diff -r
--exclude=__pycache__ ba/comms imu_camera/comms` must be empty; build caches —
`__pycache__`, `*.pyc`, `*.nbc/.nbi` — are git-ignored). All its internal imports
are RELATIVE, so the copy works as `ba.comms` unchanged. **Never hand-edit it** —
change `imu_camera/comms` and re-vendor.

Public API the BA process uses: `LocalPubSub`, `IPCPubSub(role="server"|"client")`,
`IPCPublisher`, `IPCSubscriber`, `RingRegistry`, `topics`, and `wire.WireCalibBundle`.
The `keyframe`, `pose.refined`, `ba.state`, `ba.window` topics and the `Keyframe` /
`PoseMsg` / `BackendState` / `BaWindow` messages already live in the shared comms
contract. (The reactive `Module` / `Step` classes are still **defined** in the
vendored comms — other processes use them — but `ba/` no longer imports them: its
pipeline is plain procedural Python, see below.)

### `ba/engine/` — the in-process solve runner; the math lives in `sky.*`

The windowed-BA + tight-VIO algorithms are in the shared `sky/` leaf library; `ba`
owns only the *execution* glue:

- `ba/engine/` — the in-process engine that drives the heavy keyframe solve.
  `make_ba_engine` wraps `sky.backend.windowed.WindowedBAMap` (loose, vision-only)
  and `make_vi_engine` wraps `sky.vio.window.WindowedVIOMap` (tight, joint visual +
  IMU window). **`ba` is one process that runs its solve IN-PROCESS** — `submit`
  does the whole solve synchronously on the worker thread, `poll` returns its one
  result, byte-identical to the old in-VIO in-process path. **There is no
  worker-child engine** (the pre-split opt-in worker child existed only to free the
  camera read loop's GIL — a motivation that is gone now that BA has its own
  process). The engine knows nothing about the bus — pure machinery called by the
  module steps.

This is now the **SINGLE home** of the engine runner: it was a verbatim copy of the
old `vio.engine`, and `vio/engine/` was **deleted** once the backend moved here.

### `ba/modules/` — the procedural pipeline (no reactive `Module` / `Step`)

The back-end half of the old in-VIO `vio.modules.pipeline` pair, lifted verbatim so
the refined-pose output (and the offline byte-parity argument that depends on the
SAME frozen solve) is unchanged — only the package the comms / engine come from
moved (`vio` → `ba`). The files:

- `backend.py` — `run_ba(engine, tight, kf)`: submit the keyframe's snapshot to the
  engine and return `(refined PoseMsg, backend_state)` (or `None` → chain
  short-circuit). LOOSE submits the 5-tuple (`T_cw`, track ids/px, depth, at-rest
  gravity accel); TIGHT submits the SUPERSET 6-tuple (+ keyframe `ts_ns` + the raw
  inter-keyframe IMU block for preintegration).
- `publishers.py` — the two backend taps: `publish_refined` → `pose.refined`
  (terminal), `publish_ba_window` → `ba.window` (opt-in `--ba-window`).
- `pipeline.py` — the orchestration: `process_kf` per keyframe + the worker thread
  `BackendWorker` (legacy alias `BackendModule`). **THE ENTRY POINT.** `tight=True`
  is a clean engine switch (`make_vi_engine` instead of `make_ba_engine`), NOT a
  pipeline fork. Single `keyframe` input → strict FIFO (`latest_only=False`): every
  keyframe must be solved in order so the refined output matches the in-VIO path. The
  first END is terminal (no join); END is forwarded to `pose.refined` (+ `ba.window`
  when the capture engine is built).

#### TIGHT feed-forward — `ba.state` (the one behavioural change vs the in-VIO original)

The in-VIO backend republished its optimised bias on the **intra-process** local-bus
`backend.state` topic for the same process's `propagate_imu`. Across the split that
hop becomes an **IPC** one: under `--tight`, `process_kf` publishes the optimised
`(bg, ba)` on the new IPC POD topic **`ba.state`** (`BackendState`), and `vio` opens
a read-only client on the `ba` endpoint to drain it into `propagate_imu` (the IPC
analog of slam's `loop.correction` channel; the carried `seq` survives the wire so
the consumer's staleness gate makes the async hop tolerable). On the loose / oracle
path `ba.state` is read by nothing → `pose.refined` byte-parity (`gap = 0`) is
unaffected.

#### BA-window visualiser — `ba.window` (opt-in `--ba-window`, loose-only)

Under `--ba-window` the LOOSE backend's capture-aware engine snapshots each solve
(window keyframe poses + 3D landmarks + observation rays + reprojection error) and
publishes it on the IPC POD topic **`ba.window`** for the UI's "BA Window" view. VIO
bridges it back (alongside `pose.refined`) and re-emits it on the VIO endpoint, so
the UI reads it from the single VIO endpoint — unchanged across the split. LOOSE-only
(`--tight` overrides it: the tight map has no capture overlay), so `ba.window` is
published ONLY when `--ba-window and not --tight`; a consumer never waits on a topic
that will never emit. Oracle-safe: the capture engine runs the SAME frozen `run_ba`
solve, default OFF.

### `ba/main.py` — the BA process

A single-client startup against the **VIO** endpoint: a **calib client** blocks on
the retained `calib.bundle` (VIO re-broadcasts it after allocating its `kf_*` rings,
so its arrival proves VIO is up, intrinsics are known, and the keyframe rings exist),
then `ba` attaches to VIO's `kf_gray` / `kf_depth` rings, builds the `BackendWorker`,
and mirrors `pose.refined` (+ `ba.state` under `--tight`, + `ba.window` under
`--ba-window`) onto its own `IPCPubSub` server with an `IPCPublisher`, re-broadcasting
the retained `calib.bundle` as a readiness barrier. The windowed solve runs
**in-process** on the `BackendWorker` thread — its IPC recv is a separate thread
feeding the FIFO inbox. (`ba` deliberately does **not** subscribe to capture at all —
it is a pure consumer of VIO's output.) Same SIGTERM / drain / `os._exit` lifecycle
as the template.

**CLI** (the backend knobs that used to be in-VIO route here):

| Flag | Effect |
|---|---|
| `--vio-endpoint` / `--endpoint` | the VIO endpoint to read (default `oak.vio`) / this process's endpoint (default `oak.ba`) |
| `--tight` | select the TIGHT-coupled VIO backend (`WindowedVIOMap`, `imu_info_weight=True`) instead of the default LOOSE windowed BA, and publish the `ba.state` feed-forward bias for VIO. Opt-in; the loose default is byte-identical to the in-VIO backend. |
| `--backend-window` | LOOSE sliding-window size in keyframes (default 6); inert on the tight path (which uses `WindowedVIOConfig` defaults). |
| `--backend-iters` | LOOSE max Gauss-Newton iterations per solve (default 5); inert on the tight path. |
| `--stabilize-velocity` | **tight only** Phase-4 velocity regularisation (CV prior + gated ZUPT). Opt-in; ignored on the loose path. |
| `--depth-icp` | **tight only** Phase-4 dense-ICP relative-pose factor. Opt-in; ignored on the loose path. |
| `--ba-window` | publish `ba.window` solve snapshots for the UI's BA Window. LOOSE-only — ignored under `--tight`; oracle byte-identical. |
| `--calib-timeout` | seconds to wait for the `calib.bundle` on boot (default 30). |

> **Spawn gate (`--no-ba`).** The launcher's `--no-ba` is a **spawn** gate (mirror of
> `--no-slam`): it simply skips spawning the `ba` process — VIO then gets no
> `--ba-endpoint`, so it never wires the pass-through (no `pose.refined`, inert bias
> feed). For a one-way flight that never revisits, pair it with `--no-slam`.

## Run

```bash
# capture (replay) serves oak.capture; vio subscribes oak.capture + serves oak.vio;
# ba subscribes oak.vio (keyframe) + serves oak.ba; vio re-emits ba.pose.refined.
python -m imu_camera.main --session sessions/gold/lab_loop_30s &
python -m vio.main  --capture-endpoint oak.capture --endpoint oak.vio \
                    --ba-endpoint oak.ba &
python -m ba.main   --vio-endpoint oak.vio --endpoint oak.ba
```

The whole stack is normally launched by `launcher.main` / `run.sh`, which spawns
`ba` between `vio` and `slam`.

## Verify

```bash
cd /Users/bao/skydev/flight-vio

# 1. comms byte-identical (build caches excluded; they are git-ignored)
diff -r --exclude=__pycache__ ba/comms imu_camera/comms && echo "COMMS BYTE-IDENTICAL"

# 2. import smoke
.venv/bin/python -c "import ba.main, ba.modules.pipeline, ba.engine; print('BA IMPORT OK')"

# 3. FUNCTIONAL: capture (replay) + vio + ba over IPC on a gold session — assert the
#    ba process actually publishes pose.refined (finite T_world_cam, info['refined']).
.venv/bin/python -m ba.tests.ba_refined_functional_selftest
.venv/bin/python -m ba.tests.ba_refined_functional_selftest --tight   # tight backend

# 4. launcher forwarding of the backend knobs into ba.main:
.venv/bin/python -m launcher.tests.ba_window_forward_selftest
.venv/bin/python -m launcher.tests.stabilize_velocity_forward_selftest
.venv/bin/python -m launcher.tests.depth_icp_forward_selftest
.venv/bin/python -m launcher.tests.no_ba_no_slam_forward_selftest
```

> The deterministic **byte-parity** of the windowed solve itself is proven by the
> in-process oracle, which drives the `sky.*` solve directly (no `ba` process) — see
> [`verification/`](../verification/README.md) and `vio.tests.vio_ba_selftest`.
