# `slam/` — the loop-closure SLAM project (Phase 4 of the split)

The **fourth** of the five split projects (`imu_camera`, `depth`, `vio`, `slam`,
`ui`), built by replicating the **proven `imu_camera` / `vio` template**. `slam`
subscribes to the VIO process over IPC, runs ORB loop closure + SE(3) pose-graph
optimisation over the keyframe stream, and republishes its results on its own IPC
endpoint for the UI / tools.

```
imu_camera.main ──(oak.capture)──▶ vio.main ──(oak.vio)──▶ slam.main ──(oak.slam)──▶ ui / tools
   capture proc        IPC          VIO proc      IPC         SLAM proc      IPC
```

It owns the **SLAM map** (ORB feature index + pose graph). The VIO map (windowed
BA) lives in the VIO process; the two maps are **independent by design**. The
correction stream is **one-way**: SLAM publishes `loop.correction` for the UI but
**never closes the loop back into VIO** — behaviour unchanged from the pre-split
`ours.proc.slam`.

It was ported **VERBATIM** from the reference oracle (`ours/`): only import roots
were re-rooted and Flow/Task/Bus classes were renamed (Flow → Module, Task → Step,
Bus → LocalPubSub, Ipc*Bus/Flow → IPCPubSub/IPCPublisher/IPCSubscriber). **No
algorithm changed**, so the numerical output is byte-identical to the oracle —
proved by `slam.tests.loop_closure_selftest` (its numbers match
`ours/tools/posegraph_selftest.py` line-for-line) and by the 3-process smoke
matching the oracle loop count.

## Layers

| Package | Role | Source it was ported from |
|---------|------|---------------------------|
| `slam/comms/` | the **FROZEN** vendored comms contract | copied **bit-identically** from `imu_camera/comms` |
| `slam/mathlib/` | the math SLAM owns (loop + engine) + its FORCED deps | `ours/lib/{loop,engine}` (+ forced `odometry/pnp`, `imu/imu`, `backend/bundle`) |
| `slam/modules/` | the loop-closure reactive module | `ours/flows/slam` |
| `slam/main.py` | the SLAM process | `ours/proc/slam.py` |
| `slam/tests/` | regression self-tests | `ours/tools/posegraph_selftest.py` |

### `slam/comms/` — byte-identical, do not hand-edit

`slam/comms` is **copied bit-identically** from `imu_camera/comms`. A gate runs
`diff -r slam/comms imu_camera/comms` and it must be empty (build caches —
`__pycache__`, `*.pyc`, `*.nbc/.nbi` — are git-ignored and excluded). All its
internal imports are RELATIVE, so the copy works as `slam.comms` unchanged. **Never
hand-edit it** — change `imu_camera/comms` and re-vendor.

Public API the SLAM process uses: `LocalPubSub`, `IPCPubSub(role="server"|"client")`,
`IPCPublisher`, `IPCSubscriber`, `Module`, `Step`, `RingRegistry`, `topics`, and
`wire.WireCalibBundle`. The `keyframe`, `loop.correction`, `slam.map` topics and
the `Keyframe` / `LoopCorrection` / `SlamOverlay` messages already live in the
shared comms contract.

### `slam/mathlib/` — the math SLAM owns + the FORCED-vendor deps

The math SLAM genuinely owns is the verbatim port of `ours/lib/loop` + a private
copy of `ours/lib/engine`:

- `slam/mathlib/loop/` — `orb` (from-scratch oriented-FAST + rotated-BRIEF +
  Hamming matcher + fundamental-matrix RANSAC, **no cv2**), `loopclosure`
  (appearance gate + geometric verification → metric `T_cur_old`), `posegraph`
  (SE(3) Gauss-Newton/LM PGO with a Huber kernel on loop edges), and `slam`
  (`SlamMap` / `SlamConfig` — the persistent-keyframe orchestrator).
- `slam/mathlib/engine/` — SLAM's **own** copy of the swappable in-process /
  subprocess runners (byte-copied; `worker=False` is byte-identical offline,
  `worker=True` runs the solve in a child process so it never holds the read
  loop's GIL).

**FORCED-vendor dependencies** (resolved from the loop-closure import graph;
vendored at the **minimal** surface, mirroring how `vio` had to vendor `imu`):

| Vendored | Why it is forced | Importer |
|----------|------------------|----------|
| `slam/mathlib/odometry/pnp.py` | `solve_pnp_ransac` is the metric verifier of every loop | `loop/loopclosure.py` → `..odometry.pnp` |
| `slam/mathlib/imu/imu.py` | `so3_exp` (SO(3) helper); numpy-only, self-contained | `odometry/pnp.py` → `..imu.imu` (transitive) |
| `slam/mathlib/backend/bundle.py` | `se3_exp` + `skew` (SE(3)/Lie helpers) drive the PGO | `loop/posegraph.py` → `..backend.bundle` |

Nothing else is vendored. In particular the **windowed BA** is *not* vendored:
the byte-copied engine carries a `make_ba_engine` / `_ba_worker_main` path with a
**lazy** `..backend.windowed` import, but SLAM only ever calls `make_slam_engine`,
so that BA path never fires — the exact mirror of how `vio`'s byte-copied engine
carries a never-fired lazy `..loop.slam` import. The relative import layout under
`mathlib` is preserved, so every `from ..odometry.pnp` / `..imu.imu` /
`..backend.bundle` / `..loop.slam` resolves unchanged.

**ARCHITECTURE RULE.** The math-coupled config builder lives in `slam/mathlib/`,
**not** in the generic, bit-identical `slam/comms/`:

- `slam/mathlib/resolution_build.py` — `loop_config(res)` (ported verbatim from
  the pre-split `ResolutionProfile.loop`), which imports SLAM's own
  `loop.loopclosure.LoopConfig`. The profile in `slam.comms.lib.config.resolution`
  stays data-only and headless.

> **No `warmup.py`.** Unlike `vio` (which warms its KLT numba kernel), SLAM has
> **no numba JIT** to pre-compile — its ORB frontend is pure NumPy — so no warmup
> module exists.

### `slam/modules/` — the reactive pipeline (Flow → Module, Task → Step)

`SlamModule` subscribes `keyframe` and publishes `loop.correction`. It wraps
`SlamMap` behind a swappable engine; every keyframe is submitted (the map's own
motion gate may skip redundant ones), and on a confirmed loop the pose graph is
optimised and the rewritten keyframe poses are published as a correction.

The single-purpose steps each own one responsibility: `SlamStep` (submit + poll
the engine → `LoopCorrection` on a loop), `PublishCorrection` (emit on
`loop.correction`), `PublishSlamMap` (poll the cheap overlay → `slam.map`).

Two key behaviours are preserved verbatim from the oracle:

- **`publish_map` flag** (LIVE-only, defaults `False`). When on, SlamModule emits
  a continuous `slam.map` overlay so the UI draws keyframe dots **every** keyframe
  instead of only after a loop closes. The offline path (flag off) is byte-identical.
- **`_RunCorrectionChain`** — `Module.on` keeps **one** step list per topic, and
  `SlamStep` returns `None` on every non-loop keyframe (which short-circuits the
  chain). So the live path wraps `[SlamStep(), PublishCorrection()]` in one step
  that always returns the keyframe, letting the outer chain continue to
  `PublishSlamMap` (which polls the overlay **after** the submit). One combined
  chain, correct order, zero impact on the `loop.correction` semantics.

### `slam/main.py` — the SLAM process

A single-client startup against the **VIO** endpoint: a **calib client** blocks on
the retained `calib.bundle` (VIO re-broadcasts it after allocating its `kf_*`
rings, so its arrival proves VIO is up, intrinsics are known, and the keyframe
rings exist), then SLAM attaches to VIO's keyframe rings and builds the local
graph with the **live** config:
`SlamConfig(loop_max_odom_rot_deg=30.0, kf_min_trans_m=0.1, kf_min_rot_deg=5.0)`,
`latest_only=True`, `publish_map=True`. It mirrors `loop.correction` + `slam.map`
onto its own `IPCPubSub` server with an `IPCPublisher`, re-broadcasting the
retained `calib.bundle` as a readiness barrier. The worker-engine subprocess
boundary (`--worker`) stays on stdlib pickle (`multiprocessing.Queue`,
same-project classes) — it is **not** routed through the cross-process codec. Same
SIGTERM / drain / `os._exit` lifecycle as the template.

## Run

```bash
# capture (replay) serves oak.capture; vio subscribes + serves oak.vio;
# slam subscribes oak.vio + serves oak.slam.
python -m imu_camera.main --session sessions/gold/lab_loop_30s &
python -m vio.main  --capture-endpoint oak.capture --endpoint oak.vio &
python -m slam.main --vio-endpoint oak.vio --endpoint oak.slam
```

## Verify

```bash
cd /Users/bao/skydev/oak-d

# 1. comms byte-identical (build caches excluded; they are git-ignored)
diff -r --exclude=__pycache__ slam/comms imu_camera/comms && echo "COMMS BYTE-IDENTICAL"

# 2. import smoke
.venv/bin/python -c "import slam.main, slam.modules.pipeline; print('SLAM IMPORT OK')"

# 3. math byte-parity vs the oracle (== ours/tools/posegraph_selftest.py numbers)
.venv/bin/python -m slam.tests.loop_closure_selftest

# 4. 3-PROC smoke: imu_camera (replay) + vio + slam over a gold loop session.
#    Asserts all 3 procs rc=0, slam.map advances (kf dots), and loop.correction
#    n_loops matches the oracle (4 on lab_loop_30s).
.venv/bin/python -m slam.tests.proc3_smoke_selftest \
    --session sessions/gold/lab_loop_30s --expect-loops 4
```
