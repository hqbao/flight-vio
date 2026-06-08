# 4-Process Live Architecture

> **Status:** design (decided 2026-06-07 by user). Replaces the single-process
> live path under `ours.app.build_live` with four cooperating processes that
> communicate over a stdlib-only IPC layer. The OFFLINE / replay path
> (`ours.app.run_replay` + `flow_replay_selftest`) keeps the single-process
> codepath unchanged — determinism + byte-identical output depend on it.

## 1. Motivation

The single-process live graph already worked (out-of-process *engine* for the
heavy BA/SLAM solve, in-process Bus for everything else), but it has three
limits we want gone:

1. **One blow-up kills everything.** A Qt UI crash or a SLAM solver wedge
   takes the whole pipeline down (incl. the device pipeline, which then trips
   the OAK-D firmware watchdog and forces a USB replug).
2. **VIO and SLAM share a Python interpreter** → still pay for incidental GIL
   sharing whenever a Python step in one of them runs. `SubprocessEngine` only
   moves the inner solve out — the flow tasks (`RunBA`, `SlamStep`, the bus
   forwarders) still run in the main interpreter.
3. **Calibration / visualisation tools steal the device.** Today every wizard
   first stops the VIO source (the OAK-D is single-client) and reopens its own
   depthai pipeline. With a dedicated **capture** process that owns the device
   forever, the wizards subscribe to its stream and never touch the link.

## 2. Process layout (the decisions)

Four long-lived processes, plus transient tool processes that come and go:

| Process    | Owns                                  | Subscribes (IPC)               | Publishes (IPC) |
|---         |---                                    |---                             |---|
| `capture`  | OAK-D device + `CamFlow` + `ImuCamFlow` | —                            | `cam.sync`, `imu.raw`, `imucam.sample`, `frame.depth`, `calib.bundle` |
| `vio`      | `OdometryFlow` + `BackendFlow`         | `imucam.sample`, `frame.depth`, `calib.bundle` | `pose.odom`, `pose.vo` (pure-vision, LIVE-only), `keyframe`, `frame.tracks`, `frame.inliers`, `pose.refined`, `vio.map` |
| `slam`     | `SlamFlow` (loop closure + pose graph) | `keyframe`, `calib.bundle`     | `loop.correction` (loop-event pose-graph rewrite) **and** `slam.map` (continuous keyframe overlay, LIVE-only) |
| `ui`       | Qt `MainWindow`, single 5-trajectory viewer3D + View/Visualize/Calibration menus | `pose.odom`, `pose.vo`, `pose.refined`, `calib.bundle` (vio); `slam.map`, `calib.bundle` (slam); on-demand: `imucam.sample`, `frame.depth`, `imu.raw` (capture) + `frame.tracks`, `frame.inliers` (vio) | — (sink) |

The UI's Visualize / Calibration windows are no longer separate transient
processes: they run **in the UI process** as plain child `QMainWindow` / modal
dialogs, fed over IPC by the adapters in `ours/proc/ui_ipc_sources.py` (see §6).
Each adapter opens its own read-only `IpcClientBus` subscription on demand (when
the menu action fires) and tears it down when the window/dialog closes — exactly
the same read-only subscription pattern the long-lived trajectory sources use:

| In-UI window / dialog         | Adapter (`ui_ipc_sources.py`) | Subscribes (endpoint · topics) |
|---                            |---                            |---|
| Gyro / Accel calibration      | `IpcImuRawSource`             | capture · `imu.raw` (RAW IMU) |
| Camera + Depth + IMU triplet  | `IpcTripletWorker`           | capture · `imucam.sample`, `frame.depth` |
| Keypoint Depth Tracker        | `IpcKeypointWorker`          | capture · `frame.depth`  +  vio · `frame.tracks`, `frame.inliers` |

The crucial design rule: **nothing but `capture` opens the OAK-D**. The UI
windows/dialogs open a read-only subscription to the capture (and VIO) IpcBus,
exactly the way the UI's trajectory sources do. Nobody fights `capture` for the device,
and the UI process imports no depthai — it is device-agnostic by contract.

### 2.1 Decisions captured (asked + answered 2026-06-07)

| Question | Decision |
|---|---|
| Who owns the device? | Dedicated `capture` process (4 procs total). |
| What is "VIO's own map"? | VIO = frame-to-frame PnP + windowed BA (`BackendFlow`). |
| IPC mechanism? | `multiprocessing.Queue` for metadata + `shared_memory` ring for images. |
| UI display modes? | A SINGLE `Viewer3D` (the former VIO / SLAM tabs were collapsed) drawing 5 toggleable trajectory lines: VO / VIO / VIO-BA / SLAM-corrected VIO / SLAM. |
| Calib / visualise tools? | Subscribe to capture's stream via IPC; don't open the device. |
| Offline replay? | **Stays single-process** — determinism + byte-identical output. |

## 3. IPC layer — `ours/lib/ipc/`

A thin substrate that exposes the same `publish(topic, msg)` / `subscribe(topic, handler)`
API as the in-process `Bus`, but works across processes. Everything is **stdlib only**
(`multiprocessing`, `multiprocessing.connection`, `multiprocessing.shared_memory`,
`pickle`, `struct`). The existing flows do not change; they still use the
in-process `Bus` inside their own process — a tiny **bridge flow** wires the
in-proc Bus to the IpcBus at the process boundary.

### 3.1 `ours/lib/ipc/shared_array.py` — `SharedArrayRing`

A fixed-shape, fixed-dtype ring of `N` slots for one stream (e.g. one ring for
`gray_left`, one for `depth_m`). The ring is backed by **ONE `SharedMemory`
segment** named exactly `{name}` (no per-slot `.{i}` suffix), sized
`N * nbytes`; slot `i` is the byte-offset window `[i*nbytes : (i+1)*nbytes]`
(a numpy offset-view). The producer rotates `slot = seq % N`; consumers read by
slot index out of metadata. Subscribers who need to hold the array beyond one
frame must copy it (cheap: a single numpy copy of one frame is ~0.1 ms for
640×400).

- The single-segment layout keeps the open-file-descriptor cost a **small
  constant per ring, independent of `N`** (CPython's `SharedMemory(create=True)`
  holds ~2 fds per segment on macOS). The earlier design used one segment per
  slot, so fd cost scaled linearly with slots — a capture process attaching 3
  rings × 64 slots tripped macOS's 256-fd default (`shm_open` → EMFILE,
  `OSError: [Errno 24] Too many open files`) at boot. Total RAM is identical
  either way.

- `N` is sized so a moderate consumer backlog can never wrap around (default
  `N=8` at 20 fps = 0.4 s of slack — well above the 50 ms / 60 ms latest-only
  inbox cadence used downstream).
- No locks: rotation is single-producer single-cursor. The consumer's
  responsibility is to read fast or copy out. Worst case: a stuck consumer
  reads stale frames, but the live latest-only sinks already drop those.

### 3.2 `ours/lib/ipc/bus.py` — `IpcBus`

Pub/sub over `multiprocessing.connection.Listener` (Unix-domain socket on
macOS/Linux). One central socket per *publisher process*. Every subscriber
connects with a `SUBSCRIBE([topic, ...])` handshake; the publisher then
forwards each `publish` to all matching connections via `Connection.send`
(pickle under the hood). Big numpy arrays are **never** pickled — they ride
the `SharedArrayRing` and the wire-message carries only `(slot, shape, dtype, ts)`.

The API mirrors `ours.lib.flow.pubsub.Bus`:

```python
bus = IpcServerBus(endpoint="ipc.capture")          # publisher side
bus.publish("imucam.sample", IpcImuCamRef(seq, ts, slot_gray, slot_right, imu_ts, gyro, accel))
```

```python
bus = IpcClientBus(endpoint="ipc.capture")          # subscriber side
bus.subscribe("imucam.sample", lambda ref: ...)
bus.start()
```

### 3.3 `ours/lib/ipc/messages.py` — wire messages

For every existing message type that crosses a process boundary, a sibling
wire-message exists carrying only:

- POD fields (`seq`, `ts_ns`, ids, etc.)
- `SharedArrayRef(ring_name, slot, shape, dtype)` for every large array.

The receiving bridge flow re-hydrates by copying from shared memory back into
a regular `np.ndarray` and constructing the in-proc dataclass (`ImuCamPacket`,
`DepthFrame`, `Keyframe`, ...). The wire messages live next to the in-proc
ones so the contract is visible in one place.

## 4. Bridge flows — `ours/flows/bridge/`

The bridge keeps the existing flows unchanged. Each side has one tiny flow:

- **`IpcPublisherFlow`** — subscribes to N in-proc topics, copies the payload
  into a shared-memory slot (if it has arrays), wraps it in the matching wire
  message, and `IpcServerBus.publish`es it. One per process boundary.
- **`IpcSubscriberFlow`** — subscribes (via `IpcClientBus`) to topics on a
  remote publisher, re-hydrates wire messages into in-proc dataclasses, and
  publishes them on the local in-proc Bus. Other flows in this process
  consume from the in-proc Bus exactly as before.

The whole IPC layer is therefore invisible to `OdometryFlow`, `BackendFlow`,
`SlamFlow`, the UI sinks, and every existing self-test.

## 5. Process entry points — `ours/proc/`

One module per process, each exposes a `main()` so it can be spawned as a
standalone Python process.

### 5.1 `ours/proc/capture.py`

```
LocalBus
  ├── CamFlow (LiveCamSource or ReplayCamSource)
  ├── ImuCamFlow (LiveImuSource or ReplayImuSource)
  └── IpcPublisherFlow → IpcServerBus(endpoint="oak.capture")
      └── publishes: cam.sync, imu.raw, imucam.sample, frame.depth, calib.bundle
```

`calib.bundle` is a one-shot retained message: when a new subscriber connects
it gets the latest cached bundle immediately (so VIO / SLAM can boot without
guessing). Re-published on device re-open.

### 5.2 `ours/proc/vio.py`

```
IpcClientBus(endpoint="oak.capture")
  └── IpcSubscriberFlow → LocalBus
        ├── OdometryFlow(publish_vo=True)   (LIVE-only: also emits pose.vo)
        ├── BackendFlow            (worker=False — solve in-process here is fine;
        │                           this whole process is already "out-of-main")
        └── IpcPublisherFlow → IpcServerBus(endpoint="oak.vio")
              └── publishes: pose.odom, pose.vo, keyframe, frame.tracks,
                             frame.inliers, pose.refined, vio.map
```

`pose.vo` (`topics.POSE_VO`) is the PURE-VISION frame-to-frame trajectory — raw
PnP R/t only, **no gyro fusion, no tilt leveling, no BA**. It is accumulated by
`RGBDVisualOdometry.pose_vo` (a separate accumulator from the gyro-fused
`pose`, `ours/lib/odometry/odometry.py`) and emitted by the `PublishVo` task
(`ours/flows/odometry/publish_vo.py`), wired into the frame chain **only** when
`OdometryFlow` is built with `publish_vo=True` (`ours/proc/vio.py` sets it;
`POSE_VO` is in its `_OUTPUT_TOPICS`). It is **LIVE-only**: the offline /
deterministic path leaves `publish_vo=False`, so `PublishVo` never runs and
`pose.odom` byte-parity is unaffected (see §9 invariant 15). VIO carries the VO,
VIO (`pose.odom`) and VIO-BA (`pose.refined`) lines; SLAM carries the rest.

`vio.map` is the windowed-BA refined-keyframe trajectory (the same payload
`Engine.poll_overlay()` produces today), published as a periodic snapshot.

> **`pose.refined` is now drawn as the VIO-BA line; `vio.map` is still unwired.**
> The single view consumes `pose.refined` directly as the blue **VIO-BA** line
> (the windowed-BA keyframe trajectory — see §6.1). The separate periodic
> `vio.map` snapshot / `WireVioMap` wire type remains **defined but unconsumed**:
> the UI reads the per-keyframe `pose.refined` stream, not the `vio.map`
> aggregate. Do not read parity between them — `vio.map` is reserved for a future
> aggregate-overlay consumer.

### 5.3 `ours/proc/slam.py`

```
IpcClientBus(endpoint="oak.vio")
  └── IpcSubscriberFlow → LocalBus
        ├── SlamFlow(latest_only=True, publish_map=True, worker=False)
        │                          (latest-only LIVE inbox; in-process solve by default)
        └── IpcPublisherFlow → IpcServerBus(endpoint="oak.slam")
              └── publishes: loop.correction, slam.map
```

SLAM subscribes to `keyframe` **from VIO** (not capture), so SLAM never sees a
keyframe VIO hasn't already accepted. The pose-graph is SLAM's own map.

The proc4 `slam` process builds `SlamConfig(kf_min_trans_m=0.1,
kf_min_rot_deg=5.0)` (`ours/proc/slam.py`): a new keyframe is admitted to the
pose graph **only if the camera moved ≥10 cm OR rotated ≥5°** since the last
inserted keyframe, so a hovering / near-stationary drone stops adding redundant
near-identical keyframes (bounds the graph by trajectory length, not run time).
This is a proc4-LIVE setting; the offline `SlamFlow` keeps the `SlamConfig`
default `kf_min_trans_m=0.0` / `kf_min_rot_deg=0.0` (gate off), so offline
scoring is unchanged (see §9 invariant 16).

The proc4 SLAM flow is built with **`latest_only=True`** — a coalescing
(latest-only) in-process inbox. The ORB + pose-graph solve cost grows with the
map, so a strict FIFO inbox backed up without bound and the `slam.map` overlay
lagged further and further behind real time ("SLAM updates slowly"). A
coalescing inbox **drops the backlog and always solves the FRESHEST keyframe**,
so the live map stays current; it skips intermediate keyframes only when
overloaded, and `END` is never coalesced so clean shutdown still propagates.
This is the LIVE viewer's behaviour only — the offline / replay scoring path
(`ours.tools.vio_run` / `run_replay`) keeps the `SlamFlow` default
`latest_only=False` (strict FIFO) for determinism.

`worker=False` is the **default**: the heavy BA/SLAM solves run **in-process**
(this process is already off the main interpreter). `--worker` is an **opt-in**
flag on the launcher (off by default) that runs those solves GIL-free in child
subprocesses; with it off there is no worker subprocess and therefore no
`resource_tracker` semaphore noise at shutdown / Restart.

The process builds `SlamFlow` with `publish_map=True` (the LIVE-only flag), which
adds the `PublishSlamMap` task so SLAM emits **two** topics, distinct in cadence:

- `loop.correction` — the loop-event pose-graph rewrite, emitted ONLY on a
  confirmed loop closure (`SlamStep` returns a result only then). Byte-identical
  to the offline path.
- `slam.map` — a CONTINUOUS overlay published EVERY keyframe (`SlamOverlay`),
  carrying the current corrected camera-optical keyframe positions + `n_loops` +
  `last_match`. LIVE-only: the offline path keeps `publish_map=False`, so neither
  the task nor the topic exists there and the deterministic `loop.correction`
  scoring stays byte-identical (see §9 invariant 12).

### 5.4 `ours/proc/ui.py`

```
IpcClientBus(endpoint="oak.vio")       # pose.odom, pose.vo, pose.refined, calib.bundle (always)
IpcClientBus(endpoint="oak.slam")      # slam.map, calib.bundle (always)
IpcClientBus(endpoint="oak.capture")   # imu.raw, imucam.sample, frame.depth  (on-demand, menus)
  └── IpcSubscriberFlows → LocalBus → Qt MainWindow
        ├── ONE Viewer3D (no tabs): live marker = pose.odom (vio), drawing 5 lines
        │     VO                 : pose.vo     (vio) — grey,  pure vision
        │     VIO                : pose.odom   (vio) — green, f2f PnP + gyro
        │     VIO-BA             : pose.refined(vio) — blue,  windowed BA
        │     SLAM-corrected VIO : pose.odom deformed by slam.map corrections —
        │                          orange (teleport segments red)
        │     SLAM               : slam.map    (slam) — cyan kf line + amber dots
        ├── Controls toolbar (always-visible, top of window):
        │     [VO][VIO][VIO-BA][SLAM-corrected VIO][SLAM]  : per-line show/hide
        │     Clear Trail  : clear the live trajectory trail
        │     Restart      : quit with RESTART_EXIT_CODE=42 → launcher respawns all
        └── Menu bar (renders in-window on every platform; setNativeMenuBar(False)):
              View         : VIEW_PRESETS / Follow Camera (on the single viewer)
              Visualize    : triplet window  ← capture imucam.sample/frame.depth
                             keypoint tracker ← capture frame.depth + vio tracks/inliers
              Calibration  : gyro / accel dialogs ← capture imu.raw (RAW)
```

A single `SlamMapTracker` subscribes `slam.map` (slam endpoint) plus `pose.odom`
/ `pose.vo` / `pose.refined` (vio endpoint) for the lifetime of the process and
exposes one snapshot getter per line; `IpcPoseSource` feeds the live green marker
+ trail off `pose.odom`. The **menu** subscriptions are opened lazily by the
`ui_ipc_sources.py` adapters only when a Visualize/Calibration action fires, and
closed when the window/dialog closes (so an unused tracker holds no subscriber).

The Qt main thread sees only the in-proc Bus, so the existing UI sinks
(`UiTracksFlow`, `UiTripletFlow`) and the existing `ours.ui` calib dialogs are
reused unchanged — the adapters republish the IPC topics onto the very same
local Bus those sinks already read. See §6 for the adapter contract.

### 5.5 Two different optimisers: VIO = windowed BA, SLAM = PGO

VIO and SLAM run **two distinct optimisers** — this is the key fact behind the
five UI lines, so state it precisely:

- **VIO runs windowed Bundle Adjustment (BA).** `BackendFlow` (`RunBA`) solves a
  sliding window jointly over **keyframe poses AND landmarks** (3D points),
  minimising reprojection error — analytic Schur in `ours/lib/backend/`. Output:
  `pose.refined`, the blue **VIO-BA** line. BA refines the *local* geometry of the
  recent window.
- **SLAM runs Pose-Graph Optimization (PGO).** `SlamFlow` (`SlamStep`) runs ORB
  loop detection, then on a confirmed loop optimises a graph of **poses only — no
  landmarks** (`ours/lib/loop/`). The graph has odometry edges (relative motion
  between consecutive keyframes) + loop-closure edges (the relative motion implied
  by a revisited place); PGO **distributes the accumulated drift over the whole
  trajectory** so the loop closes consistently. Output: `loop.correction` (the
  loop-event rewrite) + `slam.map` (the continuous corrected keyframe map, the
  cyan **SLAM** line).

So BA ≠ PGO: BA is a local windowed landmark+pose solve (metric refinement); PGO
is a global pose-only solve fired by loop closure (drift redistribution). The five
lines make the progression visible: **VO** (vision only) → **VIO** (+ IMU) →
**VIO-BA** (+ windowed BA) → **SLAM-corrected VIO** (the dense VIO path deformed
by SLAM's loop correction) ; **SLAM** = the corrected keyframe map PGO converged
on.

## 6. UI — `ours/proc/ui.py` + `ours/proc/ui_ipc_sources.py`

The proc4 UI (`ours/proc/ui.py`, started by `./run.sh --proc`) is a single
`QMainWindow` with **one** `Viewer3D` (the former VIO / SLAM tabs were collapsed
into a single view) **and a menu bar**. It imports **no depthai**: everything it
shows is fed over IPC. Critically, the existing `ours/ui` windows and calib
dialogs are reused **byte-for-byte unchanged** — `ours/proc/ui.py` only builds
the viewer + toolbar + menus and wires them, and `ours/proc/ui_ipc_sources.py`
supplies three injectable adapters that bridge the IPC topics onto the same
in-process `Bus` those windows already read.

### 6.1 The single 5-trajectory view

One `Viewer3D` draws all five trajectory streams, each from its own source, each
with an independent show/hide toggle on the Controls toolbar (§6.2). The lines
form a progression — pure vision → +IMU → +windowed BA → +loop closure on the
dense path → the corrected keyframe map:

| # | Line | Colour (`ours/ui/theme`) | Source topic | Meaning |
|---|---|---|---|---|
| 1 | **VO**                 | grey   (`VO_PATH`)        | `pose.vo` (vio)         | PURE-VISION frame-to-frame path — raw PnP R/t, **no IMU, no BA**. Drifts most. |
| 2 | **VIO**                | green  (`TRACE_PATH`)     | `pose.odom` (vio)       | Frame-to-frame RGB-D PnP **+ gyro fusion**, no BA. The responsive live marker + trail (never lags — never waits on a back-end). |
| 3 | **VIO-BA**             | blue/violet (`BA_PATH`)   | `pose.refined` (vio)    | Windowed **Bundle Adjustment** keyframe trajectory (landmarks + poses). Sparse. (Previously labelled "BA".) |
| 4 | **SLAM-corrected VIO** | orange (`CORRECTED_PATH`) | `pose.odom` deformed by `slam.map` | The dense VIO trail rubber-sheeted by SLAM's per-keyframe pose-graph correction (np.interp of the per-keyframe correction delta, matched by keyframe seq). Segments where the correction magnitude exceeds ~0.15 m (`TELEPORT_M`) are flagged "teleport" and drawn in **red** (`TELEPORT`), highlighting where loop closure pulled the path. |
| 5 | **SLAM**               | cyan   (`REFINED_PATH`)   | `slam.map` (slam)       | The loop-corrected keyframe path + **amber keyframe dots**, with the just-revisited keyframes flashed on each loop closure (`last_match` + `n_loops`). |

The live green marker + VIO trail come from `IpcPoseSource` (`pose.odom`) feeding
the viewer's `PoseHistory`. The other four lines are fed by snapshot getters on a
single `SlamMapTracker` (`vo_snapshot`, `ba_snapshot`, `corrected_vio_snapshot`,
`refined_path_snapshot` + `slam_overlay_snapshot`), which subscribes — across two
IPC clients — to `slam.map` on the slam endpoint and `pose.odom` / `pose.vo` /
`pose.refined` on the vio endpoint. The SLAM-corrected VIO line needs BOTH the
dense `pose.odom` trail (with frame seqs) and the per-keyframe corrected positions
SLAM publishes (with their source seqs in `kf_ids`): the tracker matches each
keyframe to its dense VIO anchor, computes the correction delta, and interpolates
it piecewise-linearly by seq across the dense trail (flat-clamped outside the
keyframe range) before adding it back — so the orange line is the green VIO path
deformed onto SLAM's loop-corrected anchors.

`slam.map` **supersedes** the old `loop.correction`-driven overlay, which only
fired ON a loop closure — so there were no keyframe dots along the path until the
first loop closed (the bug this design fixes). `loop.correction` is still
published (it is the loop-event pose-graph rewrite), but the live keyframe-dots
overlay no longer waits on it.

All sources subscribe for the lifetime of the process; the recv threads marshal
poses onto the GUI thread via the existing `PoseSource` callback contract, and the
four tracker lines are latched by the `SlamMapTracker` background subscriber and
polled by the viewer at GUI rate.

### 6.2 Controls toolbar + menu bar

A small always-visible **Controls** `QToolBar` (docked at the top of the window,
`setMovable(False)`) carries first the **five per-line toggle buttons**, then the
two actions the operator reaches for most.

- **Line toggles** — five checkable `QPushButton`s, labelled exactly
  **VO** / **VIO** / **VIO-BA** / **SLAM-corrected VIO** / **SLAM** (in
  back-to-front / drift order). All start CHECKED (visible); each `toggled(bool)`
  drives its viewer visibility setter (`set_vo_visible`, `set_vio_visible`,
  `set_ba_visible`, `set_corrected_visible`, `set_slam_visible`) so the operator
  can isolate any one trajectory (e.g. show only VO vs VIO to read raw drift, or
  only SLAM vs SLAM-corrected VIO to read the loop correction).
- **Clear Trail** — clears the live trajectory trail (`history.clear()`). With
  one viewer there is no "active tab" — it targets the single `PoseHistory`
  directly. It used to live buried in the View menu; the operator hits it often
  (per-run trajectory reset), so it gets a real button.
- **Restart** — "chạy lại từ đầu": respawn the whole pipeline fresh. Because the
  IPC bus is one-way (server→client) the UI **cannot** reset vio/slam in place,
  so it sets a flag and calls `app.quit()`; `run_ui` then returns
  **`RESTART_EXIT_CODE = 42`**. The launcher's restart loop sees code 42,
  `_cleanup_orphans()`es the prior generation, and respawns capture + vio + slam
  + ui from scratch (any other exit code ends the launcher normally). See §7 for
  the launcher side and §9 invariant 13.

The menu bar mirrors the single-process `ours.ui.mainwindow`, but every action
drives the unchanged windows/dialogs over IPC — there is no `_release_device`
dance because the UI never owns the device.

The menu is plain Qt (`QMenuBar` / `QAction`) and is **not** macOS-specific — it
already runs on Linux. `ui.py` calls `mbar.setNativeMenuBar(False)` so the
View / Visualize / Calibration bar renders **in-window on every platform**
(Linux/Windows do this anyway; macOS would otherwise hoist it into the global
top-of-screen bar). That only fixes WHERE the bar draws, for cross-platform
consistency.

- **View** — `VIEW_PRESETS` camera presets and **Follow Camera** (checkable).
  Each acts on the single `Viewer3D`. **Clear Trail** moved out of this menu onto
  the always-visible Controls toolbar above. There is deliberately no "Clear
  Keyframes" — proc4 has no UI→SLAM channel, so it would be a dead action.
- **Visualize** — **"Camera + Depth + IMU (triplet)…"** (`SyncedViewWindow`,
  driven by `IpcTripletWorker`) and **"Keypoint Depth Tracker…"**
  (`KeypointTrackWindow`, driven by `IpcKeypointWorker`). The raw-stereo
  "Camera + IMU synced" view is intentionally **not** restored — the triplet is
  its superset. Each window is cached on the main window so repeated opens reuse
  the one IPC worker instead of stacking subscribers.
- **Calibration** — **"Gyroscope Bias…"** (`GyroCalibDialog`) and
  **"Accelerometer (6-position)…"** (`AccelCalibDialog`). Each opens with a fresh
  `IpcImuRawSource` injected as its `stream`, so the dialog does not own (or stop)
  the stream — the menu handler does, in its `finally`.

### 6.3 IPC adapters — `ours/proc/ui_ipc_sources.py`

Three drop-in adapters let the unchanged `ours/ui` windows/dialogs run with no
in-process acquisition graph. The module is **device-agnostic by contract**: it
consumes only the abstract IPC topics + wire POD types and never imports depthai
(the guarantee the future multi-chip port depends on).

| Adapter             | Duck-types / extends                        | Consumes (endpoint · topics)                                   | Notes |
|---                  |---                                          |---                                                            |---|
| `IpcImuRawSource`   | `ours.flows.imu_cam.imu_stream.ImuStream`   | capture · `imu.raw`                                            | Subscribes capture's **RAW** IMU and re-emits one `(3,)` gyro+accel sample at a time with a **seconds** timestamp (the calib collector's shape). RAW — not calibrated — is correct: calibrating off an already-calibrated stream would be circular. |
| `IpcTripletWorker`  | `ours.ui.synced_window.TripletWorker`       | capture · `imucam.sample`, `frame.depth`                       | `_drive` republishes both topics onto a local `Bus`; the unchanged `UiTripletFlow` sink joins them by `seq` and renders the triplet. |
| `IpcKeypointWorker` | `ours.ui.keypoints_window.KeypointWorker`   | capture · `frame.depth`  +  **vio** · `frame.tracks`, `frame.inliers` | Two endpoints: depth imagery from capture, KLT tracks + PnP inliers from VIO. The unchanged `UiTracksFlow` sink joins them by `seq`. Keeps `FrameTracks` pure POD so VIO never writes capture's rings (see §9 invariant 6). |

Each adapter opens its own read-only `IpcClientBus` on demand, attaches only the
capture rings it needs, and surfaces a connect failure (e.g. capture down) as a
clear reason on the worker/dialog rather than a raw shared-memory path error. No
second device, no second SGM — depth is already published by capture.

### 6.4 Calibration semantic — "saves for the NEXT capture start"

In proc4 the UI does **not** own the device; `capture` does. So a calibration
the UI saves is **not** applied live mid-run. The dialog keys the saved value
(gyro bias / accel calib) by `device_id` (taken from the calib bundle, see §3.3
/ §9) and writes it to the per-device store; `capture` **loads** it by the same
key on its **next start** (`load_gyro_bias` / `load_accel_calib` in
`ours/lib/device/live_calib.py`). The dialog already shows
"Saved for device `<id>`" to make the deferred effect explicit. This is the
correct contract because the UI cannot retro-fit the running capture pipeline,
and keying both sides by the identical `device_id` is what makes the saved value
actually take effect.

## 7. Launcher — `ours/proc/launcher.py` + `run.sh`

`launcher.py main()` spawns the three background processes (capture → vio →
slam, in that order so each subscriber boots after its publisher's endpoint
exists), waits a few hundred ms between each, then runs the UI process in
the foreground. On UI exit it sends `SIGTERM` to the three background processes
and joins them; on any of them dying it shuts the others down with a clear
diagnostic.

**Restart loop.** The spawn → run-UI → teardown sequence is a **loop**. Each
iteration `_spawn_pipeline()`s a fresh capture + vio + slam generation, blocks on
`ui_proc.wait()`, then `_terminate()`s that generation on the main thread (no
waitpid race — the UI is already reaped by `wait()`). If the UI returned
`RESTART_EXIT_CODE` (42, from the Restart toolbar button) the loop
`_cleanup_orphans()`es and respawns the whole pipeline from scratch; any other
exit code breaks the loop and the launcher exits normally. A full respawn is the
robust restart because the IPC bus is one-way (server→client), so the UI can't
reset vio/slam in place. The endpoint names are computed ONCE (the `--auto-suffix`
derives them from the launcher PID), so each restart re-creates the same-named
endpoints + rings; `_cleanup_orphans()` reclaims the prior generation's stale SHM
each iteration.

The **`--no-ui`** path runs the pipeline exactly **once** (there is no Restart
button without a UI, so it bypasses the restart loop): it spawns
capture + vio + slam, waits for capture to exit, lets vio + slam drain, then
tears them down.

The launcher's **SIGTERM handler** (registered once) forwards SIGTERM to the
current generation's children and `os._exit`s immediately. It deliberately does
**not** call `_terminate()` — `_terminate` polls `Popen.poll()`
(`os.waitpid(pid, WNOHANG)`) on the same pid the main thread is blocked in
`ui_proc.wait()` on, and the two waitpid callers would race for the single reap
event (the loser's `returncode` sticks at `None`, spinning the full 10 s deadline
and SIGKILLing a UI that already exited cleanly).

**`--worker` is an opt-in (default off).** With it off, vio + slam run their heavy
BA/SLAM solves **in-process** and SLAM stays responsive via its latest-only inbox
(§5.3) — no worker subprocess, so no `resource_tracker` semaphore noise at
shutdown / Restart. Passing `--worker` propagates `--worker` to both the vio and
slam children so the solves run GIL-free in child subprocesses.

`run.sh` gains:
- `./run.sh ...` — unchanged single-process live (default; the existing
  `ours.tools.view_pose3d` path stays for one release as a fallback).
- `./run.sh --proc ...` — the 4-process launcher (now `launcher.main`, see §7.1).
- `./run.sh --proc-old ...` — the PRE-split `ours.proc.launcher` (this oracle),
  kept reachable for the Phase 7 verification harness.

The offline `ours.app --session ...` path is untouched.

### 7.1 Split launcher — `launcher/main.py` + `launcher/comms/`

Phase 6 ships `launcher/`, a **behaviour-for-behaviour** port of
`ours/proc/launcher.py` retargeted onto the four split projects' `<project>.main`
entrypoints (`imu_camera.main` / `vio.main` / `slam.main` / `ui.main`). It is the
target of `./run.sh --proc`; `ours.proc.launcher` survives as `--proc-old` so the
reference oracle stays runnable. The restart loop, `--no-ui` once-path,
`_cleanup_orphans` SHM/sock reclaim, `_RING_NAMES_BY_ROLE`
(cap=`gray_left`/`gray_right`/`depth_m`, vio=`kf_gray`/`kf_depth`, slm=none),
`--auto-suffix` endpoint naming (`oak.cap.l<pidhex>` etc., longest ring shm name
23 chars ≤ the 30-char POSIX cap), and the waitpid-race-safe SIGTERM handler
(forward + `os._exit(143)`, never `_terminate`) are all carried over verbatim.

Two intentional differences from the oracle, forced by the new projects' actual
argparse (confirmed by reading each `main.py`):

- **capture argv inversion** — `imu_camera.main` DEFAULTS to replay and takes an
  explicit `--live` for hardware (the old `ours.proc.capture` defaulted to live),
  so the launcher's live branch passes `--live` and the replay branch passes
  `--session PATH [--max-frames N]`.
- **slam dropped `--capture-endpoint`** — the new `slam.main` is a pure consumer
  of VIO's output ("we deliberately don't subscribe to capture at all") and
  removed that flag, so the launcher wires slam with only
  `--vio-endpoint` / `--endpoint` (passing the old `--capture-endpoint` would
  make slam's argparse abort on startup).

`launcher/comms/` is a **byte-identical** vendored copy of `imu_camera/comms/`
(CI `diff -r` gate). The launcher only needs `SharedArrayRing.cleanup_stale` +
`ring_registry` for orphan reclaim, but the full copy is vendored for consistency
with the other projects. `launcher.main` stays **Qt-free** (it imports only
`RESTART_EXIT_CODE` from `ui.main`, which lazy-imports PyQt6 inside `run_ui`).

## 8. Testing

| Test | Scope | Status |
|---|---|---|
| `ours.tools.ipc_bus_selftest` | `SharedArrayRing` roundtrip + wrap, `IpcServerBus`/`IpcClientBus` 2-proc echo, `IpcPublisherFlow → IpcSubscriberFlow` byte-for-byte `ImuCamPacket` roundtrip. | PASSING |
| `ours.tools.proc4_replay_selftest` | All four processes (capture + vio + slam + headless UI sink) spawned against a recorded session. Asserts identical pose.odom count / density to the single-proc `flow_replay_selftest`, plus expected keyframe + refined counts and clean END propagation. | PASSING |
| `ours.tools.proc4_ui_selftest` | Same 4-proc spawn but drives `IpcPoseSource` + `SlamMapTracker` + a Qt `MainWindow` construction. Catches GUI-side regressions without needing a display in the event loop. | PASSING |
| Existing `flow_replay_selftest` + every `_selftest` | **Unchanged.** Single-process path is the reference and must stay green. | PASSING |

Verified end-to-end (`proc4_replay_selftest` against `sessions/gold/lab_loop_30s`):
- 30 frames → 30 odom, 6 keyframes, 4 refined poses
- 60 frames → 60 odom, 12 keyframes, 10 refined poses
- Identical to single-proc `flow_replay_selftest` (60 odom, 10 refined).

## 9. Invariants

1. The IPC layer is stdlib-only. No new pip deps.
2. Existing flows (`OdometryFlow`, `BackendFlow`, `SlamFlow`, every UI sink)
   are reused unchanged. The bridge flow is the only new flow type.
3. The offline replay path (`ours.app`, `flow_replay_selftest`) stays
   byte-identical and single-process.
4. Tools never open the OAK-D. Capture is the only owner of the device.
5. No process holds another process's data (every numpy array crossing the
   bridge is copied out of shared memory on the receiving side before any
   downstream task runs).
6. **Ring slots > IPC outbox capacity.** A wire message in an outbox
   references a ring slot the producer must NOT have overwritten by the time
   the consumer reads it. Default slots=64 strictly exceeds outbox cap=32
   so a publisher that's outbox-full is at most 32 frames ahead of a
   consumer, and the 32 outbox-queued items reference slots `[N-32, N-1] mod
   64` — all distinct from the producer's next-write slot `N mod 64`. See
   `default_capture_specs` / `default_vio_specs` and
   `SharedArrayRing.create` for the assertion.
7. **Drain before stop.** `Flow.stop()` checks `_stop` at the TOP of every
   loop iteration and discards any items still queued. So a process that
   wants to publish END must wait for the flow's `done` event (set inside
   `_handle_end` after `expected_ends` ENDs have been processed) BEFORE
   calling `stop()`. Capture waits on `imu_flow.done`; VIO on `odom.done` +
   `backend.done`; SLAM on `slam.done`. Discovered the hard way -- without
   it, capture lost 4 of 5 frames.
8. **IpcServerBus.close drains outbox.** `close()` sets `_stopped` to gate
   new publishes, then puts BYE on each subscriber's outbox and joins the
   fanout thread (which sends every pending wire message in order, then BYE,
   then exits). `state.alive` is ONLY flipped by send-errors -- close does
   NOT flip it. Older code that flipped alive in close caused the fanout to
   discard everything queued at close-time.
9. **`SharedMemory(track=False)` on attach.** The attaching process must
   not register the shm with its own resource_tracker (the creator does).
   `SharedArrayRing.attach` passes `track=False` (Python ≥ 3.13). Without
   it, the attacher prints spurious "leaked shared_memory" warnings on exit
   even though only the creator should unlink. See
   https://bugs.python.org/issue38119.
10. **Readiness barrier = retained calib.bundle re-publish.** VIO subscribes
    to capture's retained `calib.bundle`, then re-publishes the SAME bundle
    on its own retained endpoint AFTER allocating its kf_* rings. SLAM (and
    the UI) wait on VIO's calib bundle as a "VIO is ready, rings exist"
    signal. Without this barrier, downstream procs race the ring creation
    and fail with `FileNotFoundError`.
11. **`WireCalibBundle.device_id` is the calibration key, carried on the bundle.**
    `WireCalibBundle` (`ours/lib/ipc/messages.py`) carries an OPTIONAL
    `device_id: str | None = None` field. It is the per-device key for the IMU
    calibration store. **Producer:** capture fills it from the real device id
    (`LiveFrontEndCalib.device_id` in `ours/lib/device/live_calib.py` →
    `_build_calib_bundle_live` in `ours/proc/capture.py`); replay sets it to
    `None`. VIO **re-broadcasts the same bundle**, so the UI reads `device_id`
    off VIO's bundle. **Consumer:** the UI keys any calibration it saves (gyro
    bias / accel) by this id, which is IDENTICALLY the key capture LOADS with
    (`load_gyro_bias` / `load_accel_calib`) on its NEXT start — that match is
    what makes a UI-saved calibration actually take effect (it is NOT applied
    live; see §6.4). When `device_id` is `None` (replay) the UI falls back to
    `"default"`.
    *This is a cross-language wire contract.* `device_id` is a deliberate
    **additive, backward-compatible OPTIONAL** field: it has a default and is
    placed AFTER the existing optional fields, so pickling stays safe and any
    older subscriber simply ignores it.
12. **`slam.map` is a LIVE-ONLY overlay; it never touches the offline scoring
    path.** SLAM publishes a continuous keyframe-map overlay on
    `slam.map` (`topics.SLAM_MAP`), carrying the local POD `SlamOverlay`
    (`ours/lib/flow/messages.py`) over the wire as `WireSlamMap`
    (`ours/lib/ipc/messages.py`); the bridge converters are registered in
    `ours/flows/bridge/converters.py`. **Producer:** the `PublishSlamMap` task
    (`ours/flows/slam/publish_slam_map.py`) emits it EVERY keyframe — but ONLY
    when `SlamFlow` is built with `publish_map=True`
    (`ours/flows/slam/slam_flow.py`). The proc4 `slam` process sets that flag
    (`ours/proc/slam.py`) and adds `SLAM_MAP` to its IPC outputs; the **offline /
    replay path keeps `publish_map=False`**, so neither the task nor the topic
    exists there. **Consumer:** the UI's SLAM line (`SlamMapTracker` in
    `ours/proc/ui.py`) draws the continuous keyframe dots from `slam.map` instead
    of waiting on `loop.correction`. The invariant: `slam.map` is **purely
    additive and live-only** — `loop.correction` (the loop-event pose-graph
    rewrite) and the deterministic offline scoring path stay **byte-identical**
    whether or not the overlay exists. The overlay polls
    `engine.poll_overlay()` (the same continuous `slam_overlay` the single-process
    UI drew from), which has no return-value coupling to `SlamStep` /
    `loop.correction`; `PublishSlamMap` returns `None` so it never alters the
    task chain's result.
13. **Restart = full respawn via `RESTART_EXIT_CODE`; there is no reverse IPC
    channel.** The IPC bus is one-way (server→client), so the UI cannot reset
    vio/slam in place. The Restart toolbar button (`ours/proc/ui.py`) instead
    quits the Qt loop and `run_ui` returns `RESTART_EXIT_CODE = 42`; the launcher
    (`ours/proc/launcher.py`) loops on that code, `_cleanup_orphans()`es, and
    respawns capture + vio + slam + ui from scratch. Any other UI exit code ends
    the launcher normally. The `--no-ui` path bypasses the loop (runs once). The
    launcher's SIGTERM handler must NOT call `_terminate()` (waitpid race with the
    main thread's `ui_proc.wait()` — see §7).
14. **proc4 SLAM uses a latest-only (coalescing) inbox; the offline scoring path
    does not.** The `slam` process builds `SlamFlow(latest_only=True, ...)`
    (`ours/proc/slam.py`) so the LIVE viewer drops a keyframe backlog and always
    solves the freshest keyframe — the `slam.map` overlay stays current instead of
    lagging as the ORB + pose-graph solve cost grows. `END` is never coalesced, so
    shutdown still propagates. The deterministic offline / replay path
    (`ours.tools.vio_run` / `run_replay`) keeps the `SlamFlow` default
    `latest_only=False` (strict FIFO), so its scoring stays byte-identical. The
    in-process solve is the default (`worker=False`); `--worker` is an opt-in that
    moves the heavy solves to GIL-free child subprocesses (see §7).
15. **`pose.vo` is LIVE-only; `pose.odom` byte-parity is preserved.** The
    pure-vision frame-to-frame trajectory (`topics.POSE_VO`, the UI's VO line) is
    emitted by the `PublishVo` task (`ours/flows/odometry/publish_vo.py`), wired
    into `OdometryFlow`'s frame chain **only** when built with `publish_vo=True`.
    The proc4 `vio` process sets that flag (`ours/proc/vio.py`, `POSE_VO` in
    `_OUTPUT_TOPICS`); the offline / deterministic path leaves `publish_vo=False`,
    so `PublishVo` never runs and never publishes. `pose_vo` is a SEPARATE
    accumulator on `RGBDVisualOdometry` (raw PnP R/t — no gyro, no tilt, no BA)
    that is read-only w.r.t. the gyro-fused `pose`, so adding it does not perturb
    the `pose.odom` solve: offline `pose.odom` output stays **byte-identical**.
16. **SLAM keyframe motion-gating is a proc4-LIVE setting; the offline default is
    0/0 (gate off).** The `slam` process builds
    `SlamConfig(kf_min_trans_m=0.1, kf_min_rot_deg=5.0)` (`ours/proc/slam.py`) so a
    keyframe joins the pose graph only after ≥10 cm of translation OR ≥5° of
    rotation since the last inserted keyframe (skips redundant near-identical
    keyframes; bounds the graph by trajectory length). The `SlamConfig` defaults
    are `kf_min_trans_m=0.0` / `kf_min_rot_deg=0.0` (`ours/lib/loop/slam.py`), so
    the offline `SlamFlow` keeps the gate OFF and its deterministic scoring is
    unchanged.

## 10. Migration order

The phases match the todo list. Each is shippable on its own. **Done items
in bold.**

1. **IPC primitives** + selftest. No process changes yet.
2. **Bridge flows** + selftest. Still single-process; bridge is exercised by
   a unit test that wires both ends in-process.
3. **Capture process** + smoke test (replay backend → IPC publish).
4. **VIO process** + smoke test (replay capture → VIO → collect pose.odom).
5. **SLAM process** + smoke test (replay capture → VIO → SLAM → collect).
6. **UI process** — single 5-trajectory `Viewer3D` + `IpcPoseSource` +
   `SlamMapTracker`. (`pose.refined` is now drawn as the VIO-BA line, so the
   former "VIO refined overlay unwired" gap is closed.)
7. **Calib / visualize tools rewired to the IPC capture stream** — done via the
   in-UI **View / Visualize / Calibration** menus, fed by the
   `ours/proc/ui_ipc_sources.py` adapters (§6). The earlier "UI is pose-only"
   limitation no longer holds: the triplet view, keypoint tracker, and gyro /
   accel calibration dialogs are restored, all device-agnostic over IPC.
8. **`run.sh --proc`** launcher (`ours.proc.launcher`). It passes
   `--capture-endpoint` to the UI (in addition to `--vio-endpoint` /
   `--slam-endpoint`) so the Visualize / Calibration menus can reach capture's
   stream. Old single-process path stays as the default `./run.sh` for one
   release, removed only after live validation.
