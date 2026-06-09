# `depth/` — the stereo-depth project (SOURCE-OF-TRUTH for the SGM math)

The **second** of the five split projects (`imu_camera`, `depth`, `vio`, `slam`,
`ui`), built by replicating the **proven `imu_camera` template**. `depth` owns the
from-scratch SGM dense-stereo matcher + the two depth steps
(`compute_depth` → `publish_depth`).

> **`depth/` is the canonical home of the stereo math.** The capture project
> (`imu_camera`) vendors a **byte-identical copy** because depth runs **INLINE**
> on the capture process's `imu_cam` thread in the live topology today — so the
> launcher never spawns a depth process. A `diff -r` gate keeps the two copies in
> lock-step, and this tree is where the stereo math is edited and where a future
> "depth as its own process" promotion would graduate from.

`depth.main` is the **standalone harness** that proves the source tree already
runs as its own independent project: it subscribes to raw `cam.sync` over IPC,
computes metric depth with the SGM matcher, and publishes `frame.depth` on its
own endpoint.

```
imu_camera.main ──(oak.capture: cam.sync raw L/R + calib.bundle)──▶ depth.main ──(oak.depth: frame.depth)──▶ consumers
  capture proc                    IPC                                depth proc            IPC
```

## Layers

| Package | Role | Source |
|---------|------|--------|
| `depth/comms/` | the **FROZEN** vendored comms contract | copied **bit-identically** from `imu_camera/comms` |
| `depth/mathlib/stereo/` | the SGM matcher + rectifiers depth **OWNS** (the source of truth) | the canonical copy; `imu_camera/mathlib/stereo` vendors it |
| `depth/io/` | recorded-session reading (used **only** for the full stereo calibration) | re-rooted copy of `imu_camera/io` |
| `depth/modules/` | the `compute_depth` + `publish_depth` steps | re-rooted copy of `imu_camera/modules/{compute_depth,publish_depth}.py` |
| `depth/main.py` | the standalone depth process | new (mirrors `imu_camera.modules.pipeline` wiring + `vio.main` IPC topology) |
| `depth/tests/` | the SGM-vs-chip-depth regression self-test | re-rooted copy of `imu_camera/tests/stereo_sgm_selftest.py` |

### `depth/comms/` — byte-identical, do not hand-edit

`depth/comms` is **copied bit-identically** from `imu_camera/comms`. A gate runs
`diff -r --exclude=__pycache__ depth/comms imu_camera/comms` and it must be empty.
All its internal imports are RELATIVE, so the copy works as `depth.comms`
unchanged. **Never hand-edit it** — change `imu_camera/comms` and re-vendor.

Public API depth uses: `LocalPubSub`, `IPCPubSub(role="server"|"client")`,
`IPCPublisher`, `IPCSubscriber`, `Module`, `Step`, `RingRegistry`, `topics`,
`messages.{DepthFrame,CamSync,END}`, `converters`, and `wire.WireCalibBundle`.

### `depth/mathlib/stereo/` — the source of truth, kept in lock-step

`depth/mathlib/stereo` is the **canonical** SGM math (numpy + numba, fully
self-contained — no top-level cv2 / project imports). A gate runs `diff -r
--exclude=__pycache__ depth/mathlib/stereo imu_camera/mathlib/stereo`; the only
permitted delta is the three docstring lines that name the host project's
`io.reader.StereoCalib` (`depth.` vs `imu_camera.`). Every line of MATH is
byte-identical, so the depth the standalone process runs is **numerically
identical** to the depth the capture process computes inline (proven —
`depth.tests.stereo_sgm_selftest` reports the same numbers as
`imu_camera.tests.stereo_sgm_selftest` line-for-line).

* `SGMStereoMatcher` + `SGMConfig` — semi-global block matching with built-in
  left/right rectification.
* `StereoMatcher` + `StereoConfig` — the sparse block matcher the self-test uses.

#### Density-preserving disparity denoise (live preset only)

The raw SGM disparity carries salt-pepper mismatches + isolated "flying" blobs
that survive the L/R / uniqueness gates and make the 3D map look exploded.
`SGMConfig` exposes two **post-filters** that clean the disparity map *after* the
WTA/uniqueness/LR gates — so they never reject more matches (keypoint depth
density is preserved; the rejection thresholds — `uniqueness` / `lr_max_diff` /
census — are left untouched, which would otherwise starve PnP):

* `median_disp` — `cv2.medianBlur` aperture on the disparity (e.g. `3` for 3×3;
  `0` = off). Kills salt-pepper without shifting edges; a median over a
  mostly-valid window stays valid, so a hole only opens where the neighbourhood
  was already mostly invalid.
* `speckle_window` / `speckle_range` + `speckle_cv2` — small-blob removal. With
  `speckle_cv2=True` it uses `cv2.filterSpeckles` (fast C, quantised int16 grid
  used only to GROUP blobs — survivors keep their float sub-pixel disparity);
  otherwise it falls back to the numba `_speckle_filter` flood fill.

Both run at the **computed** (post-downscale) resolution, where the map is small,
so the measured per-frame cost is a fraction of a millisecond. They are **OFF by
default** (`SGMConfig()` is byte-identical to before) and **ON only in
`SGMConfig.live()`** (`median_disp=3`, `speckle_window=20`, `speckle_cv2=True`),
i.e. the live / replay-preview depth — the path that feeds the live 3D map.
Gate: `python -m imu_camera.tests.sgm_denoise_bench` (latency increase bounded,
keypoint density not dropped, speckle proxy ≥30% lower).

### Why `depth/io/` (the calibration the wire bundle doesn't carry)

The matcher's rectifiers (`RightRectifier` / `LeftRectifier`) need the **full
per-camera stereo calibration** — `K_left` / `K_right`, the per-camera distortion,
and the `T_left_right` rigid transform. That calibration is **NOT** on the wire
`calib.bundle` (`WireCalibBundle` broadcasts only the rectified-left intrinsic +
the IMU extrinsics — everything VIO/SLAM need, since they never recompute depth).

So — exactly as the capture project builds its matcher from `reader.calib`
(replay) / `cal.calib` (live) — `depth.main` builds the matcher from the recorded
session's `calib.json` (`--session`). The raw stereo frames themselves still
arrive **over IPC** on `cam.sync`; the session is read **only** for the
calibration. The wire bundle is used as the readiness barrier + frame sizing, and
is re-broadcast on depth's endpoint so a `frame.depth` consumer that connects there
boots with the bundle cached.

## `depth.main` — the standalone depth process

1. Open a **calib client** on the capture endpoint; block until the retained
   `calib.bundle` arrives (readiness barrier + frame size).
2. Build the `SGMStereoMatcher` from the session's full `StereoCalib`.
3. Attach capture's rings (consumer-side) so the subscriber bridge can read
   `cam.sync`'s raw left/right out of shared memory; create depth's **own** rings
   for the `frame.depth` output.
4. Open depth's **output** `IPCPubSub` server (`blocking=False`) + an
   `IPCPublisher` for `frame.depth`; re-broadcast the retained `calib.bundle`.
5. Wire a `DepthModule` on a `LocalPubSub` running `[ComputeDepthStep,
   PublishDepthStep]` on `cam.sync` (matcher in `ctx.state["matcher"]`) — the
   **same two steps, wired the same way** the capture project composes them inline
   in `imu_camera.modules.pipeline.ImuCamModule`.
6. Open the **input** `IPCPubSub` client + `IPCSubscriber` bridge for `cam.sync`.
7. Run until capture sends `END` on `cam.sync`, the `--max-frames` cap is hit, or
   SIGTERM / Ctrl-C; then a clean drain → stop bridges → flush → close server →
   unlink rings (mirrors the `imu_camera.main` / `vio.main` shutdown lifecycle,
   with `os._exit` so no lingering thread holds the process open).

CLI: `--capture-endpoint` (default `oak.capture`), `--endpoint` (default
`oak.depth`), `--session`, `--max-frames`, `--depth-fast`, `--calib-timeout`.

## Run

```bash
# the SGM-vs-chip-depth regression self-test (no device; offline gold sessions)
.venv/bin/python -m depth.tests.stereo_sgm_selftest

# depth-as-a-process pair (replay; no device needed). Capture publishes cam.sync
# on its endpoint; depth consumes it and publishes frame.depth on its own.
.venv/bin/python -m imu_camera.main --session sessions/gold/lab_loop_30s \
    --endpoint oak.capture
.venv/bin/python -m depth.main --capture-endpoint oak.capture \
    --endpoint oak.depth --session sessions/gold/lab_loop_30s --max-frames 20
```

## Gates (run from the repo root)

```bash
diff -r --exclude=__pycache__ depth/comms imu_camera/comms          # must be empty
diff -r --exclude=__pycache__ depth/mathlib/stereo imu_camera/mathlib/stereo  # must be empty
.venv/bin/python -c "import depth.main, depth.modules.compute_depth"  # imports OK
.venv/bin/python -m depth.tests.stereo_sgm_selftest                  # byte-parity vs ours stereo
```
