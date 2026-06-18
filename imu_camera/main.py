"""capture process: own the OAK-D (or a recorded session) + publish cam/IMU/depth.

Wires the same ``read_cam`` + ``imu_cam`` (with depth steps) front-end the
in-process :func:`imu_camera.modules.pipeline.build_live_frontend` /
:func:`~imu_camera.modules.pipeline.build_replay_frontend` builds, but adds an
:class:`~imu_camera.comms.IPCPublisher` that mirrors the local
:class:`~imu_camera.comms.LocalPubSub` topics onto an
:class:`~imu_camera.comms.IPCPubSub` server at the canonical endpoint
``"oak.capture"``. The calibration bundle is broadcast once on the **retained**
``calib.bundle`` topic so any subscriber that connects later (UI / SLAM / a calib
tool) immediately receives the latest copy.

Two modes share the same downstream wiring:

* ``--live`` -- :func:`~imu_camera.modules.pipeline.build_live_frontend` (real
  OAK-D). Hardware only.
* ``--session PATH`` (default) -- :func:`build_replay_frontend` over a recorded
  session, so the whole stack runs without a device on CI.

Run::

    python -m imu_camera.main --session sessions/gold/lab_loop_30s
    python -m imu_camera.main --live --width 640 --height 400 --fps 20
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from imu_camera.comms import (                                    # noqa: E402
    IPCPublisher, IPCPubSub, LocalPubSub, RingRegistry, topics,
)
from imu_camera.comms.ring_registry import default_capture_specs  # noqa: E402
from imu_camera.comms.wire import WireCalibBundle, WireCalibStereo  # noqa: E402
from imu_camera.io.reader import SessionReader                    # noqa: E402

LOG = logging.getLogger("imu_camera.main")

#: Canonical endpoint name -- VIO / SLAM / UI / tools all connect here.
DEFAULT_ENDPOINT = "oak.capture"

#: Bridge-forwarded topics. Calibration travels on its own RETAINED topic so a
#: late subscriber boots with the bundle already cached.
_DATA_TOPICS = [
    topics.CAM_SYNC,
    topics.IMU_RAW,
    topics.IMUCAM_SAMPLE,
    topics.FRAME_DEPTH,
]
_CALIB_TOPIC = "calib.bundle"
#: Sibling RETAINED topic carrying the FULL stereo calib (both intrinsics +
#: distortion + the left->right extrinsic) the rectifiers need. Published ALONGSIDE
#: ``calib.bundle`` (which carries only the rectified-left K) so the UI's Epipolar
#: window can build a Left/RightRectifier from a live raw pair. ADDITIVE: the
#: oracle never consumes it and ``calib.bundle`` is byte-unchanged -> gap=0 holds.
_CALIB_STEREO_TOPIC = "calib.stereo"


def _build_calib_bundle_replay(reader: SessionReader) -> WireCalibBundle:
    """Wire-bundle from a recorded session's `calib.json`."""
    from imu_camera.modules.pipeline import _replay_imu_startup
    R_imu_cam, accel_align, gyro_bias = _replay_imu_startup(reader, use_gyro=True)
    T = reader.calib.T_imu_left if reader.calib.has_imu_extrinsics else None
    return WireCalibBundle(
        K=np.asarray(reader.K, dtype=np.float64),
        width=int(reader.calib.left.width),
        height=int(reader.calib.left.height),
        fps=20,
        T_imu_left=(None if T is None else np.asarray(T, dtype=np.float64)),
        R_imu_cam=(None if R_imu_cam is None
                   else np.asarray(R_imu_cam, dtype=np.float64)),
        accel_align=(None if accel_align is None
                     else np.asarray(accel_align, dtype=np.float64)),
        gyro_bias=(None if gyro_bias is None
                   else np.asarray(gyro_bias, dtype=np.float64)),
        # Replay has no live device -> the UI falls back to "default" when it
        # keys any IMU calibration it saves.
        device_id=None,
    )


def _scale_bundle_to_tof(bundle: WireCalibBundle, *,
                         src_w: int, src_h: int) -> WireCalibBundle:
    """Return a copy of ``bundle`` with K + dims scaled to the ToF grid.

    The ToF frame is a NON-uniform resize of the source (54/W_src != 42/H_src),
    so K is scaled ANISOTROPICALLY: fx, cx by ``TOF_W/src_w`` and fy, cy by
    ``TOF_H/src_h``. Depth metres are unchanged (the world distance a pixel sees
    does not change when the image is resized -- only the focal length in pixels
    does). Every other field (extrinsics, IMU calib, device id) carries through.
    """
    from imu_camera.modules.pipeline import TOF_W, TOF_H
    import dataclasses

    sx = TOF_W / float(src_w)
    sy = TOF_H / float(src_h)
    K = np.asarray(bundle.K, dtype=np.float64).copy()
    K[0, 0] *= sx          # fx
    K[0, 2] *= sx          # cx
    K[1, 1] *= sy          # fy
    K[1, 2] *= sy          # cy
    return dataclasses.replace(bundle, K=K, width=TOF_W, height=TOF_H)


def _build_calib_bundle_live(cal) -> WireCalibBundle:
    """Wire-bundle from a live `read_live_calibration` result."""
    T = cal.calib.T_imu_left if cal.calib.has_imu_extrinsics else None
    gyro_bias = (cal.imu_calibration.gyro_bias
                 if cal.imu_calibration is not None else None)
    return WireCalibBundle(
        K=np.asarray(cal.K, dtype=np.float64),
        width=int(cal.calib.left.width),
        height=int(cal.calib.left.height),
        fps=20,
        T_imu_left=(None if T is None else np.asarray(T, dtype=np.float64)),
        R_imu_cam=(None if cal.R_imu_cam is None
                   else np.asarray(cal.R_imu_cam, dtype=np.float64)),
        accel_align=(None if cal.accel_align is None
                     else np.asarray(cal.accel_align, dtype=np.float64)),
        gyro_bias=(None if gyro_bias is None
                   else np.asarray(gyro_bias, dtype=np.float64)),
        # Carry the live device id so the UI keys any saved IMU calib by the SAME
        # id capture/VIO use -> the saved calib takes effect on the next start.
        device_id=cal.device_id,
    )


def _build_calib_stereo(calib) -> WireCalibStereo:
    """Build the retained ``calib.stereo`` wire message from a ``StereoCalib``.

    ``calib`` is the SAME :class:`depth.io.reader.StereoCalib` (replay:
    ``reader.calib``; live: ``cal.calib``) the depth path already uses to build its
    stereo matcher -- this just EXPOSES the rectifier-relevant fields on the wire.
    It carries exactly what ``Left/RightRectifier.from_calib`` read
    (``sky/depth/stereo.py``): both cameras' ``K`` + distortion, the left->right
    rigid transform, and the (left) image dimensions the rectifier maps span.

    The native source-resolution calib is published as-is (no ToF K-scaling): the
    rectifiers operate on the raw full-resolution left+right the UI receives on
    ``imucam.sample``, so the calib must match THAT grid, not a downsampled one.
    """
    return WireCalibStereo(
        left_K=np.asarray(calib.left.K, dtype=np.float64).reshape(3, 3),
        left_dist=np.asarray(calib.left.dist, dtype=np.float64).reshape(-1),
        right_K=np.asarray(calib.right.K, dtype=np.float64).reshape(3, 3),
        right_dist=np.asarray(calib.right.dist, dtype=np.float64).reshape(-1),
        T_left_right=np.asarray(calib.T_left_right, dtype=np.float64).reshape(4, 4),
        width=int(calib.left.width),
        height=int(calib.left.height),
    )


def _drain_wait(done_evt, ceiling_s: float, stop) -> None:
    """Wait up to ``ceiling_s`` for ``done_evt``, polling so a late SIGINT/SIGTERM
    (``stop[0]``) short-circuits the wait. The caller then forces the worker out
    via ``.stop()`` instead of blocking on an END that will never arrive."""
    waited = 0.0
    while waited < ceiling_s and not stop[0]:
        if done_evt.wait(timeout=0.1):
            return
        waited += 0.1


def _poll_with_watchdog(stop, cam_module, imu_module) -> None:
    """Poll until the cam read thread exits, with a frame-flow WATCHDOG.

    A frozen OAK does NOT kill the worker -- it BLOCKS on ``_inbox.get()``, alive
    but starved, so a bare ``while is_alive(): sleep`` spins SILENTLY and a device
    freeze leaves NO trace in run.log (the operator's "runs a bit then freezes,
    console says nothing"). ``imu_module.frames_out`` is the count of
    ``IMUCAM_SAMPLE`` frames published downstream -- the data VIO actually
    consumes -- so a frozen count means "VIO is getting nothing", whatever
    stalled (device read OR the pack stage). We emit a periodic fps heartbeat and
    a LOUD, throttled WARNING the instant frames stall + when they RECOVER, so
    the log always carries a timestamped trace + a cause hint to grep for.
    """
    _HB, _STALL = 5.0, 3.0                        # heartbeat / stall-warn (s)
    last_n = imu_module.frames_out
    t_frame = time.monotonic()                    # when frames last advanced
    t_hb, hb_n = t_frame, last_n                  # heartbeat anchor
    stalled = False
    while not stop[0] and cam_module.is_alive():
        time.sleep(0.2)
        now = time.monotonic()
        n = imu_module.frames_out
        if n != last_n:                           # frames advancing
            if stalled:
                LOG.warning("capture: RECOVERED after %.1fs stall (frame %d)",
                            now - t_frame, n)
                stalled = False
            last_n, t_frame = n, now
        elif not stalled and now - t_frame >= _STALL:         # frames stopped
            stalled = True
            LOG.warning("capture: STALLED -- no frame for %.1fs (last frame %d, "
                        "read-thread alive=%s). OAK crashed/hung? check `dmesg` + "
                        ".cache/depthai/crashdumps", now - t_frame, last_n,
                        cam_module.is_alive())
        if now - t_hb >= _HB:                      # periodic heartbeat
            fps = (n - hb_n) / (now - t_hb)
            if stalled:
                LOG.warning("capture: %.1f fps (frame %d) [STALLED %.1fs]",
                            fps, n, now - t_frame)
            else:
                LOG.info("capture: %.1f fps (frame %d)", fps, n)
            t_hb, hb_n = now, n
    LOG.info("capture: loop exit (stop=%s, read-alive=%s, frames=%d)",
             stop[0], cam_module.is_alive(), imu_module.frames_out)


# --------------------------------------------------------------------------- #
def run_capture_replay(session: Path, endpoint: str, *,
                       width: int, height: int,
                       max_frames: int = 0,
                       depth_fast: bool = True,
                       tof_sim: bool = False) -> int:
    """Replay-driven capture: SessionReader -> bridge -> IPC."""
    from imu_camera.modules.pipeline import (
        TOF_W, TOF_H, _replay_imu_startup, build_replay_frontend)
    from sky.sensors.imu_calib import ImuCalibration

    reader = SessionReader(session)
    # Use the session's native resolution so the rings line up with the frames.
    width, height = int(reader.calib.left.width), int(reader.calib.left.height)

    # VL53L9CX simulation: depth is computed at the session SOURCE resolution
    # (width x height, where SGM works) but the PUBLISHED frames/depth are
    # downsampled to the fixed ToF grid by ToFDownsampleStep. The rings + the
    # broadcast calib bundle must therefore match the 54x42 grid, not the source.
    pub_w, pub_h = (TOF_W, TOF_H) if tof_sim else (width, height)

    rings = RingRegistry().create_all(default_capture_specs(
        endpoint=endpoint, width=pub_w, height=pub_h))
    # Live mode uses non-blocking publish (drop-oldest on stall so the OAK-D
    # firmware watchdog never fires). Replay mode below uses blocking=True so
    # every replayed frame reaches VIO.
    server = IPCPubSub(endpoint, role="server",
                       retain_topics={_CALIB_TOPIC, _CALIB_STEREO_TOPIC},
                       blocking=False)
    local = LocalPubSub()

    # In the ToF path cam.sync carries the SOURCE-res stereo pair (640x400) on the
    # local bus only; it must NOT cross the IPC boundary because the rings are
    # sized to 54x42 (writing a 640x400 array would raise). No IPC consumer needs
    # cam.sync (VIO subscribes only to imucam.sample + frame.depth), so drop it
    # from the bridged topics for the ToF run.
    data_topics = ([t for t in _DATA_TOPICS if t != topics.CAM_SYNC]
                   if tof_sim else _DATA_TOPICS)

    # Build the publisher BEFORE the front-end so subscribers connecting at any
    # time are wired in (the publisher subscribes to the local bus eagerly).
    pub = IPCPublisher(local, server, rings, data_topics, endpoint=endpoint)
    pub.start()

    # Replay startup IMU references for the imu_cam module's calibration.
    _, _, gyro_bias = _replay_imu_startup(reader, use_gyro=True)
    calibration = (ImuCalibration(gyro_bias=gyro_bias)
                   if gyro_bias is not None else None)

    cam_module, imu_module = build_replay_frontend(
        bus=local, reader=reader, depth_fast=depth_fast,
        max_frames=int(max_frames), calibration=calibration, tof_sim=tof_sim)

    # Broadcast the retained calibration bundle BEFORE starting the front-end
    # so any subscriber that connects mid-run gets the cached one immediately.
    # ToF: scale K to the 54x42 grid so a consumer that reads calib.bundle solves
    # against the SAME pixel grid the published frames/depth use.
    bundle = _build_calib_bundle_replay(reader)
    if tof_sim:
        bundle = _scale_bundle_to_tof(bundle, src_w=width, src_h=height)
    server.publish(_CALIB_TOPIC, bundle)
    # Sibling retained FULL stereo calib for the UI's Epipolar window (additive;
    # the oracle never reads it). Built from the SAME StereoCalib the depth path
    # uses, at native source resolution (the rectifiers run on the raw pair).
    server.publish(_CALIB_STEREO_TOPIC, _build_calib_stereo(reader.calib))
    LOG.info("capture[%s] replay session=%s frames=%d src=%dx%d pub=%dx%d%s",
             endpoint, session, len(reader), width, height, pub_w, pub_h,
             " (vl53l9cx ToF sim)" if tof_sim else "")

    # Install SIGTERM handler BEFORE starting the modules so the launcher's
    # SIGTERM is observed even if the producer is mid-frame. Without this the
    # `cam_module` join below blocks until the source is fully drained -- a
    # 30-second replay session would block shutdown for ~30 s and the launcher
    # would SIGKILL the process at the 10 s deadline, leaking every ring slot.
    stop = [False]
    def _on_sigterm(_signo, _frame):
        stop[0] = True
        # cam_module is a producer thread; setting its _stop flag breaks out of
        # the produce loop at the next item boundary so the join() returns.
        cam_module.stop()
    # SIGINT (Ctrl-C) and SIGTERM (launcher) both request the SAME clean stop:
    # handling SIGINT here (vs the default KeyboardInterrupt) means teardown can
    # NEVER abort on a raw traceback -- the operator Ctrl-Cs once and we exit.
    signal.signal(signal.SIGTERM, _on_sigterm)
    signal.signal(signal.SIGINT, _on_sigterm)

    imu_module.start()
    cam_module.start()
    LOG.info("capture: cam + imu modules started, waiting for join ...")
    try:
        # Poll until the read thread exits, with the frame-flow watchdog
        # (heartbeat + stall/recovery logging; a frozen source is otherwise a
        # silent spin here). Signal handler stays prompt at the 0.2 s cadence.
        _poll_with_watchdog(stop, cam_module, imu_module)
    finally:
        # ReadCamModule (producer thread) emits END on CAM_SYNC when its produce
        # loop returns; ImuCamWorker forwards END from CAM_SYNC to IMU_RAW +
        # IMUCAM_SAMPLE + FRAME_DEPTH (its `_downstream` list). The publisher
        # bridge converts those local-bus ENDs to WireEnd and sends them on the
        # IPC server.
        #
        # CRITICAL: wait for ImuCamWorker's drain to chew through every queued
        # CAM_SYNC + the END BEFORE calling imu_module.stop(). stop() sets
        # `_stop`, which the drain checks at the TOP of every loop iteration --
        # so if we stop while CAM_SYNC items are still queued, we discard them
        # AND the END. `done` is set inside `_handle_end` after the END is
        # drained (single-input worker -> the first END is terminal).
        #
        # On interrupt the operator wants a fast exit, NOT a full drain -- END
        # will never arrive from a half-killed producer. `_drain_wait` polls
        # stop[0] so a late SIGINT/SIGTERM short-circuits the wait. Natural
        # end-of-replay keeps the generous 120 s ceiling so a busy backend can
        # finish.
        cam_module.stop()
        drain_timeout = 2.0 if stop[0] else 120.0
        LOG.info("capture: waiting for imu module to drain (timeout=%.1fs) ...",
                 drain_timeout)
        _drain_wait(imu_module.done, drain_timeout, stop)
        LOG.info("capture: imu_module.done=%s", imu_module.done.is_set())
        imu_module.stop()
        # Give the bridge a brief window to flush the buffered WireEnds onto
        # the socket before we tear down the server.
        time.sleep(0.3)
        pub.stop()
        server.close()
        rings.unlink()
        rings.close()
        LOG.info("capture: shutdown complete")
    return 0


def run_capture_live(endpoint: str, *,
                     width: int, height: int, fps: int,
                     depth_fast: bool = True,
                     use_gyro: bool = True,
                     recalibrate_bias: bool = False,
                     use_camera_calib: bool = False,
                     tof_sim: bool = False,
                     model: str | None = None) -> int:
    """Live OAK-D capture: device -> bridge -> IPC."""
    from imu_camera.modules.pipeline import TOF_W, TOF_H, build_live_frontend

    # ToF sim: depth runs at the SOURCE width x height (SGM), but the published
    # frames/depth + the rings + the calib bundle are the 54x42 ToF grid.
    pub_w, pub_h = (TOF_W, TOF_H) if tof_sim else (width, height)

    rings = RingRegistry().create_all(default_capture_specs(
        endpoint=endpoint, width=pub_w, height=pub_h))
    # Live: the IPC server is non-blocking (drop-oldest on stall) so a slow
    # downstream subscriber never stalls the OAK-D producer (firmware watchdog
    # ~1.5 s). The local front-end stays FIFO (`latest_only=False`): coalescing
    # imucam.sample / frame.depth here would drop frames BEFORE the bridge,
    # breaking VIO (the gyro continuity required by PreintegratePrior and the KLT
    # continuity required by TrackFeatures). Backpressure belongs at the IPC
    # boundary, not at the VIO inputs.
    server = IPCPubSub(endpoint, role="server",
                       retain_topics={_CALIB_TOPIC, _CALIB_STEREO_TOPIC},
                       blocking=False)
    local = LocalPubSub()
    # ToF: drop cam.sync from the bridge -- it carries the source-res pair, which
    # would not fit the 54x42 rings, and no IPC consumer reads it (see the replay
    # path for the full rationale).
    data_topics = ([t for t in _DATA_TOPICS if t != topics.CAM_SYNC]
                   if tof_sim else _DATA_TOPICS)
    pub = IPCPublisher(local, server, rings, data_topics, endpoint=endpoint)
    pub.start()

    try:
        device, cam_module, imu_module, cal = build_live_frontend(
            bus=local, width=width, height=height, fps=fps,
            use_gyro=use_gyro, depth_fast=depth_fast,
            recalibrate_bias=recalibrate_bias,
            use_camera_calib=use_camera_calib, latest_only=False,
            tof_sim=tof_sim, model=model)
    except Exception as e:                                         # noqa: BLE001
        LOG.error("capture: live build failed: %s", e)
        pub.stop()
        server.close()
        rings.unlink()
        rings.close()
        return 1

    bundle = _build_calib_bundle_live(cal)
    if tof_sim:
        bundle = _scale_bundle_to_tof(bundle, src_w=width, src_h=height)
    server.publish(_CALIB_TOPIC, bundle)
    # Sibling retained FULL stereo calib for the UI's Epipolar window (additive;
    # the oracle never reads it). Built from the SAME live StereoCalib the depth
    # path uses, at native source resolution (the rectifiers run on the raw pair).
    server.publish(_CALIB_STEREO_TOPIC, _build_calib_stereo(cal.calib))
    LOG.info("capture[%s] live src=%dx%d pub=%dx%d@%d depth_fast=%s%s",
             endpoint, width, height, pub_w, pub_h, fps, depth_fast,
             " (vl53l9cx ToF sim)" if tof_sim else "")

    stop = [False]
    def _on_sigterm(_signo, _frame):
        stop[0] = True
    # SIGINT (Ctrl-C) and SIGTERM both request the SAME clean stop. Handling
    # SIGINT here (vs the default KeyboardInterrupt) is CRITICAL on the device
    # path: a raw Ctrl-C traceback could abort the finally mid-way through
    # device.release(), tripping the OAK-D firmware watchdog (the crash we hit
    # before). One clean stop -> orderly device close.
    signal.signal(signal.SIGTERM, _on_sigterm)
    signal.signal(signal.SIGINT, _on_sigterm)

    imu_module.start()
    cam_module.start()
    try:
        # Poll until the read thread exits, with the frame-flow watchdog
        # (heartbeat + stall/recovery logging). A frozen OAK leaves the worker
        # alive-but-starved, so without this the loop spins silently and a device
        # freeze leaves no trace -- the operator's "runs a bit then freezes,
        # console says nothing". See _poll_with_watchdog.
        _poll_with_watchdog(stop, cam_module, imu_module)
    finally:
        # Close the device FIRST: the OAK-D firmware watchdog is only ~1.5s and
        # tearing down the bridge / IPC before release() risks tripping it.
        cam_module.stop()
        imu_module.stop()
        try:
            device.release()
        except Exception:                                          # noqa: BLE001
            pass
        # Same as the replay path: the front-end threads already forward END
        # (read_cam on cam.sync, ImuCamWorker to its downstream) on disconnect;
        # the bridge mirrors them.
        time.sleep(0.3)
        pub.stop()
        server.close()
        rings.unlink()
        rings.close()
    return 0


# --------------------------------------------------------------------------- #
def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    # Surface uncaught thread exceptions (otherwise a crashed module drain
    # silently leaves the process alive with no published data).
    def _excepthook(args):
        LOG.error("THREAD CRASH in %s: %s: %s", args.thread.name,
                  args.exc_type.__name__, args.exc_value, exc_info=(
                      args.exc_type, args.exc_value, args.exc_traceback))
    threading.excepthook = _excepthook
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default=DEFAULT_ENDPOINT,
                    help=f"IPCPubSub endpoint name (default: {DEFAULT_ENDPOINT!r})")
    ap.add_argument("--live", action="store_true",
                    help="open the OAK-D instead of replaying a session")
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s",
                    help="session directory (replay mode)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--max-frames", type=int, default=0,
                    help="cap frames in replay (0 = all)")
    ap.add_argument("--depth-fast", action="store_true", default=True,
                    help="half-res SGM preset (faster)")
    ap.add_argument("--no-gyro", action="store_true",
                    help="live: disable IMU gyro use in the calibration bundle")
    ap.add_argument("--recalibrate-bias", action="store_true",
                    help="live: ignore the cached gyro bias and re-measure it")
    ap.add_argument("--use-camera-calib", action="store_true",
                    help="live: apply the operator's SAVED per-device stereo calib "
                         "(from the wizard) instead of the FACTORY calib. Default "
                         "OFF -- factory is the trusted metrology reference; this "
                         "flag opts into the stored user calib if one exists.")
    ap.add_argument("--vl53l9cx", action="store_true",
                    help="simulate a VL53L9CX-class ToF camera: compute depth at "
                         "the source resolution then downsample gray + depth to "
                         "54x42 (accurate per-pixel ToF depth + intensity + IMU)")
    ap.add_argument("--model", default=None,
                    help="live: select which OAK device to open when several are "
                         "connected, by product-name substring (e.g. 'lite') or "
                         "deviceId. Default: the single connected device. Device "
                         "capabilities (IMU presence, mono resolution) are always "
                         "auto-detected from the selected device.")
    args = ap.parse_args()

    if args.live:
        return run_capture_live(
            endpoint=args.endpoint, width=args.width, height=args.height,
            fps=args.fps, depth_fast=args.depth_fast,
            use_gyro=not args.no_gyro,
            recalibrate_bias=args.recalibrate_bias,
            use_camera_calib=args.use_camera_calib,
            tof_sim=args.vl53l9cx, model=args.model)
    return run_capture_replay(
        session=Path(args.session), endpoint=args.endpoint,
        width=args.width, height=args.height,
        max_frames=args.max_frames, depth_fast=args.depth_fast,
        tof_sim=args.vl53l9cx)


if __name__ == "__main__":
    # Same os._exit pattern as the other split process mains -- prevent any
    # lingering non-daemon thread (depthai background thread, numba pool, etc.)
    # from holding the process past the launcher's 10 s SIGTERM deadline.
    import os as _os
    _rc = main()
    LOG.info("capture: main returned, calling os._exit(%d)", int(_rc))
    logging.shutdown()
    _os.sys.stdout.flush()
    _os.sys.stderr.flush()
    _os._exit(int(_rc))
