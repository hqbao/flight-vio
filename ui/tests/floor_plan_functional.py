"""Functional floor-plan check on a GOLD replay session (saves a verifiable PNG).

Boots imu_camera(replay) + vio over IPC on a gold session, attaches an
:class:`~ui.modules.ipc_sources.IpcFloorPlanSource` to VIO's ``keyframe`` feed,
lets the replay drain into its keyframe accumulator, then runs its OFF-thread
``_build`` ONCE -- timing the rebuild. The resulting 2D occupancy raster + camera
path is drawn (the path as a polyline, the latest pose as a dot) and SAVED TO A
PNG with pure numpy + cv2 (NO GL / no display), so the result can be read back and
visually judged offscreen -- which the GL map viewers could not be on this Mac.

Run (the harness drives it; defaults to lab_loop_30s @ kf-every 5)::

    QT_QPA_PLATFORM=offscreen .venv/bin/python -m ui.tests.floor_plan_functional \\
        --session sessions/gold/lab_loop_30s --out /tmp/floor_plan_lab.png
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time

import numpy as np

from ui.comms import IPCPubSub


def _await_calib_bundle(endpoint: str, timeout_s: float):
    """Wait for the retained ``calib.bundle`` on an endpoint (W/H/K)."""
    bundle = [None]
    got = threading.Event()

    def on_calib(wm) -> None:
        bundle[0] = wm
        got.set()

    client = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    client.subscribe("calib.bundle", on_calib)
    client.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(f"no calib.bundle from {endpoint!r}")
    finally:
        client.stop()
    return bundle[0]


def _spawn_cap_vio(session: str, max_frames: int, kf_every: int):
    """Boot vio then imu_camera(replay) over IPC; return (procs, cap_ep, vio_ep)."""
    pid = os.getpid()
    cap_ep = f"oak.cap.fp{pid & 0xFFF:x}"
    vio_ep = f"oak.vio.fp{pid & 0xFFF:x}"
    py = sys.executable
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    lk = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE}
    vio = subprocess.Popen(
        [py, "-m", "vio.main", "--capture-endpoint", cap_ep,
         "--endpoint", vio_ep, "--kf-every", str(kf_every)], env=env, **lk)
    time.sleep(0.3)
    cap = subprocess.Popen(
        [py, "-m", "imu_camera.main", "--endpoint", cap_ep,
         "--session", session, "--max-frames", str(max_frames)], env=env, **lk)
    return (cap, vio), cap_ep, vio_ep


def _draw_path_on_raster(rgb: np.ndarray, path_px: np.ndarray) -> np.ndarray:
    """Overlay the camera path (polyline + latest dot) on the raster with cv2.

    Pure cv2 (no GL): draws the path the window would draw, so the SAVED PNG shows
    EXACTLY what the user sees (raster + path). ``path_px`` is ``(M,2)`` fractional
    (col,row) on the raster grid; clamp + round to integer pixels.
    """
    import cv2
    h, w = rgb.shape[:2]
    # cv2 wants BGR; convert once for drawing then back.
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if len(path_px) >= 1:
        px = np.clip(np.round(path_px).astype(np.int32), 0,
                     [w - 1, h - 1])                       # (M,2) col,row
        if len(px) >= 2:
            cv2.polylines(bgr, [px.reshape(-1, 1, 2)], isClosed=False,
                          color=(80, 255, 80), thickness=1, lineType=cv2.LINE_AA)
        # Latest pose: an amber dot.
        cv2.circle(bgr, (int(px[-1, 0]), int(px[-1, 1])), 3,
                   (0, 200, 255), -1, lineType=cv2.LINE_AA)
    return bgr


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--out", default="/tmp/floor_plan.png")
    ap.add_argument("--max-frames", type=int, default=600)
    ap.add_argument("--kf-every", type=int, default=5)
    ap.add_argument("--upscale", type=int, default=4,
                    help="integer nearest-neighbour upscale of the saved PNG")
    args = ap.parse_args()

    import cv2
    from ui.modules import IpcFloorPlanSource

    procs, cap_ep, vio_ep = _spawn_cap_vio(
        args.session, args.max_frames, args.kf_every)
    try:
        bundle = _await_calib_bundle(vio_ep, timeout_s=30.0)
        W, H, K = int(bundle.width), int(bundle.height), bundle.K
        print(f"vio ready: {W}x{H}, session={args.session}")

        src = IpcFloorPlanSource(vio_ep, K, width=W, height=H,
                                 connect_timeout_s=20.0)
        if not src._attach_or_fail():
            print(f"FAIL: attach: {src.error}")
            return 1
        client = src._make_keyframe_client()
        client.start()
        try:
            # Let the replay drain keyframes into the accumulator.
            deadline = time.monotonic() + 40.0
            last = -1
            while time.monotonic() < deadline:
                n = len(src._kf_depth)
                if n != last:
                    last = n
                if procs[0].poll() is not None and n > 0:
                    # capture finished AND we have keyframes -> give the last few
                    # a moment to land, then build.
                    time.sleep(1.0)
                    break
                time.sleep(0.3)
            n_kf = len(src._kf_depth)
            print(f"accumulated {n_kf} keyframes")
            if n_kf == 0:
                print("FAIL: no keyframes accumulated")
                return 1

            # Time the rebuild (the 2D histogram build, OFF the GUI thread).
            t0 = time.perf_counter()
            rgb, path_px, cams, extent = src._build()
            dt = time.perf_counter() - t0
            x0, x1, z0, z1 = extent.world_extent()
            print(f"rebuild: {dt*1000:.1f} ms | raster {rgb.shape[1]}x"
                  f"{rgb.shape[0]} cells | world {x1-x0:.1f}m x {z1-z0:.1f}m | "
                  f"{len(cams)} cams")
        finally:
            src.stop()
    finally:
        for p in procs:
            try:
                p.terminate()
            except Exception:                              # noqa: BLE001
                pass
        for p in procs:
            try:
                p.wait(timeout=5.0)
            except Exception:                              # noqa: BLE001
                p.kill()

    # Draw the path + save the PNG (pure cv2, no GL). Upscale (nearest) so a small
    # cell grid is legible when read back.
    bgr = _draw_path_on_raster(rgb, path_px)
    if args.upscale > 1:
        bgr = cv2.resize(bgr, (bgr.shape[1] * args.upscale,
                               bgr.shape[0] * args.upscale),
                         interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(args.out, bgr)
    print(f"SAVED: {args.out} ({bgr.shape[1]}x{bgr.shape[0]} px)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
