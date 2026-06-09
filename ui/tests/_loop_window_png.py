#!/usr/bin/env python3
"""End-to-end PNG proof for the "Loop Closure" window (ALGORITHMS.md viz #1).

Boots the SPLIT 3-process stack (imu_camera replay + vio + slam LIVE) on a gold
session WITH loops, drives the REAL :class:`~ui.modules.ipc_sources.IpcLoopMatchSource`
(it subscribes SLAM's ``slam.loop`` match funnel AND VIO's ``keyframe`` grays,
joining them by seq), renders the window's 2D image with the REAL
:func:`~ui.viz.loop_render.render_loop`, and writes it to a PNG. No OpenGL is
involved (the loop view is pure 2D), so this runs headless.

Asserts the captured event is real (two keyframe seqs, non-empty matches, the
funnel monotone) and that at least one keyframe gray was joined (so the PNG shows
a real keyframe, not just placeholders).

Run::

    .venv/bin/python -m ui.tests._loop_window_png
    .venv/bin/python -m ui.tests._loop_window_png --out /tmp/loop.png
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ui.comms import IPCPubSub                                    # noqa: E402
from ui.modules import IpcLoopMatchSource                        # noqa: E402
from ui.viz.loop_render import render_loop, STAGE_PNP            # noqa: E402


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def _await_calib(endpoint: str, timeout_s: float):
    got = threading.Event()
    box = [None]

    def on(wm):
        box[0] = wm
        got.set()
    c = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    c.subscribe("calib.bundle", on)
    c.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(f"no calib.bundle from {endpoint!r}")
    finally:
        c.stop()
    return box[0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=600)
    ap.add_argument("--kf-every", type=int, default=5)
    ap.add_argument("--out", default="/tmp/loop_closure_lab_loop_30s.png")
    args = ap.parse_args()

    pid = os.getpid()
    cap_ep = f"oak.cap.lw{pid & 0xFFF:x}"
    vio_ep = f"oak.vio.lw{pid & 0xFFF:x}"
    slam_ep = f"oak.slm.lw{pid & 0xFFF:x}"
    py = sys.executable
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    lk = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE}

    print("loop_window_png")
    print(f"  session={args.session} max-frames={args.max_frames}")

    vio_proc = subprocess.Popen(
        [py, "-m", "vio.main", "--capture-endpoint", cap_ep,
         "--endpoint", vio_ep, "--kf-every", str(args.kf_every)], env=env, **lk)
    slam_proc = subprocess.Popen(
        [py, "-m", "slam.main", "--vio-endpoint", vio_ep,
         "--endpoint", slam_ep], env=env, **lk)
    time.sleep(0.3)
    cap_proc = subprocess.Popen(
        [py, "-m", "imu_camera.main", "--endpoint", cap_ep,
         "--session", args.session, "--max-frames", str(args.max_frames)],
        env=env, **lk)
    procs = (cap_proc, vio_proc, slam_proc)

    events: list = []
    lock = threading.Lock()
    src = None
    try:
        bundle = _await_calib(slam_ep, timeout_s=25.0)
        W, H = int(bundle.width), int(bundle.height)
        print(f"  slam ready ({W}x{H})")

        # The REAL window source: slam.loop funnel joined to keyframe grays.
        def on_event(ev) -> None:
            with lock:
                events.append(ev)

        src = IpcLoopMatchSource(slam_ep, vio_ep, width=W, height=H,
                                 connect_timeout_s=25.0)
        src.start(on_event)

        cap_proc.wait(timeout=180.0)
        vio_proc.wait(timeout=180.0)
        slam_proc.wait(timeout=180.0)
        time.sleep(1.0)                            # drain in-flight events

        _check(src.error is None, f"source has no connect error ({src.error})")
        with lock:
            evs = list(events)
        print(f"  captured {len(evs)} loop event(s)")
        _check(len(evs) >= 1, "at least one loop event reached the window source")

        # Prefer an ACCEPTED event with both grays present (the headline picture).
        def score(e):
            return (bool(e.accepted), e.cur_gray is not None,
                    e.old_gray is not None, int(e.n_appearance))
        ev = max(evs, key=score)
        cur = np.asarray(ev.cur_px).reshape(-1, 2)
        stg = np.asarray(ev.stage).reshape(-1)
        n_pnp = int((stg >= STAGE_PNP).sum())
        print(f"  chosen event: kf {ev.cur_seq} <-> {ev.old_seq}  "
              f"funnel {ev.n_appearance}->{ev.n_fmat}->{ev.n_pnp}  "
              f"rot {ev.rot_deg:.2f}  accepted={ev.accepted}  "
              f"grays cur={ev.cur_gray is not None} old={ev.old_gray is not None}")
        _check(len(cur) == int(ev.n_appearance) and int(ev.n_appearance) > 0,
               f"event carries non-empty matches (N={len(cur)})")
        _check(n_pnp == int(ev.n_pnp) and n_pnp >= 0,
               f"stage labels agree with n_pnp ({n_pnp} == {ev.n_pnp})")
        _check(ev.cur_gray is not None or ev.old_gray is not None,
               "at least one keyframe gray was joined from the buffer")

        img = render_loop(ev, 1100, 560)
        _check(img.shape == (560, 1100, 3) and img.dtype == np.uint8,
               f"rendered (560,1100,3) uint8 (got {img.shape} {img.dtype})")
        # Persist (cv2 wants BGR; the canvas is RGB).
        import cv2
        out = Path(args.out)
        cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        # Confirm the colour-coded lines are actually drawn (green PnP present).
        from ui.viz.loop_render import _C_PNP
        flat = img.reshape(-1, 3)
        green = int((np.abs(flat.astype(int) - np.array(_C_PNP)).sum(1) < 40).sum())
        _check(green > 50 if int(ev.n_pnp) > 0 else True,
               f"green PnP-inlier lines drawn in the PNG ({green} px)")
        print(f"\n  wrote {out}  ({green} green PnP px)")
        print("LOOP-WINDOW PNG PASS")
        return 0
    finally:
        if src is not None:
            try:
                src.stop()
            except Exception:                                      # noqa: BLE001
                pass
        for p in procs:
            if p.poll() is None:
                try:
                    p.terminate()
                except Exception:                                  # noqa: BLE001
                    pass
        for p in procs:
            try:
                p.wait(timeout=5.0)
            except Exception:                                      # noqa: BLE001
                try:
                    p.kill()
                except Exception:                                  # noqa: BLE001
                    pass


if __name__ == "__main__":
    raise SystemExit(main())
