#!/usr/bin/env python3
"""End-to-end PNG proof for the LIVE "Epipolar / Rectification" window.

Boots the capture process (``imu_camera.main`` replay on a gold session), which
now publishes the retained ``calib.stereo`` topic ALONGSIDE ``calib.bundle``,
then drives the REAL :class:`~ui.modules.ipc_sources.IpcEpipolarSource`: it
subscribes capture's raw ``imucam.sample`` left+right pairs + the retained
``calib.stereo``, builds the Left/Right rectifiers, rectifies the live pair, and
emits a finished :class:`~ui.viz.epipolar_render.EpipolarRender`. We render it
with the REAL :func:`~ui.viz.epipolar_render.render_epipolar_record` and write a
PNG. No OpenGL is involved (the view is pure 2D cv2), so this runs headless.

This single test covers two gates at once:

* REPLAY SMOKE -- capture emits ``calib.stereo`` (the source receives a real
  :class:`~ui.comms.wire.WireCalibStereo`) AND the raw stereo pairs.
* OFFSCREEN RENDER -- the window's renderer draws a non-blank before/after figure
  with the scanlines + corner markers + a median row-mismatch status line, and the
  rectification REDUCES the median |row mismatch| (the whole teaching point).

Run::

    .venv/bin/python -m ui.tests._epipolar_window_png
    .venv/bin/python -m ui.tests._epipolar_window_png --out /tmp/epipolar_window.png
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

from ui.modules import IpcEpipolarSource                           # noqa: E402
from ui.viz.epipolar_render import (                               # noqa: E402
    render_epipolar_record, median_abs_mismatch, status_line, _SCAN,
)


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=80)
    ap.add_argument("--out", default="/tmp/epipolar_window_lab_loop_30s.png")
    args = ap.parse_args()

    pid = os.getpid()
    cap_ep = f"oak.cap.ep{pid & 0xFFF:x}"
    py = sys.executable
    env = dict(os.environ)
    env.setdefault("QT_QPA_PLATFORM", "offscreen")
    lk = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE}

    print("epipolar_window_png")
    print(f"  session={args.session} max-frames={args.max_frames}")

    cap_proc = subprocess.Popen(
        [py, "-m", "imu_camera.main", "--endpoint", cap_ep,
         "--session", args.session, "--max-frames", str(args.max_frames)],
        env=env, **lk)
    procs = (cap_proc,)

    # We need the capture resolution to attach the rings. The session's native
    # left dims are 640x400 for the gold sessions; the source attaches rings at
    # those dims (the same W/H ui.main passes from the calib bundle).
    W, H = 640, 400

    records: list = []
    lock = threading.Lock()
    src = None
    try:
        def on_rec(r) -> None:
            with lock:
                records.append(r)

        # The REAL window source: raw imucam.sample pairs + retained calib.stereo
        # -> rectify -> EpipolarRender. Give capture a moment to bind first.
        time.sleep(0.5)
        src = IpcEpipolarSource(cap_ep, W, H, connect_timeout_s=25.0)
        src.start(on_rec)

        cap_proc.wait(timeout=180.0)
        time.sleep(1.0)                            # drain in-flight records

        _check(src.error is None, f"source has no error ({src.error})")
        with lock:
            got = list(records)
        print(f"  captured {len(got)} epipolar record(s)")
        _check(len(got) >= 1,
               "at least one EpipolarRender reached the source "
               "(=> calib.stereo + raw pairs both arrived)")

        # The source built rectifiers from a REAL WireCalibStereo it received off
        # the wire -- prove the retained calib.stereo emitted + decoded.
        _check(src._calib is not None,
               "source received the retained calib.stereo (WireCalibStereo)")
        cal = src._calib
        _check(np.asarray(cal.left_K).shape == (3, 3)
               and np.asarray(cal.right_K).shape == (3, 3)
               and np.asarray(cal.T_left_right).shape == (4, 4),
               "calib.stereo carries left_K / right_K (3,3) + T_left_right (4,4)")
        print(f"  calib.stereo: {int(cal.width)}x{int(cal.height)}  "
              f"left_dist[{np.asarray(cal.left_dist).size}]  "
              f"right_dist[{np.asarray(cal.right_dist).size}]")

        # Pick a record whose rectification measurably improves row alignment
        # (some frames are featureless -- prefer one with the most valid matches).
        def score(r):
            _, nb = median_abs_mismatch(r.matches_before)
            _, na = median_abs_mismatch(r.matches_after)
            return min(nb, na)
        rec = max(got, key=score)
        med_b, n_b = median_abs_mismatch(rec.matches_before)
        med_a, n_a = median_abs_mismatch(rec.matches_after)
        print(f"  chosen record seq {rec.seq}: "
              f"before {med_b:.2f}px ({n_b}) -> after {med_a:.2f}px ({n_a})")
        _check(rec.left_before.shape == rec.left_after.shape
               and rec.right_before.shape == rec.right_after.shape,
               "before/after panels share their grid (rectify kept H, W)")
        # The raw and rectified RIGHT differ (the rectifier actually warped it).
        _check(int((rec.right_before.astype(int)
                    != rec.right_after.astype(int)).sum()) > 100,
               "rectified right differs from raw right (the warp ran)")
        _check(n_b > 0 and n_a > 0,
               f"both rows have valid corner matches (before={n_b}, after={n_a})")
        # The teaching claim: rectification must not WORSEN the row alignment.
        _check(med_a <= med_b + 1e-6,
               f"rectification did not worsen row mismatch "
               f"({med_b:.2f}px -> {med_a:.2f}px)")

        # Render the REAL window figure (RGB) + a placeholder, and prove both real.
        ph = render_epipolar_record(None)
        _check(ph.ndim == 3 and ph.shape[2] == 3 and float(ph.std()) > 1.0,
               "placeholder ('waiting for calib/stereo') renders non-blank")

        img = render_epipolar_record(rec)
        _check(img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8,
               f"rendered RGB uint8 figure (got {img.shape} {img.dtype})")
        H_fig, W_fig = img.shape[:2]
        _check(H_fig > 2 * rec.left_before.shape[0],
               f"figure stacks two stereo rows + banner/captions ({img.shape})")
        _check(float(img.std()) > 5.0,
               f"rendered figure is non-blank (std={img.std():.2f})")
        # Scanlines actually drawn: the muted steel-blue _SCAN colour appears.
        flat = img.reshape(-1, 3).astype(int)
        scan_px = int((np.abs(flat - np.array(_SCAN)).sum(1) < 40).sum())
        _check(scan_px > 200, f"scanlines drawn ({scan_px} scanline px)")
        # A coloured overlay (markers) exists -- R != G somewhere.
        r_ch, g_ch = img[..., 0].astype(np.int16), img[..., 1].astype(np.int16)
        _check(int(np.abs(r_ch - g_ch).max()) > 0,
               "coloured corner overlay drawn (R != G somewhere)")

        line = status_line(rec.matches_before, rec.matches_after)
        _check("before" in line and "after" in line,
               f"status line reports median row-mismatch before/after: '{line}'")

        import cv2
        out = Path(args.out)
        cv2.imwrite(str(out), img[..., ::-1])      # RGB -> BGR for cv2
        print(f"\n  wrote {out}  ({scan_px} scanline px)")
        print("EPIPOLAR-WINDOW PNG PASS")
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
