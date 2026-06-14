#!/usr/bin/env python3
"""Entry point — launch the 3D pose viewer for the *baseline* (DepthAI) VIO.

Standalone to the ``baseline/`` pipeline: it wires DepthAI's built-in Basalt
backends (:class:`baseline.sources.basalt_vio.OakBasaltVioSource`,
:class:`baseline.sources.basalt_slam.OakBasaltSlamSource`) into the baseline copy
of the Qt 3D viewer (:mod:`baseline.ui`). It shares nothing with the
from-scratch pipeline — that now runs as the 5-project split via ``./run.sh``
(the launcher). This tool is purely the DepthAI/Basalt reference to compare against.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

# allow running directly without `pip install -e .`
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from baseline.pose import PoseHistory                   # noqa: E402
from baseline.sources import FakePoseSource             # noqa: E402
# PyQt6 / baseline.ui are imported LAZILY inside main()'s UI branch, so the
# --no-ui (headless / FC-only) path runs with NO Qt installed.


def _build_source(name: str, args):
    name = name.lower()
    if name == "fake":
        return FakePoseSource(rate_hz=100.0, radius_m=3.0, period_s=12.0)
    # Camera resolution forwarded to Basalt via depthai's requestOutput(); --fps=0
    # keeps each source's own sensor-fps default (VIO 60, SLAM 20).
    res = {"width": args.width, "height": args.height}
    if args.fps > 0:
        res["fps"] = args.fps
    # Opt-in Basalt override: only forwarded when --vio-config is given, so the
    # default stays stock auto-config for both the VIO and SLAM sources.
    if name in ("oak", "slam") and args.vio_config:
        res["vio_config_path"] = args.vio_config
    if name == "oak":
        from baseline.sources.basalt_vio import OakBasaltVioSource
        return OakBasaltVioSource(**res)
    if name == "slam":
        from baseline.sources.basalt_slam import OakBasaltSlamSource
        return OakBasaltSlamSource(**res)
    raise SystemExit(f"unknown --source '{name}' (expected: fake|oak|slam)")


def _run_headless(source, source_name: str) -> int:
    """Run the pose source with NO UI — the FC-only / Pi deployment path.

    Starts the source, forwards every pose to ``_on_pose`` (the hook where a
    MAVLink ``VISION_POSITION_ESTIMATE`` send to the flight controller goes),
    prints a throttled status line, and tears the device down CLEANLY on
    Ctrl+C/SIGTERM (``source.stop()`` waits for the depthai pipeline to close,
    same as the UI fix — so a headless quit never crashes the OAK-D firmware).
    """
    stop = {"flag": False}

    def _sig(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    latest = {"pose": None}
    count = {"n": 0}

    def _on_pose(p) -> None:
        # === FC OUTPUT HOOK ===
        # This is the single place a real deployment forwards the pose to the
        # flight controller (e.g. MAVLink VISION_POSITION_ESTIMATE). For now we
        # just keep the latest sample for the status line below.
        latest["pose"] = p
        count["n"] += 1

    print(f"baseline headless: source={source_name}  (Ctrl+C to stop)")
    print("  pose -> stdout; wire the FC forward in _on_pose().")
    source.start(_on_pose)

    last_print = 0.0
    try:
        while not stop["flag"]:
            if source.error:
                print(f"[{source_name}] error: {source.error}", file=sys.stderr)
                return 1
            if not source.is_running():
                print(f"[{source_name}] source stopped (stream ended).")
                break
            now = time.monotonic()
            if now - last_print >= 0.5:
                last_print = now
                p = latest["pose"]
                if p is not None:
                    # RAW pose for the FC: position natively NED (X=North,
                    # Y=East, Z=Down) + the NED-world body-attitude quaternion
                    # (w,x,y,z). We send position + quaternion as-is and let the
                    # FC derive heading/euler itself.
                    x, y, z = p.pos_ned
                    qw, qx, qy, qz = p.quat_wxyz
                    print(f"  pos NED=(N{x:+.3f} E{y:+.3f} D{z:+.3f}) m"
                          f"   quat wxyz=({qw:+.4f} {qx:+.4f} {qy:+.4f} {qz:+.4f})"
                          f"   fps={source.fps:5.1f}   n={count['n']}")
            time.sleep(0.05)
    finally:
        # Clean device teardown (waits for the depthai pipeline to close) — the
        # headless counterpart of the MainWindow quit fix.
        source.stop(timeout=10.0)
        print("baseline headless: stopped cleanly.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="OAK-D 3D pose viewer (baseline)")
    ap.add_argument("--source", default="fake",
                    choices=("fake", "oak", "slam"),
                    help="pose provider (oak = Basalt VIO; slam = Basalt SLAM)")
    ap.add_argument("--width", type=int, default=640,
                    help="camera output width fed to Basalt (default 640)")
    ap.add_argument("--height", type=int, default=400,
                    help="camera output height fed to Basalt (default 400)")
    ap.add_argument("--fps", type=int, default=0,
                    help="sensor fps (0 = source default: VIO 60, SLAM 20)")
    ap.add_argument("--vio-config", type=str, default=None,
                    help="path to a Basalt vio_config.json fed to "
                         "BasaltVIO.setConfigPath (advanced; default = stock "
                         "auto-config)")
    ap.add_argument("--no-ui", action="store_true",
                    help="headless: run the source + stream pose to stdout (the "
                         "FC-output path), NO Qt viewer. Clean Ctrl+C teardown. "
                         "Needs no PyQt6 -- the Pi FC-only deployment path.")
    args = ap.parse_args()

    source = _build_source(args.source, args)

    if args.no_ui:
        return _run_headless(source, args.source)

    # UI path: import Qt + the viewer LAZILY so --no-ui needs no PyQt6 installed.
    from PyQt6.QtWidgets import QApplication
    from baseline.ui.mainwindow import MainWindow
    history = PoseHistory(capacity=8192)
    app = QApplication(sys.argv)
    win = MainWindow(history, source, source_name=args.source)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
