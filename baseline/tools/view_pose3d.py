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
import sys
from pathlib import Path

# allow running directly without `pip install -e .`
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from PyQt6.QtWidgets import QApplication       # noqa: E402

from baseline.pose import PoseHistory                   # noqa: E402
from baseline.sources import FakePoseSource             # noqa: E402
from baseline.ui.mainwindow import MainWindow           # noqa: E402


def _build_source(name: str, args):
    name = name.lower()
    if name == "fake":
        return FakePoseSource(rate_hz=100.0, radius_m=3.0, period_s=12.0)
    # Camera resolution forwarded to Basalt via depthai's requestOutput(); --fps=0
    # keeps each source's own sensor-fps default (VIO 60, SLAM 20).
    res = {"width": args.width, "height": args.height}
    if args.fps > 0:
        res["fps"] = args.fps
    if name == "oak":
        from baseline.sources.basalt_vio import OakBasaltVioSource
        return OakBasaltVioSource(**res)
    if name == "slam":
        from baseline.sources.basalt_slam import OakBasaltSlamSource
        return OakBasaltSlamSource(**res)
    raise SystemExit(f"unknown --source '{name}' (expected: fake|oak|slam)")


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
    args = ap.parse_args()

    history = PoseHistory(capacity=8192)
    source = _build_source(args.source, args)

    app = QApplication(sys.argv)
    win = MainWindow(history, source, source_name=args.source)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
