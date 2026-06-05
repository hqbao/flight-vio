#!/usr/bin/env python3
"""Entry point — launch the 3D pose viewer for the *baseline* (DepthAI) VIO.

Standalone to the ``baseline/`` pipeline: it wires DepthAI's built-in Basalt
backends (:class:`baseline.depthai_vio.OakBasaltVioSource`,
:class:`baseline.depthai_slam.OakBasaltSlamSource`) into the baseline copy of
the Qt 3D viewer (:mod:`oakd.ui`). It shares nothing with ``ours/`` — our
from-scratch VIO backends live in ``ours/tools/view_pose3d.py``.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# allow running directly without `pip install -e .`
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from PyQt6.QtWidgets import QApplication       # noqa: E402

from oakd.pose import PoseHistory              # noqa: E402
from oakd.sources import FakePoseSource        # noqa: E402
from oakd.ui.mainwindow import MainWindow      # noqa: E402


def _build_source(name: str, args):
    name = name.lower()
    if name == "fake":
        return FakePoseSource(rate_hz=100.0, radius_m=3.0, period_s=12.0)
    if name == "oak":
        from baseline.depthai_vio import OakBasaltVioSource
        return OakBasaltVioSource()
    if name == "slam":
        from baseline.depthai_slam import OakBasaltSlamSource
        return OakBasaltSlamSource()
    raise SystemExit(f"unknown --source '{name}' (expected: fake|oak|slam)")


def main() -> int:
    ap = argparse.ArgumentParser(description="OAK-D 3D pose viewer (baseline)")
    ap.add_argument("--source", default="fake",
                    choices=("fake", "oak", "slam"),
                    help="pose provider (oak = Basalt VIO; slam = Basalt SLAM)")
    args = ap.parse_args()

    history = PoseHistory(capacity=8192)
    source = _build_source(args.source, args)

    app = QApplication(sys.argv)
    win = MainWindow(history, source, source_name=args.source)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
