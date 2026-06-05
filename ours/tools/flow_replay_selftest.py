#!/usr/bin/env python3
"""Offline integration test for the unified VIO graph driven by ``ours.app``.

After the front-end unification there is no ``*_selftest`` that exercises the FULL
``ours.app`` replay graph -- the camera-reader + imu-reader front-end feeding
depth + odometry + backend + slam + ui. The app-graph parity used to be checked
only by a manual ``python -m ours.app`` run; this test makes it part of the
offline sweep so a topology regression is caught automatically.

It runs :func:`ours.app.run_replay` over a gold session for a bounded number of
frames and asserts the contract the manual run checked, plus basic sanity:

* one ``pose.odom`` per processed frame (the front-end delivered every frame and
  odometry produced a pose for each),
* the back-end emitted refined poses, at most one per keyframe
  (``ceil(n_frames / kf_every)``; the BA window warms up on the first keyframes
  so the count may be a touch lower),
* seqs are dense ``0..n-1`` and every recorded position is finite (the two-input
  join did not drop, reorder or corrupt frames),
* the graph drained on its own (``run_replay`` returned, no timeout).

Run::

    python -m ours.tools.flow_replay_selftest
    python -m ours.tools.flow_replay_selftest --session sessions/gold/lab_loop_30s
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.app import run_replay                                  # noqa: E402
from ours.lib.io.reader import SessionReader                      # noqa: E402


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=60)
    ap.add_argument("--kf-every", type=int, default=5)
    args = ap.parse_args()

    print("flow_replay_selftest")

    reader = SessionReader(Path(args.session))
    n_frames = (len(reader) if args.max_frames <= 0
                else min(args.max_frames, len(reader)))

    ui, _, elapsed = run_replay(
        args.session, kf_every=args.kf_every, use_gyro=True,
        depth_fast=True, max_frames=args.max_frames)

    print(f"  frames={n_frames} odom={len(ui.odom)} refined={len(ui.refined)} "
          f"elapsed={elapsed:.1f}s")

    _check(len(ui.odom) == n_frames,
           f"one pose.odom per frame ({len(ui.odom)}/{n_frames})")

    max_refined = math.ceil(n_frames / args.kf_every)
    _check(0 < len(ui.refined) <= max_refined,
           f"back-end refined poses, <= one per keyframe "
           f"({len(ui.refined)}/{max_refined})")

    seqs = sorted(ui.odom.keys())
    _check(seqs == list(range(n_frames)),
           f"pose.odom seqs are dense 0..{n_frames - 1} (no drop/reorder)")

    _check(all(np.all(np.isfinite(np.asarray(p))) for p in ui.odom.values()),
           "every pose.odom position is finite")

    print("\nALL FLOW REPLAY SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
