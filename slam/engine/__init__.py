"""``slam.engine`` -- the in-process runner for the heavy keyframe optimiser.

SLAM is one process that runs its loop-closure + pose-graph solve IN-PROCESS:

* :class:`InProcessEngine` -- synchronous, deterministic, byte-identical replay
  output. The live path runs it on SLAM's own worker thread; the offline / oracle
  path runs it on the deterministic FIFO inbox. There is no worker-child engine --
  the live latest-only inbox drops stale keyframes if a heavy PGO briefly blocks,
  so the map stays current without a separate process.

The engine wraps the shared loop-closure SLAM library (``sky.slam.slam``)
and knows nothing about flows or the bus -- it is pure machinery (``lib``),
called by the module steps.
"""
from __future__ import annotations

import numpy as np

from .base import Engine, SlamResult
from .inprocess import InProcessEngine
from .steps import slam_step, slam_overlay

__all__ = ["Engine", "SlamResult", "InProcessEngine", "make_slam_engine"]


def make_slam_engine(K: np.ndarray, cfg, *,
                     capture_loops: bool = False) -> Engine:
    """Build the in-process loop-closure SLAM engine.

    ``capture_loops`` (LIVE-only) makes the engine capture each verified
    candidate's match funnel so the SLAM module can publish ``slam.loop`` for the
    UI's loop-closure view. It is wired ON only on the live publish-map path; the
    OFFLINE / oracle path leaves it False, so the map runs the byte-frozen
    ``verify`` (no funnel work) and the deterministic ``loop.correction`` scoring
    stays bit-identical.
    """
    from sky.slam.slam import SlamMap
    return InProcessEngine(lambda: SlamMap(K, cfg, capture_loops=capture_loops),
                           slam_step, slam_overlay)
