"""``slam.mathlib.engine`` -- swappable runners for the heavy keyframe optimisers.

A flow picks how its optimiser runs with one ``worker`` flag:

* ``worker=False`` (default, OFFLINE) -> :class:`InProcessEngine` -- synchronous,
  deterministic, byte-identical replay output.
* ``worker=True`` (LIVE) -> :class:`~slam.mathlib.engine.subprocess.SubprocessEngine`
  -- the solve runs in a separate process so it never holds the camera read loop's
  GIL (the fast-push undershoot fix).

The engines wrap the existing algorithm libraries (``vio.mathlib.backend.windowed`` /
``slam.mathlib.loop.slam``) and know nothing about flows or the bus -- they are pure
machinery (``mathlib``), called by the module steps.
"""
from __future__ import annotations

import numpy as np

from .base import Engine, SlamResult
from .inprocess import InProcessEngine
from .steps import ba_step, slam_step, ba_overlay, slam_overlay
from .subprocess import SubprocessEngine, _ba_worker_main, _slam_worker_main

__all__ = ["Engine", "SlamResult", "InProcessEngine", "SubprocessEngine",
           "make_ba_engine", "make_slam_engine"]


def make_ba_engine(K: np.ndarray, cfg, *, worker: bool = False) -> Engine:
    """Build a windowed-BA engine (in-process unless ``worker``)."""
    if worker:
        return SubprocessEngine(_ba_worker_main, K, cfg)
    from ..backend.windowed import WindowedBAMap
    return InProcessEngine(lambda: WindowedBAMap(K, cfg), ba_step, ba_overlay)


def make_slam_engine(K: np.ndarray, cfg, *, worker: bool = False) -> Engine:
    """Build a loop-closure SLAM engine (in-process unless ``worker``)."""
    if worker:
        return SubprocessEngine(_slam_worker_main, K, cfg)
    from ..loop.slam import SlamMap
    return InProcessEngine(lambda: SlamMap(K, cfg), slam_step, slam_overlay)
