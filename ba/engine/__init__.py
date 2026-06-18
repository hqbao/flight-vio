"""``ba.engine`` -- the in-process runners for the heavy keyframe optimisers.

This is the SINGLE home of the engine runners: it was a verbatim COPY of the old
``vio.engine`` while the windowed-BA backend was being extracted into the ``ba``
process, and ``vio/engine`` has since been DELETED (the backend lives only here
now). The ``ba`` process runs its solve IN-PROCESS (it IS its own process), so the
only engine is :class:`InProcessEngine`:

* :class:`InProcessEngine` -- synchronous, deterministic, byte-identical replay
  output. Both ``make_ba_engine`` and ``make_vi_engine`` build it.

The engines wrap the shared algorithm libraries (``sky.backend.windowed``
/ ``sky.vio.window``) and know nothing about flows or the bus --
they are pure machinery (``lib``), called by the flow tasks.
"""
from __future__ import annotations

import numpy as np

from .base import Engine
from .inprocess import InProcessEngine
from .steps import ba_step, vio_step, ba_overlay, vio_overlay
from .ba_capture import ba_step_capture, ba_window_overlay

__all__ = ["Engine", "InProcessEngine", "make_ba_engine", "make_vi_engine"]


def make_ba_engine(K: np.ndarray, cfg, *,
                   capture_window: bool = False) -> Engine:
    """Build the in-process windowed-BA engine.

    ``capture_window`` (opt-in, ``--ba-window``) selects the RICHER capture step +
    overlay (:func:`~ba.engine.ba_capture.ba_step_capture` /
    :func:`~ba.engine.ba_capture.ba_window_overlay`): the SAME frozen
    ``run_ba`` solve plus a read-only PRE/POST snapshot for the UI's "BA Window"
    visualiser. Default OFF -> the historical ``ba_step`` / ``ba_overlay`` path,
    byte-identical to before (the oracle relies on this).
    """
    step = ba_step_capture if capture_window else ba_step
    overlay = ba_window_overlay if capture_window else ba_overlay
    from sky.backend.windowed import WindowedBAMap
    return InProcessEngine(lambda: WindowedBAMap(K, cfg), step, overlay)


def make_vi_engine(K: np.ndarray, cfg) -> Engine:
    """Build the in-process tight-coupled VIO engine.

    Symmetric with :func:`make_ba_engine` but wraps
    :class:`sky.vio.window.WindowedVIOMap` (the joint visual +
    IMU window optimiser) instead of the visual-only ``WindowedBAMap``. The live
    path feeds each keyframe's raw IMU segment via the snapshot, so the map is
    built with no stored IMU stream (``cfg=cfg`` only).
    """
    from sky.vio.window import WindowedVIOMap
    return InProcessEngine(lambda: WindowedVIOMap(K, cfg=cfg),
                           vio_step, vio_overlay)
