"""Engine contract: the runner for the heavy keyframe optimiser.

An *engine* owns one heavy map optimiser (loop-closure SLAM) and
exposes a tiny, uniform interface so the flow task (``SlamStep``)
never cares *how* the solve runs:

* :class:`InProcessEngine` -- runs the solve synchronously on the calling thread.
  This is the SINGLE engine: SLAM is one process that runs its solve in-process.
  The OFFLINE replay/scoring path runs it on the deterministic FIFO inbox (``submit``
  does the whole solve, ``poll`` returns its one result), and the LIVE path runs the
  same engine on SLAM's own worker thread fed by a latest-only inbox (which drops
  stale keyframes if a heavy PGO briefly blocks). The offline numbers stay
  byte-identical to the old in-thread flow.

CONTRACT (critical for offline byte-parity)
-------------------------------------------
``poll()`` is **one-shot** for the in-process engine: it returns at most the one
result stashed by the matching ``submit`` and then clears it. It is NOT
latest-wins. A warmup keyframe whose solve returns ``None`` must make ``poll``
return ``None`` -- never re-surface a previous keyframe's result -- otherwise the
back-end would emit an extra ``pose.refined`` and break the replay self-test count
(``len(refined) <= ceil(frames/kf_every)``).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class SlamResult:
    """A SLAM step's output: the rewritten keyframe poses + loop count.

    Returned by :func:`slam.engine.steps.slam_step` only on a keyframe that
    confirmed a loop (so the pose graph was optimised). ``SlamStep`` frames it
    into a :class:`~slam.comms.messages.LoopCorrection` for the bus.

    * ``kf_poses`` -- ``{keyframe seq: T_world_cam}`` after pose-graph optimise.
    * ``n_loops`` -- total confirmed loop closures so far.
    """

    kf_poses: dict[int, np.ndarray]
    n_loops: int


class Engine(ABC):
    """Runs one heavy optimiser; results are produced by ``submit`` + ``poll``."""

    @abstractmethod
    def submit(self, snapshot: Any) -> None:
        """Hand the optimiser one keyframe snapshot (non-blocking)."""

    @abstractmethod
    def poll(self) -> Any:
        """Return a ready result (:class:`SlamResult` on a loop closure) or
        ``None`` if nothing is ready."""

    @abstractmethod
    def poll_overlay(self) -> Any:
        """Return the latest MAP overlay snapshot (for the live 3D view) or
        ``None``. Separate channel from :meth:`poll` so the UI can read the map
        without stealing the correction the flow task consumes. Offline never
        calls this (no live viewer)."""

    def poll_loops(self) -> list:
        """Return the loop-match captures recorded since the last call (LIVE only).

        Each entry is ``(cur_seq, old_seq, LoopMatchCapture)`` for one verified
        loop candidate (confirmed OR rejected), so the live SLAM module can publish
        a ``slam.loop`` LoopMatch for the UI's loop-closure view. Distinct from
        :meth:`poll` (the correction the flow consumes) and :meth:`poll_overlay`
        (the map). Default is empty; :class:`InProcessEngine` overrides it to drain
        the map's captures (empty unless ``capture_loops`` is on -- the live path),
        so the offline deterministic path is untouched."""
        return []

    @abstractmethod
    def reset(self) -> None:
        """Forget the whole map and start fresh (UI "clear keyframes")."""

    @abstractmethod
    def close(self) -> None:
        """Tear down; idempotent."""
