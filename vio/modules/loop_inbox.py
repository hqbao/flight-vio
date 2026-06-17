"""Thread-safe inboxes for the live ``--tight`` cross-thread feedback paths
(SLAM ``loop.correction`` + the tight backend's ``backend.state`` bias).

The closed-loop feedback ``slam -> vio`` crosses a thread boundary: the
``loop.correction`` arrives on the VIO process's slam-endpoint IPC subscriber
thread, but it must be applied to the live nav-state on the ODOMETRY module's
thread (the single owner of ``live_nav``). This tiny holder is the safe handoff:

* the subscriber side calls :meth:`push` (under a lock),
* :class:`~vio.modules.propagate_imu.PropagateImu` calls :meth:`drain` once per
  frame on the odometry thread and applies the corrections there.

It coalesces nothing -- every queued correction is returned in arrival order so a
burst of loop closures (e.g. a long revisit) all fold into the pending delta. The
queue is bounded so a wedged consumer cannot grow it without limit; the oldest
correction is dropped on overflow (a stale correction is the safe one to lose --
the freshest pose-graph rewrite supersedes it anyway).

This module is imported ONLY by the live ``--tight`` path (``OdometryModule`` when
``loop_correct=True``, wired by ``vio.main``). The offline / oracle / loose path
never constructs it, so the closed-loop feedback is purely additive there.
"""
from __future__ import annotations

import threading
from collections import deque
from typing import Any

#: Max queued corrections before the oldest is dropped. Loop closures are rare
#: events (one per revisit), so this only ever fills if the odometry thread wedges
#: -- in which case the freshest correction is the one worth keeping.
_MAX_QUEUED = 16


class LoopCorrectionInbox:
    """A small lock-guarded queue of ``LoopCorrection`` messages."""

    __slots__ = ("_lock", "_q")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q: "deque[Any]" = deque(maxlen=_MAX_QUEUED)

    def push(self, correction: Any) -> None:
        """Enqueue a correction from the IPC subscriber thread (drops oldest on
        overflow via the bounded deque)."""
        with self._lock:
            self._q.append(correction)

    def drain(self) -> list[Any]:
        """Return + clear all queued corrections (called on the odometry thread).

        Returns them in arrival order so the pending-delta composition stacks the
        oldest unfinished correction first.
        """
        with self._lock:
            if not self._q:
                return []
            items = list(self._q)
            self._q.clear()
        return items


class BackendStateInbox:
    """Latest-wins holder for ``backend.state`` (the tight BA's optimised bias).

    Unlike :class:`LoopCorrectionInbox` (which stacks a burst of loop closures so
    they all fold into the pending delta), only the FRESHEST bias matters -- a
    single current value, not a sequence -- so this COALESCES to the latest. Same
    thread handoff: the local-bus subscriber thread :meth:`push`es, the odometry
    thread :meth:`take`s once per frame and adopts it. LIVE + --tight only.
    """

    __slots__ = ("_lock", "_latest")

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: Any = None

    def push(self, state: Any) -> None:
        """Store the newest ``backend.state`` from the subscriber thread (latest-wins)."""
        with self._lock:
            self._latest = state

    def take(self) -> Any:
        """Return + clear the freshest ``backend.state`` (odometry thread), or ``None``."""
        with self._lock:
            s, self._latest = self._latest, None
        return s
