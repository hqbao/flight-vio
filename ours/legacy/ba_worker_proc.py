"""Out-of-process sliding-window BA worker for the live ``ours-ba`` source.

Why a process and not a thread
------------------------------
The windowed-BA refine (Jacobian assembly + window bookkeeping in
``ours.lib.backend.bundle`` / ``windowed``) is mostly **pure-Python** work: only
the inner ``np.linalg.solve`` releases the GIL, the rest holds it. Measured on
the recorded ``fast_push_15s`` session a single ``run_ba()`` costs ~43 ms mean /
~74 ms peak and fires every ~250 ms, i.e. it steals **~17 % mean / ~30 % peak**
of the GIL from the device frame-read loop. When the loop is starved it drains
its camera queues *to the latest frame and drops the backlog*, so on a fast push
it processes fewer frames; each surviving frame then spans a larger motion, the
frame-to-frame PnP under-measures the translation, and the displayed path
"đẩy nhanh rồi ì lại" (undershoots). ``ours`` (the flow source) has no backend
thread, keeps a full frame rate, and never shows this.

Running the BA in a **separate process** removes the GIL contention entirely:
the read loop owns its interpreter and keeps 20 fps under BA load. The map state
lives wholly inside the child; the parent only ships keyframe snapshots in and
receives the world-frame correction ``C = inv(T_ba) @ T_cw`` out. The numbers it
produces are identical to the in-thread worker for the same inputs (the child
runs the very same :class:`WindowedBAMap`), which the self-test asserts.

The returned state dict mirrors the old in-thread worker's interface exactly
(``submit`` / ``poll`` / ``kf_every`` / ``stop`` / ``event`` / ``thread``) so the
read loop and teardown in :mod:`ours.legacy.depthai_ours_vio` are unchanged --
``thread`` is the :class:`multiprocessing.Process` (it too has ``join(timeout=)``)
and ``event`` is a tiny waker that unblocks the child's queue ``get`` on stop.
"""
from __future__ import annotations

import multiprocessing as mp
import queue
from typing import Any

import numpy as np


def _ba_worker_main(K, cfg, in_q, out_q, stop_evt) -> None:
    """Child entry point: own the BA map, refine each submitted keyframe.

    Module-level (not a closure) so it is picklable under the ``spawn`` start
    method used on macOS. Imports the heavy map lazily here so the child's
    bootstrap stays light and never touches depthai/Qt.
    """
    from ours.lib import WindowedBAMap

    ba_map = WindowedBAMap(K, cfg)
    while not stop_evt.is_set():
        try:
            snap = in_q.get(timeout=0.2)
        except queue.Empty:
            continue
        if snap is None:                      # stop sentinel
            break
        T_cw, ids, pts, depth_m, accel = snap
        ba_map.add_keyframe(T_cw, ids, pts, depth_m, accel_cam=accel)
        post = ba_map.run_ba()
        if post is not None:
            C = np.linalg.inv(post) @ T_cw
            try:                               # keep only the freshest correction
                out_q.put_nowait(C)
            except queue.Full:
                try:
                    out_q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    out_q.put_nowait(C)
                except queue.Full:
                    pass


class _QueueWaker:
    """Exposes ``.set()`` (the teardown contract) by pushing the stop sentinel."""

    def __init__(self, in_q: "mp.Queue") -> None:
        self._in_q = in_q

    def set(self) -> None:
        try:
            self._in_q.put_nowait(None)
        except queue.Full:
            pass


def start_ba_process(K: np.ndarray, cfg) -> dict[str, Any]:
    """Spawn the out-of-process BA worker and return the live-source state dict.

    Same keys as the legacy in-thread worker: ``submit(T_cw, ids, pts, depth_m,
    accel)`` (latest-wins, non-blocking -- drops if the worker is busy, exactly
    like the old ``_pending`` overwrite), ``poll() -> C | None`` (drains to the
    newest correction), ``kf_every``, plus ``stop`` / ``event`` / ``thread`` for
    the unchanged teardown.
    """
    ctx = mp.get_context("spawn")
    in_q: mp.Queue = ctx.Queue(maxsize=1)     # one pending keyframe; newest wins
    out_q: mp.Queue = ctx.Queue(maxsize=2)
    stop_evt = ctx.Event()
    proc = ctx.Process(target=_ba_worker_main,
                       args=(K, cfg, in_q, out_q, stop_evt),
                       name="OursBAWorkerProc", daemon=True)
    proc.start()

    def submit(T_cw, ids, pts, depth_m, accel) -> None:
        snap = (T_cw, ids, pts, depth_m, accel)
        try:
            in_q.put_nowait(snap)
        except queue.Full:                    # worker busy: replace the pending one
            try:
                in_q.get_nowait()
            except queue.Empty:
                pass
            try:
                in_q.put_nowait(snap)
            except queue.Full:
                pass

    def poll():
        C = None
        while True:
            try:
                C = out_q.get_nowait()
            except queue.Empty:
                break
        return C

    return {
        "submit": submit,
        "poll": poll,
        "kf_every": cfg.kf_every,
        "stop": stop_evt,
        "event": _QueueWaker(in_q),
        "thread": proc,
    }
