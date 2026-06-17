"""The windowed-BA backend over the ``keyframe`` stream, as PROCEDURAL Python.

The back-end half of the old in-VIO ``vio.modules.pipeline`` pair, extracted into
the ``ba`` process. The odometry worker (front-end, IMU prior, per-frame
publishers) stays in ``vio``; ``ba`` is a pure CONSUMER of the ``keyframe`` stream:

* :func:`process_kf` -- per keyframe: run the sliding-window solve and publish the
  refined pose (+ the opt-in ``ba.window`` snapshot), and -- TIGHT only -- the
  optimised bias for the live feed-forward.
* :class:`BackendWorker` -- the single ``keyframe`` input thread, the ``tight``
  engine switch, and the END-forward to ``pose.refined`` (+ ``ba.window`` when the
  capture engine is built).

Lifted from ``vio.modules.pipeline`` so the refined-pose output (and the offline
byte-parity argument that depends on the SAME frozen solve) is unchanged; only the
package the comms / engine come from moved (``vio`` -> ``ba``).

TIGHT feed-forward seam (the ONE behavioural change vs the in-VIO original): the
in-VIO backend republished its optimised bias on the INTRA-process local-bus
``backend.state`` topic for the SAME process's propagate_imu. Across the split that
hop becomes an IPC one -- ``ba`` publishes the bias on the new IPC POD topic
``ba.state`` (:class:`~ba.comms.messages.BackendState`), and ``vio`` opens a
read-only client on the ba endpoint to drain it (the IPC analog of slam's
``loop.correction``). The carried ``seq`` survives the wire so the consumer's
staleness gate makes the async hop tolerable.
"""
from __future__ import annotations

import logging
import queue
import threading
from typing import Any

import numpy as np

from ba.comms import LocalPubSub, topics
from ba.comms.messages import BackendState, END
from sky.backend.bundle import BAConfig
from sky.backend.windowed import WindowedConfig
from sky.vio.window import WindowedVIOConfig
from ba.engine import make_ba_engine, make_vi_engine
from .backend import run_ba
from .publishers import publish_ba_window, publish_refined

LOG = logging.getLogger("ba.pipeline")

#: Inbox sentinel to unblock ``queue.get`` on ``stop()``. Mirrors ``Module._SENTINEL``.
_SENTINEL = object()
#: Inbox payload marker for the coalescing path: "the real message is the current
#: self._latest". Mirrors the old ``Module._LATEST`` token.
_LATEST = object()


# =========================================================================== #
# Backend: the windowed BA over the keyframe stream
# =========================================================================== #
def process_kf(engine, bus: LocalPubSub, tight: bool, capture_window: bool,
               kf) -> None:
    """Run one keyframe through the windowed-BA backend.

    Byte-identical order to the old in-VIO chain. ``run_ba`` submits the keyframe's
    snapshot (loose 5-tuple / tight 6-tuple) to the engine and returns
    ``(refined PoseMsg, backend_state)`` (or ``None`` -> chain short-circuit). With
    capture on, ``publish_ba_window`` runs between ``run_ba`` and ``publish_refined``
    (it forwards the pose UNCHANGED, so ``pose.refined`` is byte-identical to the
    no-capture chain).

    TIGHT feed-forward (PLAN P1): when the tight backend returns its latest
    optimised bias, publish it on the IPC ``ba.state`` topic
    (:class:`~ba.comms.messages.BackendState`) for the ``vio`` process's
    propagate_imu to adopt over its read-only ba-endpoint client. The topic is the
    IPC analog of the in-VIO local-bus ``backend.state``; it is read by nothing on
    the loose / oracle path -> ``pose.refined`` byte-parity (gap = 0) is unaffected.
    """
    result = run_ba(engine, tight, kf)
    if result is None:               # no tracks this kf / no refined pose yet
        return
    msg, backend_state = result
    if capture_window:
        msg = publish_ba_window(engine, bus, msg)
    publish_refined(bus, msg)
    if tight and backend_state is not None:
        bg, ba = backend_state
        bus.publish(topics.BA_STATE,
                    BackendState(seq=int(kf.seq),
                                 bg=np.asarray(bg, dtype=np.float64).reshape(-1),
                                 ba=np.asarray(ba, dtype=np.float64).reshape(-1),
                                 degraded=bool(msg.info.get("vio_degraded", False))))


class BackendWorker(threading.Thread):
    """The windowed back-end over the ``keyframe`` stream: a plain thread.

    A procedural replacement for the old reactive ``BackendModule(Module)``. Two
    selectable backends, picked by ``tight`` (a clean engine switch, NOT a
    pipeline fork):

    * ``tight=False`` (default, LOOSE) -- :func:`~ba.engine.make_ba_engine` builds
      the vision-only ``WindowedBAMap`` (reproj + depth + optional VO/gravity
      priors). Byte-identical to the pre-tight build; the offline oracle relies on
      this.
    * ``tight=True`` (``--tight``, opt-in) -- :func:`~ba.engine.make_vi_engine`
      builds the tight-coupled ``WindowedVIOMap`` (joint visual + IMU window
      optimiser). The IMU factor is weighted by the per-edge information square root
      (``imu_info_weight=True``); ``run_ba`` then submits the SUPERSET snapshot
      (keyframe ts + raw inter-keyframe IMU block).

    The heavy solve runs behind an :class:`~ba.engine.base.Engine`. ``ba`` is its
    OWN process, so it always uses the in-process engine (``worker=False``) -- the
    solve runs synchronously in this thread, byte-identical to the old in-VIO
    in-process path. (The ``vio`` process used ``worker=True`` to push the solve off
    the camera read loop's GIL; that motivation is gone once BA has its own process,
    so the engine here is in-process only.)

    Single ``keyframe`` input, so the first END is terminal (no join). END is
    forwarded to ``pose.refined`` (+ ``ba.window`` when the capture engine built).
    """

    def __init__(self, bus: LocalPubSub, K: np.ndarray,
                 window: int = 6, iters: int = 5,
                 latest_only: bool = False,
                 tight: bool = False, stabilize_velocity: bool = False,
                 depth_icp: bool = False, capture_window: bool = False) -> None:
        super().__init__(name="backend", daemon=True)
        self.bus = bus
        self.tight = bool(tight)
        # BA-window capture (opt-in, --ba-window) is a LOOSE-backend-only viz: the
        # capture-aware ``WindowedBAMap`` engine snapshots each solve for the UI's
        # "BA Window". Ignored on the tight path + OFF by default so the oracle
        # stays byte-identical.
        self.capture_window = bool(capture_window) and not tight
        if tight:
            # Tight backend: enable the covariance-correct IMU weight (Phase 1's
            # opt-in flag) on a copy of WindowedVIOConfig's validated defaults.
            # ``imu_info_weight`` is the only baseline override -- everything else
            # (window, lock_tilt, tight vel/pos sigmas, kf_every) keeps the values
            # the vio_ba_selftest / vio oracle entries were tuned against.
            vio_cfg = WindowedVIOConfig()
            vio_cfg.vio.imu_info_weight = True
            # Phase-4 velocity regularisation (opt-in, LIVE --tight only): the
            # single ``stabilize_velocity`` knob makes ``run_ba`` flip on BOTH
            # the CV smoothness prior and the excitation-gated ZUPT for every
            # solve, curbing the 54x42 / shake window-velocity divergence. Left
            # OFF by default so the tight-without-flag path -- and the oracle --
            # stay byte-identical; only the operator's --stabilize-velocity sets it.
            if stabilize_velocity:
                vio_cfg.stabilize_velocity = True
                LOG.info("ba: tight velocity-stabilize ON "
                         "(CV prior + gated ZUPT)")
            # Phase-4 dense-ICP relative-pose factor (opt-in, LIVE --tight only):
            # ``depth_icp`` makes ``run_ba`` add an IMU-seeded point-to-plane ICP
            # relative-pose factor between adjacent in-window keyframes, anchoring
            # the inter-keyframe TRANSLATION the feature-starved 54x42 frontend
            # leaves unobservable. OFF by default so the tight-without-flag path
            # and the oracle stay byte-identical; only --depth-icp sets it.
            if depth_icp:
                vio_cfg.depth_icp = True
                LOG.info("ba: tight dense-ICP relative-pose factor ON "
                         "(translation anchor for feature-starved frames)")
            self.engine = make_vi_engine(K, vio_cfg, worker=False)
        else:
            cfg = WindowedConfig(window=window, ba=BAConfig(max_iters=iters))
            self.engine = make_ba_engine(K, cfg, worker=False,
                                         capture_window=self.capture_window)
            if self.capture_window:
                LOG.info("ba: BA-window capture ON (--ba-window) -- publishing "
                         "ba.window solve snapshots for the UI visualiser")

        # END is forwarded to whatever this worker publishes (was forwards_to):
        # pose.refined always, ba.window when the capture engine is built.
        self._downstream = [topics.POSE_REFINED]
        if self.capture_window:
            self._downstream.append(topics.BA_WINDOW)

        self._latest_only = bool(latest_only)
        self._inbox: "queue.Queue" = queue.Queue()
        self._latest: Any = _SENTINEL          # single-slot newest unprocessed kf
        self._latest_lock = threading.Lock()
        self._stop = threading.Event()
        self.done = threading.Event()          #: set after END is handled
        self._emitted_end = False

        # Subscribe the inbox feeder to keyframe in __init__ (old Module.on timing).
        self.bus.subscribe(topics.KEYFRAME, self._on_keyframe)

    # -- inbox feeder (runs on the PUBLISHER's thread, kept cheap) ----------- #
    def _on_keyframe(self, msg: Any) -> None:
        """Bus handler for ``keyframe``: enqueue (coalescing or strict FIFO)."""
        if not self._latest_only:
            self._inbox.put(msg)
            return
        # Coalescing (LIVE visualiser-fed graphs): keep only the newest
        # unprocessed keyframe; enqueue a token only when nothing pending -- EXCEPT
        # END, which always enqueues. Byte-for-byte the old Module._coalesce,
        # specialised to this worker's single keyframe topic.
        with self._latest_lock:
            pending = self._latest is not _SENTINEL
            self._latest = msg
            enqueue = (not pending) or (msg is END)
        if enqueue:
            self._inbox.put(_LATEST)

    # -- thread body -------------------------------------------------------- #
    def stop(self) -> None:
        self._stop.set()
        self._inbox.put(_SENTINEL)             # unblock the queue.get

    def run(self) -> None:
        # Close the engine on THIS thread when the loop exits (stop sentinel or
        # _stop). The in-process engine's close is a no-op, but keep the same
        # lifecycle shape as the original for clarity / future engines.
        try:
            self._loop()
        finally:
            self.engine.close()

    def _loop(self) -> None:
        while not self._stop.is_set():
            item = self._inbox.get()
            if item is _SENTINEL:
                break
            if item is _LATEST:
                with self._latest_lock:
                    msg, self._latest = self._latest, _SENTINEL
                if msg is _SENTINEL:
                    continue
            else:
                msg = item                      # strict-FIFO payload
            if msg is END:
                self._handle_end()
                continue
            process_kf(self.engine, self.bus, self.tight,
                       self.capture_window, msg)

    def _handle_end(self) -> None:
        # Single-input sink (one keyframe topic), so the first END is terminal.
        if not self._emitted_end:
            self._emitted_end = True
            for t in self._downstream:
                self.bus.publish(t, END)
        self.done.set()


#: Public alias kept symmetric with ``vio.modules`` (the procedural worker, not a
#: reactive Module).
BackendModule = BackendWorker
