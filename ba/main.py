"""ba process: subscribe to VIO's keyframes, run the windowed BA, republish.

Subscribes (over IPC) to the ``vio`` endpoint for ``keyframe`` and the retained
``calib.bundle`` (intrinsics only); runs the procedural windowed-BA backend
(:class:`~ba.modules.pipeline.BackendWorker` ->
:func:`~ba.modules.pipeline.process_kf`) that was the back-end half of the in-VIO
pipeline; then republishes ``pose.refined`` -- and, under ``--tight``, the
optimised bias on ``ba.state`` -- on its own :class:`~ba.comms.IPCPubSub` endpoint
``"oak.ba"`` for the UI / VIO.

This shell is PROCEDURAL -- there is no reactive ``Module`` / ``Step`` graph. The
keyframe arrives over IPC on the :class:`~ba.comms.IPCSubscriber` recv thread,
which drops it onto the local bus; the :class:`~ba.modules.pipeline.BackendWorker`
runs :func:`~ba.modules.pipeline.process_kf` per keyframe (strict FIFO -- the
offline byte-parity argument needs every keyframe solved in order). ``ba`` is its
OWN process, so the heavy solve runs in-process in the worker thread (the in-VIO
``--worker`` subprocess engine was there to free the camera read loop's GIL, a
motivation that is gone once BA has its own process).

This process owns the windowed-BA map (sliding window of keyframe poses +
landmarks). The SLAM map (ORB index, pose-graph) lives in the ``slam`` process;
the two maps are independent by design -- they consume different things and serve
different views. ``ba`` is a pure CONSUMER of vio's keyframe output (``emit_keyframe``
stays in ``vio`` on its odometry thread).

Calibration handshake
---------------------
Same as SLAM -- a dedicated calib client blocks until the retained ``calib.bundle``
arrives on the VIO endpoint. VIO republishes the same calib it got from capture
AFTER allocating its kf_* rings, so receiving it here proves (a) VIO is up, (b)
intrinsics are known, and (c) the kf_gray / kf_depth rings we need to attach to
already exist. (We deliberately don't subscribe to capture at all -- ``ba`` is a
pure consumer of VIO's output.)

Tight feed-forward
------------------
Under ``--tight`` the backend publishes its latest optimised bias on the IPC POD
topic ``ba.state`` here. The CLOSED-LOOP feedback is wired on the OTHER side: when
VIO runs with ``--tight`` it opens its own read-only client on THIS endpoint and
subscribes to ``ba.state``, feeding the bias into propagate_imu's dead-reckoning
(PLAN P1/P2). This is the IPC analog of slam's ``loop.correction`` channel; the
carried ``seq`` survives the wire so the consumer's staleness gate makes the async
hop tolerable.

BA-window visualiser
--------------------
Under ``--ba-window`` (LOOSE only) the backend's capture-aware engine snapshots each
solve and publishes it on the IPC POD topic ``ba.window`` here. VIO bridges it back
(alongside ``pose.refined``) and re-emits it on the VIO endpoint, so the UI's "BA
Window" source keeps reading it from the single VIO endpoint -- unchanged across the
split. Opt-in + oracle-safe (the SAME frozen ``run_ba`` solve, default OFF).

Run::

    python -m ba.main
    python -m ba.main --vio-endpoint oak.vio.test --endpoint oak.ba.test --tight
    python -m ba.main --vio-endpoint oak.vio.test --endpoint oak.ba.test --ba-window
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ba.comms import (                                              # noqa: E402
    IPCPublisher, IPCSubscriber, IPCPubSub, LocalPubSub, RingRegistry, topics,
)
from ba.comms.messages import END                                  # noqa: E402
from ba.comms.wire import WireCalibBundle                          # noqa: E402
from ba.comms.ring_registry import default_vio_specs               # noqa: E402
from ba.modules import BackendWorker                               # noqa: E402

LOG = logging.getLogger("ba.main")

DEFAULT_VIO_ENDPOINT = "oak.vio"
DEFAULT_BA_ENDPOINT = "oak.ba"

#: Topic BA subscribes to from VIO.
_INPUT_TOPICS = [topics.KEYFRAME]
#: Topics BA republishes. POSE_REFINED is the BA-refined pose (pure POD, no ring),
#: always emitted. BA_STATE (the tight feed-forward bias) and BA_WINDOW (the
#: --ba-window solve snapshot) are pure POD too and appended ONLY when their producer
#: is built -- BA_STATE under --tight, BA_WINDOW under loose + --ba-window -- so a
#: consumer never waits on a topic that will never emit.
_OUTPUT_TOPICS = [topics.POSE_REFINED]


# --------------------------------------------------------------------------- #
def _await_calib_bundle(endpoint: str, timeout_s: float) -> WireCalibBundle:
    """Open a dedicated client, block until the retained calib bundle arrives."""
    bundle: list[WireCalibBundle | None] = [None]
    got = threading.Event()

    def on_calib(wm: WireCalibBundle) -> None:
        bundle[0] = wm
        got.set()

    client = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    client.subscribe("calib.bundle", on_calib)
    client.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(
                f"ba: no calib.bundle from {endpoint!r} in {timeout_s}s")
    finally:
        client.stop()
    assert bundle[0] is not None
    return bundle[0]


def _drain_wait(done_evt, ceiling_s: float, stop) -> None:
    """Wait up to ``ceiling_s`` for ``done_evt``, polling so a late SIGINT/SIGTERM
    (``stop[0]``) short-circuits the wait. The caller then forces the worker out
    via ``.stop()`` instead of blocking on an END that will never arrive."""
    waited = 0.0
    while waited < ceiling_s and not stop[0]:
        if done_evt.wait(timeout=0.1):
            return
        waited += 0.1


# --------------------------------------------------------------------------- #
def run_ba_proc(*,
                vio_endpoint: str = DEFAULT_VIO_ENDPOINT,
                endpoint: str = DEFAULT_BA_ENDPOINT,
                tight: bool = False,
                worker: bool = False,
                backend_window: int = 6,
                backend_iters: int = 5,
                stabilize_velocity: bool = False,
                depth_icp: bool = False,
                ba_window: bool = False,
                calib_timeout_s: float = 30.0) -> int:
    """Run the BA process until END / SIGTERM / Ctrl-C.

    ``tight`` selects the TIGHT-coupled VIO backend (the joint visual + IMU window
    optimiser, ``imu_info_weight=True``) instead of the default LOOSE windowed-BA
    backend, AND turns on the ``ba.state`` feed-forward publish for the ``vio``
    process. Opt-in: when False the path is the loose windowed BA and no ``ba.state``
    is emitted.

    ``backend_window`` / ``backend_iters`` size the LOOSE windowed-BA solve
    (``WindowedConfig.window`` + ``BAConfig.max_iters``); they are inert on the tight
    path (which uses the validated ``WindowedVIOConfig`` defaults). ``stabilize_velocity``
    / ``depth_icp`` are the TIGHT-only Phase-4 knobs (CV-prior + gated ZUPT, and the
    dense-ICP relative-pose factor); they are forwarded to the tight ``WindowedVIOMap``
    and ignored on the loose path. All four are opt-in and never touch ``pose.refined``
    on the default path -> the byte-parity oracle is unchanged with or without them.

    ``ba_window`` enables the opt-in BA-window visualiser snapshot stream
    (``--ba-window``): the LOOSE backend's capture-aware engine snapshots each solve
    (window keyframe poses + landmarks + observation rays + reprojection error) on the
    IPC ``ba.window`` topic for the UI's "BA Window". LOOSE-only -- ``--tight`` overrides
    it (the tight map has no capture overlay), so ``ba.window`` is published ONLY when
    ``ba_window and not tight``; a consumer never waits on a topic that will never emit.
    Oracle-safe: the capture engine runs the SAME frozen ``run_ba`` solve, and the
    default-OFF path never captures it.

    ``worker`` is ACCEPTED for argparse symmetry with the other process shells but
    is a NO-OP for ``ba``: the backend already runs the solve in-process in its own
    process (the in-VIO subprocess engine existed only to free the camera read
    loop's GIL, which ``ba`` does not share). It is logged when set so the operator
    knows it had no effect.
    """
    if worker:
        LOG.info("ba: --worker is a NO-OP for the ba process (the solve already "
                 "runs in-process in its own process); ignoring it")
    # BA-window capture is LOOSE-only; --tight overrides it (the tight map has no
    # capture overlay). Publish ba.window only when the capture engine is actually
    # built, so a consumer never waits on a topic that will never emit. (Mirrors the
    # pre-split in-VIO ``ba_window_on`` gate.)
    ba_window_on = bool(ba_window and not tight)
    # 1. Block until VIO's retained calib bundle arrives. VIO republishes the
    #    same calib it got from capture AFTER allocating its kf_* rings, so
    #    receiving it here proves (a) VIO is up, (b) intrinsics are known, and
    #    (c) the kf_gray / kf_depth rings we need to attach to already exist.
    #    (We deliberately don't subscribe to capture at all -- ba is a pure
    #    consumer of VIO's output.)
    LOG.info("ba: waiting for calib.bundle on %s ...", vio_endpoint)
    bundle = _await_calib_bundle(vio_endpoint, calib_timeout_s)
    width, height = int(bundle.width), int(bundle.height)
    LOG.info("ba: got calib %dx%d", width, height)

    # 2. Attach to VIO's keyframe rings. BA needs each keyframe's ``depth_m`` for
    #    the windowed solve; the keyframe converter (_keyframe_to_local) reads BOTH
    #    the kf_gray and kf_depth ring slots, so we attach both specs (same as
    #    SLAM). VIO is the single writer of these rings; ba reads at the keyframe
    #    cadence.
    vio_rings = RingRegistry().attach_all(default_vio_specs(
        endpoint=vio_endpoint, width=width, height=height))

    # 3. Build the local bus + the backend worker (the windowed BA). Strict FIFO
    #    (latest_only=False): ba is the deterministic backend, not a live-only
    #    viewer -- every keyframe must be solved in order so the refined-pose
    #    output matches the in-VIO path. The worker subscribes itself to `keyframe`
    #    on construction. The backend knobs (window / iters size the loose solve;
    #    stabilize_velocity / depth_icp the tight Phase-4 factors; capture_window the
    #    --ba-window viz) are threaded through here EXACTLY as the pre-split in-VIO
    #    BackendModule passed them.
    local = LocalPubSub()
    backend = BackendWorker(local, bundle.K,
                            window=backend_window, iters=backend_iters,
                            latest_only=False, tight=tight,
                            stabilize_velocity=stabilize_velocity,
                            depth_icp=depth_icp, capture_window=ba_window_on)
    if tight:
        LOG.info("ba: TIGHT-coupled VIO backend selected (--tight) "
                 "[imu_info_weight=True] -- publishing ba.state feed-forward")
    # Publish the opt-in republished topics only when their producer is actually
    # built, so a consumer never waits on a topic that will never emit: ba.state
    # (the tight feed-forward bias) only under --tight, and ba.window (the BA-window
    # solve snapshot) only when the capture engine is built (loose + --ba-window).
    out_topics = list(_OUTPUT_TOPICS)
    if tight:
        out_topics.append(topics.BA_STATE)
    if ba_window_on:
        out_topics.append(topics.BA_WINDOW)
        LOG.info("ba: BA-window visualiser ON (--ba-window) -- publishing ba.window "
                 "solve snapshots on %s for the UI", endpoint)

    # 4. Open output IPCPubSub server + publisher for the refined pose (+ ba.state).
    #    Both republished topics are pure POD (pose / bias -- no images), so the
    #    publisher's ring registry is effectively unused for them; we hand it the
    #    attached vio_rings (the converters ignore rings for POD topics). Retain
    #    `calib.bundle` and re-broadcast VIO's bundle so consumers that talk to
    #    *this* endpoint (UI, smoke selftest) can use the calib arrival as a
    #    readiness barrier.
    server = IPCPubSub(endpoint, role="server", retain_topics={"calib.bundle"})
    pub = IPCPublisher(local, server, vio_rings, out_topics, endpoint=endpoint)
    pub.start()
    server.publish("calib.bundle", bundle)

    # 5. Open input IPCPubSub client + subscriber bridge: VIO keyframes -> local.
    in_client = IPCPubSub(vio_endpoint, role="client")
    in_bridge = IPCSubscriber(local, in_client, vio_rings, _INPUT_TOPICS)

    # 6. END-watch: capture's END propagates through VIO to here.
    finished = threading.Event()

    def _end_watch(_msg) -> None:
        if _msg is END:
            finished.set()
    for t in _INPUT_TOPICS:
        local.subscribe(t, _end_watch)

    LOG.info("ba[%s] subscribing to %s for keyframes", endpoint, vio_endpoint)

    # 7. Start everything. The worker already subscribed itself to `keyframe` on
    #    construction (step 3), so its inbox feeder is wired; start its thread
    #    BEFORE the bridge starts pushing keyframes onto the local bus so the
    #    inbox is being drained from the first message.
    backend.start()
    in_bridge.start()

    stop = [False]

    def _on_sigterm(_signo, _frame):
        stop[0] = True
    # SIGINT (Ctrl-C) and SIGTERM (launcher) both request the SAME clean stop:
    # set the flag and let the run loop + drain react. Handling SIGINT here (vs
    # the default KeyboardInterrupt) means teardown can NEVER abort on a raw
    # traceback -- the operator Ctrl-Cs once and we exit cleanly.
    signal.signal(signal.SIGTERM, _on_sigterm)
    signal.signal(signal.SIGINT, _on_sigterm)

    try:
        while not stop[0] and not finished.is_set():
            time.sleep(0.1)
    finally:
        # Drain order: stop the input bridge so no more keyframes arrive, then
        # wait for the backend to finish its inbox (END is already in flight). On
        # interrupt the operator wants a fast exit and VIO is also shutting down so
        # END will never arrive -- `_drain_wait` polls stop[0] so a late
        # SIGINT/SIGTERM short-circuits the wait and BackendWorker.stop() unblocks
        # the worker, otherwise the launcher SIGKILLs us at its deadline.
        in_bridge.stop()
        drain_timeout = 2.0 if stop[0] else 120.0
        _drain_wait(backend.done, drain_timeout, stop)
        backend.stop()
        # The worker forwards END to pose.refined (+ ba.window) when it drains END
        # (process_kf path); the publisher bridge mirrors that onto IPC. No
        # explicit publish_end.
        time.sleep(0.3)
        pub.stop()
        server.close()
        vio_rings.close()
        LOG.info("ba: shutdown complete")
    return 0


# --------------------------------------------------------------------------- #
def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--vio-endpoint", default=DEFAULT_VIO_ENDPOINT,
                    help=f"VIO IPC endpoint (default: {DEFAULT_VIO_ENDPOINT!r})")
    ap.add_argument("--endpoint", default=DEFAULT_BA_ENDPOINT,
                    help=f"this process's IPC endpoint (default: {DEFAULT_BA_ENDPOINT!r})")
    ap.add_argument("--tight", action="store_true",
                    help="select the TIGHT-coupled VIO backend (joint visual + IMU "
                         "window optimiser, imu_info_weight=True) instead of the "
                         "default LOOSE windowed-BA backend, and publish the "
                         "ba.state feed-forward bias for VIO. Opt-in; the default "
                         "(loose) path is byte-identical to the in-VIO backend.")
    ap.add_argument("--worker", action="store_true",
                    help="ACCEPTED for symmetry with the other shells but a NO-OP "
                         "for ba (the solve already runs in-process in its own "
                         "process); logged + ignored when set.")
    ap.add_argument("--backend-window", type=int, default=6,
                    help="LOOSE windowed-BA sliding-window size (keyframes); inert "
                         "on the tight path (which uses WindowedVIOConfig defaults).")
    ap.add_argument("--backend-iters", type=int, default=5,
                    help="LOOSE windowed-BA max Gauss-Newton iterations per solve; "
                         "inert on the tight path.")
    ap.add_argument("--stabilize-velocity", action="store_true",
                    help="tight only: enable Phase-4 velocity regularisation (CV "
                         "prior + gated ZUPT) to curb 54x42/shake velocity "
                         "divergence. Opt-in; ignored on the loose path.")
    ap.add_argument("--depth-icp", action="store_true",
                    help="tight only: enable the Phase-4 dense-ICP relative-pose "
                         "factor (anchors inter-keyframe translation at 54x42). "
                         "Opt-in; ignored on the loose path.")
    ap.add_argument("--ba-window", action="store_true",
                    help="publish ba.window solve snapshots (window keyframe poses + "
                         "3D landmarks + observation rays + reprojection error) for "
                         "the UI's BA Window visualiser. LOOSE-only -- ignored under "
                         "--tight; oracle byte-identical (the SAME frozen solve).")
    ap.add_argument("--calib-timeout", type=float, default=30.0,
                    help="seconds to wait for the calib.bundle on boot")
    args = ap.parse_args()

    return run_ba_proc(
        vio_endpoint=args.vio_endpoint,
        endpoint=args.endpoint,
        tight=args.tight,
        worker=args.worker,
        backend_window=args.backend_window,
        backend_iters=args.backend_iters,
        stabilize_velocity=args.stabilize_velocity,
        depth_icp=args.depth_icp,
        ba_window=args.ba_window,
        calib_timeout_s=args.calib_timeout,
    )


if __name__ == "__main__":
    # Use os._exit (not SystemExit / return-from-main) so a lingering non-daemon
    # thread -- IPCSubscriber's recv loop, a numba thread pool, etc -- cannot keep
    # the process alive past `ba: shutdown complete`. Without this the launcher
    # waits its full deadline and SIGKILLs us. Mirrors the same pattern in
    # `slam.main` / `vio.main`.
    import os as _os
    _rc = main()
    LOG.info("ba: main returned, calling os._exit(%d)", int(_rc))
    logging.shutdown()
    _os.sys.stdout.flush()
    _os.sys.stderr.flush()
    _os._exit(int(_rc))
