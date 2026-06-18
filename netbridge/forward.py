"""netbridge.forward -- runs on the Pi: local IPC -> TCP (the re-encode point).

For each canonical endpoint (capture / vio / slam) the forward process:

1. Opens a local ``comms.ipc.IPCPubSub(ep, role="client")`` and wraps it in a
   ``comms.bridge.IPCSubscriber`` with the Pi-side rings + the topic allowlist.
   ``IPCSubscriber`` runs ``comms.converters.to_local`` for every inbound message,
   which ``read_copy``-s each ``SharedArrayRef`` out of the Pi's shared memory into
   a REAL ndarray and rebuilds the local dataclass. The resolved dataclass is
   published on a PRIVATE ``LocalPubSub``.

2. Taps that private bus. For each message it builds the ref-FREE wire form (full
   ndarrays inline -- via :mod:`netbridge.wire_full` for the four image topics, or
   ``comms.converters.to_wire`` with a no-op ring registry for the POD topics),
   ``comms.codec.encode``s it, and publishes the bytes over the per-endpoint
   :class:`netbridge.tcp_transport.TcpServer`.

This step is the ONLY re-encode point in the whole bridge, and the place that
GUARANTEES full-ndarray (0x08) on the wire. A DEFENSIVE assert refuses to let any
``SharedArrayRef`` survive into the TCP encode -- shipping a ref the Mac cannot
``read_copy`` would corrupt the UI silently, so we fail loud instead.

The three endpoints share ONE ``TcpServer`` (one TCP port): the topic namespaces
are disjoint across capture/vio/slam, so a single server with the union allowlist
re-serves all three cleanly, and the Mac opens one TCP connection.

Backpressure: image topics forward non-blocking latest-wins (drop stale on a WiFi
stall, never back-pressure the flight stack); POD + retained topics forward
reliably. That policy lives in :class:`TcpServer` (``image_topics`` set); forward
just passes the encoded bytes through.

Run (on the Pi)::

    OAKD_NETBRIDGE_KEY=<secret> python -m netbridge.forward \\
        --listen 0.0.0.0:8787 \\
        --capture-endpoint oak.capture --vio-endpoint oak.vio \\
        --slam-endpoint oak.slam --width 54 --height 42
"""
from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from netbridge.comms import codec                                             # noqa: E402
from netbridge.comms.bridge import IPCSubscriber                              # noqa: E402
from netbridge.comms.converters import to_wire                                # noqa: E402
from netbridge.comms.ipc import IPCPubSub                                     # noqa: E402
from netbridge.comms.pubsub import LocalPubSub                                # noqa: E402
from netbridge.comms.ring_registry import (                                   # noqa: E402
    RingRegistry, default_capture_specs, default_vio_specs,
)
from netbridge.comms.shared_array import SharedArrayRef                       # noqa: E402
from netbridge.comms.wire import WireEnd                                      # noqa: E402

from netbridge import topics_allowlist as allow                    # noqa: E402
from netbridge import wire_full                                     # noqa: E402
from netbridge.tcp_transport import TcpServer, install_sigterm     # noqa: E402

LOG = logging.getLogger("netbridge.forward")


# --------------------------------------------------------------------------- #
# The defensive no-ref guard: nothing with a SharedArrayRef may reach the wire.
# --------------------------------------------------------------------------- #
def _assert_no_shared_ref(topic: str, wm) -> None:
    """Fail LOUD if any field of ``wm`` is still a ``SharedArrayRef``.

    The whole point of forward is to MATERIALISE every shared-memory ref into a
    real ndarray before it crosses the network. A ref on the wire would carry only
    metadata (ring name / slot) the Mac cannot resolve -- the UI would read
    garbage. So we check every field and refuse to encode if one slipped through
    (e.g. a future image topic added to the allowlist without a ``wire_full``
    entry). This is cheap (a handful of fields) and runs per message.
    """
    from dataclasses import fields, is_dataclass
    if not is_dataclass(wm):
        return
    for f in fields(wm):
        if isinstance(getattr(wm, f.name), SharedArrayRef):
            raise AssertionError(
                f"netbridge.forward: topic {topic!r} field {f.name!r} is a "
                f"SharedArrayRef on the TCP wire -- the Mac cannot read it. The "
                f"forward path must materialise it to a full ndarray first.")


# --------------------------------------------------------------------------- #
# A RingRegistry that NEVER writes -- POD converters take (msg, rings, endpoint)
# but never touch the rings, so we pass this and a forgotten ring access fails
# loud (instead of silently writing the wrong place) on a future mis-wiring.
# --------------------------------------------------------------------------- #
class _NoRings(RingRegistry):
    """A RingRegistry whose :meth:`get` raises -- POD converters must not ring."""

    def get(self, name: str):                       # type: ignore[override]
        raise AssertionError(
            f"netbridge.forward: a POD converter tried to access ring {name!r}; "
            f"only the four ref-bearing image topics use rings, and they go "
            f"through netbridge.wire_full, not comms.converters.to_wire.")


# --------------------------------------------------------------------------- #
# One forwarded endpoint: local IPCSubscriber -> private LocalPubSub -> TCP.
# --------------------------------------------------------------------------- #
class EndpointForwarder:
    """Bridges ONE local endpoint's allowlisted topics onto the shared TcpServer.

    Owns the local IPC client + subscriber + the private local bus; does NOT own
    the ``TcpServer`` (shared across all endpoints, started/closed by the caller).
    """

    def __init__(self, role: str, local_endpoint: str, server: TcpServer,
                 rings: RingRegistry, *, connect_timeout_s: float = 30.0,
                 include_images: bool = True) -> None:
        self.role = role
        self.local_endpoint = local_endpoint
        self.server = server
        self._no_rings = _NoRings()
        # In pose-only mode the heavy image topics are dropped here, at the single
        # source of truth, so this forwarder NEVER subscribes/forwards them.
        all_t = allow.all_topics(role, include_images=include_images)
        # Convertible (image + POD) topics ride the IPCSubscriber -> to_local ->
        # tap -> ref-free wire. Direct-wire (retained config) topics have NO
        # converter, so they ride a RAW IPC subscription that hands us the Wire*
        # object to forward verbatim.
        self._conv_topics = [t for t in all_t
                             if t not in allow.DIRECT_WIRE_TOPICS]
        self._direct_topics = [t for t in all_t
                               if t in allow.DIRECT_WIRE_TOPICS]
        self._topics = all_t

        # Private local bus the IPCSubscriber republishes onto; we tap it for the
        # convertible topics only.
        self._local_bus = LocalPubSub()
        for t in self._conv_topics:
            self._local_bus.subscribe(t, self._make_tap(t))

        # Local IPC client + subscriber: read the Pi endpoint, resolve refs to
        # real ndarrays (to_local), republish on the private bus -- convertible
        # topics ONLY (the converter would KeyError on the direct-wire ones).
        self._client = IPCPubSub(local_endpoint, role="client",
                                 connect_timeout_s=connect_timeout_s)
        self._subscriber = IPCSubscriber(
            self._local_bus, self._client, rings, self._conv_topics)

        # A SECOND raw client for the direct-wire (retained config) topics: it
        # subscribes them on the same endpoint and hands us the Wire* object,
        # which we encode + forward verbatim (no to_local round-trip).
        self._direct_client: IPCPubSub | None = None
        if self._direct_topics:
            self._direct_client = IPCPubSub(
                local_endpoint, role="client",
                connect_timeout_s=connect_timeout_s)
            for t in self._direct_topics:
                self._direct_client.subscribe(t, self._make_direct(t))

    # ------------------------------------------------------------------ #
    def _make_tap(self, topic: str):
        """Closure: build the ref-free wire form, encode, publish over TCP.

        The image-vs-POD backpressure policy is applied by :class:`TcpServer`
        (it knows the image-topic set), so the tap is policy-free: it just
        materialises, encodes, and hands the bytes to ``publish_encoded``.
        """
        def _tap(msg) -> None:
            # The wire-level END arrives as the local END sentinel here
            # (to_local maps WireEnd -> END); forward it as a wire END frame so a
            # late Mac subscriber still sees a clean end-of-stream per topic.
            if isinstance(msg, WireEnd):
                wm = msg
            elif _is_end(msg):
                wm = WireEnd(topic)
            elif topic in wire_full.REF_BEARING_TOPICS:
                # Image topic: ndarrays inline (the 0x09 -> 0x08 re-materialisation).
                wm = wire_full.local_to_full_wire(topic, msg)
            else:
                # POD topic: the standard converter ships every array inline; the
                # no-op ring registry proves it never touches a ring.
                wm = to_wire(topic, msg, self._no_rings, self.local_endpoint)
            self._encode_and_publish(topic, wm)

        return _tap

    def _make_direct(self, topic: str):
        """Closure: forward a direct-wire (retained config) Wire* object verbatim.

        The raw client delivers the ``Wire*`` instance (e.g. ``WireCalibBundle``)
        directly -- no ``to_local`` (there is no converter for these). We encode +
        forward it as-is; the TCP server caches it (retained) so a late Mac UI
        gets it on connect.
        """
        def _direct(wm) -> None:
            self._encode_and_publish(topic, wm)

        return _direct

    def _encode_and_publish(self, topic: str, wm) -> None:
        """Guard against a stray ref, encode, hand the bytes to the TcpServer."""
        try:
            _assert_no_shared_ref(topic, wm)           # fail loud, never ship a ref
            payload = codec.encode(topic, wm)
        except Exception as e:                                      # noqa: BLE001
            LOG.warning("forward %s/%s: encode failed: %s",
                        self.role, topic, e)
            return
        self.server.publish_encoded(topic, payload)

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Start the local subscribers (the TcpServer is started by the caller)."""
        self._subscriber.start()
        if self._direct_client is not None:
            self._direct_client.start()
        LOG.info("forward[%s]: subscribing %s on %r -> TCP",
                 self.role, sorted(self._topics), self.local_endpoint)

    def stop(self) -> None:
        try:
            self._subscriber.stop()
        except Exception:                                          # noqa: BLE001
            pass
        if self._direct_client is not None:
            try:
                self._direct_client.stop()
            except Exception:                                      # noqa: BLE001
                pass


def _is_end(msg) -> bool:
    """True if ``msg`` is the local END sentinel (imported lazily to avoid a hard
    dep on its identity across copies)."""
    from netbridge.comms.messages import END
    return msg is END


# --------------------------------------------------------------------------- #
def _await_calib_bundle(endpoint: str, timeout_s: float):
    """Block until capture's retained ``calib.bundle`` arrives; return it.

    The bundle carries the ACTUAL grid (54x42 under ``--vl53l9cx``, else full res),
    so the forwarder attaches its Pi-side rings at the resolution capture REALLY
    created -- NOT the raw ``--width``/``--height`` the launcher passes (too large
    under ToF -> ``buffer is too small for requested array``). Mirrors
    ``vio.main._await_calib_bundle`` / ``receive.py``'s calib-driven sizing.
    """
    box: list = [None]
    got = threading.Event()

    def _on(wm) -> None:
        box[0] = wm
        got.set()

    client = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    client.subscribe("calib.bundle", _on)
    client.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(
                f"forward: no calib.bundle from {endpoint!r} in {timeout_s}s")
    finally:
        client.stop()
    assert box[0] is not None
    return box[0]


def _attach_all_retry(specs, timeout_s: float, label: str) -> "RingRegistry":
    """Attach a ring set, RETRYING while the producer is still creating it.

    The rings the forwarders read are created by SEPARATE processes (capture /
    vio) that boot CONCURRENTLY with this forwarder -- e.g. vio allocates its
    ``kf_gray``/``kf_depth`` rings only after it has come up and awaited capture's
    calib, which is AFTER this forwarder gets capture's calib and races ahead. So
    a ring can legitimately not exist the instant we attach. Retry on
    FileNotFoundError until the producer has created it (or the timeout -> a clear
    error, not a startup-race crash). The loopback selftest pre-creates every ring
    so it never exercised this; the real launcher does.
    """
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return RingRegistry().attach_all(specs)
        except FileNotFoundError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.2)


def run_forward(*, host: str, port: int,
                capture_endpoint: str, vio_endpoint: str, slam_endpoint: str,
                width: int, height: int, slots: int = 64,
                connect_timeout_s: float = 30.0,
                calib_timeout_s: float = 60.0,
                pose_only: bool = False,
                ready_event: threading.Event | None = None,
                stop_event: threading.Event | None = None) -> None:
    """Start the TCP server + the three endpoint forwarders; block until SIGTERM.

    ``width`` / ``height`` size the Pi-side rings the forwarder ATTACHES to (it
    must match what capture/vio created -- the same ToF 54x42 vs 640x400 contract
    the rest of the stack honours). ``pose_only`` is the low-bandwidth mode: the
    heavy shm-backed image topics are EXCLUDED from every forwarder (and from the
    TcpServer's latest-wins set), so the bridge carries only the small POD + retained
    topics the trajectory + map UI needs -- and never attaches the image rings at all
    (there is nothing to read from them). ``ready_event`` (test hook) is set once the
    TCP server is listening and all forwarders have started. ``stop_event`` (test
    hook) lets a caller running this OFF the main thread (where SIGTERM cannot be
    installed) stop it cleanly; when ``None`` a fresh event is used and SIGTERM /
    Ctrl-C drive the teardown.
    """
    include_images = not pose_only
    if pose_only:
        LOG.info("forward: POSE-ONLY mode (image topics NOT bridged -- camera/"
                 "depth/keyframe frames are excluded; the trajectory + map UI is "
                 "unaffected, only the opt-in camera Visualize windows lose frames)")

    # The launcher passes the RAW camera res, but under --vl53l9cx capture creates
    # its rings at the ToF grid (54x42). Await capture's retained calib.bundle (the
    # source of truth for the actual grid) and attach at THAT res -- else attach_all
    # sees a too-small shm buffer ("buffer is too small for requested array").
    bundle = _await_calib_bundle(capture_endpoint, calib_timeout_s)
    width, height = int(bundle.width), int(bundle.height)
    LOG.info("forward: calib received (%dx%d) -> attaching Pi rings at that res",
             width, height)

    # Pi-side rings the forwarders attach to (consumer side -- capture/vio own
    # them). Retry: capture/vio create these asynchronously as they boot, so a
    # ring (esp. vio's kf_gray/kf_depth) may not exist the instant we attach. In
    # pose-only mode no image topic is forwarded, so the rings would never be read;
    # skip attaching them entirely (empty registries) -- one less startup race.
    if pose_only:
        cap_rings = RingRegistry()
        vio_rings = RingRegistry()
    else:
        cap_rings = _attach_all_retry(
            default_capture_specs(endpoint=capture_endpoint,
                                  width=width, height=height, slots=slots),
            calib_timeout_s, "capture")
        vio_rings = _attach_all_retry(
            default_vio_specs(endpoint=vio_endpoint,
                              width=width, height=height, slots=slots),
            calib_timeout_s, "vio")
    # SLAM owns no rings (all its forwarded topics are POD), so an empty registry.
    slam_rings = RingRegistry()

    # ONE TcpServer for all three endpoints: union the image + retained sets so the
    # latest-wins / replay policy is applied per topic regardless of role. In
    # pose-only mode image_topics() returns empty, so the union is empty and the
    # server treats every forwarded topic as reliable POD/retained.
    image_union: set[str] = set()
    retained_union: set[str] = set()
    # SLAM is OPTIONAL: under the lean flight config (`--no-slam`) the launcher
    # passes an EMPTY slam endpoint, because no SLAM process exists -- attaching
    # the slam forwarder anyway would block on `oak.slam` and crash the whole
    # forward (and with it the remote UI) after the 30 s connect timeout.
    bridge_slam = bool(slam_endpoint)
    roles = ("capture", "vio", "slam") if bridge_slam else ("capture", "vio")
    for role in roles:
        image_union |= allow.image_topics(role, include_images=include_images)
        retained_union |= allow.retained_topics(role)
    server = TcpServer(host, port,
                       retain_topics=retained_union,
                       image_topics=image_union)

    forwarders = [
        EndpointForwarder("capture", capture_endpoint, server, cap_rings,
                          connect_timeout_s=connect_timeout_s,
                          include_images=include_images),
        EndpointForwarder("vio", vio_endpoint, server, vio_rings,
                          connect_timeout_s=connect_timeout_s,
                          include_images=include_images),
    ]
    if bridge_slam:
        forwarders.append(
            EndpointForwarder("slam", slam_endpoint, server, slam_rings,
                              connect_timeout_s=connect_timeout_s,
                              include_images=include_images))
    else:
        LOG.info("forward: SLAM bridge DISABLED (no slam endpoint -- --no-slam); "
                 "bridging capture + vio only")

    stop = stop_event if stop_event is not None else threading.Event()

    def _on_sigterm(_signo, _frame):
        LOG.info("forward: SIGTERM -> stopping")
        stop.set()
    install_sigterm(_on_sigterm)               # main-thread only (no-op threaded)

    try:
        server.start()
        for fwd in forwarders:
            fwd.start()
        if ready_event is not None:
            ready_event.set()
        LOG.info("forward: serving %s:%d (Ctrl-C / SIGTERM to stop)", host, port)
        while not stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        LOG.info("forward: SIGINT -> stopping")
    finally:
        LOG.info("forward: shutting down ...")
        for fwd in forwarders:
            fwd.stop()
        try:
            server.close()
        except Exception:                                          # noqa: BLE001
            pass
        cap_rings.close()
        vio_rings.close()
        slam_rings.close()
        LOG.info("forward: bye")


# --------------------------------------------------------------------------- #
def _parse_hostport(s: str, *, default_host: str = "0.0.0.0") -> tuple[str, int]:
    """Parse ``HOST:PORT`` (or bare ``:PORT`` / ``PORT``) -> ``(host, port)``."""
    s = s.strip()
    if ":" in s:
        host, _, port = s.rpartition(":")
        host = host or default_host
    else:
        host, port = default_host, s
    return host, int(port)


def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--listen", default="0.0.0.0:8787",
                    help="HOST:PORT to bind the TCP server (default 0.0.0.0:8787)")
    ap.add_argument("--capture-endpoint", default="oak.capture")
    ap.add_argument("--vio-endpoint", default="oak.vio")
    ap.add_argument("--slam-endpoint", default="oak.slam")
    ap.add_argument("--width", type=int, default=640,
                    help="capture width (must match the running capture process)")
    ap.add_argument("--height", type=int, default=400,
                    help="capture height (must match the running capture process)")
    ap.add_argument("--slots", type=int, default=64,
                    help="ring depth (must match the producers' create slots)")
    ap.add_argument("--connect-timeout", type=float, default=30.0)
    ap.add_argument("--pose-only", action="store_true",
                    help="LOW-BANDWIDTH mode: do NOT bridge the heavy image topics "
                         "(camera / depth / keyframe frames, ~51 Mbit/s). Only the "
                         "small POD + retained topics cross the wire, so the main "
                         "trajectory + map UI works fully over a slow WiFi link; the "
                         "opt-in camera Visualize windows just have no frames. The "
                         "Mac-side netbridge.receive MUST be run with --pose-only too.")
    args = ap.parse_args()

    host, port = _parse_hostport(args.listen)
    run_forward(host=host, port=port,
                capture_endpoint=args.capture_endpoint,
                vio_endpoint=args.vio_endpoint,
                slam_endpoint=args.slam_endpoint,
                width=args.width, height=args.height, slots=args.slots,
                connect_timeout_s=args.connect_timeout,
                pose_only=args.pose_only)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
