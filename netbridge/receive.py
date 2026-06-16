"""netbridge.receive -- runs on the Mac: TCP -> local IPC (re-serve the endpoints).

The mirror image of :mod:`netbridge.forward`. It opens ONE
:class:`netbridge.tcp_transport.TcpClient` to the Pi, decodes every inbound frame
(all full-ndarray 0x08 -- forward guarantees no ``SharedArrayRef`` on the wire),
and RE-SERVES the canonical ``oak.capture`` / ``oak.vio`` / ``oak.slam`` AF_UNIX
endpoints so the Mac UI attaches EXACTLY as if it were running on the Pi -- the UI
is byte-for-byte unchanged.

Per re-served endpoint there is one ``comms.ipc.IPCPubSub(ep, role="server")``
(with the role's retained topics, so a late UI subscriber gets calib replayed) and
one ``comms.bridge.IPCPublisher`` against MAC-LOCAL rings. For each inbound frame:

* image topics -> :func:`netbridge.wire_full.full_wire_to_local` -> the local
  dataclass -> publish on the endpoint's private local bus -> ``IPCPublisher``
  writes the arrays into the Mac rings (0x09 refs over AF_UNIX) -> UI ``read_copy``s.
* POD topics -> ``comms.converters.to_local`` -> local dataclass -> same path
  (the POD converters never touch rings).
* retained topics (calib.bundle / calib.stereo / vio.map) -> published DIRECTLY as
  their ``Wire*`` form onto the server (``server.publish(topic, wire)``), exactly
  as capture/vio publish them in-host; the UI reads them straight off the wire.

CRITICAL -- ring sizing from calib: the Mac rings MUST be created at the SAME
resolution the Pi produced (54x42 for a ToF run, 640x400 otherwise). Hardcoding
640x400 would corrupt a 54x42 stream. So receive AWAITS the forwarded
``calib.bundle`` FIRST (like ``ui.main`` does), reads ``width`` / ``height`` from
it, and only THEN creates the rings + starts the publishers. The bundle (and any
other retained topic that arrived during the await) is then re-published so the UI
still gets it.

Run (on the Mac)::

    OAKD_NETBRIDGE_KEY=<secret> python -m netbridge.receive \\
        --connect <pi-host>:8787 \\
        --capture-endpoint oak.capture --vio-endpoint oak.vio \\
        --slam-endpoint oak.slam
"""
from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from netbridge.comms import topics                                            # noqa: E402
from netbridge.comms.bridge import IPCPublisher                               # noqa: E402
from netbridge.comms.converters import to_local                               # noqa: E402
from netbridge.comms.ipc import IPCPubSub                                     # noqa: E402
from netbridge.comms.pubsub import LocalPubSub                                # noqa: E402
from netbridge.comms.ring_registry import (                                   # noqa: E402
    RingRegistry, default_capture_specs, default_vio_specs,
)
from netbridge.comms.wire import WireCalibBundle, WireEnd                     # noqa: E402

from netbridge import topics_allowlist as allow                    # noqa: E402
from netbridge import wire_full                                     # noqa: E402
from netbridge.tcp_transport import TcpClient, install_sigterm     # noqa: E402

LOG = logging.getLogger("netbridge.receive")

#: Topics PUBLISHED DIRECTLY as their wire form onto the server (no local-dataclass
#: round-trip), mirroring how capture/vio publish them in-host. The single source
#: of truth is the allowlist (these are exactly the retained config topics).
_DIRECT_WIRE_TOPICS = allow.DIRECT_WIRE_TOPICS


# --------------------------------------------------------------------------- #
# One re-served endpoint: TCP frames -> private local bus -> IPCPublisher -> UI.
# --------------------------------------------------------------------------- #
class EndpointServer:
    """Re-serves ONE canonical endpoint (capture / vio / slam) on AF_UNIX.

    Built AFTER the calib resolution is known so its rings are sized correctly.
    Holds the ``IPCPubSub`` server (with the role's retained topics), the private
    local bus, the ``IPCPublisher`` (for image + POD topics), and the Mac-local
    rings. Retained topics bypass the publisher and ride ``server.publish`` directly.
    """

    def __init__(self, role: str, endpoint: str, *,
                 width: int, height: int, slots: int = 64,
                 include_images: bool = True) -> None:
        self.role = role
        self.endpoint = endpoint
        # In pose-only mode the image topics are excluded everywhere -- so this
        # endpoint neither serves them nor allocates their Mac-local rings.
        self._topics = allow.all_topics(role, include_images=include_images)
        self._retained = allow.retained_topics(role)
        # Topics that go through the publisher (image + POD = everything that is
        # NOT a direct-wire retained topic).
        self._published = [t for t in self._topics
                           if t not in _DIRECT_WIRE_TOPICS]

        # Mac-local rings the IPCPublisher writes into. Only capture + vio own
        # rings (the four ref-bearing image topics); slam is POD-only. In pose-only
        # mode no image topic is published, so there is nothing to write into a ring
        # -- skip allocating them entirely (empty registry for every role).
        if include_images and role == "capture":
            self.rings = RingRegistry().create_all(
                default_capture_specs(endpoint=endpoint,
                                      width=width, height=height, slots=slots))
        elif include_images and role == "vio":
            self.rings = RingRegistry().create_all(
                default_vio_specs(endpoint=endpoint,
                                  width=width, height=height, slots=slots))
        else:
            self.rings = RingRegistry()            # slam, or pose-only: no rings

        # The IPCPubSub server re-serving this endpoint. retain_topics so a late
        # UI subscriber gets calib / vio.map replayed off this server.
        self.server = IPCPubSub(endpoint, role="server",
                                retain_topics=set(self._retained))
        self._local_bus = LocalPubSub()
        # IPCPublisher mirrors the private local bus' published topics onto the
        # server, writing arrays into our rings (only the published, non-direct
        # topics; the direct-wire retained topics ride server.publish straight).
        self.publisher = IPCPublisher(
            self._local_bus, self.server, self.rings, self._published)

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Start the IPCPublisher (which starts the IPCPubSub server)."""
        self.publisher.start()
        LOG.info("receive[%s]: re-serving %s on %r",
                 self.role, sorted(self._topics), self.endpoint)

    def feed(self, topic: str, wm) -> None:
        """Route one decoded TCP frame onto this endpoint's serving path."""
        if isinstance(wm, WireEnd):
            # Per-topic clean end-of-stream: forward the END through the local bus
            # so any UI sink that guards WireEnd sees it (matches the in-host END).
            self.server.publish_end(topic)
            return
        if topic in _DIRECT_WIRE_TOPICS:
            # Retained config: publish the wire object straight (the server caches
            # + replays it; the UI reads it directly off the wire).
            self.server.publish(topic, wm)
            return
        if topic in wire_full.REF_BEARING_TOPICS:
            msg = wire_full.full_wire_to_local(topic, wm)
        else:
            msg = to_local(topic, wm, self.rings)
        # Publish on the private local bus; the IPCPublisher converts (writing
        # arrays into our rings) + forwards to the UI over AF_UNIX.
        self._local_bus.publish(topic, msg)

    def stop(self) -> None:
        try:
            self.publisher.stop()
        except Exception:                                          # noqa: BLE001
            pass
        self.rings.close()
        self.rings.unlink()


# --------------------------------------------------------------------------- #
def _topic_roles(topic: str, include_images: bool = True) -> list[str]:
    """Every re-served endpoint a topic belongs to (capture / vio / slam).

    Most topics belong to exactly ONE role, but ``calib.bundle`` is re-served on
    ALL THREE (capture publishes it; vio + slam republish it -- and the UI awaits
    it on the vio AND slam endpoints). The topic string on the wire is
    endpoint-agnostic, so receive fans such a topic out to every endpoint whose
    allowlist contains it, reproducing the in-host multi-publisher fan-out.

    ``include_images`` MUST match the mode the endpoints were built with: in
    pose-only mode the image topics are not in any allowlist, so an (unexpected)
    image frame would not resolve to a role -- consistent with the forward never
    sending one.
    """
    roles = [role for role in ("capture", "vio", "slam")
             if topic in allow.all_topics(role, include_images=include_images)]
    if not roles:
        raise KeyError(f"receive: topic {topic!r} not in any role allowlist")
    return roles


def _union_allowlist(include_images: bool = True) -> list[str]:
    """Every allowlisted topic across all three roles, de-duplicated, in order.

    One TcpClient carries all three endpoints' topics; the server forwards only
    what we subscribe, so we ask for the union. ``include_images=False`` (pose-only
    mode) leaves the heavy image topics OUT of the subscription, so the server never
    even sends them -- the bandwidth never leaves the Pi.
    """
    union: list[str] = []
    for role in ("capture", "vio", "slam"):
        for t in allow.all_topics(role, include_images=include_images):
            if t not in union:
                union.append(t)
    return union


# --------------------------------------------------------------------------- #
# Frame router: buffers inbound frames until the rings are sized, then routes.
# --------------------------------------------------------------------------- #
class _FrameRouter:
    """Single TcpClient handler that buffers, then routes, frames -- race-free.

    BEFORE the Mac rings are sized (no resolution known yet) every inbound frame
    is BUFFERED. The retained replay burst (calib.bundle / calib.stereo / vio.map)
    plus any live frames that race in during ring construction all land in the
    buffer, in arrival order. :meth:`go_live` (called under the same lock the
    recv-thread handler uses) drains the buffer through the now-built endpoints and
    flips to direct routing, so NO frame is ever lost or double-delivered at the
    handoff.

    ``self._lock`` is held for the whole buffer-append / drain-and-flip, so a frame
    the recv thread delivers concurrently is either fully buffered (and later
    drained) or fully routed -- never split.
    """

    def __init__(self, include_images: bool = True) -> None:
        self._lock = threading.Lock()
        self._live = False
        self._include_images = include_images
        self._pending: list[tuple[str, object]] = []
        self._endpoints: dict[str, "EndpointServer"] = {}
        self.calib: WireCalibBundle | None = None
        self.calib_event = threading.Event()

    # Called on the TcpClient recv thread for every decoded frame.
    def __call__(self, topic: str, wm) -> None:
        with self._lock:
            if not self._live:
                self._pending.append((topic, wm))
                # Surface the calib bundle so the main thread can size the rings.
                if (topic == topics.CALIB_BUNDLE
                        and not isinstance(wm, WireEnd)):
                    self.calib = wm
                    self.calib_event.set()
                return
        # Live path: route outside the lock (feed can be slow; the recv thread
        # must not hold the buffer lock through a ring write).
        self._route(topic, wm)

    def _route(self, topic: str, wm) -> None:
        # Fan out to EVERY endpoint that re-serves the topic (calib.bundle goes to
        # all three; everything else to exactly one).
        try:
            roles = _topic_roles(topic, include_images=self._include_images)
        except KeyError as e:
            LOG.warning("receive: %s", e)
            return
        for role in roles:
            try:
                self._endpoints[role].feed(topic, wm)
            except Exception as e:                                 # noqa: BLE001
                LOG.warning("receive: route %s -> %s failed: %s", topic, role, e)

    def go_live(self, endpoints: dict[str, "EndpointServer"]) -> None:
        """Install the endpoints, drain the buffer through them, flip to routing.

        Holds the lock across the drain + flip so a concurrently-arriving frame is
        serialised behind the buffered ones (preserving arrival order) and lands on
        exactly one path.
        """
        with self._lock:
            self._endpoints = endpoints
            buffered = list(self._pending)
            self._pending.clear()
            for topic, wm in buffered:
                self._route(topic, wm)
            self._live = True


def run_receive(*, host: str, port: int,
                capture_endpoint: str, vio_endpoint: str, slam_endpoint: str,
                slots: int = 64, calib_timeout_s: float = 60.0,
                pose_only: bool = False,
                ready_event: threading.Event | None = None,
                stop_event: threading.Event | None = None) -> None:
    """Connect, learn the resolution, build the Mac rings, re-serve. Block on SIGTERM.

    ``pose_only`` is the low-bandwidth mode and MUST match the Pi-side forward: the
    image topics are excluded from the subscription (so the server never sends
    them), and no image rings are allocated. receive still awaits the retained
    ``calib.bundle`` and re-serves the pose / map / overlay POD topics normally, so
    the main trajectory + map UI is unaffected -- only the opt-in camera Visualize
    windows have no frames. ``ready_event`` (test hook) is set once all three
    endpoints are serving and the buffered + live stream is flowing. ``stop_event``
    (test hook) lets a caller running this OFF the main thread (where SIGTERM cannot
    be installed) stop it cleanly; when ``None`` a fresh event is used and SIGTERM /
    Ctrl-C drive the teardown.
    """
    include_images = not pose_only
    if pose_only:
        LOG.info("receive: POSE-ONLY mode (image topics NOT subscribed -- the "
                 "forward must be in --pose-only too; the trajectory + map UI works "
                 "fully, only the camera Visualize windows have no frames)")
    # 1. Connect ONE TcpClient subscribed to the union allowlist; a single
    #    _FrameRouter buffers every inbound frame until the rings are sized. (Like
    #    ui.main awaits the bundle before building views -- we await it before
    #    building any ring; the bundle's W/H is the ONLY thing that sizes them.) In
    #    pose-only mode the image topics are NOT in the union, so they are never
    #    subscribed and never cross the wire.
    LOG.info("receive: connecting to %s:%d, awaiting calib.bundle ...", host, port)
    router = _FrameRouter(include_images=include_images)
    client = TcpClient(host, port, connect_timeout_s=calib_timeout_s)
    for t in _union_allowlist(include_images=include_images):
        client.subscribe(t, router)
    client.start()

    if not router.calib_event.wait(timeout=calib_timeout_s):
        try:
            client.stop()
        except Exception:                                          # noqa: BLE001
            pass
        raise TimeoutError(
            f"receive: no calib.bundle from {host}:{port} in {calib_timeout_s}s")
    bundle = router.calib
    assert bundle is not None
    width, height = int(bundle.width), int(bundle.height)
    LOG.info("receive: calib received (%dx%d) -> sizing Mac rings", width, height)

    # 2. Build + start the three re-served endpoints at the LEARNED resolution.
    #    (pose-only -> include_images False -> no image rings, no image serving.)
    endpoints = {
        "capture": EndpointServer("capture", capture_endpoint,
                                  width=width, height=height, slots=slots,
                                  include_images=include_images),
        "vio": EndpointServer("vio", vio_endpoint,
                              width=width, height=height, slots=slots,
                              include_images=include_images),
        "slam": EndpointServer("slam", slam_endpoint,
                               width=width, height=height, slots=slots,
                               include_images=include_images),
    }
    for ep in endpoints.values():
        ep.start()

    # 3. Install endpoints + drain the buffer (the calib + everything that raced in
    #    during ring construction) + flip the router to live -- atomically, so no
    #    frame is lost or double-delivered at the handoff.
    router.go_live(endpoints)

    if ready_event is not None:
        ready_event.set()

    stop = stop_event if stop_event is not None else threading.Event()

    def _on_sigterm(_signo, _frame):
        LOG.info("receive: SIGTERM -> stopping")
        stop.set()
    install_sigterm(_on_sigterm)               # main-thread only (no-op threaded)

    try:
        LOG.info("receive: serving cap=%r vio=%r slam=%r (Ctrl-C / SIGTERM to stop)",
                 capture_endpoint, vio_endpoint, slam_endpoint)
        while not stop.is_set():
            if client.error:
                LOG.warning("receive: TCP client error: %s", client.error)
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        LOG.info("receive: SIGINT -> stopping")
    finally:
        LOG.info("receive: shutting down ...")
        try:
            client.stop()
        except Exception:                                          # noqa: BLE001
            pass
        for ep in endpoints.values():
            ep.stop()
        LOG.info("receive: bye")


# --------------------------------------------------------------------------- #
def _parse_hostport(s: str, *, default_host: str = "127.0.0.1") -> tuple[str, int]:
    """Parse ``HOST:PORT`` (or bare ``PORT``) -> ``(host, port)``."""
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
    ap.add_argument("--connect", required=True,
                    help="Pi HOST:PORT to connect the TCP client to")
    ap.add_argument("--capture-endpoint", default="oak.capture")
    ap.add_argument("--vio-endpoint", default="oak.vio")
    ap.add_argument("--slam-endpoint", default="oak.slam")
    ap.add_argument("--slots", type=int, default=64,
                    help="Mac ring depth (>= the IPCPubSub outbound cap)")
    ap.add_argument("--calib-timeout", type=float, default=60.0,
                    help="seconds to wait for the forwarded calib.bundle")
    ap.add_argument("--pose-only", action="store_true",
                    help="LOW-BANDWIDTH mode: do NOT subscribe the heavy image "
                         "topics (camera / depth / keyframe frames). The Pi-side "
                         "netbridge.forward MUST be run with --pose-only too. The "
                         "main trajectory + map UI works fully; the opt-in camera "
                         "Visualize windows just have no frames.")
    args = ap.parse_args()

    host, port = _parse_hostport(args.connect)
    run_receive(host=host, port=port,
                capture_endpoint=args.capture_endpoint,
                vio_endpoint=args.vio_endpoint,
                slam_endpoint=args.slam_endpoint,
                slots=args.slots, calib_timeout_s=args.calib_timeout,
                pose_only=args.pose_only)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
