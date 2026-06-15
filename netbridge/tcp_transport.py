"""AF_INET frame transport for the cross-machine bridge.

The network analogue of the AF_UNIX :class:`comms.ipc.IPCPubSub` boundary. Where
``IPCPubSub`` length-frames ``codec.encode`` bytes over a Unix-domain socket on one
host, this carries the SAME codec bytes over a TCP socket between hosts, with:

* **Authkey (HMAC challenge-response).** ``multiprocessing.connection`` performs a
  standard sha256 challenge-response **at connect time** (NOT per message) when
  constructed with ``authkey=<bytes>``. The key comes from the ``OAKD_NETBRIDGE_KEY``
  environment variable; when that is unset both ends fall back to the SAME public
  :data:`DEFAULT_AUTHKEY` so the bridge connects with no setup (see
  :func:`resolve_authkey`). The bridge is therefore ALWAYS authenticated -- it never
  opens a no-auth socket. A peer with the wrong key fails the handshake ->
  ``AuthenticationError`` -> connection refused.

  HONEST scope: the authkey AUTHENTICATES the peer but does NOT encrypt the stream,
  and being a one-time handshake it costs nothing once streaming. The default key is
  PUBLIC (in the source), so it is convenience auth for a trusted LAN, not a secret;
  export a real ``OAKD_NETBRIDGE_KEY`` for security. For an untrusted network, run it
  inside a Wireguard tunnel or an SSH ``-L`` forward (netbridge then sees only
  loopback and the tunnel encrypts) -- the tunnel provides confidentiality.

* **Framing.** Every published message rides as raw bytes:
  ``conn.send_bytes(codec.encode(topic, wire_msg))`` on the wire, decoded with
  ``codec.decode(conn.recv_bytes())`` on the far side. ``send_bytes`` /
  ``recv_bytes`` already length-frame on the socket, so the codec body carries NO
  extra length prefix -- exactly the ``IPCPubSub`` contract.

* **``_BYE`` sentinel.** A fixed 3-byte frame (``b"BYE"``) marks a clean
  end-of-stream, identical to ``comms.ipc._BYE``.

* **Retained-topic replay on connect.** A port of ``IPCPubSub._accept_loop``'s
  retained replay: the server caches the latest encoded payload per retained topic
  (calib.bundle / calib.stereo / vio.map) and sends them to a peer immediately
  after the handshake, so a UI that connects LATE never misses the one-shot
  calibration it needs to size its rings.

Roles
-----
* :class:`TcpServer` (``role="server"``, runs on the Pi inside ``forward``): bind +
  accept, per-connection fan-out thread, latest-wins drop on a slow/stalled WiFi
  link for image topics (never back-pressure the flight stack), reliable delivery
  for POD + retained topics.
* :class:`TcpClient` (``role="client"``, runs on the Mac inside ``receive``):
  connect (with retry), declare the topic allowlist in the handshake, and invoke a
  per-topic handler for every inbound frame on its recv thread.
"""
from __future__ import annotations

import json
import logging
import os
import queue
import signal
import socket
import threading
import time
from collections import defaultdict
from multiprocessing import connection
from typing import Callable, Iterable

from netbridge.comms import codec

LOG = logging.getLogger("netbridge.tcp")

#: Environment variable carrying the shared HMAC authkey (UTF-8). REQUIRED on both
#: ends -- the transport refuses to start if it is unset or empty.
AUTHKEY_ENV = "OAKD_NETBRIDGE_KEY"

#: Control sentinel marking a clean end-of-stream (matches ``comms.ipc._BYE``).
_BYE = b"BYE"

#: A per-connection outbound queue is bounded so in-flight memory is predictable.
#: Image topics drop-oldest on a full queue (latest-wins); POD + retained topics
#: block briefly so a momentary stall never silently drops a calib / pose.
_DEFAULT_OUTBOUND_CAP = 64

#: Handler invoked on the client recv thread for each decoded ``(topic, wire_msg)``.
Handler = Callable[[str, object], None]


# --------------------------------------------------------------------------- #
# Authkey resolution
# --------------------------------------------------------------------------- #
def install_sigterm(handler) -> bool:
    """Register ``handler`` for SIGTERM, but ONLY on the main thread.

    ``signal.signal`` raises ``ValueError`` off the main thread, which is exactly
    where ``run_forward`` / ``run_receive`` run under the loopback selftest (each
    in its own daemon thread). The ``python -m netbridge.{forward,receive}`` entry
    points always call this on the main thread (clean SIGTERM teardown), so we
    register there and SILENTLY skip when threaded -- the selftest stops the
    workers via their ``stop`` event instead. Returns True if registered.
    """
    if threading.current_thread() is not threading.main_thread():
        return False
    try:
        signal.signal(signal.SIGTERM, handler)
        return True
    except ValueError:
        return False


#: Built-in fallback authkey used when ``OAKD_NETBRIDGE_KEY`` is unset. It is PUBLIC
#: (it lives right here in the source), so it is NOT a secret -- it authenticates
#: against accidental / casual connects on a trusted LAN and spares you typing a key
#: while testing. Set ``OAKD_NETBRIDGE_KEY`` to a real secret on an untrusted network.
DEFAULT_AUTHKEY = "oakd-netbridge-default-key"


def resolve_authkey() -> bytes:
    """Return the HMAC authkey: ``$OAKD_NETBRIDGE_KEY`` if set, else a built-in DEFAULT.

    Never raises and never returns ``None`` -- the bridge is ALWAYS authenticated, so
    it can never open a no-auth socket. The auth is a CONNECT-TIME HMAC
    challenge-response (``multiprocessing.connection``), NOT a per-message MAC, and it
    does NOT encrypt the stream.

    * ``OAKD_NETBRIDGE_KEY`` set  -> use it (a real shared secret; for untrusted nets).
    * unset                       -> both ends fall back to the SAME public
      :data:`DEFAULT_AUTHKEY` and connect with no setup. Convenient for trusted-LAN
      testing; it is NOT a secret (anyone with the source knows it), so a custom key
      is still what gives real security. A custom key is no slower -- the handshake is
      one-time either way.
    """
    raw = os.environ.get(AUTHKEY_ENV, "")
    if raw:
        return raw.encode("utf-8")
    LOG.warning(
        "%s unset -- using the built-in DEFAULT bridge key (fine for a trusted LAN; "
        "it is PUBLIC, so set %s to a real secret on an untrusted network).",
        AUTHKEY_ENV, AUTHKEY_ENV)
    return DEFAULT_AUTHKEY.encode("utf-8")


# --------------------------------------------------------------------------- #
# Per-subscriber-connection state held by the server role.
# --------------------------------------------------------------------------- #
class _ConnState:
    """One accepted TCP subscriber: its socket, topic set, and outbound queue."""

    __slots__ = ("conn", "topics", "outbox", "thread", "alive")

    def __init__(self, conn: "connection.Connection",
                 topics: set[str], cap: int) -> None:
        self.conn = conn
        self.topics: set[str] = topics
        self.outbox: "queue.Queue" = queue.Queue(maxsize=cap)
        self.thread: threading.Thread | None = None
        self.alive = True


# --------------------------------------------------------------------------- #
# Server role -- runs on the Pi (inside forward).
# --------------------------------------------------------------------------- #
class TcpServer:
    """AF_INET publish side: accept subscribers, fan out encoded frames.

    Built by ``netbridge.forward``. The forward bridge taps each local topic, then
    calls :meth:`publish_encoded` with the topic + the codec bytes (the bytes are
    encoded ONCE in forward -- the single re-encode point -- not per-subscriber, so
    the full-ndarray guarantee holds and the encode cost is paid once).

    ``retain_topics`` are cached (latest encoded payload per topic) and replayed to
    each new subscriber right after the handshake.

    ``image_topics`` use latest-wins drop on a full outbox (a stale frame beats a
    stalled producer over WiFi); every other topic blocks briefly so a one-shot
    calib / a pose is never silently dropped.
    """

    def __init__(self, host: str, port: int, *,
                 retain_topics: Iterable[str] = (),
                 image_topics: Iterable[str] = (),
                 outbound_cap: int = _DEFAULT_OUTBOUND_CAP) -> None:
        self._authkey = resolve_authkey()          # env key or the built-in default
        self._host = str(host)
        self._port = int(port)
        self._retain_topics = set(retain_topics)
        self._image_topics = set(image_topics)
        self._outbound_cap = int(outbound_cap)
        #: latest ENCODED payload per retained topic (already codec.encode'd).
        self._retained: dict[str, bytes] = {}
        self._listener: connection.Listener | None = None
        self._accept_thread: threading.Thread | None = None
        self._conns: list[_ConnState] = []
        self._lock = threading.Lock()
        self._stopped = threading.Event()

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Bind the TCP socket and start accepting subscribers (idempotent)."""
        with self._lock:
            if self._listener is not None:
                return
            # AF_INET listener with the HMAC authkey; backlog so a UI restart can
            # reconnect without the accept queue overflowing.
            self._listener = connection.Listener(
                address=(self._host, self._port), family="AF_INET",
                authkey=self._authkey, backlog=8)
            self._accept_thread = threading.Thread(
                target=self._accept_loop, name="netbridge-tcp-accept",
                daemon=True)
            self._accept_thread.start()
            LOG.info("TcpServer listening on %s:%d (retain=%s, image=%s)",
                     self._host, self._port, sorted(self._retain_topics),
                     sorted(self._image_topics))

    # ------------------------------------------------------------------ #
    def publish_encoded(self, topic: str, payload: bytes) -> None:
        """Fan ``payload`` (already ``codec.encode``'d) to every subscriber of ``topic``.

        Encoding happens ONCE in the caller (forward), so this method never sees a
        wire object -- only the finished bytes. Retained topics are cached here so a
        late subscriber gets the latest. After :meth:`close` this is a no-op.
        """
        if self._stopped.is_set():
            return
        if topic in self._retain_topics:
            with self._lock:
                self._retained[topic] = payload
        with self._lock:
            conns = [c for c in self._conns if c.alive and topic in c.topics]
        blocking = topic not in self._image_topics
        for c in conns:
            self._enqueue(c, (topic, payload), blocking)

    # ------------------------------------------------------------------ #
    def close(self) -> None:
        """Stop accepting + drain each subscriber (BYE) + close the socket. Idempotent.

        Mirrors ``IPCPubSub.close``: gate further publishes, then put a ``_BYE`` on
        each outbox and join the fan-out thread so everything already in flight is
        delivered before the connection is torn down.
        """
        if self._stopped.is_set():
            return
        self._stopped.set()
        if self._listener is not None:
            try:
                self._listener.close()
            except Exception:                                      # noqa: BLE001
                pass
            self._listener = None
        with self._lock:
            conns = list(self._conns)
            self._conns.clear()
        for c in conns:
            try:
                c.outbox.put(_BYE, timeout=1.0)
            except queue.Full:
                # Force room then BYE so the drain loop always sees the sentinel.
                try:
                    c.outbox.get_nowait()
                except queue.Empty:
                    pass
                try:
                    c.outbox.put_nowait(_BYE)
                except queue.Full:
                    pass
        for c in conns:
            if c.thread is not None:
                c.thread.join(timeout=5.0)
            c.alive = False
            try:
                c.conn.close()
            except Exception:                                      # noqa: BLE001
                pass

    # ------------------------------------------------------------------ #
    # Server internals
    # ------------------------------------------------------------------ #
    def _accept_loop(self) -> None:
        listener = self._listener
        while not self._stopped.is_set() and listener is not None:
            try:
                conn = listener.accept()           # does the HMAC handshake
            except OSError:                         # listener closed -> exit
                return
            except Exception as e:                 # AuthenticationError etc.
                # A wrong-authkey peer raises here. Log + keep accepting (one bad
                # client must not take down the bridge).
                LOG.warning("TcpServer accept/auth failed: %s", e)
                continue
            # Read the subscribe handshake (one short JSON byte frame, blocking).
            try:
                hello = json.loads(conn.recv_bytes().decode("utf-8"))
            except Exception as e:                                 # noqa: BLE001
                LOG.warning("TcpServer handshake failed: %s", e)
                try:
                    conn.close()
                except Exception:                                  # noqa: BLE001
                    pass
                continue
            topics = (set(hello.get("topics", ()))
                      if isinstance(hello, dict) else set())
            state = _ConnState(conn, topics, self._outbound_cap)
            state.thread = threading.Thread(
                target=self._fanout_loop, args=(state,),
                name="netbridge-tcp-out", daemon=True)
            with self._lock:
                self._conns.append(state)
                # Replay retained topics this subscriber asked for (calib first,
                # so the receive side can size its rings before any frame lands).
                retained = [(t, p) for t, p in self._retained.items()
                            if t in topics]
            for t, p in retained:
                # Retained replay is RELIABLE (blocking): the UI's ring sizing
                # depends on calib.bundle, so it must never be dropped.
                self._enqueue(state, (t, p), blocking=True)
            state.thread.start()
            LOG.info("TcpServer subscriber connected for %s", sorted(topics))

    def _enqueue(self, state: _ConnState, item, blocking: bool) -> None:
        """Drop ``item`` (a ``(topic, payload)`` pair or ``_BYE``) into the outbox.

        ``blocking`` (POD + retained): :meth:`Queue.put` with a short timeout in a
        loop, giving up only if the subscriber dies or the server shuts down --
        nothing is silently dropped. NON-blocking (image topics): drop-oldest,
        append-newest (latest-wins) so a WiFi stall never back-pressures the
        flight stack -- a stale frame is preferable to a stalled producer.
        """
        if not state.alive:
            return
        if blocking:
            while True:
                try:
                    state.outbox.put(item, timeout=0.1)
                    return
                except queue.Full:
                    if not state.alive or self._stopped.is_set():
                        return
        # Latest-wins (image topics).
        try:
            state.outbox.put_nowait(item)
        except queue.Full:
            try:
                state.outbox.get_nowait()
            except queue.Empty:
                pass
            try:
                state.outbox.put_nowait(item)
            except queue.Full:
                pass

    def _fanout_loop(self, state: _ConnState) -> None:
        """Drain ``state.outbox`` onto the TCP socket until BYE / EOF.

        Each item is already-encoded bytes (no per-connection re-encode): pop, send
        with ``send_bytes`` (which length-frames), repeat. ``_BYE`` is checked
        BEFORE ``state.alive`` so a clean :meth:`close` delivers the rest of the
        queue first -- the same ordering ``IPCPubSub._fanout_loop`` relies on.
        """
        try:
            while True:
                item = state.outbox.get()
                if item is _BYE:
                    try:
                        state.conn.send_bytes(_BYE)
                    except Exception:                              # noqa: BLE001
                        pass
                    return
                if not state.alive:
                    return
                _topic, payload = item
                try:
                    state.conn.send_bytes(payload)
                except (BrokenPipeError, EOFError, OSError):
                    state.alive = False
                    return
                except Exception as e:                             # noqa: BLE001
                    LOG.warning("TcpServer send failed: %s", e)
                    state.alive = False
                    return
        finally:
            try:
                state.conn.close()
            except Exception:                                      # noqa: BLE001
                pass


# --------------------------------------------------------------------------- #
# Client role -- runs on the Mac (inside receive).
# --------------------------------------------------------------------------- #
class TcpClient:
    """AF_INET subscribe side: connect, declare topics, decode + dispatch frames.

    Built by ``netbridge.receive``. :meth:`subscribe` registers a handler per
    topic BEFORE :meth:`start`; the topic list is sent in the connect handshake so
    the server forwards only what the UI needs. The recv thread does the
    ``codec.decode`` and invokes the per-topic handler with ``(topic, wire_msg)``.
    """

    def __init__(self, host: str, port: int, *,
                 connect_timeout_s: float = 30.0,
                 connect_retry_s: float = 0.5) -> None:
        self._authkey = resolve_authkey()          # env key or the built-in default
        self._host = str(host)
        self._port = int(port)
        self._connect_timeout_s = float(connect_timeout_s)
        self._connect_retry_s = float(connect_retry_s)
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._subs_lock = threading.Lock()         # guards _subs vs the recv thread
        self._conn: "connection.Connection | None" = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = threading.Event()
        self._error: str | None = None

    @property
    def error(self) -> str | None:
        return self._error

    # ------------------------------------------------------------------ #
    def subscribe(self, topic: str, handler: Handler) -> None:
        """Register ``handler`` for ``topic`` (must be before :meth:`start`)."""
        if self._started.is_set():
            raise RuntimeError("TcpClient already started -- subscribe first")
        self._subs[topic].append(handler)

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Connect (with retry + HMAC handshake), send the topic list, recv loop."""
        if self._started.is_set():
            return
        conn = self._connect_with_retry()
        try:
            hello = json.dumps({"role": "subscriber",
                                "topics": list(self._subs.keys())})
            conn.send_bytes(hello.encode("utf-8"))
        except Exception as e:                                     # noqa: BLE001
            try:
                conn.close()
            except Exception:                                      # noqa: BLE001
                pass
            raise ConnectionError(f"TcpClient handshake failed: {e}") from e
        self._conn = conn
        self._thread = threading.Thread(
            target=self._recv_loop, name="netbridge-tcp-in", daemon=True)
        self._thread.start()
        self._started.set()

    def stop(self, timeout: float = 2.0) -> None:
        """Close the connection + join the recv thread. Idempotent."""
        self._stop.set()
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:                                      # noqa: BLE001
                pass
            self._conn = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=timeout)

    # ------------------------------------------------------------------ #
    def _connect_with_retry(self) -> "connection.Connection":
        """Retry :func:`connection.Client` until the server is up or timeout.

        The Pi-side server may not be listening yet when the Mac UI launches;
        rather than crash, wait up to ``connect_timeout_s``. A WRONG authkey raises
        ``AuthenticationError`` inside ``Client`` -- that is NOT retried (it is a
        configuration error, not a not-yet-up condition), so it surfaces fast.
        """
        deadline = time.monotonic() + self._connect_timeout_s
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return connection.Client(
                    address=(self._host, self._port), family="AF_INET",
                    authkey=self._authkey)
            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                # Server not up yet (or transient network) -> retry.
                last_err = e
                time.sleep(self._connect_retry_s)
        raise TimeoutError(
            f"TcpClient could not connect to {self._host}:{self._port} within "
            f"{self._connect_timeout_s}s (last error: {last_err})")

    def _recv_loop(self) -> None:
        conn = self._conn
        try:
            while not self._stop.is_set() and conn is not None:
                try:
                    raw = conn.recv_bytes()
                except EOFError:
                    return                            # server closed
                except (OSError, BrokenPipeError):
                    return
                except Exception as e:                             # noqa: BLE001
                    self._error = f"recv failed: {e}"
                    LOG.warning("TcpClient recv failed: %s", e)
                    return
                if raw == _BYE:
                    return
                try:
                    topic, msg = codec.decode(raw)
                except Exception as e:                             # noqa: BLE001
                    # A corrupt frame must not kill the recv loop.
                    LOG.warning("TcpClient decode failed: %s", e)
                    continue
                # Snapshot the handler list under the lock so a concurrent
                # rebind_all (receive's buffer->router swap) is atomic wrt us.
                with self._subs_lock:
                    handlers = list(self._subs.get(topic, ()))
                for h in handlers:
                    try:
                        h(topic, msg)
                    except Exception as e:                         # noqa: BLE001
                        LOG.warning("TcpClient handler for %s raised: %s",
                                    topic, e)
        finally:
            self._started.clear()
