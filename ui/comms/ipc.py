"""Cross-process pub/sub bus -- the wire side of :class:`comms.pubsub.LocalPubSub`.

A *publisher process* exposes ONE ``IPCPubSub(endpoint, role="server")`` on a
Unix-domain socket (macOS / Linux); each subscriber process opens ONE
``IPCPubSub(endpoint, role="client")`` per publisher it cares about. The API
mirrors the in-process :class:`comms.pubsub.LocalPubSub` so the modules + the
bridge modules look almost identical at the call site::

    # publisher (server role)
    bus = IPCPubSub("oak.capture", role="server")
    bus.start()
    bus.publish("imucam.sample", wire_msg)
    ...
    bus.close()

    # subscriber (client role)
    bus = IPCPubSub("oak.capture", role="client")
    bus.subscribe("imucam.sample", on_imucam)
    bus.subscribe("calib.bundle", on_calib)
    bus.start()                             # starts the background recv thread
    ...

Wire protocol
-------------
- Unix-domain socket via :class:`multiprocessing.connection.Listener` /
  :func:`multiprocessing.connection.Client`. Auth disabled (``authkey=None``)
  on Linux/macOS: the socket file lives under ``$TMPDIR/ours_ipc/`` with
  mode 0600 so only the current uid can connect. (Acceptable for a desktop dev
  tool; production would add HMAC.)
- Each connection starts with a one-line handshake carrying
  ``{"role": "subscriber", "topics": [...]}`` as a JSON byte frame
  (``send_bytes`` / ``recv_bytes``). The server records the topic list and only
  forwards matching messages to that connection.
- Every published message rides as RAW BYTES: ``conn.send_bytes(codec.encode(
  topic, wire_msg))``; the receiver does ``codec.decode(conn.recv_bytes())``.
  This replaces the implicit pickle ``conn.send`` / ``conn.recv`` of the old
  design so the wire format is class-path-INDEPENDENT (see :mod:`comms.codec`).
  ``send_bytes`` / ``recv_bytes`` already length-frame on the socket, so the
  codec body carries NO extra length prefix of its own.
- A control sentinel ``BYE`` is a fixed 3-byte frame (``b"BYE"``).
- Retained topics (e.g. ``calib.bundle``): when a subscriber connects, the server
  first replays the latest cached message for every retained topic the subscriber
  is interested in, so booting late never misses a one-shot configuration.

Threading
---------
``role="server"`` uses one *accept thread* and one *fan-out thread per
connection*. Publish is non-blocking from the caller's perspective: it drops the
wire message into each subscriber's outbound :class:`queue.Queue` (bounded,
latest-wins or blocking back-pressure depending on ``blocking``).

``role="client"`` uses one *receive thread* that reads frames and invokes the
user-registered handlers on that thread (the same actor model the in-proc bus
uses -- handlers typically drop into a module's inbox, so the real work runs on
the consuming module's own thread).

OFFLINE replay never imports this module; the single-process oracle path stays on
:class:`comms.pubsub.LocalPubSub`.
"""
from __future__ import annotations

import json
import logging
import os
import queue
import tempfile
import threading
from collections import defaultdict
from multiprocessing import connection
from typing import Any, Callable

from . import codec

LOG = logging.getLogger("comms.ipc")

Handler = Callable[[Any], None]

# A subscriber's outbound queue is bounded; in non-blocking mode, if it fills
# (slow consumer) the producer drops the oldest message and inserts the newest.
# This matches the "latest-only" inbox semantics used by the live modules and
# guarantees ``publish`` never blocks the producer thread.
_DEFAULT_OUTBOUND_CAP = 32

#: Control sentinel sent on the socket to mark a clean end-of-stream.
_BYE = b"BYE"


# --------------------------------------------------------------------------- #
# Endpoint helpers
# --------------------------------------------------------------------------- #
def _endpoint_path(name: str) -> str:
    """Resolve a logical endpoint name to a Unix-domain socket path.

    All sockets live under ``$TMPDIR/ours_ipc/`` (one directory per user,
    chmod 0700). The socket itself is created by :class:`Listener` with chmod
    0600 enforced below.
    """
    root = os.path.join(tempfile.gettempdir(), "ours_ipc")
    try:
        os.makedirs(root, mode=0o700, exist_ok=True)
    except OSError:
        pass
    try:
        os.chmod(root, 0o700)
    except OSError:
        pass
    return os.path.join(root, f"{name}.sock")


# --------------------------------------------------------------------------- #
# Per-subscriber-connection state held by the server role.
# --------------------------------------------------------------------------- #
class _ConnState:
    """Per-subscriber-connection state held by the server role."""

    __slots__ = ("conn", "topics", "outbox", "thread", "alive")

    def __init__(self, conn: "connection.Connection",
                 topics: set[str], cap: int) -> None:
        self.conn = conn
        self.topics: set[str] = topics
        self.outbox: "queue.Queue" = queue.Queue(maxsize=cap)
        self.thread: threading.Thread | None = None
        self.alive = True


class IPCPubSub:
    """Cross-process pub/sub over one Unix-domain socket.

    ``role="server"`` -- the PUBLISHING process: bind the socket, accept
    subscribers, and :meth:`publish` wire messages to the ones that asked for the
    topic. ``role="client"`` -- the SUBSCRIBING process: :meth:`subscribe` to N
    topics (before :meth:`start`), then :meth:`start` the background recv thread.

    The merged class keeps every behaviour of the old split server/client buses;
    the ONLY wire-format change is that messages ride raw codec bytes via
    ``send_bytes`` / ``recv_bytes`` instead of implicit pickle.
    """

    def __init__(self, endpoint: str, *, role: str = "server",
                 retain_topics: set[str] | None = None,
                 outbound_cap: int = _DEFAULT_OUTBOUND_CAP,
                 blocking: bool = True,
                 connect_timeout_s: float = 10.0,
                 connect_retry_s: float = 0.2) -> None:
        """Construct an unstarted bus.

        ``role`` is ``"server"`` (publish) or ``"client"`` (subscribe).

        Server-only knobs (``blocking`` back-pressure when a subscriber's outbox
        fills):

        * ``True`` (default) -- :meth:`publish` blocks the caller until space is
          available. The producer is throttled to the slowest consumer's rate;
          nothing is dropped. This is what offline replay + the smoke tests need,
          where every frame must reach VIO for correctness.
        * ``False`` -- the OLDEST queued message is dropped and the new one is
          appended (latest-wins semantics). Use for live operation where a stale
          marker beats a stalled producer.

        Either way the outbox is bounded at ``outbound_cap`` to keep in-flight
        memory predictable; in blocking mode the bound just becomes a throttle.

        Client-only knobs: ``connect_timeout_s`` / ``connect_retry_s`` govern how
        long :meth:`start` waits for the publisher's socket to appear.
        """
        role = str(role).lower()
        if role not in ("server", "client"):
            raise ValueError(f"role must be 'server' or 'client', got {role!r}")
        self.endpoint = endpoint
        self.role = role
        self._path = _endpoint_path(endpoint)

        # --- server-role state -------------------------------------------- #
        self._retain_topics = set(retain_topics or ())
        self._retained: dict[str, Any] = {}      # latest msg per retained topic
        self._outbound_cap = int(outbound_cap)
        self._blocking = bool(blocking)
        self._listener: connection.Listener | None = None
        self._accept_thread: threading.Thread | None = None
        self._conns: list[_ConnState] = []
        self._lock = threading.Lock()
        self._stopped = threading.Event()

        # --- client-role state -------------------------------------------- #
        self._connect_timeout_s = float(connect_timeout_s)
        self._connect_retry_s = float(connect_retry_s)
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._conn: "connection.Connection | None" = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._started = threading.Event()
        self._error: str | None = None

    @property
    def error(self) -> str | None:
        return self._error

    # ================================================================== #
    # Lifecycle dispatch
    # ================================================================== #
    def start(self) -> None:
        """Bring the bus up. Server: bind + accept. Client: connect + recv."""
        if self.role == "server":
            self._start_server()
        else:
            self._start_client()

    # ================================================================== #
    # Server role -- the publishing process
    # ================================================================== #
    def _start_server(self) -> None:
        """Bind the socket and start accepting subscribers (idempotent).

        Thread-safe: two modules that share one server bus may both call
        ``start`` from their own ``run`` thread (e.g. VIO runs two
        :class:`comms.bridge.IPCPublisher` against one server). Only the first
        caller binds; the rest return immediately.
        """
        with self._lock:
            if self._listener is not None:
                return
            # Clear any stale socket from a previous crash.
            try:
                if os.path.exists(self._path):
                    os.unlink(self._path)
            except OSError:
                pass
            # AF_UNIX listener, no authkey -- access controlled by FS perms.
            self._listener = connection.Listener(
                address=self._path, family="AF_UNIX")
            try:
                os.chmod(self._path, 0o600)
            except OSError:
                pass
            self._accept_thread = threading.Thread(
                target=self._accept_loop, name=f"ipc-{self.endpoint}-accept",
                daemon=True)
            self._accept_thread.start()
            LOG.info("IPCPubSub[%s] server listening on %s",
                     self.endpoint, self._path)

    def publish(self, topic: str, msg: Any) -> None:
        """Deliver ``msg`` (a ``Wire*`` instance) to every subscriber of ``topic``.

        Non-blocking from the caller's perspective: messages drop into each
        connection's bounded outbox; a fan-out thread codec-encodes + sends them.
        If a subscriber's outbox is full (slow consumer) the OLDEST queued message
        is dropped so the latest always wins (non-blocking mode).

        After :meth:`close` (``self._stopped`` is set) publishes become no-ops --
        the in-flight queue is still drained by the fan-out threads, but no
        further messages enter the system.
        """
        if self._stopped.is_set():
            return
        if topic in self._retain_topics:
            with self._lock:
                self._retained[topic] = msg
        with self._lock:
            conns = [c for c in self._conns if c.alive and topic in c.topics]
        for c in conns:
            self._enqueue(c, (topic, msg))

    def publish_end(self, topic: str) -> None:
        """Wire-side END sentinel for one topic (replay path)."""
        from .wire import WireEnd
        self.publish(topic, WireEnd(topic))

    def close(self) -> None:
        """Stop accepting + close every subscriber connection. Idempotent.

        Drains each subscriber's outbox before tearing down the socket: every
        message the publisher has already enqueued is delivered, then a BYE
        marker, then the connection is closed. This is what the offline replay
        path needs -- if a producer published N frames and immediately closes, we
        must NOT discard those N frames just because the close ran before the
        fanout thread caught up.

        We do gate further :meth:`publish` calls (via ``_stopped``) so new
        publishes after close are no-ops, but EVERYTHING already in flight gets
        through.

        For a client-role bus this delegates to :meth:`stop`.
        """
        if self.role == "client":
            self.stop()
            return
        if self._stopped.is_set():
            return
        self._stopped.set()
        # Stop accepting.
        if self._listener is not None:
            try:
                self._listener.close()
            except Exception:                                      # noqa: BLE001
                pass
            self._listener = None
        # Snapshot connections; signal each to drain + BYE.
        with self._lock:
            conns = list(self._conns)
            self._conns.clear()
        for c in conns:
            try:
                c.outbox.put(_BYE, timeout=1.0)
            except queue.Full:
                # The outbox was already at capacity; force-make-room then BYE.
                try:
                    c.outbox.get_nowait()
                except queue.Empty:
                    pass
                try:
                    c.outbox.put_nowait(_BYE)
                except queue.Full:
                    pass
        # Join each fanout thread -- it processes the rest of its outbox in order,
        # hits BYE, sends it, and exits. Generous timeout per conn so a slow
        # subscriber gets its data.
        for c in conns:
            if c.thread is not None:
                c.thread.join(timeout=5.0)
            c.alive = False
            try:
                c.conn.close()
            except Exception:                                      # noqa: BLE001
                pass
        # Best-effort socket unlink.
        try:
            if os.path.exists(self._path):
                os.unlink(self._path)
        except OSError:
            pass

    # ------------------------------------------------------------------ #
    # Server internals
    # ------------------------------------------------------------------ #
    def _accept_loop(self) -> None:
        listener = self._listener
        while not self._stopped.is_set() and listener is not None:
            try:
                conn = listener.accept()
            except OSError:                       # listener closed -> exit
                return
            except Exception as e:                                 # noqa: BLE001
                LOG.warning("IPCPubSub[%s] accept failed: %s",
                            self.endpoint, e)
                return
            # Read the subscribe handshake (one short JSON byte frame, blocking).
            try:
                hello = json.loads(conn.recv_bytes().decode("utf-8"))
            except Exception as e:                                 # noqa: BLE001
                LOG.warning("IPCPubSub[%s] handshake failed: %s",
                            self.endpoint, e)
                try:
                    conn.close()
                except Exception:                                  # noqa: BLE001
                    pass
                continue
            topics = set(hello.get("topics", ())) if isinstance(hello, dict) else set()
            state = _ConnState(conn, topics, self._outbound_cap)
            state.thread = threading.Thread(
                target=self._fanout_loop, args=(state,),
                name=f"ipc-{self.endpoint}-out", daemon=True)
            with self._lock:
                self._conns.append(state)
                # Replay retained messages this subscriber asked for, in
                # declaration order (calib first, then everything else).
                retained = [(t, m) for t, m in self._retained.items()
                            if t in topics]
            for t, m in retained:
                self._enqueue(state, (t, m))
            state.thread.start()
            LOG.info("IPCPubSub[%s] subscriber connected for %s",
                     self.endpoint, sorted(topics))

    def _enqueue(self, state: _ConnState, item) -> None:
        """Drop ``item`` (a ``(topic, msg)`` pair or ``_BYE``) into the outbox.

        Behaviour on a full outbox depends on the server's ``blocking`` flag:

        * blocking -- :meth:`Queue.put` with a short timeout in a loop, giving up
          only if the subscriber dies or the server is being shut down. Throttles
          the producer to the slowest consumer (offline / replay).
        * non-blocking -- drop oldest, append newest (live latest-wins).
        """
        if not state.alive:
            return
        if self._blocking:
            while True:
                try:
                    state.outbox.put(item, timeout=0.1)
                    return
                except queue.Full:
                    if not state.alive or self._stopped.is_set():
                        return
        # Latest-wins (non-blocking).
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
        """Drain ``state.outbox`` onto the socket until BYE / EOF.

        Each data item is codec-encoded HERE (on the fanout thread, off the
        producer's thread) and sent with ``send_bytes`` (which length-frames on
        the socket -- no extra length prefix needed).

        Order matters: BYE is checked BEFORE ``state.alive``. ``alive`` is only
        set False by send-errors (BrokenPipe etc.); :meth:`close` does NOT set it,
        because we must let the drain finish first. So a normal server shutdown::

            publisher -> publish N items -> close()
            close puts BYE on the queue, joins this thread
            this thread: pops item 1..N, encodes+sends them, pops BYE, sends, exits

        ...delivers every message that was in flight at close time.
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
                    # Set by a previous send-error: the conn is dead, no point
                    # spending time on remaining items.
                    return
                topic, msg = item
                try:
                    payload = codec.encode(topic, msg)
                except Exception as e:                             # noqa: BLE001
                    # A bad message must not kill the connection; log + skip it.
                    LOG.warning("IPCPubSub[%s] encode %s failed: %s",
                                self.endpoint, topic, e)
                    continue
                try:
                    state.conn.send_bytes(payload)
                except (BrokenPipeError, EOFError, OSError):
                    state.alive = False
                    return
                except Exception as e:                             # noqa: BLE001
                    LOG.warning("IPCPubSub[%s] send failed: %s",
                                self.endpoint, e)
                    state.alive = False
                    return
        finally:
            try:
                state.conn.close()
            except Exception:                                      # noqa: BLE001
                pass

    # ================================================================== #
    # Client role -- the subscribing process
    # ================================================================== #
    def subscribe(self, topic: str, handler: Handler) -> None:
        """Register ``handler`` for every message on ``topic`` (client role).

        Must be called BEFORE :meth:`start` (the topic list is sent in the connect
        handshake). Calling after start raises ``RuntimeError``.
        """
        if self.role != "client":
            raise RuntimeError(
                f"IPCPubSub[{self.endpoint}] subscribe is a client-role op "
                f"(this bus is role={self.role!r})")
        if self._started.is_set():
            raise RuntimeError(
                f"IPCPubSub[{self.endpoint}] already started -- "
                f"subscribe before start()")
        self._subs[topic].append(handler)

    def _start_client(self) -> None:
        """Connect to the server and start the receive thread.

        Blocks until either the connection succeeds, the connect timeout elapses
        (raises ``TimeoutError``), or the connect fails (raises
        ``ConnectionError``). The receive thread then runs in the background;
        :meth:`stop` to join it.
        """
        if self._started.is_set():
            return
        conn = self._connect_with_retry()
        # Send the handshake (subscribed topics) as a JSON byte frame.
        try:
            hello = json.dumps({"role": "subscriber",
                                "topics": list(self._subs.keys())})
            conn.send_bytes(hello.encode("utf-8"))
        except Exception as e:                                     # noqa: BLE001
            try:
                conn.close()
            except Exception:                                      # noqa: BLE001
                pass
            raise ConnectionError(
                f"IPCPubSub[{self.endpoint}] handshake failed: {e}") from e
        self._conn = conn
        self._thread = threading.Thread(
            target=self._recv_loop, name=f"ipc-{self.endpoint}-in",
            daemon=True)
        self._thread.start()
        self._started.set()

    def stop(self, timeout: float = 2.0) -> None:
        """Close the connection and join the receive thread (client). Idempotent."""
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
    # Client internals
    # ------------------------------------------------------------------ #
    def _connect_with_retry(self) -> "connection.Connection":
        """Retry :func:`connection.Client` until the socket file exists.

        The publisher may not have called server :meth:`start` yet when the
        subscriber boots; rather than crash, we wait up to ``connect_timeout_s``
        for the socket to appear.
        """
        import time
        deadline = time.monotonic() + self._connect_timeout_s
        last_err: Exception | None = None
        while time.monotonic() < deadline:
            try:
                return connection.Client(address=self._path, family="AF_UNIX")
            except (FileNotFoundError, ConnectionRefusedError) as e:
                last_err = e
                time.sleep(self._connect_retry_s)
            except Exception as e:                                 # noqa: BLE001
                last_err = e
                time.sleep(self._connect_retry_s)
        raise TimeoutError(
            f"IPCPubSub[{self.endpoint}] could not connect to {self._path} "
            f"within {self._connect_timeout_s}s (last error: {last_err})")

    def _recv_loop(self) -> None:
        conn = self._conn
        try:
            while not self._stop.is_set() and conn is not None:
                try:
                    raw = conn.recv_bytes()
                except EOFError:
                    return                            # publisher closed
                except (OSError, BrokenPipeError):
                    return
                except Exception as e:                             # noqa: BLE001
                    self._error = f"recv failed: {e}"
                    LOG.warning("IPCPubSub[%s] recv failed: %s",
                                self.endpoint, e)
                    return
                if raw == _BYE:
                    return
                try:
                    topic, msg = codec.decode(raw)
                except Exception as e:                             # noqa: BLE001
                    # A corrupt frame must not kill the receive loop.
                    LOG.warning("IPCPubSub[%s] decode failed: %s",
                                self.endpoint, e)
                    continue
                for h in list(self._subs.get(topic, ())):
                    try:
                        h(msg)
                    except Exception as e:                         # noqa: BLE001
                        # Handler errors must not kill the receive loop; log and
                        # keep delivering subsequent messages.
                        LOG.warning("IPCPubSub[%s] handler for %s raised: %s",
                                    self.endpoint, topic, e)
        finally:
            self._started.clear()
