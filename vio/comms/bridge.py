"""Bridge modules -- glue the in-process :class:`comms.pubsub.LocalPubSub` to the
cross-process :class:`comms.ipc.IPCPubSub`.

The whole point of these bridges is so the existing modules (``OdometryModule``,
``BackendModule``, ``SlamModule``, every UI sink) **do not change at all** when
the graph is split across processes. Each process keeps a local bus and runs its
own modules on it; the bridge modules translate at the boundary:

* :class:`IPCPublisher` -- a sink that subscribes to N local topics and publishes
  the matching wire messages on a server-role :class:`comms.ipc.IPCPubSub`. Lives
  in the producing process.
* :class:`IPCSubscriber` -- a source that pulls wire messages off a client-role
  :class:`comms.ipc.IPCPubSub` and republishes them on the local bus. Lives in the
  consuming process.

Both inherit from :class:`threading.Thread` only for lifecycle parity with the
other modules (start/stop/join): they have NO inbox / step chain because the
bridge is a pure subscribe-and-forward (all the real work runs inline in the
subscribe handlers, on the producing/consuming module's own thread). The mapping
between local dataclasses and wire-message types lives in :mod:`comms.converters`.
"""
from __future__ import annotations

import logging
import threading
from typing import Iterable

from .pubsub import LocalPubSub
from .ipc import IPCPubSub
from .converters import to_local, to_wire
from .ring_registry import RingRegistry

LOG_PUB = logging.getLogger("comms.bridge.pub")
LOG_SUB = logging.getLogger("comms.bridge.sub")


# --------------------------------------------------------------------------- #
# Publish side: local bus -> IPC server
# --------------------------------------------------------------------------- #
class IPCPublisher(threading.Thread):
    """A "sink" module that mirrors local topics onto a server-role IPCPubSub.

    ``server`` is a started-or-startable ``IPCPubSub(role="server")`` for the
    publisher ``endpoint`` (e.g. ``"oak.capture"``); ``rings`` must already be
    created via :meth:`RingRegistry.create_all` so the converters can write into
    the slots; ``topics`` is the list of local-bus topic names to forward. The
    server is started by this module (and closed on :meth:`stop`).

    The module holds **no inbox** -- it's a pure subscribe-and-forward bridge.
    The ``Thread.run`` body just waits for the stop event so the standard
    ``start()`` / ``join()`` lifecycle still works.
    """

    def __init__(self, local_bus: LocalPubSub, server: IPCPubSub,
                 rings: RingRegistry, topics: Iterable[str],
                 *, endpoint: str | None = None,
                 ring_endpoint: str | None = None) -> None:
        super().__init__(name=f"ipc-pub-{server.endpoint}", daemon=True)
        self.local_bus = local_bus
        self.server = server
        self.rings = rings
        self.endpoint = endpoint or server.endpoint
        # Ring names are namespaced by the producing endpoint -- e.g. capture
        # publishes "oak.capture.gray_left". A re-publisher (VIO republishing
        # capture's frames) may want a different ring namespace; default to the
        # server's endpoint.
        self.ring_endpoint = ring_endpoint or self.endpoint
        self._topics = list(topics)
        self._stop = threading.Event()
        # Subscriptions are made eagerly so messages published while this module
        # is "starting" are not lost. The server is started in `run` so the socket
        # only exists once we're committed to the lifecycle.
        for t in self._topics:
            local_bus.subscribe(t, self._make_forwarder(t))

    # ------------------------------------------------------------------ #
    def _make_forwarder(self, topic: str):
        """Closure that converts + publishes one message for ``topic``."""
        server = self.server
        rings = self.rings
        ring_endpoint = self.ring_endpoint

        def _forward(msg) -> None:
            try:
                wm = to_wire(topic, msg, rings, ring_endpoint)
            except Exception as e:                                 # noqa: BLE001
                LOG_PUB.warning("ipc-pub %s/%s: convert failed: %s",
                                self.endpoint, topic, e)
                return
            try:
                server.publish(topic, wm)
            except Exception as e:                                 # noqa: BLE001
                LOG_PUB.warning("ipc-pub %s/%s: send failed: %s",
                                self.endpoint, topic, e)

        return _forward

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        """Start the server, then idle until :meth:`stop`."""
        try:
            self.server.start()
        except Exception as e:                                     # noqa: BLE001
            LOG_PUB.error("ipc-pub %s: server.start failed: %s", self.endpoint, e)
            return
        self._stop.wait()

    def stop(self) -> None:
        """Idempotent shutdown: stop the wait, close the server socket."""
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            self.server.close()
        except Exception:                                          # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Subscribe side: IPC client -> local bus
# --------------------------------------------------------------------------- #
class IPCSubscriber(threading.Thread):
    """A "source" module that mirrors remote topics onto a local LocalPubSub.

    ``client`` must be an *unstarted* ``IPCPubSub(role="client")`` (this module
    calls ``.subscribe`` for every requested topic, then ``.start``). ``rings`` is
    the consumer-side :class:`RingRegistry` attached to the producer's shared
    memory. ``topics`` is the list of remote topic names to forward to the local
    bus.

    Threading is supplied by the underlying IPCPubSub recv thread: it invokes the
    per-topic handler on its own thread, which does the conversion and the local
    publish inline. The client is started in this module's ``run`` so a process
    can build its graph + bridges synchronously, then call :meth:`start` once to
    bring everything up.
    """

    def __init__(self, local_bus: LocalPubSub, client: IPCPubSub,
                 rings: RingRegistry, topics: Iterable[str]) -> None:
        super().__init__(name=f"ipc-sub-{client.endpoint}", daemon=True)
        self.local_bus = local_bus
        self.client = client
        self.rings = rings
        self._topics = list(topics)
        self._stop = threading.Event()
        for t in self._topics:
            client.subscribe(t, self._make_forwarder(t))

    # ------------------------------------------------------------------ #
    def _make_forwarder(self, topic: str):
        """Closure that re-hydrates + republishes one wire message."""
        local_bus = self.local_bus
        rings = self.rings

        def _forward(wm) -> None:
            try:
                msg = to_local(topic, wm, rings)
            except Exception as e:                                 # noqa: BLE001
                LOG_SUB.warning("ipc-sub %s/%s: convert failed: %s",
                                self.client.endpoint, topic, e)
                return
            local_bus.publish(topic, msg)

        return _forward

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        try:
            self.client.start()
        except Exception as e:                                     # noqa: BLE001
            LOG_SUB.error("ipc-sub %s: client.start failed: %s",
                          self.client.endpoint, e)
            return
        self._stop.wait()

    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        try:
            self.client.stop()
        except Exception:                                          # noqa: BLE001
            pass
