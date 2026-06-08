"""Thread-safe publish/subscribe bus for inter-module communication.

Modules are independent threads. They never call each other directly; they
exchange data only through this bus. A publisher calls
``bus.publish(topic, msg)`` and every handler registered for that topic is
invoked synchronously on the publisher's thread. Modules register a handler that
drops the message into their own inbox queue, so the real work always runs on the
*subscribing* module's thread (actor model) -- the publish call itself stays
cheap and non-blocking.

Topics are plain strings. The canonical set used by the live pipeline lives in
:mod:`comms.topics`.

This is the in-process transport: it passes Python objects DIRECTLY (zero
serialization), so the offline deterministic-replay path stays byte-for-byte
identical. The cross-process transport is :class:`comms.ipc.IPCPubSub`.
"""
from __future__ import annotations

import threading
from collections import defaultdict
from typing import Any, Callable

Handler = Callable[[Any], None]


class LocalPubSub:
    """A minimal thread-safe in-process pub/sub bus (zero serialization)."""

    def __init__(self) -> None:
        self._subs: dict[str, list[Handler]] = defaultdict(list)
        self._lock = threading.Lock()

    def subscribe(self, topic: str, handler: Handler) -> None:
        """Register ``handler`` to be called for every message on ``topic``."""
        with self._lock:
            self._subs[topic].append(handler)

    def publish(self, topic: str, msg: Any) -> None:
        """Deliver ``msg`` to every subscriber of ``topic``.

        The subscriber list is copied under the lock and the handlers are then
        invoked outside the lock, so a handler may itself publish without
        deadlocking.
        """
        with self._lock:
            handlers = list(self._subs.get(topic, ()))
        for handler in handlers:
            handler(msg)
