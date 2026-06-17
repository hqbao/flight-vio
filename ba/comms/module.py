"""Module: one thread that runs a fixed list of steps sequentially.

A Module owns a single thread. Inside it, steps run one after another -- the
output of one step is the input of the next. Modules never call each other
directly; they communicate only through :class:`comms.pubsub.LocalPubSub`.

Two shapes of module:

``SourceModule``
    Produces messages on its own (e.g. grabbing frames from the camera).
    Subclass and override :meth:`SourceModule.produce` to yield raw items; each
    item is pushed through the step chain on the module's thread.

``Module`` (reactive)
    Waits for messages from the bus. Register a step chain per input topic with
    :meth:`Module.on`. Incoming messages are queued and fed through the matching
    chain on this module's thread, so heavy work never runs on the publisher.

Steps publish their results to the bus via ``ctx.bus.publish(topic, msg)`` --
usually in a small dedicated "publish" step at the end of the chain.
"""
from __future__ import annotations

import queue
import threading
from typing import Any, Iterable, Sequence

from .messages import END
from .pubsub import LocalPubSub
from .step import Step

_SENTINEL = object()
#: inbox payload marker: "the real message is the current self._latest[topic]".
_LATEST = object()


class ModuleContext:
    """Shared state handed to every step in a module.

    Exposes the bus (for publishing) and a module-local ``state`` dict that steps
    use to keep stateful helpers (the VO object, the stereo matcher, counters).
    """

    def __init__(self, bus: LocalPubSub, name: str) -> None:
        self.bus = bus
        self.name = name
        self.state: dict[str, Any] = {}


class _BaseModule(threading.Thread):
    def __init__(self, name: str, bus: LocalPubSub) -> None:
        super().__init__(name=name, daemon=True)
        self.bus = bus
        self.ctx = ModuleContext(bus, name)
        self._stop = threading.Event()
        self._downstream: list[str] = []

    def forwards_to(self, *topics: str) -> "_BaseModule":
        """Declare the topics this module publishes, so END is forwarded to them."""
        self._downstream.extend(topics)
        return self

    def _emit_end(self) -> None:
        for topic in self._downstream:
            self.bus.publish(topic, END)

    def stop(self) -> None:
        self._stop.set()

    @staticmethod
    def _run_chain(ctx: ModuleContext, steps: Sequence[Step], msg: Any) -> None:
        for step in steps:
            msg = step.run(ctx, msg)
            if msg is None:
                return


class Module(_BaseModule):
    """A reactive module: drains an inbox and routes messages by topic.

    By default the inbox is an unbounded FIFO: every published message is
    processed in order (needed for the VIO + deterministic offline replay, where
    dropping a frame would corrupt the result). A realtime *visualiser* graph, by
    contrast, must stay fresh: if its consumer chain is even slightly slower than
    the producer, a FIFO inbox grows without bound and the view falls seconds
    behind. Such a module is built with ``latest_only=True`` -- a **coalescing**
    inbox that keeps only the most-recent unprocessed message per topic, so the
    consumer always works on the freshest frame and the backlog is dropped.
    Latency is then bounded to ~one frame per stage regardless of the rate
    mismatch. ``END`` is a control signal and is never coalesced away.
    """

    def __init__(self, name: str, bus: LocalPubSub, *,
                 latest_only: bool = False) -> None:
        super().__init__(name, bus)
        self._inbox: "queue.Queue" = queue.Queue()
        self._routes: dict[str, list[Step]] = {}
        self._latest_only = bool(latest_only)
        self._latest: dict[str, Any] = {}        # topic -> newest unprocessed msg
        self._latest_lock = threading.Lock()
        self.done = threading.Event()  #: set after all expected ENDs are handled
        self.expected_ends = 1  #: a sink subscribing N END-bearing topics sets this to N
        self._ends_seen = 0
        self._emitted_end = False

    def on(self, topic: str, steps: Sequence[Step]) -> "Module":
        """Run ``steps`` (in order) whenever a message arrives on ``topic``."""
        self._routes[topic] = list(steps)
        if self._latest_only:
            self.bus.subscribe(topic, lambda m, t=topic: self._coalesce(t, m))
        else:
            self.bus.subscribe(topic, lambda m, t=topic: self._inbox.put((t, m)))
        return self

    def _coalesce(self, topic: str, msg: Any) -> None:
        """Keep only the newest unprocessed ``msg`` per topic (latest-only mode).

        The inbox carries just a topic *token*; the message itself lives in
        ``self._latest[topic]`` and is overwritten by each newer arrival, so a
        backlog never builds. A token is enqueued only when there was nothing
        pending for the topic (one token drives one drain) -- except ``END``,
        which always enqueues a token so it is delivered even if it overwrites a
        pending data frame (losing the last frame is fine; dropping END is not).
        """
        with self._latest_lock:
            pending = topic in self._latest
            self._latest[topic] = msg
            enqueue = (not pending) or (msg is END)
        if enqueue:
            self._inbox.put((topic, _LATEST))

    def on_end(self) -> None:
        """Hook called once END has been received. Override for custom drain."""

    def stop(self) -> None:
        super().stop()
        self._inbox.put((_SENTINEL, _SENTINEL))  # unblock the queue.get

    def run(self) -> None:
        while not self._stop.is_set():
            topic, msg = self._inbox.get()
            if msg is _SENTINEL:
                break
            if msg is _LATEST:
                # Coalescing inbox: the token names a topic; pull its current
                # newest message (None if already drained by an earlier token).
                with self._latest_lock:
                    msg = self._latest.pop(topic, _SENTINEL)
                if msg is _SENTINEL:
                    continue
            if msg is END:
                self._handle_end()
                continue
            self._run_chain(self.ctx, self._routes.get(topic, ()), msg)

    def _handle_end(self) -> None:
        self._ends_seen += 1
        # Emit our own END only once EVERY END-bearing input has drained
        # (expected_ends). A single-input module keeps the old behaviour
        # (expected_ends defaults to 1 -> emits on the first END); a
        # multi-input join (e.g. odometry on imucam.sample + frame.depth)
        # waits for all of them so it never signals "done" early.
        if self._ends_seen >= self.expected_ends and not self._emitted_end:
            self._emitted_end = True
            self._emit_end()
        self.on_end()
        if self._ends_seen >= self.expected_ends:
            self.done.set()


class SourceModule(_BaseModule):
    """A producing module: pushes self-generated items through one step chain."""

    def __init__(self, name: str, bus: LocalPubSub, steps: Sequence[Step]) -> None:
        super().__init__(name, bus)
        self.steps = list(steps)
        self.done = threading.Event()

    def produce(self) -> Iterable[Any]:
        """Yield raw items to feed into the step chain. Override in subclass."""
        return ()

    def run(self) -> None:
        for item in self.produce():
            if self._stop.is_set():
                break
            self._run_chain(self.ctx, self.steps, item)
        self._emit_end()
        self.done.set()
