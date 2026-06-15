"""``comms`` -- the vendored comms contract shared bit-identically by every project.

This package is the merge of the old ``ours.lib.flow`` + ``ours.lib.ipc`` +
``ours.flows.bridge`` (plus the foundational ``ours.lib.{misc,config}``), renamed
for the split. It is COPIED bit-identically into each of the 5 projects
(imu_camera, depth, vio, slam, ui); CI diffs the copies, so all internal imports
are RELATIVE and the package pulls NO depthai / PyQt6 (headless-safe).

Two transports
--------------
* :class:`LocalPubSub` -- in-process pub/sub that passes Python objects DIRECTLY
  (zero serialization). The offline deterministic-replay path; byte-for-byte
  unchanged from the pre-split code.
* :class:`IPCPubSub` -- cross-process pub/sub over a Unix-domain socket. The wire
  encoding is the class-path-INDEPENDENT :mod:`comms.codec` (replacing pickle),
  keyed by ``(topic -> Wire* class, dataclass-field-order)``; large arrays travel
  through :class:`SharedArrayRing` shared memory, only metadata rides the codec.

Modules + steps
---------------
* :class:`Module` / :class:`SourceModule` / :class:`ModuleContext` -- the threaded
  reactive substrate (was Flow / SourceFlow / FlowContext).
* :class:`Step` -- the smallest input->output stage (was Task).

Bridges
-------
* :class:`IPCPublisher` / :class:`IPCSubscriber` -- glue a LocalPubSub <-> an
  IPCPubSub at the process boundary using :mod:`comms.converters` + the rings.
"""
from __future__ import annotations

from . import topics
from .codec import decode, encode
from .ipc import IPCPubSub
from .module import Module, ModuleContext, SourceModule
from .pubsub import LocalPubSub
from .ring_registry import RingRegistry, RingSpec
from .shared_array import SharedArrayRef, SharedArrayRing
from .step import Step
from .bridge import IPCPublisher, IPCSubscriber

__all__ = [
    # transports
    "LocalPubSub",
    "IPCPubSub",
    # modules / steps
    "Module",
    "SourceModule",
    "ModuleContext",
    "Step",
    # shared memory
    "SharedArrayRing",
    "SharedArrayRef",
    "RingRegistry",
    "RingSpec",
    # bridges
    "IPCPublisher",
    "IPCSubscriber",
    # topics + codec
    "topics",
    "encode",
    "decode",
]
