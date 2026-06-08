"""Base class for a Step: one clear input -> one clear output stage.

A Step is the smallest unit of work inside a Module. It receives the module's
shared context and one message, and returns the message to hand to the next step
in the chain. Returning ``None`` stops the chain for that input (e.g. "no
keyframe this frame, nothing to forward").

Keep steps small and single-purpose. If a step file grows too long, split the
work into two steps.
"""
from __future__ import annotations

from typing import Any


class Step:
    """A single stage in a module's step chain."""

    #: Human-readable name, used in logs/diagnostics.
    name: str = "step"

    def run(self, ctx: Any, msg: Any) -> Any:
        """Process ``msg`` and return the value for the next step.

        ``ctx`` is the owning module's :class:`comms.module.ModuleContext`
        (gives access to the bus and the module-local ``state`` dict). Return
        ``None`` to halt the chain for this message.
        """
        raise NotImplementedError
