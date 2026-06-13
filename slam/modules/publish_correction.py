"""``publish_correction`` -- emit a loop correction on ``loop.correction``.

A plain function (not a ``Step`` subclass): the bus is passed in explicitly. The
caller only invokes this when there IS a correction to emit (the old
``PublishCorrection.run`` was reached only when ``SlamStep`` returned non-None),
so there is no None-guard here -- the procedural chain in
:mod:`slam.modules.pipeline` makes that condition explicit at the call site.
"""
from __future__ import annotations

from slam.comms import LocalPubSub, topics
from slam.comms.messages import LoopCorrection


def publish_correction(bus: LocalPubSub, msg: LoopCorrection) -> None:
    """Publish the loop correction on ``loop.correction``."""
    bus.publish(topics.LOOP_CORRECTION, msg)
