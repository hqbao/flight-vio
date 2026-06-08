"""``publish_correction`` step: emit the loop correction on ``loop.correction``."""
from __future__ import annotations

from slam.comms import topics
from slam.comms.messages import LoopCorrection
from slam.comms.step import Step


class PublishCorrection(Step):
    name = "publish_correction"

    def run(self, ctx, msg: LoopCorrection):
        ctx.bus.publish(topics.LOOP_CORRECTION, msg)
        return None
