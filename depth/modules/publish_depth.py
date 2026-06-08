"""``publish_depth`` step: emit the computed depth frame on ``frame.depth``."""
from __future__ import annotations

from depth.comms import Step, topics
from depth.comms.messages import DepthFrame


class PublishDepthStep(Step):
    name = "publish_depth"

    def run(self, ctx, msg: DepthFrame):
        ctx.bus.publish(topics.FRAME_DEPTH, msg)
        return None
