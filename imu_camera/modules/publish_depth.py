"""``publish_depth`` step: emit the computed depth frame on ``frame.depth``."""
from __future__ import annotations

from imu_camera.comms import Step, topics
from imu_camera.comms.messages import DepthFrame


class PublishDepthStep(Step):
    name = "publish_depth"

    def run(self, ctx, msg: DepthFrame):
        ctx.bus.publish(topics.FRAME_DEPTH, msg)
        return None
