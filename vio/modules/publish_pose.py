"""``publish_pose`` task: emit the per-frame pose on ``pose.odom``."""
from __future__ import annotations

from vio.comms import topics
from vio.comms.messages import PoseMsg
from vio.comms import Step as StepBase
from .step import Step


class PublishPose(StepBase):
    name = "publish_pose"

    def run(self, ctx, step: Step):
        ctx.bus.publish(topics.POSE_ODOM,
                        PoseMsg(step.frame.seq, step.frame.ts_ns,
                                step.pose, step.info))
        return step
