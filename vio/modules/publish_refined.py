"""``publish_refined`` task: emit the BA-refined pose on ``pose.refined``."""
from __future__ import annotations

from vio.comms import topics
from vio.comms.messages import PoseMsg
from vio.comms import Step


class PublishRefined(Step):
    name = "publish_refined"

    def run(self, ctx, msg: PoseMsg):
        ctx.bus.publish(topics.POSE_REFINED, msg)
        return None
