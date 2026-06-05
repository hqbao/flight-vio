"""``publish_imucam`` task: route the packed bundle onto the bus."""
from __future__ import annotations

from ...lib.flow import topics
from ...lib.flow.messages import ImuCamPacket
from ...lib.flow.task import Task


class PublishImuCam(Task):
    name = "publish_imucam"

    def run(self, ctx, msg: ImuCamPacket):
        ctx.bus.publish(topics.IMUCAM_SAMPLE, msg)
        return None
