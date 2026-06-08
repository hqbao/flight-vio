"""``publish_imucam`` step: route the packed bundle onto the bus."""
from __future__ import annotations

from imu_camera.comms import Step, topics
from imu_camera.comms.messages import ImuCamPacket


class PublishImuCamStep(Step):
    name = "publish_imucam"

    def run(self, ctx, msg: ImuCamPacket):
        ctx.bus.publish(topics.IMUCAM_SAMPLE, msg)
        return msg                         # pass on to depth (when matcher wired)
