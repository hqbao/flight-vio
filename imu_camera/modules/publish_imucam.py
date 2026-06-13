"""``publish_imucam`` step: route the packed bundle onto the bus."""
from __future__ import annotations

from imu_camera.comms import LocalPubSub, topics
from imu_camera.comms.messages import ImuCamPacket


def publish_imucam(bus: LocalPubSub, msg: ImuCamPacket) -> ImuCamPacket:
    """Publish the calibrated packet on ``IMUCAM_SAMPLE``; return it for depth.

    Was ``PublishImuCamStep(Step)``; identical publish, the bus passed explicitly.
    """
    bus.publish(topics.IMUCAM_SAMPLE, msg)
    return msg                         # pass on to depth (when matcher wired)
