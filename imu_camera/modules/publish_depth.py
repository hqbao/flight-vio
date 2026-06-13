"""``publish_depth`` step: emit the computed depth frame on ``frame.depth``."""
from __future__ import annotations

from imu_camera.comms import LocalPubSub, topics
from imu_camera.comms.messages import DepthFrame


def publish_depth(bus: LocalPubSub, frame: DepthFrame) -> None:
    """Publish the depth frame on ``FRAME_DEPTH``.

    Was ``PublishDepthStep(Step)``; identical publish, the bus passed explicitly.
    """
    bus.publish(topics.FRAME_DEPTH, frame)
