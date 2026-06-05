"""imu_cam flow: buffer raw IMU, pack + depth against each camera trigger.

The other half of the acquisition front-end (``cam`` is the other). It reads the
IMU continuously into a timestamped buffer and, for each
:class:`~ours.lib.flow.messages.CamSync` trigger the cam flow publishes on
``cam.sync``, drains the buffer up to that frame's device timestamp and publishes
an :class:`~ours.lib.flow.messages.ImuCamPacket` (the frames bundled with exactly
the inertial samples in that frame's interval) on ``imucam.sample``. When given a
stereo matcher it also computes dense depth for the same pair inline and publishes
``frame.depth`` -- depth is a task in this flow, not a separate one, since it is
just a transform of the stereo pair this flow already produces.

Per the architecture, this subpackage is exactly ONE flow: the ``ImuCamFlow``
plus its pack/publish/depth tasks and the IMU sample sources (replay offline /
live OAK-D). depthai is only touched by
:class:`~ours.flows.imu_cam.sources.LiveImuSource`, imported lazily.
"""
from .imu_cam_flow import ImuCamFlow

__all__ = ["ImuCamFlow"]
