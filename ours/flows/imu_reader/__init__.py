"""imu-reader flow: buffer raw IMU, pack it against each camera trigger.

The other half of the split-acquisition front-end (``cam_reader`` is the other).
It reads the IMU continuously into a timestamped buffer and, for each
:class:`~ours.lib.flow.messages.CamSync` trigger the cam-reader publishes on
``cam.sync``, drains the buffer up to that frame's device timestamp and publishes
an :class:`~ours.lib.flow.messages.ImuCamPacket` (the frames bundled with exactly
the inertial samples in that frame's interval) on ``imucam.sample``.

Per the architecture, this subpackage is exactly ONE flow: the ``ImuReaderFlow``
plus its pack/publish tasks and the IMU sample sources (replay offline / live
OAK-D). depthai is only touched by :class:`~ours.flows.imu_reader.sources.LiveImuSource`,
imported lazily.
"""
from .imu_reader_flow import ImuReaderFlow

__all__ = ["ImuReaderFlow"]
