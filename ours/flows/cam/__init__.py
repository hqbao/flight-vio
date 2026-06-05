"""cam flow: pull stereo on a schedule, trigger the IMU pack.

One half of the acquisition front-end (``imu_cam`` is the other). It owns the
*schedule*: one stereo pair per scheduler tick (``fps`` Hz). For each pair it
publishes a single :class:`~ours.lib.flow.messages.CamSync` (the frames + their
device timestamp) on ``cam.sync`` -- the trigger the
:class:`~ours.flows.imu_cam.ImuCamFlow` reacts to.

Per the architecture, this subpackage is exactly ONE flow: the ``CamFlow`` plus
its publish task and the pull-based frame sources (replay offline / live OAK-D).
depthai is only touched by :class:`~ours.flows.cam.sources.LiveCamSource`,
imported lazily, so the offline path never pulls the device library.
"""
from .cam_flow import CamFlow

__all__ = ["CamFlow"]
