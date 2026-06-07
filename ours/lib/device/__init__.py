"""``ours.lib.device`` -- live OAK-D acquisition + boot-time calibration.

The only hardware-touching corner of ``ours.lib``: opening the single shared
device and reading the boot references the live flow graph needs. ``depthai`` is
imported lazily inside these modules, so importing this package never pulls the
device library (keeps the offline path depthai-free).

* :class:`~ours.lib.device.oak_live.SharedLiveDevice` -- one reference-counted
  pipeline (stereo + IMU) shared by the cam/imu reader flows.
* :func:`~ours.lib.device.live_calib.read_live_calibration` -- intrinsics +
  IMU->camera extrinsics + the startup gravity-align / cached gyro bias.
"""
from __future__ import annotations

from .oak_live import SharedLiveDevice
from .live_calib import LiveFrontEndCalib, read_live_calibration

__all__ = ["SharedLiveDevice", "LiveFrontEndCalib", "read_live_calibration"]
