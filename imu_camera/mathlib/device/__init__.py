"""``imu_camera.mathlib.device`` -- live OAK-D acquisition + boot-time calibration.

The only hardware-touching corner of the project's mathlib: opening the single
shared device and reading the boot references the live pipeline needs. ``depthai``
is imported lazily inside these modules, so importing this package never pulls the
device library (keeps the offline / replay path depthai-free).

* :class:`~imu_camera.mathlib.device.oak_live.SharedLiveDevice` -- one
  reference-counted pipeline (stereo + IMU) shared by the cam / imu reader
  modules.
* :func:`~imu_camera.mathlib.device.live_calib.read_live_calibration` --
  intrinsics + IMU->camera extrinsics + the startup gravity-align / cached gyro
  bias.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from .oak_live import SharedLiveDevice

__all__ = ["SharedLiveDevice", "LiveFrontEndCalib", "read_live_calibration"]

if TYPE_CHECKING:                       # pragma: no cover -- type-checkers only
    from .live_calib import LiveFrontEndCalib, read_live_calibration


def __getattr__(name: str):
    """Lazily re-export the live-calibration API.

    ``live_calib`` reads boot-time references via ``ResolutionProfile`` (from the
    vendored comms config) and is HARDWARE-only; deferring its import keeps the
    package importable on the offline / replay path -- the live front-end builder
    imports it explicitly only when ``--live`` is used.
    """
    if name in ("LiveFrontEndCalib", "read_live_calibration"):
        from . import live_calib
        return getattr(live_calib, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
