"""``apply_calibration`` task: correct the packet's IMU before it is published.

The imu_cam flow publishes the raw IMU separately (``topics.IMU_RAW``); the synced
:class:`~ours.lib.flow.messages.ImuCamPacket` that downstream state-estimation
consumes must carry the CALIBRATED inertial data when a per-device calibration
exists. This task applies the gyro-bias + six-position accel correction
(:class:`~ours.lib.imu.imu_calib.ImuCalibration`) to the drained samples and
returns a new packet with the corrected ``gyro`` / ``accel``.

The calibration is resolved lazily through an optional ``provider`` callable,
because on the live path the device id (the cache key) is only known once the
shared device has opened -- after this flow was constructed. The provider is
called at most once and its result memoised. With no calibration (replay, or an
uncalibrated device) the packet passes through unchanged: raw == calibrated.
"""
from __future__ import annotations

from collections.abc import Callable

from ...lib.flow.messages import ImuCamPacket
from ...lib.flow.task import Task
from ...lib.imu.imu_calib import ImuCalibration


class ApplyCalibration(Task):
    name = "apply_calibration"

    def __init__(self, calibration: ImuCalibration | None = None, *,
                 provider: Callable[[], ImuCalibration | None] | None = None
                 ) -> None:
        self._cal = calibration
        self._provider = provider
        self._resolved = calibration is not None or provider is None

    def _calibration(self) -> ImuCalibration | None:
        if not self._resolved:
            try:
                self._cal = self._provider()
            except Exception:
                self._cal = None           # never let a load error kill the flow
            self._resolved = True
        return self._cal

    def run(self, ctx, msg: ImuCamPacket):
        cal = self._calibration()
        if cal is None or cal.is_identity:
            return msg                      # nothing cached -> raw passes through
        gyro_cal, accel_cal = cal.apply(msg.gyro, msg.accel)
        return ImuCamPacket(
            seq=msg.seq, ts_ns=msg.ts_ns,
            gray_left=msg.gray_left, gray_right=msg.gray_right,
            imu_ts=msg.imu_ts, gyro=gyro_cal, accel=accel_cal,
        )
