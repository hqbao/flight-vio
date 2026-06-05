"""``publish_imu_raw`` task: emit the uncalibrated IMU for a frame interval.

This runs BEFORE calibration in the imu_cam chain: it takes the freshly
packed (raw) :class:`~ours.lib.flow.messages.ImuCamPacket`, publishes its
inertial samples on ``topics.IMU_RAW`` as an :class:`~ours.lib.flow.messages.ImuRaw`,
and passes the same packet through unchanged so the next task can calibrate it.
Publishing here -- not from the IMU I/O thread -- keeps all bus traffic on the
flow thread (the honest "what the sensor reported" snapshot, per frame).
"""
from __future__ import annotations

from ...lib.flow import topics
from ...lib.flow.messages import ImuCamPacket, ImuRaw
from ...lib.flow.task import Task


class PublishImuRaw(Task):
    name = "publish_imu_raw"

    def run(self, ctx, msg: ImuCamPacket):
        ctx.bus.publish(topics.IMU_RAW, ImuRaw(
            seq=msg.seq, ts_ns=msg.ts_ns,
            imu_ts=msg.imu_ts, gyro=msg.gyro, accel=msg.accel,
        ))
        return msg                         # pass the raw packet on to calibration
