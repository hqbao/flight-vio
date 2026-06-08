"""``publish_imu_raw`` step: emit the uncalibrated IMU for a frame interval.

This runs BEFORE calibration in the imu_cam chain: it takes the freshly packed
(raw) :class:`~imu_camera.comms.messages.ImuCamPacket`, publishes its inertial
samples on ``topics.IMU_RAW`` as an :class:`~imu_camera.comms.messages.ImuRaw`,
and passes the same packet through unchanged so the next step can calibrate it.
Publishing here -- not from the IMU I/O thread -- keeps all bus traffic on the
module thread (the honest "what the sensor reported" snapshot, per frame).
"""
from __future__ import annotations

from imu_camera.comms import Step, topics
from imu_camera.comms.messages import ImuCamPacket, ImuRaw


class PublishImuRawStep(Step):
    name = "publish_imu_raw"

    def run(self, ctx, msg: ImuCamPacket):
        ctx.bus.publish(topics.IMU_RAW, ImuRaw(
            seq=msg.seq, ts_ns=msg.ts_ns,
            imu_ts=msg.imu_ts, gyro=msg.gyro, accel=msg.accel,
        ))
        return msg                         # pass the raw packet on to calibration
