"""``pack_synced`` step: drain the buffer to the frame time, build the packet."""
from __future__ import annotations

from imu_camera.comms.messages import CamSync, ImuCamPacket
from sky.imu.timed_buffer import TimedImuBuffer


def pack_synced(buffer: TimedImuBuffer, wait_timeout: float,
                msg: CamSync) -> ImuCamPacket:
    """Drain the IMU buffer up to the frame timestamp and bundle the packet.

    Was ``PackSyncedStep(Step)``; the same logic with the buffer + wait_timeout
    passed explicitly instead of held as instance state.
    """
    # Block (bounded) until the IMU stream has covered this frame's time, so
    # the interval is never short-changed by thread scheduling; the last
    # frame (ts past the final IMU sample) just drains what is present.
    buffer.wait_until(msg.ts_ns, timeout=wait_timeout)
    imu_ts, gyro, accel = buffer.drain_until(msg.ts_ns)
    return ImuCamPacket(
        seq=msg.seq, ts_ns=msg.ts_ns,
        gray_left=msg.gray_left, gray_right=msg.gray_right,
        imu_ts=imu_ts, gyro=gyro, accel=accel,
    )
