"""odometry flow: the real-time RGB-D visual odometry (+ gyro prior).

Subscribes ``frame.depth`` (the depth flow's output) and ``imu.sample`` (the
capture flow's one-shot IMU chunk). Publishes ``pose.odom`` every frame and a
``keyframe`` every few frames for the back-end / SLAM flows.

This reproduces, task by task, the validated offline f2f driver in
``ours.tools.vio_run`` (``--backend f2f --depth ours``): build a gyro
preintegrator, gravity-align the first frame, hand PnP a per-frame rotation
prior, run :class:`~ours.lib.odometry.odometry.RGBDVisualOdometry`.
"""
from .odometry_flow import OdometryFlow

__all__ = ["OdometryFlow"]
