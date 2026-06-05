"""odometry flow: the real-time RGB-D visual odometry (+ gyro prior).

Subscribes ``frame.depth`` (the depth flow's output) and ``imucam.sample`` (the
imu-reader's synced frame+IMU packet -- the SAME stream depth consumes). Owns the
IMU->prior fusion itself (``PreintegratePrior``) instead of a separate capture
flow. Publishes ``pose.odom`` every frame and a ``keyframe`` every few frames for
the back-end / SLAM flows.

This reproduces, task by task, the validated offline f2f driver in
``ours.tools.vio_run`` (``--backend f2f --depth ours``): integrate the packet's
gyro into a per-frame rotation prior, gravity-align the first frame, run
:class:`~ours.lib.odometry.odometry.RGBDVisualOdometry`.
"""
from .odometry_flow import OdometryFlow

__all__ = ["OdometryFlow"]
