"""Canonical pub/sub topic names for the live ``ours`` pipeline.

Each constant is the string key used on the :class:`~ours.lib.flow.pubsub.Bus`.
Keeping them in one place documents the data flow between flows (edges below are
exactly the ``self.on(...)`` subscriptions wired in each ``*_flow.py``):

    capture  --frame.raw-------> depth
    capture  --imu.sample------> odometry
    depth    --frame.depth-----> odometry
    odometry --pose.odom-------> ui-collector, ui-render
    odometry --keyframe--------> backend, slam
    backend  --pose.refined----> ui-collector
    slam     --loop.correction-> ui-collector

Note: the back-end and SLAM flows both trigger off ``keyframe`` (NOT
``pose.odom``); the SLAM ``loop.correction`` is currently consumed only by the
UI collector -- it is not yet fed back into odometry (no closed loop on the live
pose path). Wire an odometry ``self.on(LOOP_CORRECTION, ...)`` when that
feedback is added.
"""
from __future__ import annotations

FRAME_RAW = "frame.raw"
IMU_SAMPLE = "imu.sample"
FRAME_DEPTH = "frame.depth"
POSE_ODOM = "pose.odom"
KEYFRAME = "keyframe"
POSE_REFINED = "pose.refined"
LOOP_CORRECTION = "loop.correction"
