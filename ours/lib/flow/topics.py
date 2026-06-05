"""Canonical pub/sub topic names for the live ``ours`` pipeline.

Each constant is the string key used on the :class:`~ours.lib.flow.pubsub.Bus`.
Keeping them in one place documents the data flow between flows (edges below are
exactly the ``self.on(...)`` subscriptions wired in each ``*_flow.py``).

There is ONE acquisition front-end (``cam`` + ``imu_cam``); the odometry flow
consumes its synced ``imucam.sample`` and the ``frame.depth`` its depth task
emits::

    cam      --cam.sync------> imu_cam       (trigger: stereo pair + ts)
    imu_cam  --imu.raw-------> (visualiser)   (raw IMU for the interval)
    imu_cam  --imucam.sample-> odometry       (frames + CALIBRATED IMU)
    imu_cam  --frame.depth---> odometry       (depth task output)
    odometry --pose.odom-----> ui-collector, ui-render
    odometry --keyframe------> backend, slam
    backend  --pose.refined--> ui-collector
    slam     --loop.correction-> ui-collector

``cam.sync`` carries the frames so the ``imu_cam`` flow packs them with the
inertial samples it drains up to the frame timestamp. For each trigger the
``imu_cam`` flow emits the uncalibrated samples on ``imu.raw`` (honest, what the
sensor reported), the frames bundled with the *calibrated* IMU on
``imucam.sample``, and -- in the VIO path -- the ``frame.depth`` computed by its
own depth task. The odometry flow owns the IMU->prior fusion itself (no separate
capture flow).

Note: the back-end and SLAM flows both trigger off ``keyframe`` (NOT
``pose.odom``); the SLAM ``loop.correction`` is currently consumed only by the
UI collector -- it is not yet fed back into odometry (no closed loop on the live
pose path). Wire an odometry ``self.on(LOOP_CORRECTION, ...)`` when that
feedback is added.
"""
from __future__ import annotations

FRAME_DEPTH = "frame.depth"
POSE_ODOM = "pose.odom"
KEYFRAME = "keyframe"
POSE_REFINED = "pose.refined"
LOOP_CORRECTION = "loop.correction"

# Acquisition front-end (``cam`` <-> ``imu_cam``).
CAM_SYNC = "cam.sync"
IMUCAM_SAMPLE = "imucam.sample"
# Raw IMU for each frame interval, published BEFORE calibration (honest, what
# the sensor reported). ``imucam.sample`` carries the CALIBRATED IMU.
IMU_RAW = "imu.raw"
