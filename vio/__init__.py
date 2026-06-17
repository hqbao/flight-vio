"""``vio`` -- the visual-inertial odometry PROJECT (Phase 3 of the split).

Subscribes to the ``imu_camera`` capture process over IPC (``imucam.sample`` +
``frame.depth`` + the retained ``calib.bundle``), runs the RGB-D visual odometry
(+ gyro prior + the live ``--tight`` dead-reckon nav-state), and republishes
``pose.odom`` / ``pose.vo`` / ``keyframe`` / ``frame.tracks`` / ``frame.inliers``
on its own IPC endpoint for BA / SLAM / UI / tools. The sliding-window bundle
adjustment moved to the ``ba`` process (6th project): ``ba`` consumes vio's
``keyframe`` and publishes ``pose.refined`` -- which vio re-emits on its own
endpoint as a pass-through (``--ba-endpoint``) so the UI keeps one endpoint.

Built by replicating the PROVEN ``imu_camera`` template:

* :mod:`vio.comms` -- the FROZEN vendored comms contract, COPIED bit-identically
  from ``imu_camera.comms`` (a CI ``diff -r`` gate enforces byte-parity); this
  project only consumes its public API.
* :mod:`vio.resolution_build` / :mod:`vio.warmup` -- the resolution-driven
  frontend/odometry config builders and the JIT warmup, the math-coupled glue VIO
  owns at the project root.
* :mod:`vio.modules` -- the odometry pipeline (was ``ours.flows.odometry``), wired
  by :class:`~vio.modules.pipeline.OdometryModule`. The windowed-BA worker now
  lives in the ``ba`` project (``ba.modules``).
* :mod:`vio.main` -- the VIO process: a calib client + a data client onto the
  capture endpoint, the local odometry graph, the read-only ba/slam feedback
  clients, and an :class:`~vio.comms.IPCPublisher` mirroring its outputs onto the
  ``oak.vio`` endpoint.
"""
