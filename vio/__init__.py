"""``vio`` -- the visual-inertial odometry PROJECT (Phase 3 of the split).

Subscribes to the ``imu_camera`` capture process over IPC (``imucam.sample`` +
``frame.depth`` + the retained ``calib.bundle``), runs the same RGB-D visual
odometry (+ gyro prior) and sliding-window bundle adjustment the pre-split
in-process graph ran, and republishes ``pose.odom`` / ``pose.vo`` /
``pose.refined`` / ``keyframe`` / ``frame.tracks`` / ``frame.inliers`` on its own
IPC endpoint for SLAM / UI / tools.

Built by replicating the PROVEN ``imu_camera`` template:

* :mod:`vio.comms` -- the FROZEN vendored comms contract, COPIED bit-identically
  from ``imu_camera.comms`` (a CI ``diff -r`` gate enforces byte-parity); this
  project only consumes its public API.
* :mod:`vio.mathlib` -- the math VIO owns (frontend KLT, odometry, backend BA +
  VIO window, the engine runners, and the IMU/SO(3) helpers they depend on),
  ported verbatim from ``ours.lib`` with the math-coupled config builders +
  JIT warmup living in ``vio.mathlib`` per the architecture rule.
* :mod:`vio.modules` -- the odometry + backend reactive modules (was
  ``ours.flows.{odometry,backend}``), wired by
  :class:`~vio.modules.pipeline.OdometryModule` /
  :class:`~vio.modules.pipeline.BackendModule`.
* :mod:`vio.main` -- the VIO process: a calib client + a data client onto the
  capture endpoint, the local odometry / backend graph, and an
  :class:`~vio.comms.IPCPublisher` mirroring its outputs onto the ``oak.vio``
  endpoint.
"""
