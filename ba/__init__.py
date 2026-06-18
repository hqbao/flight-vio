"""``ba`` -- the windowed bundle-adjustment PROJECT (6th project of the split).

Extracts the VIO windowed-BA backend (loose ``WindowedBAMap`` + tight
``WindowedVIOMap``) into its own process with an independent lifecycle, so the
heavy keyframe optimiser is fault-isolated from the live odometry and the
``libsky*`` port boundary is clean. Goal = architectural cleanliness, NOT
performance (the pre-split in-VIO worker-child engine already ran the solve
GIL-free; ``ba`` is its own process, so it runs the solve in-process).

Subscribes to the ``vio`` process over IPC (``keyframe`` + the retained
``calib.bundle``), runs the SAME frozen windowed solve the in-VIO backend ran, and
republishes ``pose.refined`` -- plus, under ``--tight``, the optimised bias on the
IPC ``ba.state`` topic for the ``vio`` process's live feed-forward (the IPC analog
of slam's ``loop.correction`` channel). ``ba`` is a pure CONSUMER of vio's keyframe
output: ``emit_keyframe`` stays in ``vio`` (it rides vio's odometry thread); ``ba``
only ingests the resulting keyframe.

Built by replicating the PROVEN ``imu_camera`` / ``vio`` / ``slam`` template:

* :mod:`ba.comms` -- the FROZEN vendored comms contract, COPIED bit-identically
  from ``imu_camera.comms`` (a ``diff -r`` gate enforces byte-parity); this project
  only consumes its public API.
* :mod:`ba.engine` -- the in-process runner that drives the heavy keyframe solve
  (this is now the SINGLE home -- ``vio.engine`` was deleted once the backend moved
  here; the ``ba`` process runs its solve in-process). The algorithm itself lives in
  the shared :mod:`sky.backend` / :mod:`sky.vio` libraries.
* :mod:`ba.modules` -- the windowed-BA pipeline (was the back-end half of
  ``vio.modules.pipeline``), PROCEDURAL: the plain function
  :func:`~ba.modules.pipeline.process_kf` driven by the plain worker thread
  :class:`~ba.modules.pipeline.BackendWorker`.
* :mod:`ba.main` -- the BA process: a calib + keyframe client onto the VIO
  endpoint, the local backend worker, and an :class:`~ba.comms.IPCPublisher`
  mirroring ``pose.refined`` (+ ``ba.state`` under ``--tight``) onto the ``oak.ba``
  endpoint.
"""
