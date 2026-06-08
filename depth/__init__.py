"""``depth`` -- the stereo-depth PROJECT (SOURCE-OF-TRUTH for the SGM math).

depth owns the from-scratch SGM dense-stereo matcher + the two depth steps
(``compute_depth`` -> ``publish_depth``). It is the canonical copy of that math:
the capture project (:mod:`imu_camera`) vendors a BYTE-IDENTICAL copy because
depth runs INLINE on the capture process's ``imu_cam`` thread at runtime today.
A ``diff -r`` gate keeps the two copies in lock-step, so this tree is the place
the stereo math is edited and the place a future "depth-as-its-own-process"
promotion would graduate from.

This package is a STANDALONE, independently-runnable source tree (it is NOT
spawned by the launcher -- depth runs inline in imu_camera in the live
topology): :mod:`depth.main` is the harness that proves depth runs as its own
project. It SUBSCRIBES to raw ``cam.sync`` (left/right) over IPC, computes metric
depth with the SGM matcher, and PUBLISHES ``frame.depth`` (rectified-left +
metric depth) on its own endpoint.

Layers
------
* :mod:`depth.comms` -- the FROZEN vendored comms contract (bit-identical across
  all five split projects); this project only consumes its public API.
* :mod:`depth.mathlib.stereo` -- the SGM stereo math this project OWNS (the
  source of truth; imu_camera vendors a byte-identical copy).
* :mod:`depth.io` -- recorded-session reading, used ONLY to read the full
  :class:`~depth.io.reader.StereoCalib` the matcher's rectifiers need (the wire
  ``calib.bundle`` carries only the rectified-left intrinsic, not the per-camera
  calibration).
* :mod:`depth.modules` -- the ``compute_depth`` + ``publish_depth`` steps.
* :mod:`depth.main` -- the standalone depth process: subscribes to ``cam.sync``,
  runs SGM, publishes ``frame.depth`` on an :class:`~depth.comms.IPCPubSub`
  server.
"""
