"""``ui.mathlib`` -- the minimal math the UI process owns (forced-vendor).

The UI is a sink: it renders trajectories + imagery fed over IPC and never runs
odometry / BA / SLAM, so it needs almost no math. The one exception is the
in-window **Calibration** menu: the gyro / accel calib dialogs
(:mod:`ui.qt.calib_dialogs`) drive the tested stillness-gate / six-face collector
state machines and persist the result. Those collectors + the per-device store
are vendored here (ported verbatim from ``ours.lib.imu``, like ``vio`` vendored
its IMU helpers) so the UI stays a self-contained project with no ``ours.*``
import edge.

* :mod:`ui.mathlib.imu` -- the calibration collectors + store + the underlying
  accelerometer model the dialogs consume.
"""
