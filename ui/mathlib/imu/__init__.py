"""``ui.mathlib.imu`` -- the IMU calibration math the Calibration menu needs.

Forced-vendor of the MINIMAL ``ours.lib.imu`` surface the gyro / accel calib
dialogs (:mod:`ui.qt.calib_dialogs`) + the triplet window's CALIBRATED badge
(:mod:`ui.qt.synced_window`) depend on, ported verbatim (only the package root
changed -- internal imports stay relative, so it is byte-for-byte the proven
math):

* :mod:`ui.mathlib.imu.accel_calib` -- the 6-position accelerometer model
  (:class:`AccelCalibration` + the least-squares solve).
* :mod:`ui.mathlib.imu.calib_collect` -- the stillness gate + six-face collector
  state machines (:class:`StaticCollector` / :class:`SixFaceCollector` + the
  ``face_name`` / ``gyro_bias_verdict`` helpers the dialogs call).
* :mod:`ui.mathlib.imu.calib_store` -- the per-device JSON store
  (``save_gyro_bias`` / ``save_accel_calib`` written by the dialogs;
  ``load_*`` read by the same key capture loads).
* :mod:`ui.mathlib.imu.imu_calib` -- :class:`ImuCalibration` (the triplet window
  reads it to show the CALIBRATED badge).
"""
