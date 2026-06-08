"""``imu_camera.mathlib`` -- the math this project OWNS.

* :mod:`~imu_camera.mathlib.device` -- the shared live OAK-D + boot calibration
  (depthai-backed; imported lazily so the offline/replay path never pulls it).
* :mod:`~imu_camera.mathlib.imu` -- IMU calibration, preintegration, the
  timestamped sample buffer, packet decode.
* :mod:`~imu_camera.mathlib.stereo` -- the from-scratch SGM dense-stereo matcher
  + rectifiers (numba-accelerated).
"""
