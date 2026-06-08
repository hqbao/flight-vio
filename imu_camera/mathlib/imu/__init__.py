"""``imu_camera.mathlib.imu`` -- IMU calibration, preintegration, sample buffers.

Pure-numpy inertial math the acquisition pipeline composes:

* :mod:`~imu_camera.mathlib.imu.imu_calib` -- the per-device gyro-bias + accel
  correction (:class:`ImuCalibration`).
* :mod:`~imu_camera.mathlib.imu.imu` -- SO(3) preintegration + gyro integration.
* :mod:`~imu_camera.mathlib.imu.timed_buffer` -- the thread-safe timestamped IMU
  buffer the ``imu_cam`` module drains per camera trigger.
* :mod:`~imu_camera.mathlib.imu.calib_collect` / ``calib_store`` /
  ``accel_calib`` -- the six-face collector + on-disk calibration store.
* :mod:`~imu_camera.mathlib.imu.decode` -- depthai IMU packet decode (live only).
* :mod:`~imu_camera.mathlib.imu.inertial_filter` -- the inertial translation
  filter.
"""
