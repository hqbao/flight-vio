"""``imu_camera.io`` -- recorded-session reading (the replay data source).

imu_camera owns session reading: the replay front-end feeds the SAME pipeline the
live device feeds, off a recorded gold session on disk.

* :class:`~imu_camera.io.reader.SessionReader` -- frames + IMU + calibration from
  a ``sessions/gold/*`` directory (:class:`Frame`, :class:`StereoCalib`,
  :class:`CameraCalib`).
* :class:`~imu_camera.io.synced.SyncedSample` + :func:`iter_synced` /
  :func:`slice_imu` -- per-frame frame+IMU sync helpers.
"""
