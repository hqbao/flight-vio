"""``depth.io`` -- recorded-session reading (the offline data source).

depth vendors the same session reader the capture project uses so the standalone
depth process (and the SGM self-test) can run a recorded gold session WITHOUT an
OAK-D attached: the matcher needs the full :class:`StereoCalib` (per-camera
intrinsics + the left->right rigid transform) to build its rectifiers, and that
calibration lives in the session's ``calib.json`` -- it is NOT carried on the
:class:`~depth.comms.wire.WireCalibBundle` (which only broadcasts the
rectified-left intrinsic + IMU extrinsics). The raw stereo frames themselves
arrive over IPC on ``cam.sync``; the session is read only for the calibration.

* :class:`~depth.io.reader.SessionReader` -- frames + IMU + calibration from a
  ``sessions/gold/*`` directory (:class:`Frame`, :class:`StereoCalib`,
  :class:`CameraCalib`).
* :class:`~depth.io.synced.SyncedSample` + :func:`iter_synced` /
  :func:`slice_imu` -- per-frame frame+IMU sync helpers.
"""
