"""``imu_camera.modules`` -- the threaded acquisition pipeline.

The two halves of the acquisition front-end and the steps they compose (was
``ours.flows.cam`` + ``ours.flows.imu_cam``):

* :class:`~imu_camera.modules.read_cam.ReadCamModule` -- a source module that
  emits one ``cam.sync`` per scheduled stereo pair (replay / live sources).
* :class:`~imu_camera.modules.pipeline.ImuCamModule` -- the reactive module that
  buffers IMU, packs a synced packet per camera trigger, and (when given a
  matcher) computes dense depth inline. Composes the
  pack / publish_imu_raw / apply_calibration / publish_imucam /
  compute_depth / publish_depth steps.
* :func:`~imu_camera.modules.pipeline.build_replay_frontend` /
  :func:`~imu_camera.modules.pipeline.build_live_frontend` -- the live/replay
  front-end wiring (was ``ours.app.build_*``).
"""
