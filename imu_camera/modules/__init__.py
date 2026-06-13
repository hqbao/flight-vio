"""``imu_camera.modules`` -- the threaded acquisition pipeline.

The two halves of the acquisition front-end and the step functions they compose
(was ``ours.flows.cam`` + ``ours.flows.imu_cam``; flattened from a reactive
``Step``/``Module`` graph to procedural Python):

* :class:`~imu_camera.modules.read_cam.ReadCamModule` -- a producer thread that
  emits one ``cam.sync`` per scheduled stereo pair (replay / live sources).
* :class:`~imu_camera.modules.pipeline.ImuCamWorker` (aliased
  ``ImuCamModule``) -- the worker thread that buffers IMU, packs a synced packet
  per camera trigger, and (when given a matcher) computes dense depth inline. Its
  :func:`~imu_camera.modules.pipeline.process_cam_sync` calls the step functions
  pack_synced / publish_imu_raw / apply_calibration / publish_imucam /
  compute_depth / publish_depth (or tof_downsample) in order.
* :func:`~imu_camera.modules.pipeline.build_replay_frontend` /
  :func:`~imu_camera.modules.pipeline.build_live_frontend` -- the live/replay
  front-end wiring (was ``ours.app.build_*``).
"""
