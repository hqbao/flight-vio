"""odometry flow: real-time RGB-D visual odometry (+ gyro prior).

Wires the odometry tasks (one file each) into a reactive flow that joins the two
edges of the unified acquisition front-end:

* ``imucam.sample`` ->
  [:class:`~ours.flows.odometry.preintegrate_prior.PreintegratePrior`]
* ``frame.depth`` -> [:class:`~ours.flows.odometry.track_features.TrackFeatures`,
  :class:`~ours.flows.odometry.publish_tracks.PublishTracks`,
  :class:`~ours.flows.odometry.align_gravity.AlignGravity`,
  :class:`~ours.flows.odometry.pull_prior.PullPrior`,
  :class:`~ours.flows.odometry.estimate_motion.EstimateMotion`,
  :class:`~ours.flows.odometry.publish_inliers.PublishInliers`,
  :class:`~ours.flows.odometry.publish_pose.PublishPose`,
  :class:`~ours.flows.odometry.emit_keyframe.EmitKeyframe`]

Both inputs come from the SAME flow: the imu_cam flow publishes ``imucam.sample``
and, with its depth task, ``frame.depth``. This flow owns the IMU->prior fusion
itself (``PreintegratePrior``). The frame-chain splits the visual odometry into
small single-purpose tasks. ``TrackFeatures`` (KLT) is the only numba-parallel
section and holds the parallel lock; everything after it is pure NumPy and runs
lock-free, so the heavy motion solve overlaps the next frame's depth matcher
instead of serialising against it. ``PublishTracks`` emits the same KLT tracks on
``frame.tracks`` for the keypoint-depth visualiser. ``AlignGravity`` does the
one-shot startup attitude bootstrap; ``PullPrior`` is the IMU<->vision join that
pops the preintegrated prior for the frame's ``seq``; ``EstimateMotion`` is then
just the RGB-D PnP (+ gyro fusion) solve. ``PublishInliers`` then emits that
solve's PnP inlier track ids on ``frame.inliers`` so the visualiser can mark the
clean subset the motion estimate actually trusted. The
:class:`~ours.flows.odometry.tracked.Tracked` carrier threads the frame + tracks
down to ``PullPrior``, which swaps it for the
:class:`~ours.flows.odometry.primed.Primed` carrier (tracks + joined prior); the
:class:`~ours.flows.odometry.step.Step` carrier then threads the result through
the rest of the chain.

Joining two END-bearing inputs (``imucam.sample`` + ``frame.depth``, both from the
imu_cam flow) means the flow must see BOTH ENDs before draining:
``expected_ends = 2``.

``R_imu_cam`` (IMU->camera rotation) drives the gyro prior; ``accel_align`` is the
one-shot startup gravity reference (camera frame) the front-end measured, seeded
here so ``EstimateMotion`` levels the initial attitude. Both may be ``None`` (pure
vision / no usable IMU).
"""
from __future__ import annotations

import numpy as np

from ...lib.flow import Flow, Bus, topics
from ...lib.odometry.odometry import OdometryConfig, RGBDVisualOdometry
from .preintegrate_prior import PreintegratePrior
from .track_features import TrackFeatures
from .publish_tracks import PublishTracks
from .align_gravity import AlignGravity
from .pull_prior import PullPrior
from .estimate_motion import EstimateMotion
from .publish_inliers import PublishInliers
from .publish_pose import PublishPose
from .emit_keyframe import EmitKeyframe


class OdometryFlow(Flow):
    def __init__(self, bus: Bus, K: np.ndarray,
                 R_imu_cam: np.ndarray | None = None,
                 accel_align: np.ndarray | None = None,
                 odom_cfg: OdometryConfig | None = None,
                 kf_every: int = 5, use_gyro: bool = True,
                 latest_only: bool = False) -> None:
        super().__init__("odometry", bus, latest_only=latest_only)
        self.ctx.state["vo"] = RGBDVisualOdometry(K, odom_cfg or OdometryConfig())
        self.ctx.state["kf_every"] = int(kf_every)
        self.ctx.state["use_gyro"] = bool(use_gyro)
        self.ctx.state["priors"] = {}
        self.ctx.state["R_imu_cam"] = (
            None if R_imu_cam is None else np.asarray(R_imu_cam, dtype=np.float64))
        if accel_align is not None:
            self.ctx.state["accel_align"] = np.asarray(accel_align, dtype=np.float64)
        self.expected_ends = 2          # imucam.sample + frame.depth both end
        self.on(topics.IMUCAM_SAMPLE, [PreintegratePrior()])
        self.on(topics.FRAME_DEPTH,
                [TrackFeatures(), PublishTracks(), AlignGravity(), PullPrior(),
                 EstimateMotion(), PublishInliers(), PublishPose(),
                 EmitKeyframe()])
        self.forwards_to(topics.POSE_ODOM, topics.KEYFRAME, topics.FRAME_TRACKS,
                         topics.FRAME_INLIERS)
