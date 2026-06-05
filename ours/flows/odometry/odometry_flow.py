"""odometry flow implementation.

IMU chain (one message, ``imu.sample``):

* ``_BuildPrior`` -- build the gyro preintegrator + the gravity-align accel.

Frame chain (per ``frame.depth``):

1. ``_ProcessVO``     -- gravity-align on the first frame, compute the gyro
   rotation prior for ``[prev_ts, ts]``, run RGB-D PnP odometry.
2. ``_PublishPose``   -- publish the resulting pose on ``pose.odom``.
3. ``_EmitKeyframe``  -- every ``kf_every`` frames, publish a ``keyframe``
   carrying the pose, image, depth and the current track snapshot.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...lib import topics
from ...lib.flow import Flow
from ...lib.imu.imu import GyroPreintegrator
from ...lib.messages import DepthFrame, ImuChunk, Keyframe, PoseMsg
from ...lib.odometry.odometry import OdometryConfig, RGBDVisualOdometry
from ...lib.pubsub import Bus
from ...lib.runtime import NUMBA_PARALLEL_LOCK
from ...lib.task import Task


@dataclass
class _Step:
    """Internal carrier threading one frame's result through the task chain."""

    frame: DepthFrame
    pose: np.ndarray
    info: dict


class _BuildPrior(Task):
    name = "build_prior"

    def run(self, ctx, msg: ImuChunk):
        if not (msg.has_extrinsics and ctx.state.get("use_gyro", True)):
            return None
        ctx.state["pre"] = GyroPreintegrator(msg.ts_ns, msg.gyro, msg.T_imu_cam)
        R_imu_cam = np.asarray(msg.T_imu_cam, float)[:3, :3]
        t0 = int(msg.ts_ns[0])
        win = msg.ts_ns <= t0 + int(0.3 * 1e9)        # first ~0.3 s (near static)
        if win.any():
            accel_imu = msg.accel[win].mean(axis=0)
            ctx.state["accel_align"] = R_imu_cam @ accel_imu
        return None


class _ProcessVO(Task):
    name = "process_vo"

    def run(self, ctx, msg: DepthFrame):
        vo: RGBDVisualOdometry = ctx.state["vo"]
        if not ctx.state.get("aligned") and "accel_align" in ctx.state:
            vo.align_to_gravity(ctx.state["accel_align"])
            ctx.state["aligned"] = True
        pre = ctx.state.get("pre")
        prev_ts = ctx.state.get("prev_ts")
        R_prior = (pre.delta_rotation(prev_ts, msg.ts_ns)
                   if (pre is not None and prev_ts is not None) else None)
        with NUMBA_PARALLEL_LOCK:        # KLT tracker uses numba parallel=True
            pose = vo.process(msg.gray_left, msg.depth_m, R_prior=R_prior)
        ctx.state["prev_ts"] = msg.ts_ns
        return _Step(msg, pose.copy(), dict(vo.last_info))


class _PublishPose(Task):
    name = "publish_pose"

    def run(self, ctx, step: _Step):
        ctx.bus.publish(topics.POSE_ODOM,
                        PoseMsg(step.frame.seq, step.frame.ts_ns,
                                step.pose, step.info))
        return step


class _EmitKeyframe(Task):
    name = "emit_keyframe"

    def run(self, ctx, step: _Step):
        n = ctx.state.get("kf_count", 0) + 1
        if n < ctx.state["kf_every"]:
            ctx.state["kf_count"] = n
            return None
        ctx.state["kf_count"] = 0
        vo: RGBDVisualOdometry = ctx.state["vo"]
        tr = vo.frontend.tracks
        ids = tr.ids.copy() if tr is not None and tr.ids is not None else None
        px = tr.points.copy() if tr is not None and tr.points is not None else None
        ctx.bus.publish(topics.KEYFRAME,
                        Keyframe(step.frame.seq, step.pose,
                                 step.frame.gray_left, step.frame.depth_m,
                                 track_ids=ids, track_px=px))
        return None


class OdometryFlow(Flow):
    def __init__(self, bus: Bus, K: np.ndarray,
                 odom_cfg: OdometryConfig | None = None,
                 kf_every: int = 5, use_gyro: bool = True) -> None:
        super().__init__("odometry", bus)
        self.ctx.state["vo"] = RGBDVisualOdometry(K, odom_cfg or OdometryConfig())
        self.ctx.state["kf_every"] = int(kf_every)
        self.ctx.state["use_gyro"] = bool(use_gyro)
        self.on(topics.IMU_SAMPLE, [_BuildPrior()])
        self.on(topics.FRAME_DEPTH,
                [_ProcessVO(), _PublishPose(), _EmitKeyframe()])
        self.forwards_to(topics.POSE_ODOM, topics.KEYFRAME)
