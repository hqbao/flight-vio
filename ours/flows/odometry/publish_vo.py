"""``publish_vo`` task: emit the pure-vision frame-to-frame pose on ``pose.vo``.

Mirrors :class:`~ours.flows.odometry.publish_pose.PublishPose`, but publishes the
PURE-VISION (no-IMU, no-BA) accumulated pose
(:attr:`~ours.lib.odometry.odometry.RGBDVisualOdometry.pose_vo`) instead of the
gyro-fused VIO pose. The UI draws this as its "VO" line to compare against the
VIO ``pose.odom``.

LIVE-only: wired into :class:`~ours.flows.odometry.odometry_flow.OdometryFlow`
only when ``publish_vo=True`` (the proc4 live builder sets it). The offline
deterministic path leaves it off, so it never publishes there and pose.odom byte
parity is unaffected.

``pose_vo`` is updated upstream by :class:`EstimateMotion`, so this task must run
AFTER it in the frame chain. The per-frame ``seq`` / ``ts_ns`` come from the same
:class:`Step` carrier ``PublishPose`` uses; the pose is read live off the shared
:class:`~ours.lib.odometry.odometry.RGBDVisualOdometry` instance in ``ctx.state``.
"""
from __future__ import annotations

from ...lib.flow import topics
from ...lib.flow.messages import PoseMsg
from ...lib.flow.task import Task
from ...lib.odometry.odometry import RGBDVisualOdometry
from .step import Step


class PublishVo(Task):
    name = "publish_vo"

    def run(self, ctx, step: Step):
        vo: RGBDVisualOdometry = ctx.state["vo"]
        # Copy the accumulator: the same instance keeps mutating pose_vo on the
        # next frame, so the published message must own an independent snapshot
        # (the wire/bridge layer reads it asynchronously).
        ctx.bus.publish(topics.POSE_VO,
                        PoseMsg(step.frame.seq, step.frame.ts_ns,
                                vo.pose_vo.copy(), step.info))
        return step
