"""``emit_keyframe`` task: every ``kf_every`` frames, publish a ``keyframe``.

The keyframe carries the pose, image, depth, the current track snapshot and --
only when the camera was at rest -- the gravity accel for the back-end (a moving
keyframe's lateral acceleration would bias the gravity direction).
"""
from __future__ import annotations

import numpy as np

from vio.comms import topics
from vio.comms.messages import Keyframe
from vio.comms import Step as StepBase
from vio.mathlib.odometry.odometry import RGBDVisualOdometry
from .step import Step


class EmitKeyframe(StepBase):
    name = "emit_keyframe"

    def run(self, ctx, step: Step):
        n = ctx.state.get("kf_count", 0) + 1
        if n < ctx.state["kf_every"]:
            ctx.state["kf_count"] = n
            return step
        ctx.state["kf_count"] = 0
        vo: RGBDVisualOdometry = ctx.state["vo"]
        tr = vo.frontend.tracks
        ids = tr.ids.copy() if tr is not None and tr.ids is not None else None
        px = tr.points.copy() if tr is not None and tr.points is not None else None
        accel = step.accel_cam if step.at_rest else None
        inl = step.info.get("inlier_ids")        # PnP inliers this frame (clean subset)
        inl = None if inl is None else np.asarray(inl).copy()
        ctx.bus.publish(topics.KEYFRAME,
                        Keyframe(step.frame.seq, step.pose,
                                 step.frame.gray_left, step.frame.depth_m,
                                 track_ids=ids, track_px=px, accel=accel,
                                 inlier_ids=inl))
        return step
