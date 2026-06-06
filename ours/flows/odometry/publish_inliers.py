"""``publish_inliers`` task: emit the frame's PnP inliers on ``frame.inliers``.

Runs right after :class:`~ours.flows.odometry.estimate_motion.EstimateMotion` in
the frame-chain, so the RGB-D PnP has already solved and recorded which tracks it
kept as inliers (``info["inlier_ids"]`` on the :class:`Step` it produced). It
publishes that clean subset for the keypoint-depth visualiser to mark -- a REAL
odometry output, never a re-derivation -- and passes the ``Step`` through unchanged
so ``PublishPose`` / ``EmitKeyframe`` still run on it.
"""
from __future__ import annotations

import numpy as np

from ...lib.flow import topics
from ...lib.flow.messages import FrameInliers
from ...lib.flow.task import Task
from .step import Step


class PublishInliers(Task):
    name = "publish_inliers"

    def run(self, ctx, step: Step):
        ids = step.info.get("inlier_ids")
        if ids is None:
            ids = np.empty((0,), dtype=np.int64)
        frame = step.frame
        ctx.bus.publish(topics.FRAME_INLIERS,
                        FrameInliers(frame.seq, frame.ts_ns,
                                     np.asarray(ids, dtype=np.int64)))
        return step
