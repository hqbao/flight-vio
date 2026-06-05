"""depth flow implementation: SGM dense depth.

Tasks (run sequentially per ``frame.raw``):

1. ``_ComputeDepth`` -- run the SGM matcher on (rectified left, raw right).
2. ``_PublishDepth``  -- publish the :class:`~ours.lib.messages.DepthFrame`.
"""
from __future__ import annotations

from ...lib import topics
from ...lib.flow import Flow
from ...lib.messages import DepthFrame, RawFrame
from ...lib.pubsub import Bus
from ...lib.runtime import NUMBA_PARALLEL_LOCK
from ...lib.stereo.stereo import SGMStereoMatcher
from ...lib.task import Task


class _ComputeDepth(Task):
    name = "compute_depth"

    def run(self, ctx, msg: RawFrame):
        matcher: SGMStereoMatcher = ctx.state["matcher"]
        with NUMBA_PARALLEL_LOCK:        # SGM uses numba parallel=True
            depth = matcher.dense_depth(msg.gray_left, msg.gray_right)
        return DepthFrame(msg.seq, msg.ts_ns, msg.gray_left, depth)


class _PublishDepth(Task):
    name = "publish_depth"

    def run(self, ctx, msg: DepthFrame):
        ctx.bus.publish(topics.FRAME_DEPTH, msg)
        return None


class DepthFlow(Flow):
    def __init__(self, bus: Bus, matcher: SGMStereoMatcher) -> None:
        super().__init__("depth", bus)
        self.ctx.state["matcher"] = matcher
        self.on(topics.FRAME_RAW, [_ComputeDepth(), _PublishDepth()])
        self.forwards_to(topics.FRAME_DEPTH)
