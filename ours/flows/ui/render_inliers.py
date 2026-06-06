"""``render_inliers`` task: hand each ``frame.inliers`` to the viewer callback."""
from __future__ import annotations

from ...lib.flow.messages import FrameInliers
from ...lib.flow.task import Task


class RenderInliers(Task):
    name = "render_inliers"

    def run(self, ctx, msg: FrameInliers):
        ctx.state["on_inliers"](msg)
        return None
