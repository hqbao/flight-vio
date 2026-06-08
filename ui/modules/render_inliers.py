"""``render_inliers`` step: hand each ``frame.inliers`` to the viewer callback."""
from __future__ import annotations

from ui.comms.messages import FrameInliers
from ui.comms import Step


class RenderInliers(Step):
    name = "render_inliers"

    def run(self, ctx, msg: FrameInliers):
        ctx.state["on_inliers"](msg)
        return None
