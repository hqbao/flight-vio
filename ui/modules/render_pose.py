"""``render_pose`` step: hand each ``pose.odom`` to the viewer callback."""
from __future__ import annotations

from ui.comms.messages import PoseMsg
from ui.comms import Step


class RenderPose(Step):
    name = "render"

    def run(self, ctx, msg: PoseMsg):
        ctx.state["on_pose"](msg)
        return None
