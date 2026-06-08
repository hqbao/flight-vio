"""``collect_refined`` step: record each ``pose.refined`` position by sequence."""
from __future__ import annotations

from ui.comms.messages import PoseMsg
from ui.comms import Step


class CollectRefined(Step):
    name = "collect_refined"

    def run(self, ctx, msg: PoseMsg):
        ctx.state["refined"][msg.seq] = msg.T_world_cam[:3, 3].copy()
        return None
