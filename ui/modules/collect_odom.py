"""``collect_odom`` step: record each ``pose.odom`` position by sequence."""
from __future__ import annotations

from ui.comms.messages import PoseMsg
from ui.comms import Step


class CollectOdom(Step):
    name = "collect_odom"

    def run(self, ctx, msg: PoseMsg):
        ctx.state["odom"][msg.seq] = msg.T_world_cam[:3, 3].copy()
        return None
