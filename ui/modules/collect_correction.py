"""``collect_correction`` step: append each ``loop.correction`` to the buffer."""
from __future__ import annotations

from ui.comms.messages import LoopCorrection
from ui.comms import Step


class CollectCorrection(Step):
    name = "collect_correction"

    def run(self, ctx, msg: LoopCorrection):
        ctx.state["corrections"].append(msg)
        return None
