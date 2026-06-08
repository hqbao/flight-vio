"""ui collector module: record the streamed trajectory for offline scoring.

Wires three tiny steps (one file each), one per subscribed topic, each stashing
its message into the module's public buffers:

* :class:`~ui.modules.collect_odom.CollectOdom`             -- ``pose.odom``
* :class:`~ui.modules.collect_refined.CollectRefined`       -- ``pose.refined``
* :class:`~ui.modules.collect_correction.CollectCorrection` -- ``loop.correction``
"""
from __future__ import annotations

from ui.comms import Module, LocalPubSub, topics
from ui.comms.messages import LoopCorrection
from .collect_odom import CollectOdom
from .collect_refined import CollectRefined
from .collect_correction import CollectCorrection


class UiCollectorModule(Module):
    """Sink module that records the streamed trajectory for offline scoring."""

    def __init__(self, bus: LocalPubSub) -> None:
        super().__init__("ui", bus)
        self.odom: dict[int, "object"] = {}
        self.refined: dict[int, "object"] = {}
        self.corrections: list[LoopCorrection] = []
        self.ctx.state["odom"] = self.odom
        self.ctx.state["refined"] = self.refined
        self.ctx.state["corrections"] = self.corrections
        self.expected_ends = 3       # pose.odom + pose.refined + loop.correction
        self.on(topics.POSE_ODOM, [CollectOdom()])
        self.on(topics.POSE_REFINED, [CollectRefined()])
        self.on(topics.LOOP_CORRECTION, [CollectCorrection()])
