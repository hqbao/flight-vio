"""ui render module: forward streamed poses to a callback (live viewer).

Where :class:`~ui.modules.collector.UiCollectorModule` records poses for offline
scoring, this sink hands each ``pose.odom`` message to an ``on_pose`` callback --
the bridge that drives the Qt 3D viewer. The single step lives in
:mod:`ui.modules.render_pose`.
"""
from __future__ import annotations

from typing import Callable

from ui.comms import LocalPubSub, Module, topics
from ui.comms.messages import PoseMsg
from .render_pose import RenderPose


class UiRenderModule(Module):
    """Sink module that forwards each ``pose.odom`` to ``on_pose``."""

    def __init__(self, bus: LocalPubSub,
                 on_pose: Callable[[PoseMsg], None]) -> None:
        super().__init__("ui", bus)
        self.ctx.state["on_pose"] = on_pose
        self.expected_ends = 1                       # only pose.odom carries END
        self.on(topics.POSE_ODOM, [RenderPose()])
