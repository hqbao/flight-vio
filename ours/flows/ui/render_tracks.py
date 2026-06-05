"""``render_tracks`` task: hand each ``frame.tracks`` to the viewer callback."""
from __future__ import annotations

from ...lib.flow.messages import FrameTracks
from ...lib.flow.task import Task


class RenderTracks(Task):
    name = "render_tracks"

    def run(self, ctx, msg: FrameTracks):
        ctx.state["on_tracks"](msg)
        return None
