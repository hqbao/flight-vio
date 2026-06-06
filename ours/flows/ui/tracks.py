"""ui tracks flow: forward streamed KLT tracks to a callback (keypoints view).

The sink that drives the keypoint-depth tracker window
(:class:`~ours.ui.keypoints_window.KeypointTrackWindow`). It subscribes
``frame.tracks`` -- the REAL frontend tracks the odometry flow publishes -- and
hands each :class:`~ours.lib.flow.messages.FrameTracks` to an ``on_tracks``
callback, exactly as :class:`~ours.flows.ui.render.UiRenderFlow` does for
``pose.odom``. The single task lives in :mod:`ours.flows.ui.render_tracks`.

When an ``on_inliers`` callback is given it ALSO subscribes ``frame.inliers`` --
the RGB-D PnP inlier track ids the odometry solve emits (a separate REAL output)
-- and forwards each :class:`~ours.lib.flow.messages.FrameInliers` to it (task in
:mod:`ours.flows.ui.render_inliers`), so the window can mark the clean subset the
motion estimate trusted. Both inputs are END-bearing, so ``expected_ends`` is 2
in that case.

The window keeps the overlay rendering (per-id trails + depth-coloured dots)
UI-side -- that is honest buffering of the subscribed tracks, not a parallel
detector.
"""
from __future__ import annotations

from typing import Callable

from ...lib.flow import Flow, Bus, topics
from ...lib.flow.messages import FrameInliers, FrameTracks
from .render_tracks import RenderTracks
from .render_inliers import RenderInliers


class UiTracksFlow(Flow):
    """Sink flow that forwards each ``frame.tracks`` to ``on_tracks``."""

    def __init__(self, bus: Bus, on_tracks: Callable[[FrameTracks], None], *,
                 on_inliers: Callable[[FrameInliers], None] | None = None,
                 latest_only: bool = False) -> None:
        super().__init__("ui", bus, latest_only=latest_only)
        self.ctx.state["on_tracks"] = on_tracks
        self.expected_ends = 1                       # only frame.tracks carries END
        self.on(topics.FRAME_TRACKS, [RenderTracks()])
        if on_inliers is not None:
            self.ctx.state["on_inliers"] = on_inliers
            self.expected_ends = 2                   # + frame.inliers
            self.on(topics.FRAME_INLIERS, [RenderInliers()])
