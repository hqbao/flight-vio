"""slam flow: loop closure SLAM.

Wires the two slam tasks (one file each) into a reactive flow over ``keyframe``:

1. :class:`~ours.flows.slam.slam_step.SlamStep` -- submit the keyframe to the SLAM
   engine; on a confirmed loop it returns the rewritten poses.
2. :class:`~ours.flows.slam.publish_correction.PublishCorrection` -- emit it on
   ``loop.correction``.

The heavy ORB + pose-graph solve runs behind an
:class:`~ours.lib.engine.base.Engine`: ``worker=False`` (default, offline) runs it
synchronously in-thread -- byte-identical to the old path; ``worker=True`` (live)
runs it in a separate process so it cannot hold the camera read loop's GIL.
"""
from __future__ import annotations

from ...lib.flow import Flow, Bus, topics
from ...lib.loop.slam import SlamConfig
from ...lib.engine import make_slam_engine
from .slam_step import SlamStep
from .publish_correction import PublishCorrection


class SlamFlow(Flow):
    def __init__(self, bus: Bus, K, cfg: SlamConfig | None = None,
                 latest_only: bool = False, worker: bool = False) -> None:
        super().__init__("slam", bus, latest_only=latest_only)
        self.engine = make_slam_engine(K, cfg or SlamConfig(), worker=worker)
        self.ctx.state["engine"] = self.engine
        self.on(topics.KEYFRAME, [SlamStep(), PublishCorrection()])
        self.forwards_to(topics.LOOP_CORRECTION)

    def run(self) -> None:
        # Close the engine on THIS thread when the loop exits (stop sentinel or
        # _stop), so a subprocess worker is reaped without a cross-thread race.
        try:
            super().run()
        finally:
            self.engine.close()
