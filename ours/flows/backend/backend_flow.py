"""backend flow: windowed bundle adjustment.

Wires the two backend tasks (one file each) into a reactive flow over
``keyframe``:

1. :class:`~ours.flows.backend.run_ba.RunBA` -- submit the keyframe's track
   snapshot to the BA engine; forward any refined pose it returns.
2. :class:`~ours.flows.backend.publish_refined.PublishRefined` -- emit it on
   ``pose.refined``.

The heavy solve runs behind an :class:`~ours.lib.engine.base.Engine`:
``worker=False`` (default, offline) runs it synchronously in-thread -- byte-identical
to the old path; ``worker=True`` (live) runs it in a separate process so it cannot
hold the camera read loop's GIL (the fast-push undershoot fix). The keyframe pose
``T_world_cam`` is inverted to the ``T_cw`` the BA map expects inside ``RunBA``.
"""
from __future__ import annotations

import numpy as np

from ...lib.flow import Flow, Bus, topics
from ...lib.backend.bundle import BAConfig
from ...lib.backend.windowed import WindowedConfig
from ...lib.engine import make_ba_engine
from .run_ba import RunBA
from .publish_refined import PublishRefined


class BackendFlow(Flow):
    def __init__(self, bus: Bus, K: np.ndarray,
                 window: int = 6, iters: int = 5,
                 latest_only: bool = False, worker: bool = False) -> None:
        super().__init__("backend", bus, latest_only=latest_only)
        cfg = WindowedConfig(window=window, ba=BAConfig(max_iters=iters))
        self.engine = make_ba_engine(K, cfg, worker=worker)
        self.ctx.state["engine"] = self.engine
        self.on(topics.KEYFRAME, [RunBA(), PublishRefined()])
        self.forwards_to(topics.POSE_REFINED)

    def run(self) -> None:
        # Close the engine on THIS thread when the loop exits (stop sentinel or
        # _stop), so a subprocess worker is reaped without a cross-thread race.
        try:
            super().run()
        finally:
            self.engine.close()
