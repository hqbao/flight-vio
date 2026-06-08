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

``publish_map`` (LIVE-only, mirrors the ``level_tilt`` pattern -- keeps the offline
path byte-identical) adds a continuous ``slam.map`` overlay stream so the UI draws
keyframe dots EVERY keyframe instead of only after a loop closes. It does NOT touch
the ``loop.correction`` path.
"""
from __future__ import annotations

from ...lib.flow import Flow, Bus, topics
from ...lib.flow.task import Task
from ...lib.loop.slam import SlamConfig
from ...lib.engine import make_slam_engine
from .slam_step import SlamStep
from .publish_correction import PublishCorrection


class _RunCorrectionChain(Task):
    """Run the loop-correction sub-chain, then ALWAYS pass the keyframe through.

    ``Flow.on`` keeps ONE task list per topic (``self._routes[topic] = ...``), so
    two ``on(KEYFRAME, ...)`` calls would clobber each other -- the second wins and
    SlamStep's ``engine.submit`` would never run. And ``SlamStep`` returns ``None``
    on every non-loop keyframe, which short-circuits the flow's task chain
    (:meth:`~ours.lib.flow.flow._BaseFlow._run_chain` stops on ``None``), so simply
    appending :class:`~ours.flows.slam.publish_slam_map.PublishSlamMap` after it
    would skip the overlay on exactly the keyframes that need it most.

    This wrapper runs the EXACT same ``[SlamStep(), PublishCorrection()]`` sub-chain
    the offline path uses (so ``loop.correction`` stays byte-identical -- SlamStep
    still does ``engine.submit`` then ``engine.poll``, PublishCorrection still emits
    only on a confirmed loop), then returns the keyframe unchanged so the outer
    chain continues to ``PublishSlamMap`` -- which polls the overlay AFTER the submit
    that just ran here. One combined chain, correct order, zero impact on the
    loop-correction semantics.
    """

    name = "run_correction_chain"

    def __init__(self) -> None:
        self._chain = (SlamStep(), PublishCorrection())

    def run(self, ctx, kf):
        msg = kf
        for task in self._chain:
            msg = task.run(ctx, msg)
            if msg is None:                # no loop this keyframe -> sub-chain done
                break
        return kf                          # always continue to PublishSlamMap


class SlamFlow(Flow):
    def __init__(self, bus: Bus, K, cfg: SlamConfig | None = None,
                 latest_only: bool = False, worker: bool = False,
                 publish_map: bool = False) -> None:
        super().__init__("slam", bus, latest_only=latest_only)
        self.engine = make_slam_engine(K, cfg or SlamConfig(), worker=worker)
        self.ctx.state["engine"] = self.engine
        if publish_map:
            # LIVE: one combined chain -- correction sub-chain first (does the
            # engine.submit), then the overlay poll. See _RunCorrectionChain.
            from .publish_slam_map import PublishSlamMap
            self.on(topics.KEYFRAME, [_RunCorrectionChain(), PublishSlamMap()])
            self.forwards_to(topics.LOOP_CORRECTION, topics.SLAM_MAP)
        else:
            # OFFLINE: byte-identical to the old path.
            self.on(topics.KEYFRAME, [SlamStep(), PublishCorrection()])
            self.forwards_to(topics.LOOP_CORRECTION)

    def run(self) -> None:
        # Close the engine on THIS thread when the loop exits (stop sentinel or
        # _stop), so a subprocess worker is reaped without a cross-thread race.
        try:
            super().run()
        finally:
            self.engine.close()
