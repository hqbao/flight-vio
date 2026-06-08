"""``publish_slam_map`` task: emit the live SLAM keyframe-map on ``slam.map``.

LIVE-ONLY. The offline SlamFlow does NOT register this task (``publish_map``
defaults False), so the deterministic ``loop.correction`` scoring path stays
byte-identical. This task polls the engine's cheap map overlay EVERY keyframe --
independent of :class:`~ours.flows.slam.slam_step.SlamStep`, which only emits a
``loop.correction`` ON a confirmed loop. That decoupling is the bug fix: the UI
now gets continuous keyframe dots instead of dots only after a loop closes.

It must run AFTER ``SlamStep`` has called ``engine.submit`` for this keyframe so
the polled overlay reflects the keyframe just added (see SlamFlow for the chain
ordering that guarantees this).
"""
from __future__ import annotations

import numpy as np

from ...lib.flow import topics
from ...lib.flow.messages import SlamOverlay
from ...lib.flow.task import Task


class PublishSlamMap(Task):
    name = "publish_slam_map"

    def run(self, ctx, _kf):
        engine = ctx.state["engine"]
        # (kf_seq (N,), kf_pos (N,3) optical, n_loops, match_pos (M,3)) or None
        # when the engine has no overlay yet (subprocess worker not spawned / no
        # keyframe served).
        ov = engine.poll_overlay()
        if ov is None:
            return None
        kf_seq, kf_pos, n_loops, match_pos = ov
        ctx.bus.publish(topics.SLAM_MAP, SlamOverlay(
            kf_positions=np.asarray(kf_pos, dtype=np.float64),
            n_loops=int(n_loops),
            last_match=(np.asarray(match_pos, dtype=np.float64)
                        if match_pos is not None and len(match_pos) else None),
            kf_seqs=np.asarray(kf_seq, dtype=np.int64)))
        return None
