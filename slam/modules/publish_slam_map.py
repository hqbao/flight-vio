"""``publish_slam_map`` step: emit the live SLAM keyframe-map on ``slam.map``.

LIVE-ONLY. The offline SlamModule does NOT register this step (``publish_map``
defaults False), so the deterministic ``loop.correction`` scoring path stays
byte-identical. This step polls the engine's cheap map overlay EVERY keyframe --
independent of :class:`~slam.modules.slam_step.SlamStep`, which only emits a
``loop.correction`` ON a confirmed loop. That decoupling is the bug fix: the UI
now gets continuous keyframe dots instead of dots only after a loop closes.

It must run AFTER ``SlamStep`` has called ``engine.submit`` for this keyframe so
the polled overlay reflects the keyframe just added (see SlamModule for the chain
ordering that guarantees this).
"""
from __future__ import annotations

import numpy as np

from slam.comms import topics
from slam.comms.messages import SlamOverlay
from slam.comms.step import Step


class PublishSlamMap(Step):
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
