"""``publish_loops`` step: emit per-candidate loop-match funnels on ``slam.loop``.

LIVE-ONLY. The offline SlamModule does NOT register this step (``publish_map``
defaults False), so the deterministic ``loop.correction`` scoring path stays
byte-identical and the offline engine never even captures the funnel. This step
polls the engine's loop-match capture channel EVERY keyframe -- independent of
:class:`~slam.modules.slam_step.SlamStep` (which only emits a ``loop.correction``
ON a confirmed loop). For EACH verified candidate (CONFIRMED or REJECTED) it
publishes one :class:`~slam.comms.messages.LoopMatch`, so the UI's loop-closure
window can show WHY a loop fired or got rejected (the matched ORB pixel pairs +
per-match verification stage + funnel counts + rotation-gate verdict).

It must run AFTER ``SlamStep`` has called ``engine.submit`` for this keyframe so
the polled captures reflect the candidate just verified (see SlamModule for the
chain ordering that guarantees this).

There are NO keyframe images on the wire (SLAM does not retain the gray); the UI
joins each LoopMatch to the GRAY images it buffers by seq off the ``keyframe``
topic.
"""
from __future__ import annotations

import numpy as np

from slam.comms import topics
from slam.comms.messages import LoopMatch
from slam.comms.step import Step


class PublishLoops(Step):
    name = "publish_loops"

    def run(self, ctx, kf):
        engine = ctx.state["engine"]
        # [(cur_seq, old_seq, LoopMatchCapture), ...] for every candidate verified
        # since the last poll (empty unless the engine captures -- live only).
        for cur_seq, old_seq, cap in engine.poll_loops():
            ctx.bus.publish(topics.SLAM_LOOP, LoopMatch(
                cur_seq=int(cur_seq), old_seq=int(old_seq),
                cur_px=np.asarray(cap.cur_px, dtype=np.float32).reshape(-1, 2),
                old_px=np.asarray(cap.old_px, dtype=np.float32).reshape(-1, 2),
                stage=np.asarray(cap.stage, dtype=np.uint8).reshape(-1),
                n_appearance=int(cap.n_appearance), n_fmat=int(cap.n_fmat_inliers),
                n_pnp=int(cap.n_pnp_inliers), rot_deg=float(cap.rot_deg),
                rot_gate_deg=float(cap.rot_gate_deg), accepted=bool(cap.accepted)))
        # Pass the keyframe through so the outer chain continues to the terminal
        # PublishSlamMap step (which polls the map overlay and ends the chain).
        return kf
