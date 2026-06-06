"""``run_ba`` task: submit a keyframe to the BA engine, forward any refined pose.

Offline (in-process engine) ``submit`` runs the solve synchronously and ``poll``
returns this keyframe's refined ``T_cw`` -- identical to the old in-thread path.
Live (subprocess engine) ``submit`` is async and ``poll`` returns the freshest
refined pose the worker has produced (or ``None``); the responsive marker rides
``pose.odom`` and never waits on this.
"""
from __future__ import annotations

import numpy as np

from ...lib.flow.messages import Keyframe, PoseMsg
from ...lib.flow.task import Task
from ...lib.engine import Engine


class RunBA(Task):
    name = "run_ba"

    def run(self, ctx, kf: Keyframe):
        if kf.track_ids is None or kf.track_px is None:
            return None
        engine: Engine = ctx.state["engine"]
        T_cw = np.linalg.inv(kf.T_world_cam)
        engine.submit((T_cw, kf.track_ids, kf.track_px, kf.depth_m, kf.accel))
        post = engine.poll()                     # refined latest T_cw, or None
        if post is None:
            return None
        return PoseMsg(kf.seq, 0, np.linalg.inv(post), {"refined": True})
