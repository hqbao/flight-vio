"""``slam_step`` step: submit a keyframe to the SLAM engine, forward a closure.

Offline (in-process engine) ``submit`` adds the keyframe and, on a confirmed loop,
optimises synchronously; ``poll`` returns this keyframe's :class:`SlamResult` --
identical to the old in-thread path. Live (subprocess engine) it is async; the
responsive marker rides ``pose.odom`` and never waits on this.
"""
from __future__ import annotations

from slam.comms.messages import Keyframe, LoopCorrection
from slam.comms.step import Step
from slam.mathlib.engine import Engine, SlamResult


class SlamStep(Step):
    name = "slam_step"

    def run(self, ctx, kf: Keyframe):
        engine: Engine = ctx.state["engine"]
        engine.submit((kf.T_world_cam, kf.gray_left, kf.depth_m, kf.seq))
        res: SlamResult | None = engine.poll()
        if res is None:
            return None
        return LoopCorrection(kf.seq, res.kf_poses, res.n_loops)
