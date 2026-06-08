"""``pull_prior`` task: the IMU<->vision fusion join.

Fourth task of the odometry frame-chain. The :class:`PreintegratePrior` task (on
the ``imucam.sample`` edge) buffers one :class:`~vio.comms.messages.ImuPrior`
per frame ``seq`` in ``ctx.state["priors"]``; this task pops the matching prior
for the frame now being solved and threads it forward on the :class:`Primed`
carrier. Splitting the pop out of :class:`EstimateMotion` names the place the two
front-end edges meet -- the solve downstream just consumes the joined prior. The
prior is ``None`` when none preintegrated for this frame (pure vision / no IMU).
"""
from __future__ import annotations

from vio.comms import Step
from .primed import Primed
from .tracked import Tracked


class PullPrior(Step):
    name = "pull_prior"

    def run(self, ctx, tracked: Tracked):
        prior = ctx.state["priors"].pop(tracked.frame.seq, None)
        return Primed(tracked.frame, tracked.obs, prior)
