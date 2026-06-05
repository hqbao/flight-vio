"""Internal carrier from :class:`PullPrior` to :class:`EstimateMotion`.

Not a task -- a small flow-internal message that threads one frame's tracked
``{id: pixel}`` features together with the IMU prior popped for that frame's
``seq`` (the IMU<->vision fusion join). Stays inside the odometry flow (never goes
on the Bus), the same role :class:`~ours.flows.odometry.tracked.Tracked` plays
upstream and :class:`~ours.flows.odometry.step.Step` plays downstream.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ...lib.flow.messages import DepthFrame, ImuPrior


@dataclass
class Primed:
    frame: DepthFrame
    obs: dict[int, np.ndarray]
    prior: ImuPrior | None
