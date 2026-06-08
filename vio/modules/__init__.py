"""``vio.modules`` -- the VIO reactive pipeline (odometry + windowed BA).

The two reactive modules and the single-purpose steps they compose (ported
verbatim from ``ours.flows.odometry`` + ``ours.flows.backend``; Flow -> Module,
Task -> Step):

* :class:`~vio.modules.pipeline.OdometryModule` -- joins ``imucam.sample`` (IMU
  prior preintegration) + ``frame.depth`` (KLT track -> RGB-D PnP -> gyro fusion
  -> pose) and publishes ``pose.odom`` every frame, a ``keyframe`` every few
  frames, plus ``frame.tracks`` / ``frame.inliers`` for the visualiser (and
  ``pose.vo`` when the live builder enables the pure-vision line).
* :class:`~vio.modules.pipeline.BackendModule` -- consumes ``keyframe``, runs the
  sliding-window bundle adjustment behind a swappable
  :class:`~vio.mathlib.engine.base.Engine`, and publishes the refined pose on
  ``pose.refined``.

The internal carriers (:mod:`~vio.modules.step` / :mod:`~vio.modules.primed` /
:mod:`~vio.modules.tracked`) thread one frame's state through the odometry chain;
they never go on the bus.
"""
from .pipeline import OdometryModule, BackendModule

__all__ = ["OdometryModule", "BackendModule"]
