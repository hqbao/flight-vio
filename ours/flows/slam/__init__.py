"""slam flow: ORB loop closure + pose-graph optimisation over keyframes.

Subscribes ``keyframe`` and publishes ``loop.correction``. Wraps
:class:`~ours.lib.loop.slam.SlamMap`: every keyframe is added (the map's own
motion gate may skip redundant ones); when a loop is confirmed the pose graph is
optimised and the rewritten keyframe poses are published as a correction.
"""
from .slam_flow import SlamFlow

__all__ = ["SlamFlow"]
