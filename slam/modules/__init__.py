"""``slam.modules`` -- the SLAM reactive pipeline (ORB loop closure + pose graph).

The single reactive module and the single-purpose steps it composes (ported
verbatim from ``ours.flows.slam``; Flow -> Module, Task -> Step, Bus ->
LocalPubSub):

* :class:`~slam.modules.pipeline.SlamModule` -- subscribes ``keyframe`` and
  publishes ``loop.correction``. Wraps :class:`~slam.mathlib.loop.slam.SlamMap`:
  every keyframe is added (the map's own motion gate may skip redundant ones);
  when a loop is confirmed the pose graph is optimised and the rewritten keyframe
  poses are published as a correction. The heavy ORB + pose-graph solve runs
  behind a swappable :class:`~slam.mathlib.engine.base.Engine` (in-process offline,
  subprocess live). With ``publish_map=True`` (live) it also emits a continuous
  ``slam.map`` keyframe overlay -- see :class:`~slam.modules.pipeline._RunCorrectionChain`.

The single-purpose steps (:mod:`~slam.modules.slam_step` /
:mod:`~slam.modules.publish_correction` / :mod:`~slam.modules.publish_slam_map`)
each own one responsibility on the keyframe -> correction chain.
"""
from .pipeline import SlamModule

__all__ = ["SlamModule"]
