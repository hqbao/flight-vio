"""``ours.flows.ui`` -- the flow-graph SINKS that feed the UI (NOT the GUI itself).

Two packages share the name "ui"; they are different LAYERS, don't confuse them:

* ``ours.flows.ui`` (HERE) -- reactive bus *sinks* (one thread each) that terminate
  the pipeline by consuming topics for display or scoring. They hold **NO Qt**:
  ``import ours.flows.ui`` pulls zero PyQt, so the pipeline stays GUI-free and
  offline-testable.
* ``ours.ui`` -- the actual **Qt GUI** (windows, viewer3d, panels). It builds a flow
  graph and plugs these sinks in (passing a callback the sink calls per message).
  The dependency is one-way: ``ours.ui`` imports ``ours.flows.ui``, never the reverse.

The sinks:

* :class:`~ours.flows.ui.collector.UiCollectorFlow` -- records ``pose.odom`` /
  ``pose.refined`` / ``loop.correction`` for offline scoring. A sink with
  ``expected_ends = 3``: it waits for END on all three before declaring done, so
  every upstream flow has fully drained.
* :class:`~ours.flows.ui.render.UiRenderFlow` -- bridges ``pose.odom`` to a viewer
  callback (the live 3D marker).
* :class:`~ours.flows.ui.tracks.UiTracksFlow` -- ``frame.tracks`` + ``frame.inliers``
  for the keypoint-depth window.
* :class:`~ours.flows.ui.triplet.UiTripletFlow` -- ``frame.depth`` + ``imucam.sample``
  (joined by seq) for the image|depth|IMU window.
"""
from .collector import UiCollectorFlow
from .render import UiRenderFlow
from .tracks import UiTracksFlow
from .triplet import UiTripletFlow

__all__ = ["UiCollectorFlow", "UiRenderFlow", "UiTracksFlow", "UiTripletFlow"]
