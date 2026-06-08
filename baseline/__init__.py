"""DepthAI/Basalt reference pipeline — the black box our ``ours`` work replaces.

This standalone package runs DepthAI's on-device BasaltVIO / RTABMapSLAM blobs
and renders their pose in a Qt 3D viewer, as the reference to compare ``./run.sh``
against. It depends only on its own subpackages (no ``ours``, no 5-project split):

  * ``baseline.frames``        — frame conversions (quaternion/rotation helpers)
  * ``baseline.pose``          — ``Pose`` sample + thread-safe ``PoseHistory``
  * ``baseline.sources``       — pose producers: ``FakePoseSource`` plus the
                                 DepthAI-backed ``OakBasaltVioSource`` /
                                 ``OakBasaltSlamSource`` (depthai imported lazily)
  * ``baseline.capture``       — session recording (``SessionRecorder``) + PNG I/O
  * ``baseline.ui``            — the Qt 3D viewer (main window, 3D view, panels)
  * ``baseline.tools``         — the ``view_pose3d`` entry point plus session
                                 recording / replay / comparison utilities

Launch with ``./run-baseline.sh`` (mirrors ``./run.sh``).
"""
__version__ = "0.1.0"
