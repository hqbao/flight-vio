"""``ui.tests`` -- self-tests for the visualiser project.

* :mod:`ui.tests.ui_dataflow_selftest` -- the 4-process headless smoke test:
  spawns ``imu_camera.main(replay)`` + ``vio.main`` + ``slam.main`` over IPC,
  drives :class:`ui.main.IpcPoseSource` + :class:`ui.main.SlamMapTracker`, builds
  the single 5-line Qt MainWindow + its toggles offscreen, and exercises the
  View / Visualize / Calibration menus over IPC. Ported from
  ``ours.tools.proc4_ui_selftest``.
"""
