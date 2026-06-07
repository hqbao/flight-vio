"""``ours.ui`` -- the Qt GUI (military-dark dashboard + 3D viewers): the PRESENTATION layer.

The actual PyQt6 application: ``mainwindow`` (menu + 3D pose viewer), ``viewer3d``,
``map_window`` (SLAM point cloud), the keypoint / triplet / cam-IMU windows, the
calib wizards, ``theme``, and the ``PoseSource`` bridges (``source`` / ``live_source``)
that drive them.

NOT the same as ``ours.flows.ui`` -- that is the set of flow-graph SINKS (bus
consumers, **no Qt**) that FEED these views. The dependency is one-way: ``ours.ui``
builds flow graphs and plugs in those sinks; ``ours.flows.ui`` never imports Qt, so
the pipeline stays GUI-free and offline-testable.
"""
