"""MapWindow: a standalone 3D viewer for the SLAM keyframe point cloud.

Shows the room reconstructed from every keyframe at once (the caller passes the
already-built ``points``/``colors``/``cams`` arrays; this just renders them): a
coloured point cloud plus the keyframe camera positions, on the same grid/axes
the pose viewer uses. Points come in the camera-optical world frame and are
rotated to the viewer's ENU display frame with the SAME convention as
:class:`~ui.qt.viewer3d.Viewer3D`, so a map and a trajectory line up.

The cloud is REBUILT live: an IPC source (see
:class:`~ui.modules.ipc_sources.IpcSlamMapSource`) re-fuses the keyframe depth
maps every time SLAM re-corrects the keyframe poses (after a loop closure) and
calls :meth:`update` with the fresh ``(points, colors, cams)`` -- so the room
re-snaps in place rather than being a one-shot snapshot.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph.opengl as gl
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QMainWindow

from ui.comms.lib.misc import frames
from . import theme
from .viewer3d import _make_grid, _make_world_axes, _qcolor

# Camera optical (x right, y down, z forward) -> world NED; then NED->ENU is the
# viewer's display transform (identical to Viewer3D, so maps + paths align).
_M_OPT_TO_NED = np.array([[0.0, 0.0, 1.0],
                          [1.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0]])


def _to_display(pts_opt: np.ndarray) -> np.ndarray:
    """Optical-world ``(N,3)`` -> ENU display ``(N,3)`` float32."""
    if len(pts_opt) == 0:
        return np.zeros((0, 3), np.float32)
    ned = np.asarray(pts_opt, np.float64) @ _M_OPT_TO_NED.T
    return frames.ned_to_enu(ned).astype(np.float32)


def _rgba(colors: np.ndarray) -> np.ndarray:
    """Per-point gray/RGB ``(N,3)`` in [0,1] -> opaque RGBA ``(N,4)`` float32."""
    if len(colors) == 0:
        return np.zeros((0, 4), np.float32)
    rgb = np.clip(np.asarray(colors, np.float32), 0.0, 1.0)
    return np.concatenate([rgb, np.ones((len(rgb), 1), np.float32)],
                          axis=1).astype(np.float32)


class MapWindow(QMainWindow):
    #: Carries a freshly fused cloud from a background source thread onto the GUI
    #: thread. :meth:`submit` (thread-safe) emits it; the signal is connected to
    #: :meth:`update` so the GL items are only touched on the GUI thread.
    cloud_ready = pyqtSignal(object, object, object)

    def __init__(self, points: np.ndarray | None = None,
                 colors: np.ndarray | None = None,
                 cams: np.ndarray | None = None,
                 title: str = "SLAM map") -> None:
        super().__init__()
        self.setWindowTitle(title)
        self.resize(1100, 800)

        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(QColor(theme.BG))
        self.setCentralWidget(self._view)

        self._view.addItem(_make_grid(size_m=20.0, step_m=1.0))
        for ax in _make_world_axes(length=1.0):
            self._view.addItem(ax)

        # Persistent scatter items so a live rebuild re-sets their data in place
        # (instead of stacking a new item per frame). Start empty; `update` fills
        # them. The cloud points are tiny (size 2 px); the keyframe cameras are
        # bigger amber dots so the capture path is visible.
        self._cloud = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3), np.float32), color=np.zeros((0, 4), np.float32),
            size=2.0, pxMode=True)
        self._cams = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3), np.float32),
            color=_qcolor(theme.WARN, 0.95), size=9.0, pxMode=True)
        self._view.addItem(self._cloud)
        self._view.addItem(self._cams)

        # Frame-the-cloud is done ONCE, on the first non-empty update, so the
        # orbit doesn't jump every rebuild as the room grows.
        self._framed = False

        # Route background-thread clouds through the signal so `update` (GL items)
        # runs on the GUI thread (Qt auto-queues a cross-thread signal emit).
        self.cloud_ready.connect(self.update)

        if points is not None:
            self.update(points, colors, cams)

    # ------------------------------------------------------------------ #
    def submit(self, points, colors, cams) -> None:
        """Thread-safe ingest: hand a fused cloud in from any thread.

        Emits :attr:`cloud_ready`; Qt queues the connected :meth:`update` onto the
        GUI thread, so a background IPC/rebuild thread can call this directly.
        """
        self.cloud_ready.emit(points, colors, cams)

    # ------------------------------------------------------------------ #
    def update(self, points: np.ndarray | None,
               colors: np.ndarray | None,
               cams: np.ndarray | None) -> None:
        """Replace the rendered cloud + keyframe cameras with fresh data.

        ``points`` / ``cams`` are ``(N,3)`` / ``(M,3)`` in the camera-optical
        world frame (the frame :func:`geometry.keyframe_pointcloud` returns);
        ``colors`` is ``(N,3)`` per-point intensity/RGB in [0,1]. All are rotated
        to the viewer's ENU display frame here, so the map lines up with the main
        Viewer3D. Safe to call repeatedly from the GUI thread.
        """
        pts = _to_display(points if points is not None
                          else np.zeros((0, 3), np.float32))
        rgba = _rgba(colors if colors is not None
                     else np.zeros((0, 3), np.float32))
        # Guard a colour/point length mismatch (a malformed build) so setData
        # never raises in the GUI thread -- fall back to a flat grey.
        if len(rgba) != len(pts):
            rgba = _rgba(np.full((len(pts), 3), 0.7, np.float32))
        self._cloud.setData(pos=pts, color=rgba, size=2.0, pxMode=True)

        cam_enu = _to_display(cams if cams is not None
                              else np.zeros((0, 3), np.float32))
        self._cams.setData(pos=cam_enu, color=_qcolor(theme.WARN, 0.95),
                           size=9.0, pxMode=True)

        # Centre the orbit on the cloud centroid + back off by its size, ONCE.
        if not self._framed and len(pts):
            centre = pts.mean(axis=0)
            extent = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
            self._view.opts["center"] = self._vec(centre)
            self._view.setCameraPosition(distance=max(extent * 0.8, 2.0),
                                         azimuth=45, elevation=30)
            self._framed = True

    @staticmethod
    def _vec(p):
        from PyQt6.QtGui import QVector3D
        return QVector3D(float(p[0]), float(p[1]), float(p[2]))
