"""RoomSurfaceWindow: a standalone 3D viewer for the room as CONTINUOUS SURFACES.

A COMPLEMENT to :class:`~ui.qt.map_window.MapWindow` (the sparse point-cloud SLAM
map): this window renders the mapped space as a SURFACE MESH -- the walls / floor
/ ceiling as smooth connected shaded surfaces -- so the enclosed room reads as a
continuous reconstruction, not a cloud of points. The caller (an IPC source)
passes an ALREADY-BUILT merged depth-surface mesh (``verts`` / ``faces`` /
``vertex_colors``) + the keyframe camera positions; this just renders them.

Same ENU world frame + camera conventions as :class:`~ui.qt.viewer3d.Viewer3D` /
:class:`~ui.qt.map_window.MapWindow` (the mesh vertices come in the camera-optical
world frame and are rotated to the viewer's ENU display frame with the IDENTICAL
``_to_display`` convention), so this window and the point-cloud map are directly
comparable.

The mesh is REBUILT live: :class:`~ui.modules.ipc_sources.IpcSurfaceMapSource`
re-meshes a spatially-spread subset of the VIO keyframes every couple of seconds
and calls :meth:`submit` (thread-safe) with the fresh mesh -- so the room fills in
/ re-snaps in place rather than being a one-shot snapshot. ``AA_ShareOpenGLContexts``
(set in ``ui.main``) makes a 2nd GL window safe alongside the main Viewer3D + the
SLAM-map window.
"""
from __future__ import annotations

import numpy as np
import pyqtgraph.opengl as gl
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QMainWindow

from . import theme
from .map_window import _to_display          # SAME optical-world -> ENU rotation
from .viewer3d import _make_grid, _make_world_axes, _qcolor


class RoomSurfaceWindow(QMainWindow):
    #: Carries a freshly built surface mesh from a background source thread onto
    #: the GUI thread. :meth:`submit` (thread-safe) emits it; the signal is
    #: connected to :meth:`update` so the GL items are only touched on the GUI
    #: thread.
    mesh_ready = pyqtSignal(object, object, object, object)

    def __init__(self, title: str = "Room Surface (3D mesh)") -> None:
        super().__init__()
        self.setWindowTitle(title)
        self.resize(1100, 800)

        self._view = gl.GLViewWidget()
        self._view.setBackgroundColor(QColor(theme.BG))
        self.setCentralWidget(self._view)

        # SAME grid + ENU origin triad as MapWindow / Viewer3D so the two 3D
        # windows are directly comparable.
        self._view.addItem(_make_grid(size_m=20.0, step_m=1.0))
        for ax in _make_world_axes(length=1.0):
            self._view.addItem(ax)

        # ONE persistent mesh item: a live rebuild re-sets its vertexes/faces in
        # place (instead of stacking a new item per rebuild). Start empty.
        # ``shader='shaded'`` + ``smooth=True`` averages the per-vertex normals
        # across adjacent triangles so the walls/floor read as SMOOTH continuous
        # surfaces (not faceted). Per-VERTEX colours (the height gradient)
        # interpolate across each triangle for a continuous shaded look.
        # ``glOptions='opaque'`` so depth-sorting isn't needed for the (opaque)
        # surfaces.
        self._mesh = gl.GLMeshItem(
            vertexes=np.zeros((0, 3), np.float32),
            faces=np.zeros((0, 3), np.int64),
            vertexColors=np.zeros((0, 4), np.float32),
            shader="shaded", smooth=True, drawEdges=False,
            glOptions="opaque")
        self._view.addItem(self._mesh)

        # Keyframe camera positions (amber dots) -- the capture trail, like the
        # SLAM-map window, so the user can relate the surfaces to the path.
        self._cams = gl.GLScatterPlotItem(
            pos=np.zeros((0, 3), np.float32),
            color=_qcolor(theme.WARN, 0.95), size=9.0, pxMode=True)
        self._view.addItem(self._cams)

        # Frame-the-room ONCE (on the first non-empty update) so the orbit doesn't
        # jump every rebuild as surfaces accumulate.
        self._framed = False

        # Route background-thread meshes through the signal so `update` (GL items)
        # runs on the GUI thread (Qt auto-queues a cross-thread signal emit).
        self.mesh_ready.connect(self.update)

    # ------------------------------------------------------------------ #
    def submit(self, verts, faces, vertex_colors, cams) -> None:
        """Thread-safe ingest: hand a freshly built surface mesh in from any thread.

        Emits :attr:`mesh_ready`; Qt queues the connected :meth:`update` onto the
        GUI thread, so a background IPC/rebuild thread can call this directly.
        """
        self.mesh_ready.emit(verts, faces, vertex_colors, cams)

    # ------------------------------------------------------------------ #
    def update(self, verts: np.ndarray | None,
               faces: np.ndarray | None,
               vertex_colors: np.ndarray | None,
               cams: np.ndarray | None) -> None:
        """Replace the rendered surface mesh + keyframe cameras with fresh data.

        ``verts`` ``(V,3)`` are the merged surface vertices in the camera-optical
        world frame (rotated to ENU here, like the point-cloud map); ``faces``
        ``(F,3)`` are triangle indices into ``verts``; ``vertex_colors`` ``(V,3)``
        is the per-vertex RGB height colour (or None -> a uniform surface colour).
        ``cams`` ``(M,3)`` are the keyframe camera positions. Safe to call
        repeatedly from the GUI thread.
        """
        v = _to_display(verts if verts is not None
                        else np.zeros((0, 3), np.float32))
        f = (np.asarray(faces, dtype=np.int64).reshape(-1, 3)
             if faces is not None and len(faces) else np.zeros((0, 3), np.int64))
        # Guard a malformed mesh (a face index out of range) so setMeshData never
        # raises on the GUI thread: fall back to an empty mesh rather than crash
        # the window.
        if len(f) and (f.min() < 0 or f.max() >= len(v)):
            v = np.zeros((0, 3), np.float32)
            f = np.zeros((0, 3), np.int64)
        # Per-vertex RGB -> opaque RGBA (the shaded mesh interpolates the colour
        # across each triangle). Drop the colours on a vertex/colour mismatch.
        vc = (np.asarray(vertex_colors, dtype=np.float32).reshape(-1, 3)
              if vertex_colors is not None and len(vertex_colors) else None)
        if vc is not None and len(vc) != len(v):       # mismatch -> drop colours
            vc = None
        if vc is not None:
            rgba = np.concatenate(
                [np.clip(vc, 0.0, 1.0), np.ones((len(vc), 1), np.float32)],
                axis=1).astype(np.float32)
            self._mesh.setMeshData(vertexes=v, faces=f, vertexColors=rgba)
        else:
            # No per-vertex colour -> a uniform mid surface colour.
            self._mesh.setMeshData(
                vertexes=v, faces=f,
                color=(0.7, 0.7, 0.72, 1.0))

        cam_enu = _to_display(cams if cams is not None
                              else np.zeros((0, 3), np.float32))
        self._cams.setData(pos=cam_enu, color=_qcolor(theme.WARN, 0.95),
                           size=9.0, pxMode=True)

        # Centre the orbit on the room (the mesh vertices' bounding box), ONCE.
        if not self._framed and len(v):
            centre = v.mean(axis=0)
            extent = float(np.linalg.norm(v.max(axis=0) - v.min(axis=0)))
            self._view.opts["center"] = self._vec(centre)
            self._view.setCameraPosition(distance=max(extent * 0.8, 2.0),
                                         azimuth=45, elevation=30)
            self._framed = True

    @staticmethod
    def _vec(p):
        from PyQt6.QtGui import QVector3D
        return QVector3D(float(p[0]), float(p[1]), float(p[2]))
