"""In-app Qt window: the "Loop Closure" verification funnel (ALGORITHMS.md viz #1).

The view that explains WHY a loop closure fired or got rejected -- the biggest
remaining UI gap (in Viewer3D a loop is just a flash dot). It subscribes the
SLAM engine's per-candidate match funnel on ``slam.loop`` AND the VIO keyframe
GRAY images on ``keyframe`` (buffered by seq), and for each loop event draws:

* the CURRENT and MATCHED-OLD keyframe grays side-by-side (each labelled by seq;
  an evicted gray renders as a placeholder pane -- the counts still show);
* one LINE per matched ORB keypoint, COLOUR-CODED by the verification stage it
  survived: GREY = appearance-dropped, YELLOW = epipolar inlier (not PnP),
  GREEN = PnP inlier;
* the funnel readout ("appearance 98 -> epipolar 93 -> PnP 60"), the rotation-
  gate verdict ("rot 12.98 deg <= gate 30 deg"), and an ACCEPTED/REJECTED banner.

Loops are sporadic, so the window keeps the LAST event shown until the next one.

Source model (injected ``source_factory``)
------------------------------------------
The window taps a duck-typed loop stream built by an injected zero-arg factory.
In the 4-process proc4 UI the factory is ALWAYS the IPC adapter
(:class:`~ui.modules.ipc_sources.IpcLoopMatchSource`), which subscribes SLAM's
``slam.loop`` + VIO's ``keyframe`` over IPC -- the UI never opens a device. The
source pushes each finished :class:`~ui.viz.loop_render.LoopEvent` from its IPC
recv thread; the window's QTimer renders it on the GUI thread, so the buffer
between them is guarded by a lock.

cv2 is pulled lazily via :mod:`ui.viz.loop_render` (only when this window opens);
depthai is never imported (the UI is device-free by contract).
"""
from __future__ import annotations

import threading
import time
from collections.abc import Callable

import numpy as np
from PyQt6 import QtCore
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from . import theme

#: Zero-arg factory returning an object with ``start(callback)`` / ``stop()`` /
#: ``.error`` that streams ``LoopEvent`` records to ``callback``.
SourceFactory = Callable[[], object]


class LoopClosureWindow(QWidget):
    """The loop-closure verification funnel view (two keyframes + match lines)."""

    def __init__(self, source_factory: SourceFactory, *,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent, QtCore.Qt.WindowType.Window)
        self._make_source = source_factory

        self.setWindowTitle("Loop Closure")
        self.setObjectName("LoopClosureWindow")
        self.resize(1140, 680)
        self.setMinimumSize(820, 520)
        self.setStyleSheet(theme.QSS)

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        root.addWidget(self._build_header())

        self._view = QLabel("waiting for a loop closure …")
        self._view.setObjectName("Raster")
        self._view.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._view.setMinimumHeight(440)
        self._view.setStyleSheet(f"background:{theme.BG}; color:{theme.TEXT_DIM};")
        root.addWidget(self._view, stretch=1)

        hint = QLabel(
            "GREY = appearance-dropped · YELLOW = epipolar inlier (rejected by "
            "PnP) · GREEN = PnP inlier (confirmed) · the line joins the SAME ORB "
            "feature in the two keyframes")
        hint.setObjectName("ScaleTick")
        hint.setWordWrap(True)
        root.addWidget(hint, stretch=0)

        self._status = QLabel("—")
        self._status.setObjectName("ImuCamStatus")
        root.addWidget(self._status, stretch=0)

        self._source = None
        self._buf: np.ndarray | None = None
        self._lock = threading.Lock()
        self._pending = None                      # latest LoopEvent (latest-wins)
        self._running = False
        self._failed = False
        self._first_seen = False
        self._t_start = 0.0
        self._startup_timeout_s = 30.0            # loops are sporadic -> patient

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(50)               # ~20 Hz; events are rare
        self._timer.timeout.connect(self._on_tick)

    # -- construction helpers --------------------------------------------- #
    def _build_header(self) -> QWidget:
        from PyQt6.QtWidgets import QFrame

        bar = QFrame()
        bar.setObjectName("Panel")
        h = QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(8)
        title = QLabel("LOOP CLOSURE")
        title.setObjectName("HeaderTitle")
        sub = QLabel("why a loop fired (or got rejected)")
        sub.setObjectName("HeaderSub")
        h.addWidget(title)
        h.addWidget(sub)
        h.addStretch(1)
        self._pill = QLabel("IPC")
        self._pill.setObjectName("FieldValue")
        h.addWidget(self._pill)
        return bar

    # -- lifecycle -------------------------------------------------------- #
    def ensure_started(self) -> None:
        if self._running and not self._failed:
            return
        self._teardown()
        self.start()

    def start(self) -> None:
        if self._running:
            return
        from ui.viz.loop_render import render_loop

        # Show the "waiting" frame immediately (renders without an event).
        self._render = render_loop
        self._source = self._make_source()
        with self._lock:
            self._pending = None
        self._source.start(self._on_event)        # IPC recv thread -> _pending
        self._running = True
        self._failed = False
        self._first_seen = False
        self._t_start = time.monotonic()
        self._show_rgb(self._render(None))
        self._set_status("connecting…  (loops are sporadic)", theme.TEXT_DIM)
        self._timer.start()

    def stop(self) -> None:
        self._teardown()

    def _teardown(self) -> None:
        self._timer.stop()
        if self._source is not None:
            try:
                self._source.stop()
            except Exception:
                pass
        self._source = None
        self._running = False

    # -- data flow -------------------------------------------------------- #
    def _on_event(self, ev) -> None:
        """IPC recv-thread callback: latch the latest LoopEvent (cheap)."""
        with self._lock:
            self._pending = ev                    # keep only the freshest event

    def _take_event(self):
        with self._lock:
            ev, self._pending = self._pending, None
            return ev

    # -- per-tick update -------------------------------------------------- #
    def _on_tick(self) -> None:
        ev = self._take_event()
        if ev is not None:
            self._first_seen = True
            self._show_rgb(self._render(ev))
            verdict = "ACCEPTED" if ev.accepted else "REJECTED"
            col = theme.GOOD if ev.accepted else theme.BAD
            rot = (f"{ev.rot_deg:.2f}°" if np.isfinite(ev.rot_deg) else "n/a")
            self._set_status(
                f"kf {ev.cur_seq} ↔ {ev.old_seq}  ·  appearance {ev.n_appearance} "
                f"→ epipolar {ev.n_fmat} → PnP {ev.n_pnp}  ·  rot {rot}  ·  "
                f"{verdict}", col)
        elif not self._first_seen:
            self._maybe_report_no_frame()
        # else: keep the LAST event shown (loops are sporadic).

    def _show_rgb(self, rgb: np.ndarray) -> None:
        g = np.ascontiguousarray(rgb)
        self._buf = g
        self._blit(self._view, QImage(g.data, g.shape[1], g.shape[0],
                                      3 * g.shape[1],
                                      QImage.Format.Format_RGB888))

    @staticmethod
    def _blit(label: QLabel, img: QImage) -> None:
        pix = QPixmap.fromImage(img)
        target = label.size()
        if target.width() > 1 and target.height() > 1:
            pix = pix.scaled(target, QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                             QtCore.Qt.TransformationMode.SmoothTransformation)
        label.setPixmap(pix)

    # -- failure handling ------------------------------------------------- #
    def _maybe_report_no_frame(self) -> None:
        err = self._source.error if self._source is not None else None
        if err:
            self._fail(err)
        # No timeout-to-failure here: a healthy SLAM that simply has not closed a
        # loop yet is NOT an error -- the "waiting" frame stays up. Only a real
        # connect/runtime error (surfaced on the source) fails the window.

    def _fail(self, message: str) -> None:
        if self._failed:
            return
        self._failed = True
        self._view.setStyleSheet(
            f"color: {theme.WARN}; font-size: 14px; font-weight: bold;"
            f" background:{theme.BG};")
        self._view.setText(f"⚠  {message}")
        self._set_status("not streaming — reopen from the Visualize menu to retry",
                         theme.BAD)
        self._teardown()

    def _set_status(self, text: str, color: str) -> None:
        self._status.setText(text)
        self._status.setStyleSheet(f"color: {color};")

    # -- Qt events -------------------------------------------------------- #
    def showEvent(self, event) -> None:                            # noqa: N802
        super().showEvent(event)
        self.ensure_started()

    def closeEvent(self, event) -> None:                           # noqa: N802
        try:
            self.stop()
        finally:
            super().closeEvent(event)
