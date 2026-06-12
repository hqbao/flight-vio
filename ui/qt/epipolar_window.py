"""In-app Qt window: the LIVE "Epipolar / Rectification" visualiser.

Shows, every frame, the canonical "is my rectification correct?" view on LIVE
data: the same horizontal scanlines across a left|right pair, BEFORE rectification
(top -- raw rows drift off the lines) vs AFTER (bottom -- rows snap onto the same
scanline), with a few strong corners matched both ways and the median vertical
row-mismatch collapsing from before -> after.

It subscribes the capture process's RAW ``imucam.sample`` left+right pair AND the
retained ``calib.stereo`` (the FULL stereo calib the rectifiers need -- both
intrinsics + distortion + the left->right extrinsic, which ``calib.bundle`` does
NOT carry). Once both arrive the source builds the Left/Right rectifiers once,
rectifies the live pair, and hands the window a finished
:class:`ui.viz.epipolar_render.EpipolarRender` to blit.

Threading
---------
The IPC frame recv thread (inside :class:`ui.modules.ipc_sources.IpcEpipolarSource`)
does the rectify + block-match and the callback only latches the freshest record
into a lock-guarded ``_pending`` (cheap); a ~15 Hz ``QTimer`` consumes it on the
GUI thread and renders. So the recv thread and the GUI thread never touch shared
mutable state unguarded (mirrors :class:`ui.qt.ba_window.BaWindow`).

This is a PURE CONSUMER of two always-published capture topics (``imucam.sample``
+ ``calib.stereo``) -- no flag, no opt-in engine: the window just works whenever
capture is up. cv2 is pulled lazily via :mod:`ui.viz.epipolar_render` (only when
this window opens); depthai is never imported (the UI is device-free by contract).
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
#: ``.error`` (see :class:`~ui.modules.ipc_sources.IpcEpipolarSource`).
SourceFactory = Callable[[], object]


class EpipolarWindow(QWidget):
    """The live stereo rectification / epipolar visualiser."""

    def __init__(self, source_factory: SourceFactory,
                 *, parent: QWidget | None = None) -> None:
        super().__init__(parent, QtCore.Qt.WindowType.Window)
        self._make_source = source_factory

        self.setWindowTitle("Epipolar / Rectification")
        self.setObjectName("EpipolarWindow")
        self.resize(1180, 860)
        self.setMinimumSize(860, 620)
        self.setStyleSheet(theme.QSS)

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        root.addWidget(self._build_header())

        self._view = QLabel("waiting for calib / stereo …")
        self._view.setObjectName("Raster")
        self._view.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._view.setMinimumHeight(560)
        self._view.setStyleSheet(f"background:{theme.BG}; color:{theme.TEXT_DIM};")
        root.addWidget(self._view, stretch=1)

        hint = QLabel(
            "scanlines drawn across BOTH images · top = RAW pair (rows drift off "
            "the lines) · bottom = rectified pair (rows snap onto the same line) · "
            "corner + its right match should straddle one scanline (GREEN) once "
            "rectified")
        hint.setObjectName("ScaleTick")
        hint.setWordWrap(True)
        root.addWidget(hint, stretch=0)

        self._status = QLabel("—")
        self._status.setObjectName("ImuCamStatus")
        root.addWidget(self._status, stretch=0)

        self._source = None
        self._buf: np.ndarray | None = None
        self._lock = threading.Lock()
        self._pending = None                       # latest record (latest-wins)
        self._running = False
        self._failed = False
        self._first_seen = False
        self._t_start = 0.0
        self._render = None

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(66)                # ~15 Hz
        self._timer.timeout.connect(self._on_tick)

    # -- construction helpers --------------------------------------------- #
    def _build_header(self) -> QWidget:
        from PyQt6.QtWidgets import QFrame

        bar = QFrame()
        bar.setObjectName("Panel")
        h = QHBoxLayout(bar)
        h.setContentsMargins(8, 4, 8, 4)
        h.setSpacing(8)
        title = QLabel("EPIPOLAR / RECTIFICATION")
        title.setObjectName("HeaderTitle")
        sub = QLabel("stereo rectification — do matches land on the same scanline?")
        sub.setObjectName("HeaderSub")
        h.addWidget(title)
        h.addWidget(sub)
        h.addStretch(1)
        pill = QLabel("LIVE")
        pill.setObjectName("FieldValue")
        h.addWidget(pill)
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
        from ui.viz.epipolar_render import render_epipolar_record

        self._render = render_epipolar_record
        self._source = self._make_source()
        with self._lock:
            self._pending = None
        self._source.start(self._on_record)        # frame recv thread -> _pending
        self._running = True
        self._failed = False
        self._first_seen = False
        self._t_start = time.monotonic()
        self._show_rgb(self._render(None))         # placeholder
        self._set_status("connecting… (needs capture's raw stereo + calib.stereo)",
                         theme.TEXT_DIM)
        self._timer.start()

    def stop(self) -> None:
        self._teardown()

    def _teardown(self) -> None:
        self._timer.stop()
        if self._source is not None:
            try:
                self._source.stop()
            except Exception:                                      # noqa: BLE001
                pass
        self._source = None
        self._running = False

    # -- data flow -------------------------------------------------------- #
    def _on_record(self, rec) -> None:
        """Frame recv-thread callback: latch the freshest record (cheap)."""
        with self._lock:
            self._pending = rec

    def _take_pending(self):
        with self._lock:
            rec, self._pending = self._pending, None
            return rec

    # -- per-tick update -------------------------------------------------- #
    def _on_tick(self) -> None:
        rec = self._take_pending()
        if rec is not None:
            self._first_seen = True
            self._render_record(rec)
        elif not self._first_seen:
            self._maybe_report_no_frame()
        # else: keep the last rendered frame up (no new data this tick).

    # -- render ----------------------------------------------------------- #
    def _render_record(self, rec) -> None:
        from ui.viz.epipolar_render import status_line

        self._show_rgb(self._render(rec))
        self._set_status(
            f"frame {int(getattr(rec, 'seq', 0))} · "
            f"{status_line(rec.matches_before, rec.matches_after)}",
            theme.TEXT)

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
        # No timeout-to-failure here: capture may simply not be up yet -- the
        # "waiting" frame stays until either a record arrives or the source latches
        # a hard error (capture down / mono stream).

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
