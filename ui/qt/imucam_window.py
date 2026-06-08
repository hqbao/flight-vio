"""In-app Qt window for the split camera/IMU front-end's synchronised output.

NOT part of the proc4 UI. The 4-process UI intentionally does NOT restore this
"Camera + IMU synced" view -- the triplet view (image | depth | IMU) is its
superset (see PROC4_ARCHITECTURE §6.2). This module is ported for the GUI layer's
historical completeness; ``ui.main`` never imports it.

In the single-process build this was the "visualise live, on our own UI" view: it
ran the REAL split acquisition flows -- ``CamFlow`` + ``ImuCamFlow`` (built
WITHOUT a depth matcher, so it only packs the synced frame+IMU) -- over a private
:class:`~ui.comms.LocalPubSub` and rendered every
:class:`~ui.comms.messages.ImuCamPacket` they publish straight into a Qt widget.

The layout is three honest panels, each showing exactly what the packet carries
(no parallel pipeline):

    [ left | right cameras ]              -- cv2 render (ui.viz)
    [ gyro auto-scaling line chart | accel interactive 3D vector ]  -- pyqtgraph

Device-free contract: the in-process acquisition flows + the live device source
live in the single-process codebase, NOT in this ``ui`` project (capture owns the
device). Importing this module pulls neither depthai nor any acquisition flow;
the live path is lazily guarded so it surfaces a clear reason rather than opening
a device the project cannot. cv2 and pyqtgraph are pulled only when the window is
opened, so the base UI stays lightweight.
"""
from __future__ import annotations

import queue
import time
from collections.abc import Callable

import numpy as np
from PyQt6 import QtCore
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWidgets import QLabel, QSizePolicy, QSplitter, QVBoxLayout, QWidget

from ui.comms import LocalPubSub, Module, topics
from ui.viz.imucam_render import render_cameras
from . import theme
from .imu_panels import Accel3DView, GyroPlot

# (cam source, imu source) factory -- injected so the window runs off whatever
# acquisition graph the caller wires. The cam/imu SOURCE types live in the
# single-process codebase, so the annotation is left untyped here (device-free).
SourceFactory = Callable[[], tuple]


def live_source_factory(width: int = 640, height: int = 400,
                        fps: int = 20) -> SourceFactory:
    """Default factory: the OAK-D cameras + IMU off ONE shared device.

    The in-process live device source (``SharedLiveDevice`` + the live cam/imu
    sources) lives in the single-process codebase, NOT in this device-free ``ui``
    project (capture owns the device). This window is not part of the proc4 UI, so
    the factory is never invoked there; surface a clear reason rather than a raw
    ImportError if it is.
    """
    def _make():
        raise RuntimeError(
            "ImuCamWindow's live device source is not available in the "
            "device-free proc4 UI; this synced view is not part of proc4 "
            "(use the triplet view instead).")
    return _make


class _QueueSink(Module):
    """Drop each ImuCamPacket into a queue for the Qt (display) thread."""

    def __init__(self, bus: LocalPubSub, out: "queue.Queue") -> None:
        super().__init__("imucam-ui-sink", bus)
        self._out = out
        self.on(topics.IMUCAM_SAMPLE, [self._Enqueue(out)])

    class _Enqueue:
        name = "enqueue"

        def __init__(self, out: "queue.Queue") -> None:
            self._out = out

        def run(self, ctx, msg):
            try:
                self._out.put_nowait(msg)
            except queue.Full:
                pass                        # drop to stay realtime on the UI
            return None

    def on_end(self) -> None:
        try:
            self._out.put_nowait(None)      # sentinel: stream finished
        except queue.Full:
            pass


class ImuCamWindow(QWidget):
    """Live synced camera/IMU view embedded in the pose-viewer application."""

    def __init__(self, source_factory: SourceFactory | None = None, *,
                 fps: int = 20, parent: QWidget | None = None) -> None:
        super().__init__(parent, QtCore.Qt.WindowType.Window)
        self._make_sources = source_factory or live_source_factory(fps=fps)
        self._fps = max(1, int(fps))

        self.setWindowTitle("Camera + IMU — synced (live)")
        self.setObjectName("ImuCamWindow")
        self.resize(1280, 760)
        # Keep the panels legible: never let the window collapse to a sliver.
        # A tactical viewer must not shrink telemetry away.
        self.setMinimumSize(820, 540)
        self.setStyleSheet(theme.QSS)

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(4)

        # Vertical split: cameras on top, IMU panels below — both resizable.
        split = QSplitter(QtCore.Qt.Orientation.Vertical)
        split.setChildrenCollapsible(False)

        self._view = QLabel("starting…")
        self._view.setObjectName("ImuCamView")
        self._view.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self._view.setMinimumHeight(220)
        # The camera frame is drawn by scaling the rendered pixmap to the
        # LABEL's own size. The label must NOT report that pixmap as its size
        # hint, or the window grows to fit it, which enlarges the label, which
        # scales the pixmap bigger… a feedback loop that stretches the window.
        # Ignored size policy breaks the loop: the splitter fixes the label
        # size, the pixmap just fits inside it.
        self._view.setSizePolicy(QSizePolicy.Policy.Ignored,
                                 QSizePolicy.Policy.Ignored)
        split.addWidget(self._view)

        # IMU row: auto-scaling gyro line chart | interactive 3D accel vector.
        imu_row = QSplitter(QtCore.Qt.Orientation.Horizontal)
        imu_row.setChildrenCollapsible(False)
        self._gyro = GyroPlot()
        self._accel = Accel3DView()
        imu_row.addWidget(self._gyro)
        imu_row.addWidget(self._accel)
        imu_row.setSizes([640, 640])
        split.addWidget(imu_row)
        split.setSizes([400, 360])

        self._status = QLabel("—")
        self._status.setObjectName("ImuCamStatus")
        root.addWidget(split, stretch=1)
        root.addWidget(self._status, stretch=0)

        self._queue: "queue.Queue" = queue.Queue(maxsize=8)
        # The acquisition flow types live in the single-process codebase, so the
        # member types are left untyped here (this window is not part of proc4).
        self._bus = None
        self._cam = None
        self._imu = None
        self._sink: _QueueSink | None = None
        self._running = False
        self._ended = False
        self._first_seen = False
        self._failed = False
        self._t_start = 0.0
        self._buf: np.ndarray | None = None   # keep QImage backing alive

        # If no frame arrives within this window (and nothing reports an error),
        # assume the device is unreachable/stalled rather than hanging forever.
        self._startup_timeout_s = 12.0

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(15)
        self._timer.timeout.connect(self._on_tick)

    # ------------------------------------------------------------------ #
    def ensure_started(self) -> None:
        """(Re)start streaming. Retries cleanly after a previous failure.

        Called every time the view is opened from the menu: if it is already
        streaming it is a no-op (just raise the window); if it failed before
        (e.g. the OAK-D was unplugged) it tears the dead graph down and starts
        fresh, so plugging the device in and reopening retries.
        """
        if self._running and not self._failed:
            return
        self._teardown()
        self.start()

    def start(self) -> None:
        """Build the flow graph and begin streaming into the widget.

        The in-process acquisition flows (``CamFlow`` / ``ImuCamFlow``) live in
        the single-process codebase, NOT in this device-free ``ui`` project, and
        this synced view is not part of proc4 (the triplet view supersedes it).
        ``_make_sources()`` surfaces a clear reason for the default device source;
        we mirror that here so any injected-source path also fails cleanly rather
        than referencing acquisition flows the ``ui`` project does not vendor.
        """
        if self._running:
            return
        self._clear_queue()                  # drop any stale END sentinel
        self._gyro.clear_history()
        # Surfaces the device-free guard (the default factory raises; the _QueueSink
        # sink + the Cam/ImuCam acquisition flows it would feed are not part of the
        # ``ui`` project).
        self._make_sources()

    def stop(self) -> None:
        self._teardown()

    def _teardown(self) -> None:
        self._timer.stop()
        for f in (self._cam, self._imu, self._sink):
            if f is not None:
                try:
                    f.stop()
                except Exception:
                    pass
        self._cam = self._imu = self._sink = self._bus = None
        self._running = False

    def _clear_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

    # ------------------------------------------------------------------ #
    def _on_tick(self) -> None:
        packet = self._drain_latest()
        if packet is None:
            self._maybe_report_no_frame()
            return
        self._first_seen = True
        self._show(render_cameras(packet.gray_left, packet.gray_right))
        self._gyro.add(packet.gyro)
        self._accel.set_accel(packet.accel)
        self._set_status(
            f"seq={packet.seq}   imu samples={packet.imu_ts.size}   "
            f"left {packet.gray_left.shape[1]}×{packet.gray_left.shape[0]}",
            theme.TEXT_DIM)

    def _set_status(self, text: str, color: str) -> None:
        self._status.setText(text)
        self._status.setStyleSheet(f"color: {color};")

    def _maybe_report_no_frame(self) -> None:
        """Surface a clean error/end state instead of hanging on 'starting…'."""
        if self._first_seen:
            if self._ended:
                self._set_status("stream ended", theme.WARN)
                self._timer.stop()
            return
        # No frame yet: decide whether the stream failed or simply finished.
        reason = self._failure_reason()
        threads_dead = (self._cam is not None and not self._cam.is_alive()
                        and self._imu is not None and not self._imu.is_alive())
        timed_out = (time.monotonic() - self._t_start) > self._startup_timeout_s
        if reason or self._ended or threads_dead or timed_out:
            self._fail(reason or
                       ("no frames — is the OAK-D connected and free? "
                        "(nothing else may hold the device)"))

    def _fail(self, message: str) -> None:
        if self._failed:
            return
        self._failed = True
        # A fault must read as an ALERT, not routine chrome: amber, larger.
        self._view.setStyleSheet(
            f"color: {theme.WARN}; font-size: 15px; font-weight: bold;")
        self._view.setText(f"⚠  {message}")
        self._set_status("not streaming — reopen from the Visualize menu to retry",
                         theme.BAD)
        self._teardown()          # release the dead graph so a reopen retries

    def _failure_reason(self) -> str | None:
        """The first concrete error reported by the camera or IMU source."""
        if self._cam is not None and getattr(self._cam, "error", None):
            return self._cam.error
        imu_src = getattr(self._imu, "source", None)
        if imu_src is not None and getattr(imu_src, "error", None):
            return f"IMU open failed: {imu_src.error}"
        return None

    def _drain_latest(self):
        """Return the most recent packet, dropping stale ones to stay realtime."""
        latest = None
        while True:
            try:
                item = self._queue.get_nowait()
            except queue.Empty:
                break
            if item is None:                              # END sentinel
                self._ended = True
                break
            latest = item
        return latest

    def _show(self, bgr: np.ndarray) -> None:
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])       # BGR -> RGB
        self._buf = rgb                                   # keep alive for QImage
        h, w = rgb.shape[:2]
        img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
        pix = QPixmap.fromImage(img)
        target = self._view.size()
        if target.width() > 1 and target.height() > 1:
            pix = pix.scaled(target, QtCore.Qt.AspectRatioMode.KeepAspectRatio,
                             QtCore.Qt.TransformationMode.SmoothTransformation)
        self._view.setPixmap(pix)

    # ------------------------------------------------------------------ #
    def showEvent(self, event) -> None:                              # noqa: N802
        super().showEvent(event)
        self.ensure_started()

    def closeEvent(self, event) -> None:                             # noqa: N802
        try:
            self.stop()
        finally:
            super().closeEvent(event)
