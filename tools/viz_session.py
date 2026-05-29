#!/usr/bin/env python3
"""Visualize a recorded session — one tab per pipeline checkpoint.

Usage::

    ./tools/viz_session.py sessions/2026-05-29_loop1
    ./tools/viz_session.py /tmp/oakd_rec_smoke

Tabs
----
- Overview : meta.json + calib summary + record counts
- C0 Frame : rectified-left, rectified-right, depth colormap; slider scrub
- C1 IMU   : 6-channel gyro + accel time series
- C2/C3 Pose : 3D trajectory (VIO vs SLAM overlay) + pos/quat timeseries
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from PyQt6.QtCore import Qt                                  # noqa: E402
from PyQt6.QtGui import QImage, QPixmap                      # noqa: E402
from PyQt6.QtWidgets import (                                # noqa: E402
    QApplication, QHBoxLayout, QLabel, QMainWindow, QSlider, QSplitter,
    QTabWidget, QTextEdit, QVBoxLayout, QWidget,
)

import pyqtgraph as pg                                       # noqa: E402
import pyqtgraph.opengl as gl                                # noqa: E402

from oakd.ui import theme                                    # noqa: E402


# ============================================================================
# Loaders
# ============================================================================

def _load_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    out = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def _stack_poses(records: list[dict]) -> dict[str, np.ndarray]:
    """Return ts_s, pos (N,3), quat (N,4 wxyz)."""
    if not records:
        return {
            "ts_s": np.zeros(0),
            "pos": np.zeros((0, 3)),
            "quat": np.zeros((0, 4)),
            "tracking_ok": np.zeros(0, dtype=bool),
        }
    ts = np.array([r["ts_ns"] for r in records], dtype=np.float64) / 1e9
    pos = np.array([r["pos"] for r in records], dtype=np.float64)
    quat = np.array([r["quat_wxyz"] for r in records], dtype=np.float64)
    ok = np.array([r.get("tracking_ok", True) for r in records], dtype=bool)
    return {"ts_s": ts, "pos": pos, "quat": quat, "tracking_ok": ok}


def _stack_imu(records: list[dict]) -> dict[str, np.ndarray]:
    if not records:
        return {"ts_s": np.zeros(0), "gyro": np.zeros((0, 3)),
                "accel": np.zeros((0, 3))}
    ts = np.array([r["ts_ns"] for r in records], dtype=np.float64) / 1e9
    gyro = np.array([r["gyro"] for r in records], dtype=np.float64)
    accel = np.array([r["accel"] for r in records], dtype=np.float64)
    return {"ts_s": ts, "gyro": gyro, "accel": accel}


# ============================================================================
# Tabs
# ============================================================================

class OverviewTab(QWidget):
    def __init__(self, session_dir: Path, meta: dict, calib: dict,
                 imu_n: int, frame_n: int, vio_n: int, slam_n: int,
                 parent=None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)

        title = QLabel(f"SESSION  {session_dir.name}")
        title.setObjectName("HeaderTitle")
        sub = QLabel(str(session_dir))
        sub.setObjectName("HeaderSub")
        lay.addWidget(title)
        lay.addWidget(sub)
        lay.addSpacing(8)

        txt = QTextEdit(readOnly=True)
        txt.setStyleSheet(
            f"background:{theme.PANEL}; color:{theme.TEXT}; "
            f"border:1px solid {theme.PANEL_EDGE}; padding:8px;"
        )

        lines = []
        lines.append("=== META ===")
        lines.append(json.dumps(meta, indent=2))
        lines.append("")
        lines.append("=== COUNTS (actual files) ===")
        lines.append(f"  frames    : {frame_n}")
        lines.append(f"  imu       : {imu_n}")
        lines.append(f"  vio poses : {vio_n}")
        lines.append(f"  slam poses: {slam_n}")
        dur = meta.get("duration_s", 0.0)
        if dur > 0:
            lines.append("")
            lines.append("=== RATES (avg) ===")
            lines.append(f"  frames : {frame_n/dur:6.2f} Hz")
            lines.append(f"  imu    : {imu_n/dur:6.2f} Hz")
            lines.append(f"  vio    : {vio_n/dur:6.2f} Hz")
            lines.append(f"  slam   : {slam_n/dur:6.2f} Hz")
        lines.append("")
        lines.append("=== CALIBRATION ===")
        lines.append(json.dumps(calib, indent=2))

        txt.setPlainText("\n".join(lines))
        lay.addWidget(txt, 1)


class FrameTab(QWidget):
    """C0: stereo left + right + depth colormap; scrub via slider."""

    def __init__(self, session_dir: Path, frames: list[dict], parent=None) -> None:
        super().__init__(parent)
        self.session_dir = session_dir
        self.frames = frames

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)

        self.info = QLabel("(no frames)" if not frames else "")
        self.info.setObjectName("HeaderSub")
        lay.addWidget(self.info)

        img_row = QHBoxLayout()
        img_row.setSpacing(6)
        self.lbl_left = self._make_img_label("LEFT")
        self.lbl_right = self._make_img_label("RIGHT")
        self.lbl_depth = self._make_img_label("DEPTH")
        for grp in (self.lbl_left, self.lbl_right, self.lbl_depth):
            img_row.addWidget(grp["frame"], 1)
        lay.addLayout(img_row, 1)

        ctl_row = QHBoxLayout()
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, len(frames) - 1))
        self.slider.valueChanged.connect(self._on_seek)
        ctl_row.addWidget(QLabel("frame:"))
        ctl_row.addWidget(self.slider, 1)
        lay.addLayout(ctl_row)

        if frames:
            self._on_seek(0)

    def _make_img_label(self, title: str) -> dict:
        frame = QWidget()
        frame.setObjectName("Panel")
        frame.setStyleSheet(
            f"#Panel {{ background:{theme.PANEL}; "
            f"border:1px solid {theme.PANEL_EDGE}; border-radius:4px; }}"
        )
        v = QVBoxLayout(frame)
        v.setContentsMargins(4, 4, 4, 4)
        title_lbl = QLabel(title)
        title_lbl.setObjectName("PanelTitle")
        title_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        img_lbl = QLabel()
        img_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        img_lbl.setMinimumSize(320, 200)
        img_lbl.setStyleSheet("background:#000;")
        v.addWidget(title_lbl)
        v.addWidget(img_lbl, 1)
        return {"frame": frame, "title": title_lbl, "img": img_lbl}

    def _on_seek(self, idx: int) -> None:
        if not self.frames:
            return
        rec = self.frames[idx]
        w, h = int(rec["width"]), int(rec["height"])
        base = self.session_dir / "input"

        left = cv2.imread(str(base / rec["left_path"]), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(base / rec["right_path"]), cv2.IMREAD_GRAYSCALE)
        depth = np.fromfile(base / rec["depth_path"], dtype="<u2").reshape(h, w)

        self._set_gray(self.lbl_left["img"], left)
        self._set_gray(self.lbl_right["img"], right)
        self._set_depth(self.lbl_depth["img"], depth)

        ts_s = rec["ts_ns"] / 1e9
        self.info.setText(
            f"frame  seq={rec['seq']:>5d}   t={ts_s:7.3f}s   {w}x{h}   "
            f"depth: valid={int((depth > 0).sum())}/{w*h}  "
            f"min={int(depth[depth>0].min()) if (depth>0).any() else 0}mm  "
            f"max={int(depth.max())}mm"
        )

    @staticmethod
    def _set_gray(label: QLabel, img: np.ndarray | None) -> None:
        if img is None:
            label.clear()
            return
        h, w = img.shape
        qimg = QImage(img.tobytes(), w, h, w, QImage.Format.Format_Grayscale8)
        pm = QPixmap.fromImage(qimg).scaled(
            label.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        label.setPixmap(pm)

    @staticmethod
    def _set_depth(label: QLabel, depth_u16: np.ndarray) -> None:
        valid = depth_u16 > 0
        if not valid.any():
            label.clear()
            return
        # Normalize valid range to 0..255, invalid stays black
        vmax = float(depth_u16[valid].max())
        norm = np.zeros_like(depth_u16, dtype=np.uint8)
        norm[valid] = np.clip(
            (depth_u16[valid].astype(np.float32) / vmax * 255.0), 0, 255
        ).astype(np.uint8)
        colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        colored[~valid] = 0
        h, w = colored.shape[:2]
        # BGR -> RGB
        rgb = cv2.cvtColor(colored, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.tobytes(), w, h, 3 * w, QImage.Format.Format_RGB888)
        pm = QPixmap.fromImage(qimg).scaled(
            label.size(), Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        label.setPixmap(pm)


class IMUTab(QWidget):
    """C1: 6-channel gyro + accel time series."""

    def __init__(self, imu: dict, parent=None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)

        n = len(imu["ts_s"])
        info = QLabel(f"IMU samples: {n}" if n else "(no IMU data)")
        info.setObjectName("HeaderSub")
        lay.addWidget(info)
        if n == 0:
            return

        pg.setConfigOption("background", theme.BG)
        pg.setConfigOption("foreground", theme.TEXT)

        ts = imu["ts_s"] - imu["ts_s"][0]

        self.p_gyro = pg.PlotWidget(title="GYRO (rad/s)")
        self.p_gyro.showGrid(x=True, y=True, alpha=0.3)
        self.p_gyro.addLegend()
        for i, (name, color) in enumerate(
            (("x", theme.AXIS_N), ("y", theme.AXIS_E), ("z", theme.AXIS_U))
        ):
            self.p_gyro.plot(ts, imu["gyro"][:, i],
                             pen=pg.mkPen(color, width=1), name=name)

        self.p_acc = pg.PlotWidget(title="ACCEL (m/s²)")
        self.p_acc.showGrid(x=True, y=True, alpha=0.3)
        self.p_acc.addLegend()
        for i, (name, color) in enumerate(
            (("x", theme.AXIS_N), ("y", theme.AXIS_E), ("z", theme.AXIS_U))
        ):
            self.p_acc.plot(ts, imu["accel"][:, i],
                            pen=pg.mkPen(color, width=1), name=name)

        # Link x-axes
        self.p_acc.setXLink(self.p_gyro)

        split = QSplitter(Qt.Orientation.Vertical)
        split.addWidget(self.p_gyro)
        split.addWidget(self.p_acc)
        lay.addWidget(split, 1)


class PoseTab(QWidget):
    """C2/C3: 3D trajectory + position/quaternion timeseries (overlay)."""

    def __init__(self, vio: dict, slam: dict, parent=None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)

        info = QLabel(
            f"VIO poses: {len(vio['ts_s']):>5d}   "
            f"SLAM poses: {len(slam['ts_s']):>5d}   "
            f"(FLU world frame — raw)"
        )
        info.setObjectName("HeaderSub")
        lay.addWidget(info)

        split = QSplitter(Qt.Orientation.Horizontal)

        # --- 3D trajectory ---
        gl_widget = gl.GLViewWidget()
        gl_widget.setBackgroundColor(theme.BG)
        gl_widget.setCameraPosition(distance=10, elevation=25, azimuth=-60)
        # ground grid
        grid = gl.GLGridItem()
        grid.setColor(pg.mkColor(theme.GRID))
        grid.setSize(20, 20)
        grid.setSpacing(1, 1)
        gl_widget.addItem(grid)
        # world axes
        ax_len = 1.0
        for v, color in (((ax_len, 0, 0), theme.AXIS_N),
                         ((0, ax_len, 0), theme.AXIS_E),
                         ((0, 0, ax_len), theme.AXIS_U)):
            ln = gl.GLLinePlotItem(
                pos=np.array([[0, 0, 0], v]),
                color=pg.glColor(color), width=2, antialias=True,
            )
            gl_widget.addItem(ln)
        if len(vio["pos"]) >= 2:
            gl_widget.addItem(gl.GLLinePlotItem(
                pos=vio["pos"].astype(np.float32),
                color=pg.glColor(theme.WARN), width=2, antialias=True,
            ))
        if len(slam["pos"]) >= 2:
            gl_widget.addItem(gl.GLLinePlotItem(
                pos=slam["pos"].astype(np.float32),
                color=pg.glColor(theme.GOOD), width=2, antialias=True,
            ))

        # legend overlay (Qt label)
        legend = QLabel(
            f'<span style="color:{theme.WARN}">●</span> VIO (Basalt)   '
            f'<span style="color:{theme.GOOD}">●</span> SLAM (RTABMap)'
        )
        legend.setStyleSheet(f"color:{theme.TEXT}; padding:4px;")

        left_pane = QWidget()
        lp = QVBoxLayout(left_pane)
        lp.setContentsMargins(0, 0, 0, 0)
        lp.addWidget(legend)
        lp.addWidget(gl_widget, 1)
        split.addWidget(left_pane)

        # --- timeseries (pos x/y/z, quat w/x/y/z) ---
        pg.setConfigOption("background", theme.BG)
        pg.setConfigOption("foreground", theme.TEXT)

        ts_panel = QWidget()
        tsv = QVBoxLayout(ts_panel)
        tsv.setContentsMargins(0, 0, 0, 0)

        self.p_pos = pg.PlotWidget(title="POSITION (m, FLU)")
        self.p_pos.showGrid(x=True, y=True, alpha=0.3)
        self.p_pos.addLegend()
        self.p_quat = pg.PlotWidget(title="QUATERNION (wxyz)")
        self.p_quat.showGrid(x=True, y=True, alpha=0.3)
        self.p_quat.addLegend()
        self.p_quat.setXLink(self.p_pos)

        def add_pos(src: dict, label: str, style: int) -> None:
            if not len(src["ts_s"]):
                return
            t = src["ts_s"] - src["ts_s"][0]
            for i, (axis, color) in enumerate(
                (("x", theme.AXIS_N), ("y", theme.AXIS_E), ("z", theme.AXIS_U))
            ):
                self.p_pos.plot(
                    t, src["pos"][:, i],
                    pen=pg.mkPen(color, width=1, style=style),
                    name=f"{label}.{axis}",
                )

        def add_quat(src: dict, label: str, style: int) -> None:
            if not len(src["ts_s"]):
                return
            t = src["ts_s"] - src["ts_s"][0]
            for i, (axis, color) in enumerate((
                ("w", theme.TEXT_DIM), ("x", theme.AXIS_N),
                ("y", theme.AXIS_E), ("z", theme.AXIS_U),
            )):
                self.p_quat.plot(
                    t, src["quat"][:, i],
                    pen=pg.mkPen(color, width=1, style=style),
                    name=f"{label}.{axis}",
                )

        add_pos(vio, "vio", Qt.PenStyle.SolidLine)
        add_pos(slam, "slam", Qt.PenStyle.DashLine)
        add_quat(vio, "vio", Qt.PenStyle.SolidLine)
        add_quat(slam, "slam", Qt.PenStyle.DashLine)

        ts_split = QSplitter(Qt.Orientation.Vertical)
        ts_split.addWidget(self.p_pos)
        ts_split.addWidget(self.p_quat)
        tsv.addWidget(ts_split, 1)
        split.addWidget(ts_panel)

        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 1)
        lay.addWidget(split, 1)


# ============================================================================
# Main window
# ============================================================================

class SessionViewer(QMainWindow):
    def __init__(self, session_dir: Path) -> None:
        super().__init__()
        self.setWindowTitle(f"oak-d session viewer — {session_dir.name}")
        self.resize(1400, 900)
        self.setStyleSheet(theme.QSS)

        meta = {}
        meta_p = session_dir / "meta.json"
        if meta_p.exists():
            meta = json.loads(meta_p.read_text())

        calib = {}
        calib_p = session_dir / "calib.json"
        if calib_p.exists():
            calib = json.loads(calib_p.read_text())

        frames = _load_jsonl(session_dir / "input" / "frames.jsonl")
        imu = _stack_imu(_load_jsonl(session_dir / "input" / "imu.jsonl"))
        vio = _stack_poses(_load_jsonl(session_dir / "basalt" / "vio_pose.jsonl"))
        slam = _stack_poses(_load_jsonl(session_dir / "basalt" / "slam_pose.jsonl"))

        tabs = QTabWidget()
        tabs.addTab(OverviewTab(
            session_dir, meta, calib,
            imu_n=len(imu["ts_s"]), frame_n=len(frames),
            vio_n=len(vio["ts_s"]), slam_n=len(slam["ts_s"]),
        ), "Overview")
        tabs.addTab(FrameTab(session_dir, frames), "C0 · Frame")
        tabs.addTab(IMUTab(imu), "C1 · IMU")
        tabs.addTab(PoseTab(vio, slam), "C2/C3 · Pose")
        self.setCentralWidget(tabs)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("session_dir", help="path to a recorded session folder")
    args = ap.parse_args()

    sd = Path(args.session_dir).resolve()
    if not sd.is_dir():
        print(f"not a directory: {sd}", file=sys.stderr)
        return 2

    app = QApplication(sys.argv)
    win = SessionViewer(sd)
    win.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
