"""Session recorder: dump checkpoint streams (C0/C1/C2/C3) to disk.

Folder layout follows ``docs/PIPELINE_CHECKPOINTS.md``::

    <out_dir>/
      calib.json
      meta.json                       (written by close())
      input/
        imu.jsonl                     (C1)
        frames.jsonl                  (C0 metadata)
        img/000000_L.png 000000_R.png 000000_D.raw16 ...
      basalt/
        vio_pose.jsonl                (C2: Basalt VIO, FLU world)
        slam_pose.jsonl               (C3: RTABMap SLAM, FLU world)

All ``ts_ns`` values are host-monotonic nanoseconds from the recorder's t0.
All poses are stored in the **FLU world** frame as emitted by Basalt /
RTABMap — NED conversion is a viewer-side concern and would lose
information for comparison purposes.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from threading import Lock
from typing import Any, Sequence

import cv2
import numpy as np


class SessionRecorder:
    def __init__(
        self,
        out_dir: str | Path,
        sensor_name: str = "OAK-D W",
        pipeline_name: str = "basalt_vio + rtabmap_slam",
        params: dict[str, Any] | None = None,
    ) -> None:
        self.out_dir = Path(out_dir)
        self.img_dir = self.out_dir / "input" / "img"
        self.basalt_dir = self.out_dir / "basalt"
        (self.out_dir / "input").mkdir(parents=True, exist_ok=True)
        self.img_dir.mkdir(parents=True, exist_ok=True)
        self.basalt_dir.mkdir(parents=True, exist_ok=True)

        self._lock = Lock()
        self._t0_ns = time.monotonic_ns()
        self._sensor = sensor_name
        self._pipeline = pipeline_name
        self._params = dict(params or {})

        # line-buffered JSONL writers
        self._f_imu = (self.out_dir / "input" / "imu.jsonl").open("w", buffering=1)
        self._f_frames = (self.out_dir / "input" / "frames.jsonl").open("w", buffering=1)
        self._f_vio = (self.basalt_dir / "vio_pose.jsonl").open("w", buffering=1)
        self._f_slam = (self.basalt_dir / "slam_pose.jsonl").open("w", buffering=1)

        self._frame_seq = 0
        self._imu_seq = 0
        self._vio_seq = 0
        self._slam_seq = 0
        self._closed = False

    # ---------------- timing ----------------

    def now_ns(self) -> int:
        return time.monotonic_ns() - self._t0_ns

    # ---------------- calibration ----------------

    def write_calib(self, calib: dict[str, Any]) -> None:
        with (self.out_dir / "calib.json").open("w") as f:
            json.dump(calib, f, indent=2)

    # ---------------- C0: stereo frame ----------------

    def on_stereo(
        self,
        left_u8: np.ndarray,
        right_u8: np.ndarray,
        depth_u16: np.ndarray,
        ts_ns: int | None = None,
    ) -> None:
        with self._lock:
            if self._closed:
                return
            seq = self._frame_seq
            self._frame_seq += 1
            ts = self.now_ns() if ts_ns is None else int(ts_ns)
        base = f"{seq:06d}"
        cv2.imwrite(str(self.img_dir / f"{base}_L.png"), left_u8)
        cv2.imwrite(str(self.img_dir / f"{base}_R.png"), right_u8)
        depth_u16.astype("<u2").tofile(self.img_dir / f"{base}_D.raw16")
        h, w = depth_u16.shape[:2]
        rec = {
            "ts_ns": ts,
            "seq": seq,
            "type": "stereo",
            "left_path": f"img/{base}_L.png",
            "right_path": f"img/{base}_R.png",
            "depth_path": f"img/{base}_D.raw16",
            "width": int(w),
            "height": int(h),
        }
        self._f_frames.write(json.dumps(rec) + "\n")

    # ---------------- C1: IMU sample ----------------

    def on_imu(
        self,
        gyro_xyz: Sequence[float],
        accel_xyz: Sequence[float],
        temp_c: float | None = None,
        ts_ns: int | None = None,
    ) -> None:
        with self._lock:
            if self._closed:
                return
            seq = self._imu_seq
            self._imu_seq += 1
            ts = self.now_ns() if ts_ns is None else int(ts_ns)
        rec = {
            "ts_ns": ts,
            "seq": seq,
            "gyro": [float(x) for x in gyro_xyz],
            "accel": [float(x) for x in accel_xyz],
        }
        if temp_c is not None:
            rec["temp_c"] = float(temp_c)
        self._f_imu.write(json.dumps(rec) + "\n")

    # ---------------- C2 / C3: poses ----------------

    def _write_pose(
        self,
        fp,
        seq: int,
        ts_ns: int,
        pos: Sequence[float],
        quat_wxyz: Sequence[float],
        source: str,
        tracking_ok: bool,
    ) -> None:
        rec = {
            "ts_ns": ts_ns,
            "seq": seq,
            "frame_id": "flu_world",
            "pos": [float(x) for x in pos],
            "quat_wxyz": [float(x) for x in quat_wxyz],
            "tracking_ok": bool(tracking_ok),
            "source": source,
        }
        fp.write(json.dumps(rec) + "\n")

    def on_vio_pose(
        self,
        pos_flu: Sequence[float],
        quat_wxyz: Sequence[float],
        ts_ns: int | None = None,
        tracking_ok: bool = True,
    ) -> None:
        with self._lock:
            if self._closed:
                return
            seq = self._vio_seq
            self._vio_seq += 1
            ts = self.now_ns() if ts_ns is None else int(ts_ns)
        self._write_pose(self._f_vio, seq, ts, pos_flu, quat_wxyz,
                         "basalt_vio", tracking_ok)

    def on_slam_pose(
        self,
        pos_flu: Sequence[float],
        quat_wxyz: Sequence[float],
        ts_ns: int | None = None,
        tracking_ok: bool = True,
    ) -> None:
        with self._lock:
            if self._closed:
                return
            seq = self._slam_seq
            self._slam_seq += 1
            ts = self.now_ns() if ts_ns is None else int(ts_ns)
        self._write_pose(self._f_slam, seq, ts, pos_flu, quat_wxyz,
                         "rtabmap_slam", tracking_ok)

    # ---------------- shutdown ----------------

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            duration_s = self.now_ns() / 1e9
            for fp in (self._f_imu, self._f_frames, self._f_vio, self._f_slam):
                try:
                    fp.flush()
                    fp.close()
                except Exception:
                    pass
        meta = {
            "session_id": self.out_dir.name,
            "pipeline": self._pipeline,
            "sensor": self._sensor,
            "duration_s": round(duration_s, 3),
            "counts": {
                "frames": self._frame_seq,
                "imu_samples": self._imu_seq,
                "vio_poses": self._vio_seq,
                "slam_poses": self._slam_seq,
            },
            "params": self._params,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        }
        with (self.out_dir / "meta.json").open("w") as f:
            json.dump(meta, f, indent=2)
