"""Message types passed between flows over the pub/sub bus.

These are plain immutable carriers -- one per topic in ``ours.lib.topics``. Keeping
them here documents exactly what each flow consumes and produces, and keeps the
flows themselves free of ad-hoc dicts.

The :data:`END` sentinel is published on a topic when its upstream flow has no
more data (e.g. the recorded session ran out). Reactive flows forward it to their
own downstream topics so the whole graph drains cleanly; the UI flow uses it to
know the run is finished.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

#: Published on a topic to signal "no more messages will follow on this topic".
END = object()


@dataclass(frozen=True)
class ImuChunk:
    """The session's IMU stream, published once by capture before the frames.

    Lets the odometry flow build its gyro preintegrator and gravity-align the
    initial attitude, exactly like the offline ``vio_run`` driver.
    """

    ts_ns: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    T_imu_cam: np.ndarray | None
    has_extrinsics: bool


@dataclass(frozen=True)
class RawFrame:
    """A captured stereo pair (rectified left + raw right) with its timestamp."""

    seq: int
    ts_ns: int
    gray_left: np.ndarray
    gray_right: np.ndarray | None


@dataclass(frozen=True)
class DepthFrame:
    """A left image with a metric depth map aligned to it."""

    seq: int
    ts_ns: int
    gray_left: np.ndarray
    depth_m: np.ndarray


@dataclass(frozen=True)
class PoseMsg:
    """An estimated camera pose (4x4 ``T_world_cam``) for one frame."""

    seq: int
    ts_ns: int
    T_world_cam: np.ndarray
    info: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Keyframe:
    """A keyframe handed to the back-end / SLAM flows.

    Carries both the high-level snapshot the SLAM map needs (pose + image +
    depth) and the low-level track snapshot the sliding-window BA needs
    (``track_ids`` + ``track_px`` from the odometry front-end, plus an optional
    at-rest ``accel`` for the gravity prior).
    """

    seq: int
    T_world_cam: np.ndarray
    gray_left: np.ndarray
    depth_m: np.ndarray
    track_ids: np.ndarray | None = None
    track_px: np.ndarray | None = None
    accel: np.ndarray | None = None


@dataclass(frozen=True)
class LoopCorrection:
    """A pose-graph correction: rewritten keyframe poses after loop closure."""

    seq: int
    kf_poses: dict[int, np.ndarray]
    n_loops: int
