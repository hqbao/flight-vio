"""Wire messages for cross-process pub/sub.

The in-process flows publish/subscribe rich numpy-bearing dataclasses
(``ImuCamPacket``, ``DepthFrame``, ``Keyframe``, ``PoseMsg``, ...). Those
travel fine on the in-process :class:`ours.lib.flow.pubsub.Bus` because
publisher and subscriber share memory. Across the IPC boundary we cannot ship
~1 MB numpy arrays through ``pickle`` 20 times per second per subscriber -- so
this module defines a sibling **wire message** for every topic that has to
cross processes.

Two halves
----------
* The wire message is a plain ``@dataclass`` of POD fields + zero or more
  :class:`~ours.lib.ipc.shared_array.SharedArrayRef`. It pickles cheaply.
* The bridge flow (:mod:`ours.flows.bridge`) does the conversion:
    - on publish side: copy each large array into its
      :class:`~ours.lib.ipc.shared_array.SharedArrayRing` slot, build the wire
      message with the resulting refs, send it on the IpcBus.
    - on subscribe side: ``read_copy`` each ref into a private
      ``np.ndarray``, rebuild the in-process dataclass, publish it on the
      local Bus.

All small numpy arrays (IMU rows, track ids, etc.) ride directly inside the
wire message (pickled). The shared-memory ring is only used for the few
multi-hundred-KB streams: gray_left, gray_right, depth_m.

A wire message that does NOT carry any large array is just used as-is (an
:class:`ImuRaw` interval is ~16 floats × M -- pickle is plenty).

Naming
------
``Wire<Topic>`` -- e.g. :class:`WireImuCamPacket`. The bridge maps each wire
type to its in-process counterpart in one table (``_TO_LOCAL`` /
``_TO_WIRE``).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .shared_array import SharedArrayRef


# --------------------------------------------------------------------------- #
# Acquisition: capture --> VIO / SLAM / UI / tools
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class WireCamSync:
    """Wire form of :class:`ours.lib.flow.messages.CamSync`.

    The stereo pair travels through two SharedArrayRing slots; ``right_ref`` may
    be ``None`` for mono cameras (matching the in-proc dataclass).
    """

    seq: int
    ts_ns: int
    gray_left_ref: SharedArrayRef
    gray_right_ref: SharedArrayRef | None


@dataclass(frozen=True)
class WireImuCamPacket:
    """Wire form of :class:`ours.lib.flow.messages.ImuCamPacket`.

    Frames travel through shared memory; the per-frame IMU rows (~tens of
    floats) ride pickled. ``imu_ts`` / ``gyro`` / ``accel`` may be empty
    arrays.
    """

    seq: int
    ts_ns: int
    gray_left_ref: SharedArrayRef
    gray_right_ref: SharedArrayRef | None
    imu_ts: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray


@dataclass(frozen=True)
class WireImuRaw:
    """Wire form of :class:`ours.lib.flow.messages.ImuRaw`.

    Pure POD -- no shared memory needed (only IMU samples for the interval).
    """

    seq: int
    ts_ns: int
    imu_ts: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray


@dataclass(frozen=True)
class WireDepthFrame:
    """Wire form of :class:`ours.lib.flow.messages.DepthFrame`."""

    seq: int
    ts_ns: int
    gray_left_ref: SharedArrayRef
    depth_ref: SharedArrayRef


# --------------------------------------------------------------------------- #
# Calibration (one-shot retained: a new subscriber gets the latest immediately)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class WireCalibBundle:
    """The capture process's broadcast of intrinsics + extrinsics on boot.

    The receiver (VIO / SLAM / tools) needs this BEFORE it can solve, so the
    IPC bus retains the latest published bundle and replays it to every new
    subscriber on connect. ``K`` is the rectified-left intrinsic the rest of
    the pipeline expects; ``T_imu_left`` is the IMU->camera extrinsic
    (4x4), ``None`` when the session has no IMU calibration.

    ``device_id`` is the per-device key for the IMU calibration store: the UI
    keys any calibration it saves (gyro bias / accel calib) by this id, so the
    saved values key IDENTICALLY to the id capture/VIO use on the next start and
    actually take effect. It is ``None`` in replay (no live device -> the UI
    falls back to ``"default"``).

    NOTE (IPC schema): this is the cross-language wire contract. ``device_id``
    is a deliberate, backward-compatible ADDITIVE field -- it has a default and
    is placed AFTER the existing optional fields, so pickling stays safe and old
    subscribers simply ignore it.
    """

    K: np.ndarray                                 # (3, 3) float64
    width: int
    height: int
    fps: int
    T_imu_left: np.ndarray | None = None          # (4, 4) float64
    R_imu_cam: np.ndarray | None = None           # (3, 3) float64
    accel_align: np.ndarray | None = None         # (3,) float64
    gyro_bias: np.ndarray | None = None           # (3,) float64
    device_id: str | None = None                  # per-device IMU calib key


# --------------------------------------------------------------------------- #
# VIO outputs: vio --> SLAM / UI
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class WirePoseMsg:
    """Wire form of :class:`ours.lib.flow.messages.PoseMsg`. Pose is tiny -> POD."""

    seq: int
    ts_ns: int
    T_world_cam: np.ndarray                       # (4, 4) float64
    info: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class WireFrameTracks:
    """Wire form of :class:`ours.lib.flow.messages.FrameTracks`.

    Pure POD: per-frame ids + pixels only. The image / depth used to render the
    overlay arrives separately on ``frame.depth`` (capture publishes both rings).
    See :class:`~ours.lib.flow.messages.FrameTracks` for why this split exists
    (single-writer ring contract).
    """

    seq: int
    ts_ns: int
    ids: np.ndarray
    points: np.ndarray


@dataclass(frozen=True)
class WireFrameInliers:
    """Wire form of :class:`ours.lib.flow.messages.FrameInliers`."""

    seq: int
    ts_ns: int
    ids: np.ndarray


@dataclass(frozen=True)
class WireKeyframe:
    """Wire form of :class:`ours.lib.flow.messages.Keyframe`.

    Image + depth ride shared memory; track arrays are pickled inline (a few
    hundred ints / floats per keyframe).
    """

    seq: int
    T_world_cam: np.ndarray
    gray_left_ref: SharedArrayRef
    depth_ref: SharedArrayRef
    track_ids: np.ndarray | None = None
    track_px: np.ndarray | None = None
    accel: np.ndarray | None = None
    inlier_ids: np.ndarray | None = None


@dataclass(frozen=True)
class WireVioMap:
    """Periodic snapshot of the VIO process's windowed-BA refined trajectory.

    Pure POD -- one ``(K, 3)`` array of refined keyframe world positions
    (camera-optical frame), keyed by ``kf_id``. This is the same payload
    :meth:`ours.lib.engine.subprocess.SubprocessEngine.poll_overlay` produces;
    the VIO process publishes it periodically so the UI can draw the refined
    map behind the live marker without polling an engine handle.
    """

    kf_ids: np.ndarray                            # (K,) int64
    kf_positions: np.ndarray                      # (K, 3) float64, optical frame


# --------------------------------------------------------------------------- #
# SLAM outputs: slam --> UI (and optionally back to VIO for closed-loop)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class WireLoopCorrection:
    """Wire form of :class:`ours.lib.flow.messages.LoopCorrection`.

    Pure POD -- a dict of {kf_seq: (4,4) T_world_cam} after pose-graph
    optimisation, plus the running loop count.
    """

    seq: int
    kf_poses: dict[int, np.ndarray]
    n_loops: int


@dataclass(frozen=True)
class WireSlamMap:
    """Wire form of :class:`ours.lib.flow.messages.SlamOverlay` (topic ``slam.map``).

    The continuous SLAM keyframe-map overlay, published EVERY keyframe by the
    loop-closing SLAM engine (``slam_overlay``), LIVE-only. Pure POD -- the
    keyframe positions are a handful of ``(3,)`` vectors, so they ride the
    message itself (no shared-memory ring). The UI's SLAM tab draws this.

    The converter (:func:`ours.flows.bridge.converters._slam_overlay_to_wire`)
    carries the REAL source frame seqs in ``kf_ids`` so the UI can match each
    corrected keyframe to its dense VIO pose (the rubber-sheet "corrected VIO"
    line); it falls back to ``arange(K)`` only when the seqs are missing or
    length-mismatched (the dots themselves render by POSITION). The structure
    mirrors :class:`WireVioMap`, but note ``last_match`` here is ``(M, 3)`` (the
    just-closed loop's keyframes, flashed) -- not a single ``(3,)`` point.
    """

    kf_ids: np.ndarray                            # (K,) int64 source frame seqs
    kf_positions: np.ndarray                      # (K, 3) float64, optical frame
    n_loops: int = 0
    last_match: np.ndarray | None = None          # (M, 3) optical, flash on new loop


# --------------------------------------------------------------------------- #
# Control sentinel: END signal across the IPC boundary
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class WireEnd:
    """Wire-side END sentinel; bridges back to :data:`ours.lib.flow.messages.END`.

    The in-proc :data:`~ours.lib.flow.messages.END` is the ``object()`` sentinel,
    which is not portable across processes (identity-based equality). The wire
    layer ships :class:`WireEnd` instead and the subscriber bridge rewrites it
    to the local ``END`` before publishing on the in-proc Bus.

    Only meaningful in REPLAY mode (the capture process is reading a session
    file with a finite length). Live capture never sends ``WireEnd``.
    """

    topic: str
