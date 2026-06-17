"""The per-keyframe solve, factored out so in-process and subprocess engines run
*exactly the same code* (the whole offline byte-parity argument depends on this).

Each ``*_step`` takes a live map object + one keyframe snapshot and returns the
solve result (or ``None`` when there is nothing to publish for that keyframe).
These are pure functions of (map, snapshot): no threads, no queues, no flow/bus
knowledge -- they receive the map instance so the same function drives both the
synchronous :class:`~ba.engine.inprocess.InProcessEngine` and the child of
:class:`~ba.engine.subprocess.SubprocessEngine`.

The logic is lifted verbatim from the old in-thread ``RunBA`` task so the offline
path stays identical.

The opt-in ``--ba-window`` capture variant (a richer sibling that snapshots the
full solve state for the visualizer) lives in :mod:`ba.engine.ba_capture`.
"""
from __future__ import annotations

from typing import Any


def ba_step(ba_map, snap: Any):
    """One windowed-BA keyframe: add the track snapshot, run BA.

    ``snap`` = ``(T_cw, ids, pts, depth_m, accel)`` in the raw f2f world frame
    (the flow inverts ``T_world_cam`` -> ``T_cw`` before submitting). Returns the
    refined latest ``T_cw`` (``4x4``) or ``None`` when the window has not yet
    enough structure to optimise.
    """
    T_cw, ids, pts, depth_m, accel = snap
    if ids is None or pts is None:
        return None
    ba_map.add_keyframe(T_cw, ids, pts, depth_m, accel_cam=accel)
    return ba_map.run_ba()                    # refined latest T_cw, or None


#: Health fields lifted from ``WindowedVIOMap.last_info`` onto the published pose.
#: ``vio_degraded`` is the load-bearing one (the divergence guard fired this
#: keyframe -> a detected fault the FC must see); the reprojection error + window
#: jump are the two diagnostics behind that decision. All are plain scalars so the
#: tuple crosses the subprocess pickle boundary cleanly (see ``run_ba``).
_VIO_HEALTH_KEYS = ("vio_degraded", "vio_reproj_px", "vio_window_jump_m")


def _vio_health(vio_map) -> dict:
    """Extract the picklable tight-VIO health fields from ``last_info``.

    Casts ``vio_degraded`` to ``bool`` and the diagnostics to ``float`` so the
    returned dict holds ONLY plain Python scalars (no numpy types / objects) --
    the subprocess engine pickles this across the process boundary, and the FC
    info consumer wants stable scalar types. A key absent from ``last_info`` is
    simply omitted (e.g. the guard was off / no jump computed).
    """
    src = getattr(vio_map, "last_info", None) or {}
    info: dict = {}
    for k in _VIO_HEALTH_KEYS:
        if k not in src:
            continue
        info[k] = bool(src[k]) if k == "vio_degraded" else float(src[k])
    return info


def _backend_bias(vio_map):
    """Latest keyframe's optimised ``(bg, ba)`` as plain ndarrays, or ``None``.

    Crosses the subprocess ``out_q`` with the step result (mirror ``_vio_health``'s
    plain-data discipline -- numpy arrays pickle cleanly). The live
    ``propagate_imu`` adopts these into its dead-reckoning bias: the tight
    backend->live feed-forward (PLAN P1/P2). TIGHT path only; a copy so the
    worker map's state is never aliased across the boundary.
    """
    # The optimised per-keyframe bias lives in the WindowedVIOMap's keyframe dicts
    # (``keyframes[-1]["bg"]/["ba"]``, written back after a HEALTHY solve --
    # ``sky/vio/window.py``; a degraded solve returns before the write-back so the
    # latest keyframe keeps the last-healthy bias, which the ``degraded`` flag gates
    # downstream). NOT ``vio_map.bg`` -- the map only has the ``bg0``/``ba0`` seeds.
    kfs = getattr(vio_map, "keyframes", None)
    if not kfs:
        return None
    kf = kfs[-1]
    bg, ba = kf.get("bg"), kf.get("ba")
    if bg is None or ba is None:
        return None
    return (bg.copy(), ba.copy())


def vio_step(vio_map, snap: Any):
    """One tight-coupled VIO keyframe: add the track snapshot + IMU block, solve.

    Mirrors :func:`ba_step` but for the tight backend
    (:class:`sky.vio.window.WindowedVIOMap`): the snapshot is a
    SUPERSET of the loose one carrying the keyframe timestamp + the raw
    inter-keyframe IMU segment the joint optimiser preintegrates --

        ``snap`` = ``(T_cw, ids, pts, depth_m, ts_ns, imu_seg)``

    where ``imu_seg`` is ``(ts_ns, gyro_cam, accel_cam)`` in the camera optical
    frame (or ``None`` -> the map slices its stored stream, empty live).

    Returns ``(T_cw, health, backend_bias)`` -- the refined latest ``T_cw``
    (``4x4``) PLUS the tight health fields the map stamped on ``last_info`` (at
    least ``vio_degraded``; see :func:`_vio_health`) PLUS the latest keyframe's
    optimised ``(bg, ba)`` for the live feed-forward (see :func:`_backend_bias`;
    ``None`` when no bias yet) -- or ``None`` when the window has not yet enough
    structure / IMU to optimise. The health dict carries the divergence-guard
    verdict end-to-end to the published pose / FC; without it ``vio_degraded``
    would be computed at the map and DROPPED at this boundary (a detected fault
    with no consumer). The loose :func:`ba_step` deliberately returns the bare
    ``T_cw`` (no tuple), so the loose published pose info is untouched.
    """
    T_cw, ids, pts, depth_m, ts_ns, imu_seg = snap
    if ids is None or pts is None:
        return None
    vio_map.add_keyframe(T_cw, ids, pts, depth_m, ts_ns, imu_seg=imu_seg)
    T_cw = vio_map.run_ba()                     # refined latest T_cw, or None
    if T_cw is None:
        return None
    return (T_cw, _vio_health(vio_map), _backend_bias(vio_map))


# --------------------------------------------------------------------------- #
# Overlay extractors: a cheap, picklable snapshot of the live MAP for the 3D
# viewer (the visible "refined map behind the responsive marker"). All positions
# are camera world-frame (optical); the UI applies the single optical->NED display
# transform. These read REAL map outputs (refined keyframe poses / corrected
# SLAM poses + loop events) -- never a parallel/derived pipeline.
# --------------------------------------------------------------------------- #

def ba_overlay(ba_map):
    """BA window snapshot: ``{kf_id: refined camera-world position}``.

    Keyed by the map's monotonic keyframe id so the UI can accumulate a full
    refined trajectory across the sliding window (ids that leave the window keep
    their last-refined position). ``inv(T_cw)`` maps each keyframe pose to its
    camera-in-world position.
    """
    import numpy as np
    out = {}
    for kf in ba_map.keyframes:
        T_cw = kf["T_cw"]
        out[int(kf["id"])] = (np.linalg.inv(T_cw)[:3, 3]).copy()
    return out


def vio_overlay(vio_map):
    """Tight-VIO window snapshot: ``{kf_index: refined camera-world position}``.

    Mirrors :func:`ba_overlay` but the tight map's keyframes are a plain list
    (no monotonic ``id`` field), so the snapshot is keyed by the keyframe's index
    within the current window. ``inv(T_cw)`` maps each keyframe pose to its
    camera-in-world position (camera-optical frame; the UI applies the single
    optical->NED display transform). Read-only over real refined map outputs.
    """
    import numpy as np
    out = {}
    for i, kf in enumerate(vio_map.keyframes):
        T_cw = kf["T_cw"]
        out[int(i)] = (np.linalg.inv(T_cw)[:3, 3]).copy()
    return out
