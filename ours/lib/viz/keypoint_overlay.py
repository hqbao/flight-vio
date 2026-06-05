"""Keypoint depth-track overlay.

Draws OUR KLT frontend's REAL tracks on a frame, coloured by the per-keypoint
metric depth, with a per-id trail over the last ``TRAIL_LEN`` frames.

Honest-data: the keypoints + persistent ids come straight from
:class:`~ours.lib.frontend.frontend.KLTFrontend` -- the SAME frontend the
odometry runs, not a parallel detector. The trail is the UI buffering each id's
recent positions across frames (a derived-from-real signal, clearly that and
nothing invented). Keypoints whose depth has no stereo return (``z == 0``) are
drawn as hollow grey rings -- they are NEVER given a fake depth colour.

The drawing backend is cv2 (already pulled in by :mod:`depth_render`). Everything
here is pure array logic + cv2, so it is fully unit-testable off-device.
"""
from __future__ import annotations

from collections import deque

import numpy as np

from .depth_render import turbo_bgr_array

#: How many past positions of one track to keep / draw (the user's N).
TRAIL_LEN = 20

# Neutral markers/colours pulled from theme.py (BGR, since we draw on a BGR
# frame): TEXT_DIM #8b949e for invalid-depth points, WARN #ffb000 for fresh
# tracks, pure black for the legibility halo under every dot.
_INVALID_BGR = (158, 148, 139)   # #8b949e
_FRESH_BGR = (0, 176, 255)       # #ffb000
_HALO_BGR = (0, 0, 0)


def sample_depths(depth_m: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Depth (m) under each keypoint pixel; ``0`` where invalid/out-of-bounds."""
    h, w = depth_m.shape
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] == 0:
        return np.empty((0,), dtype=np.float64)
    u = np.clip(np.round(pts[:, 0]).astype(np.int64), 0, w - 1)
    v = np.clip(np.round(pts[:, 1]).astype(np.int64), 0, h - 1)
    return np.asarray(depth_m, dtype=np.float64)[v, u]


class TrackTrails:
    """Per-id bookkeeping: recent pixel positions + first/last-seen frame.

    Lives UI-side (the frontend stays untouched). ``update`` is called once per
    produced frame, IN ORDER, so the trails are continuous even when the UI drops
    frames to stay realtime (the producer owns this object, the UI only reads the
    finished overlay).
    """

    def __init__(self, trail_len: int = TRAIL_LEN, max_stale: int = 5) -> None:
        self._len = int(trail_len)
        self._max_stale = int(max_stale)
        self._pos: dict[int, deque] = {}
        self._first: dict[int, int] = {}
        self._last: dict[int, int] = {}
        self._frame = -1
        self._new = 0
        self._cur_ids: np.ndarray = np.empty((0,), np.int64)

    def update(self, ids: np.ndarray, points: np.ndarray) -> int:
        """Append this frame's observations; evict stale tracks. Returns frame #."""
        self._frame += 1
        f = self._frame
        ids = np.asarray(ids, dtype=np.int64).reshape(-1)
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        new_count = 0
        for tid, p in zip(ids, pts):
            tid = int(tid)
            dq = self._pos.get(tid)
            if dq is None:
                dq = deque(maxlen=self._len)
                self._pos[tid] = dq
                self._first[tid] = f
                new_count += 1
            dq.append((float(p[0]), float(p[1])))
            self._last[tid] = f
        for tid in [t for t, last in self._last.items()
                    if f - last > self._max_stale]:
            self._pos.pop(tid, None)
            self._first.pop(tid, None)
            self._last.pop(tid, None)
        self._new = new_count
        self._cur_ids = ids
        return f

    def trail(self, tid: int) -> list[tuple[float, float]]:
        return list(self._pos.get(int(tid), ()))

    def age(self, tid: int) -> int:
        """Frames since the track was first seen (0 == brand new)."""
        return self._frame - self._first.get(int(tid), self._frame)

    @property
    def new_count(self) -> int:
        return self._new

    def mean_age(self) -> float:
        if self._cur_ids.size == 0:
            return 0.0
        return float(np.mean([self.age(int(t)) for t in self._cur_ids]))


def draw_overlay(gray: np.ndarray, depth_m: np.ndarray,
                 ids: np.ndarray, points: np.ndarray,
                 trails: TrackTrails, *, fresh_age: int = 3,
                 draw_trails: bool = True) -> np.ndarray:
    """Render the keypoint/depth/trail overlay; returns an ``(H, W, 3)`` RGB image.

    Background is the grayscale frame, dimmed so the colour dots pop. Each live
    track: a black halo + a depth-coloured dot (valid depth) or a hollow grey
    ring (no stereo return), an amber ring if it is a fresh track, and -- when
    ``draw_trails`` -- a faint depth-coloured polyline of its last positions.
    """
    import cv2

    bg = cv2.cvtColor(np.ascontiguousarray(gray, dtype=np.uint8),
                      cv2.COLOR_GRAY2BGR)
    bg = (bg.astype(np.float32) * 0.6).astype(np.uint8)

    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    ids = np.asarray(ids, dtype=np.int64).reshape(-1)
    if pts.shape[0] == 0:
        return np.ascontiguousarray(bg[:, :, ::-1])

    z = sample_depths(depth_m, pts)
    valid = z > 1e-6
    colors = turbo_bgr_array(z)                       # (M, 3) uint8 BGR

    if draw_trails:
        layer = bg.copy()
        for tid, c, vv in zip(ids, colors, valid):
            tr = trails.trail(int(tid))
            if len(tr) < 2:
                continue
            col = (int(c[0]), int(c[1]), int(c[2])) if vv else _INVALID_BGR
            poly = np.asarray(tr, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(layer, [poly], False, col, 1, cv2.LINE_AA)
        bg = cv2.addWeighted(layer, 0.4, bg, 0.6, 0.0)

    for (x, y), c, vv, tid in zip(pts, colors, valid, ids):
        ix, iy = int(round(x)), int(round(y))
        cv2.circle(bg, (ix, iy), 4, _HALO_BGR, -1, cv2.LINE_AA)
        if vv:
            cv2.circle(bg, (ix, iy), 3, (int(c[0]), int(c[1]), int(c[2])),
                       -1, cv2.LINE_AA)
        else:
            cv2.circle(bg, (ix, iy), 3, _INVALID_BGR, 1, cv2.LINE_AA)
        if trails.age(int(tid)) < fresh_age:
            cv2.circle(bg, (ix, iy), 5, _FRESH_BGR, 1, cv2.LINE_AA)

    return np.ascontiguousarray(bg[:, :, ::-1])
