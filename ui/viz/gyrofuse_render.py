"""Pure renderer for the gyro-fusion strip chart (ALGORITHMS.md #5).

A scrolling, window-free strip chart that explains WHY the gyro-fused VIO stays
straight where pure-vision (``pose.vo``, grey) drifts during fast yaw. It draws
exactly what a :class:`~ui.comms.messages.FrameGyroFuse` carries -- a REAL
odometry output -- and nothing computed in a parallel pipeline (honest viz).

Two stacked lanes, newest sample at the right:

* TOP lane (deg / frame) -- two traces over time:
    - ``vision_rot_deg`` (grey) -- the RAW PnP rotation that drifts under fast yaw,
    - ``gyro_rot_deg`` (cyan)  -- the near-ground-truth gyro rotation,
  with the area BETWEEN them shaded as the per-frame disagreement, plus two
  horizontal reference lines: the gate (``gate_deg``, "gyro starts taking over")
  and gate+span (``gate_deg + span_deg``, "full gyro"). When the grey trace pulls
  away from the cyan one and crosses the gate, that IS the fast-yaw regime the
  fusion rescues.
* BOTTOM lane (0..1) -- the resulting ``gain`` (vision weight: 1 = pure vision,
  0 = pure gyro) and ``t_trust`` (translation trust), so you see the correction
  collapse toward the gyro exactly as the disagreement spikes.

The renderer is window-free (returns an ``RGB`` ``uint8`` image) so the same
drawing code can feed any backend; the in-app Qt window
(:mod:`ui.qt.gyrofuse_window`) blits it to a label. cv2 is only a drawing backend
here -- importing this module is what pulls it, not the base UI.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

# Colours are RGB (the Qt blit uses Format_RGB888), matched to the app theme:
#   vision (drift) = grey, gyro (truth) = HUD cyan, gain = NVG green,
#   t_trust = amber, gate lines = caution amber / master red.
_BG = (13, 17, 23)            # theme.BG  #0d1117
_GRID = (42, 50, 61)          # theme.GRID #2a323d
_TEXT = (230, 237, 243)       # theme.TEXT
_TEXT_DIM = (139, 148, 158)   # theme.TEXT_DIM
_VISION = (150, 160, 170)     # grey -- pure-vision rotation (drifts)
_GYRO = (92, 225, 255)        # cyan -- gyro rotation (near ground-truth)
_DISAGREE = (255, 120, 90)    # shaded area between the two traces (warm)
_GAIN = (124, 255, 92)        # NVG green -- vision-correction gain
_TTRUST = (255, 176, 0)       # amber -- translation trust
_GATE = (255, 176, 0)         # amber -- gate ("gyro starts taking over")
_FULL_GYRO = (255, 59, 48)    # red  -- gate+span ("full gyro")

_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class GyroFuseSample:
    """One frame's gyro-fusion record (mirrors the wire fields the chart needs)."""

    vision_rot_deg: float
    gyro_rot_deg: float
    disagree_deg: float
    gain: float
    t_trust: float
    gate_deg: float
    span_deg: float


class GyroFuseChart:
    """Scrolling two-lane gyro-fusion strip chart fed per-frame samples."""

    #: never let the deg/frame auto-scale collapse below this (keeps a still
    #: camera's ~0 deg/frame from being blown up to full height).
    _DEG_FLOOR = 2.0
    #: expand-fast / shrink-slow hysteresis so the trace does not strobe.
    _SHRINK = 0.06

    def __init__(self, width: int = 720, height: int = 460,
                 capacity: int = 600) -> None:
        self.width = int(width)
        self.height = int(height)
        self._hist: deque[GyroFuseSample] = deque(maxlen=int(capacity))
        self._deg_span = self._DEG_FLOOR

    def clear(self) -> None:
        self._hist.clear()
        self._deg_span = self._DEG_FLOOR

    def add(self, s: GyroFuseSample) -> None:
        self._hist.append(s)

    @property
    def sample_count(self) -> int:
        return len(self._hist)

    # -- rendering ---------------------------------------------------------- #
    def render(self) -> np.ndarray:
        canvas = np.full((self.height, self.width, 3), _BG, dtype=np.uint8)
        # Split the canvas: ~62% top (deg/frame), ~38% bottom (0..1), with a
        # header band on each lane for the title + legend.
        top_y0, top_y1 = 28, int(self.height * 0.60)
        bot_y0, bot_y1 = int(self.height * 0.62) + 20, self.height - 22
        self._draw_deg_lane(canvas, top_y0, top_y1)
        self._draw_unit_lane(canvas, bot_y0, bot_y1)
        return canvas

    # -- top lane: vision vs gyro rotation (deg/frame) ---------------------- #
    def _draw_deg_lane(self, canvas, y0: int, y1: int) -> None:
        h = max(y1 - y0, 1)
        n = len(self._hist)
        # Auto-scale the deg axis to cover the data + the full-gyro line, floored,
        # with expand-fast/shrink-slow hysteresis (stable, never strobes).
        peak = self._DEG_FLOOR
        gate = span = 0.0
        if n:
            arr = self._hist
            peak = max(
                max(s.vision_rot_deg for s in arr),
                max(s.gyro_rot_deg for s in arr),
                max((s.gate_deg + s.span_deg) for s in arr),
            )
            last = arr[-1]
            gate, span = last.gate_deg, last.span_deg
        target = max(peak * 1.15, self._DEG_FLOOR)
        if target >= self._deg_span:
            self._deg_span = target
        else:
            self._deg_span += (target - self._deg_span) * self._SHRINK
        span_deg = max(self._deg_span, 1e-6)

        def y_of(v: float) -> int:
            return int(np.clip(y1 - (v / span_deg) * h, y0, y1))

        # Gridlines at 25 / 50 / 75 % of the span.
        for frac in (0.25, 0.5, 0.75):
            yy = int(y1 - frac * h)
            cv2.line(canvas, (44, yy), (self.width - 8, yy), _GRID, 1)
            cv2.putText(canvas, f"{frac * span_deg:4.1f}", (4, yy + 4),
                        _FONT, 0.34, _TEXT_DIM, 1, cv2.LINE_AA)

        # Gate reference lines (only once we know the thresholds).
        if gate > 0.0:
            yg = y_of(gate)
            cv2.line(canvas, (44, yg), (self.width - 8, yg), _GATE, 1, cv2.LINE_AA)
            cv2.putText(canvas, f"gate {gate:.1f} (gyro takes over)",
                        (48, yg - 4), _FONT, 0.38, _GATE, 1, cv2.LINE_AA)
            yf = y_of(gate + span)
            cv2.line(canvas, (44, yf), (self.width - 8, yf), _FULL_GYRO, 1,
                     cv2.LINE_AA)
            cv2.putText(canvas, f"full gyro {gate + span:.1f}",
                        (48, yf - 4), _FONT, 0.38, _FULL_GYRO, 1, cv2.LINE_AA)

        # Title + legend.
        cv2.putText(canvas, "INTER-FRAME ROTATION  (deg/frame)", (8, 18),
                    _FONT, 0.42, _TEXT, 1, cv2.LINE_AA)
        cv2.putText(canvas, "vision", (self.width - 200, 18), _FONT, 0.40,
                    _VISION, 1, cv2.LINE_AA)
        cv2.putText(canvas, "gyro", (self.width - 130, 18), _FONT, 0.40,
                    _GYRO, 1, cv2.LINE_AA)
        cv2.putText(canvas, "disagree", (self.width - 88, 18), _FONT, 0.40,
                    _DISAGREE, 1, cv2.LINE_AA)

        if n < 2:
            return
        x = np.linspace(44, self.width - 9, n)
        vis = np.array([s.vision_rot_deg for s in self._hist])
        gyr = np.array([s.gyro_rot_deg for s in self._hist])
        vis_y = np.clip(y1 - (vis / span_deg) * h, y0, y1)
        gyr_y = np.clip(y1 - (gyr / span_deg) * h, y0, y1)

        # Shade the disagreement: the band between the two traces, per column.
        band = np.concatenate([
            np.stack([x, vis_y], axis=1),
            np.stack([x[::-1], gyr_y[::-1]], axis=1),
        ]).astype(np.int32)
        overlay = canvas.copy()
        cv2.fillPoly(overlay, [band], _DISAGREE)
        cv2.addWeighted(overlay, 0.22, canvas, 0.78, 0.0, dst=canvas)

        # The two traces on top (gyro slightly thicker -- it is the trusted one).
        cv2.polylines(canvas, [np.stack([x, vis_y], axis=1).astype(np.int32)],
                      False, _VISION, 1, cv2.LINE_AA)
        cv2.polylines(canvas, [np.stack([x, gyr_y], axis=1).astype(np.int32)],
                      False, _GYRO, 2, cv2.LINE_AA)

        last = self._hist[-1]
        cv2.putText(
            canvas,
            f"vis {last.vision_rot_deg:5.2f}  gyro {last.gyro_rot_deg:5.2f}  "
            f"disagree {last.disagree_deg:5.2f}",
            (48, y1 - 6), _FONT, 0.40, _TEXT, 1, cv2.LINE_AA)

    # -- bottom lane: gain + t_trust (0..1) -------------------------------- #
    def _draw_unit_lane(self, canvas, y0: int, y1: int) -> None:
        h = max(y1 - y0, 1)
        # 0 / 0.5 / 1 gridlines.
        for frac in (0.0, 0.5, 1.0):
            yy = int(y1 - frac * h)
            cv2.line(canvas, (44, yy), (self.width - 8, yy), _GRID, 1)
            cv2.putText(canvas, f"{frac:3.1f}", (12, yy + 4), _FONT, 0.34,
                        _TEXT_DIM, 1, cv2.LINE_AA)

        cv2.putText(canvas, "CORRECTION GAIN  &  TRANSLATION TRUST  (0..1)",
                    (8, y0 - 8), _FONT, 0.42, _TEXT, 1, cv2.LINE_AA)
        cv2.putText(canvas, "gain", (self.width - 150, y0 - 8), _FONT, 0.40,
                    _GAIN, 1, cv2.LINE_AA)
        cv2.putText(canvas, "t_trust", (self.width - 92, y0 - 8), _FONT, 0.40,
                    _TTRUST, 1, cv2.LINE_AA)

        n = len(self._hist)
        if n < 2:
            return
        x = np.linspace(44, self.width - 9, n)
        gain = np.clip(np.array([s.gain for s in self._hist]), 0.0, 1.0)
        ttr = np.clip(np.array([s.t_trust for s in self._hist]), 0.0, 1.0)
        gain_y = y1 - gain * h
        ttr_y = y1 - ttr * h
        cv2.polylines(canvas, [np.stack([x, gain_y], axis=1).astype(np.int32)],
                      False, _GAIN, 2, cv2.LINE_AA)
        cv2.polylines(canvas, [np.stack([x, ttr_y], axis=1).astype(np.int32)],
                      False, _TTRUST, 1, cv2.LINE_AA)
        last = self._hist[-1]
        cv2.putText(canvas, f"gain {last.gain:4.2f}   t_trust {last.t_trust:4.2f}",
                    (48, y1 - 6), _FONT, 0.40, _TEXT, 1, cv2.LINE_AA)
