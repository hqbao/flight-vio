"""Honest depth colourisation shared by the dev tools and the Qt UI.

The metric depth from our SGM is mapped to a TURBO colormap over a **fixed**
range (:data:`D_MIN`..:data:`D_MAX` metres) so colours mean the same distance in
every frame -- a per-frame autoscale would make the scene "breathe" and hide
real depth changes. Invalid pixels (no stereo return, encoded as ``0``) stay
pure black. cv2 is only the colormap backend here; importing this module is what
pulls it, not the base UI.
"""
from __future__ import annotations

import cv2
import numpy as np

# Fixed depth range (metres) for the colormap so colours are stable across
# frames. Matches the odometry/loop-closure usable stereo band.
D_MIN = 0.3
D_MAX = 8.0


def _depth_norm_u8(depth_values: np.ndarray) -> np.ndarray:
    """Map metric depth -> the same 0..255 TURBO index used by the colormap.

    ``D_MIN..D_MAX`` clamp, then ``near = hot`` (index 255 at ``D_MIN``). Shared
    by the dense image, the scale-bar and the per-keypoint dot so a given
    distance is always the identical colour.
    """
    z = np.clip(np.asarray(depth_values, dtype=np.float64), D_MIN, D_MAX)
    t = 1.0 - (z - D_MIN) / (D_MAX - D_MIN)
    return (t * 255.0).astype(np.uint8)


_TURBO_LUT: np.ndarray | None = None


def _turbo_lut() -> np.ndarray:
    """``(256, 3)`` BGR TURBO lookup table (built once, then cached)."""
    global _TURBO_LUT
    if _TURBO_LUT is None:
        ramp = np.arange(256, dtype=np.uint8).reshape(256, 1)
        _TURBO_LUT = cv2.applyColorMap(ramp, cv2.COLORMAP_TURBO).reshape(256, 3)
    return _TURBO_LUT


def turbo_bgr_array(depth_values: np.ndarray) -> np.ndarray:
    """Per-value depth -> ``(M, 3)`` uint8 BGR, identical mapping to the image.

    Invalid depths (``<= 0``) clamp to ``D_MIN`` here (the caller masks them out
    and draws a neutral marker instead -- this never invents a colour for them).
    """
    return _turbo_lut()[_depth_norm_u8(depth_values)]


def turbo_bgr(z: float) -> tuple[int, int, int]:
    """Single metric depth -> BGR tuple (same mapping as :func:`colorize_depth`)."""
    b, g, r = _turbo_lut()[int(_depth_norm_u8(np.array([float(z)]))[0])]
    return int(b), int(g), int(r)


def colorize_depth(depth_m: np.ndarray) -> np.ndarray:
    """Metric depth (m, ``0`` == invalid) -> BGR turbo image (near = red/hot)."""
    valid = depth_m > 1e-6
    norm = np.zeros(depth_m.shape, dtype=np.uint8)
    if valid.any():
        norm[valid] = _depth_norm_u8(depth_m)[valid]
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    colored[~valid] = 0
    return colored


def depth_scale_bar(height: int, width: int = 16) -> np.ndarray:
    """Vertical TURBO gradient legend (BGR): top = :data:`D_MIN` (near/hot),
    bottom = :data:`D_MAX` (far/cold). The range is fixed, so this is rendered
    once and never changes -- an honest key to the colormap, no per-frame cost.
    """
    rows = np.linspace(0.0, 1.0, max(height, 1), dtype=np.float32)   # 0=near
    norm = ((1.0 - rows) * 255.0).astype(np.uint8)[:, None]
    norm = np.repeat(norm, max(width, 1), axis=1)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
