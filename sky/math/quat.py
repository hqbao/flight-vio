"""Quaternion <-> rotation / Euler helpers (the ``sky.math`` quaternion kernel).

These are the SAME conversions the wire-contract copies in ``*/comms/lib/misc/
frames.py`` carry, re-homed here so the leaf ``sky.*`` library has a canonical,
numpy-only quaternion kernel (the comms copies stay vendored per project as the
byte-identical wire contract -- see :mod:`sky.math` -- but ``sky`` must never
import them). Every function below reproduces the comms ``frames`` behaviour
BIT-FOR-BIT so callers can switch to this kernel without changing numerics.

Convention: quaternions are ``(w, x, y, z)`` (scalar-first), Euler is ZYX
(yaw-pitch-roll) in radians. ``rot_to_quat`` uses the numerically stable branch
that picks the largest diagonal term, so the divisor never collapses near a
180-degree rotation; ``quat_to_rpy`` clamps the pitch ``arcsin`` argument so it
stays exact (no NaN) through the +/-90-degree pitch singularity.
"""
from __future__ import annotations

import numpy as np


def quat_to_rot(q_wxyz: np.ndarray) -> np.ndarray:
    """Convert a unit quaternion (w, x, y, z) to a 3x3 rotation matrix."""
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def quat_to_rpy(q_wxyz: np.ndarray) -> tuple[float, float, float]:
    """Quaternion -> (roll, pitch, yaw) in radians, ZYX convention.

    Singularity-safe: the pitch ``arcsin`` argument is clamped to ``[-1, 1]`` so
    a quaternion at exactly +/-90-degree pitch yields ``+/-pi/2`` instead of NaN.
    """
    w, x, y, z = q_wxyz
    # roll (X)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # pitch (Y)
    sinp = 2 * (w * y - z * x)
    sinp = float(np.clip(sinp, -1.0, 1.0))
    pitch = np.arcsin(sinp)
    # yaw (Z)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return float(roll), float(pitch), float(yaw)


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion ``(w, x, y, z)``.

    Inverse of :func:`quat_to_rot`. Uses the numerically stable branch that
    picks the largest diagonal term so the divisor never collapses near a
    180-degree rotation.
    """
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)
