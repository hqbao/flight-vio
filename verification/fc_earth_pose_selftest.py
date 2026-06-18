#!/usr/bin/env python3
"""Correctness selftest for the shared earth-frame conversion SSOT.

``sky/fc/fc_earth_pose.earth_pose_from_T_world_cam`` is the ONE optical-world
``T_world_cam`` -> NED + FRD-attitude conversion shared by the UI viewer and the
``fc`` UART sender. It is flight-safety math, so it is anchored here against:

  1. KNOWN AXES -- a camera looking forward / right / down (in the optical world)
     maps to North / East / Down in NED (position direction).
  2. NOMINAL ATTITUDE -- a camera at the world origin with ``R_opt = I`` reports an
     FRD body that is axis-aligned with NED (``R_ned = I``), i.e. the nominal
     forward-facing mount is level + pointing North.
  3. R_body_cam != I -- a tilted mount rotates the reported FRD attitude by exactly
     the mount, and ``R_body_cam = I`` reduces to the verified UI form BIT-FOR-BIT.
  4. NEAR-PITCH-90 -- a pose at ~89.9-degree pitch keeps the QUATERNION exact
     (round-trips through ``quat_to_rot`` to the same R) where naive Euler would
     gimbal-lock, and the quaternion stays unit-norm.

  .venv/bin/python -m verification.fc_earth_pose_selftest
"""
from __future__ import annotations

import sys

import numpy as np

from sky.fc.fc_earth_pose import (
    _M_OPT_TO_NED, _P_OPT_TO_FRD, earth_pose_from_T_world_cam,
)
from sky.math import quat_to_rot, rot_to_quat


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


def _T(R=None, p=(0.0, 0.0, 0.0)) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def _Ry(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def main() -> int:
    print("[1] known position axes: optical world -> NED")
    # Optical world axes: X=right, Y=down, Z=forward. The pipeline's gravity-aligned
    # convention maps forward->North, right->East, down->Down.
    fwd, _, _ = earth_pose_from_T_world_cam(_T(p=(0.0, 0.0, 1.0)))   # +Z forward
    right, _, _ = earth_pose_from_T_world_cam(_T(p=(1.0, 0.0, 0.0)))  # +X right
    down, _, _ = earth_pose_from_T_world_cam(_T(p=(0.0, 1.0, 0.0)))   # +Y down
    _check(np.allclose(fwd, [1, 0, 0]), "forward (+Zopt) -> North (+N)")
    _check(np.allclose(right, [0, 1, 0]), "right (+Xopt) -> East (+E)")
    _check(np.allclose(down, [0, 0, 1]), "down (+Yopt) -> Down (+D)")

    print("[2] nominal attitude: R_opt=I -> FRD body axis-aligned with NED")
    _, q0, R0 = earth_pose_from_T_world_cam(_T(R=np.eye(3)))
    _check(np.allclose(R0, np.eye(3)),
           "R_opt=I, mount=I -> R_ned=I (level, pointing North)")
    _check(np.allclose(q0, [1, 0, 0, 0]), "identity attitude quaternion (1,0,0,0)")

    print("[3] R_body_cam != I rotates the FRD attitude; R_body_cam=I reduces "
          "to the UI form")
    rng = np.random.default_rng(7)
    max_red = 0.0
    for _ in range(1000):
        A = rng.standard_normal((3, 3))
        Q, _ = np.linalg.qr(A)
        if np.linalg.det(Q) < 0:
            Q[:, 0] = -Q[:, 0]
        T = _T(R=Q, p=rng.standard_normal(3) * 4.0)
        # R_body_cam = I path vs the OLD UI inline math (M @ R_opt @ P).
        pos_n, q_n, R_n = earth_pose_from_T_world_cam(T)               # default I
        pos_old = _M_OPT_TO_NED @ T[:3, 3]
        R_old = _M_OPT_TO_NED @ T[:3, :3] @ _P_OPT_TO_FRD
        q_old = rot_to_quat(R_old)
        max_red = max(max_red, float(np.max(np.abs(pos_n - pos_old))),
                      float(np.max(np.abs(q_n - q_old))),
                      float(np.max(np.abs(R_n - R_old))))
    _check(max_red == 0.0,
           f"R_body_cam=I is BYTE-IDENTICAL to M@R_opt@P (max|d|={max_red:.1e})")

    # A 10-degree pitch-down mount tilt (about FRD-Y) with R_opt=I: the reported
    # FRD attitude must equal Ry(10).T exactly (R_ned = I @ P @ Ry.T re-expressed),
    # i.e. a pure pitch rotation appears in the body attitude.
    tilt = _Ry(np.deg2rad(10.0))
    _, _, R_tilt = earth_pose_from_T_world_cam(_T(R=np.eye(3)), R_body_cam=tilt)
    _check(np.allclose(R_tilt, tilt.T),
           "10deg mount tilt -> R_ned == R_body_cam.T (attitude rotated by mount)")
    _check(abs(np.linalg.det(R_tilt) - 1.0) < 1e-12 and
           np.allclose(R_tilt @ R_tilt.T, np.eye(3)),
           "tilted R_ned is a proper rotation (orthonormal, det +1)")

    print("[4] near-pitch-90: the quaternion stays exact (no gimbal lock)")
    # Build an optical-world R_opt that yields an FRD body at ~89.9-degree pitch.
    # Work backwards: pick the desired R_ned (pitch about FRD-Y), then R_opt is
    # M^-1 @ R_ned @ P^-1 so earth_pose reproduces it.
    for deg in (89.9, 90.0, -89.95):
        R_ned_want = _Ry(np.deg2rad(deg))
        R_opt = _M_OPT_TO_NED.T @ R_ned_want @ np.linalg.inv(_P_OPT_TO_FRD)
        _, q, R_got = earth_pose_from_T_world_cam(_T(R=R_opt))
        _check(np.allclose(R_got, R_ned_want, atol=1e-9),
               f"pitch {deg:+.2f}deg: R_ned recovered (max|d|="
               f"{np.max(np.abs(R_got - R_ned_want)):.1e})")
        _check(abs(np.linalg.norm(q) - 1.0) < 1e-12,
               f"pitch {deg:+.2f}deg: quaternion is unit-norm")
        # The quaternion -> R -> quaternion identity holds exactly even at the
        # singularity (the proof the quaternion path is gimbal-lock-free).
        _check(np.allclose(quat_to_rot(q), R_got, atol=1e-12),
               f"pitch {deg:+.2f}deg: quat_to_rot(q) == R_ned (quaternion exact)")

    print("\nPASS -- fc_earth_pose SSOT: known axes + nominal attitude + mount "
          "extrinsic + near-pitch-90 quaternion all correct.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
