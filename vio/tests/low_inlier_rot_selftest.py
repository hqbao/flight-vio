#!/usr/bin/env python3
"""Self-test: no-gyro low-inlier freeze advances rotation by VISION (not frozen).

Pins the flight-critical fix for the "VIO sends a bit-exact IDENTITY orientation"
regression. ROOT CAUSE: with no IMU->camera extrinsic, ``R_prior`` is ``None``;
the odometry low-inlier-freeze path then advanced the pose rotation ONLY in the
``gyro_fuse and R_prior is not None`` branch, so the no-gyro path returned the
pose with the rotation UNTOUCHED -- at low resolution (few inliers) that freeze
fires constantly, so the rotation never left the identity init and the VIO
reported a constant (identity) attitude to the flight controller.

The fix adds the no-gyro ``else``: advance the rotation by the VISION rotation
``R`` (cur<-prev PnP point-rotation) while still freezing translation -- the
honest blank-wall behaviour (rotate, don't translate). The composition mirrors
the main-path pose update (``self.pose @ inv(T_pc)`` with ``T_pc[:3,:3]=R``)
exactly, with the translation zeroed.

Hard pass/fail:

1. NO-GYRO FREEZE TRACKS ROTATION (Fix A). Drive ``estimate`` with
   ``R_prior=None`` on a frame pair that lands in the low-inlier-freeze band (PnP
   solves, ninl >= min_pnp_points but < min_inliers_for_translation, IMU not
   moving). Assert the pose rotation block CHANGES (so3 angle > epsilon, ~ the
   planted yaw) -- it does NOT stay at the identity init -- AND the translation
   stays frozen (==0). The ``info["reason"]`` is the observable
   ``low_inliers_rot_vision``.

2. ROTATION DIRECTION IS CORRECT (no transpose). The vision rotation ``R`` is
   cur<-prev; the synthetic gyro prior is ``R_pc.T`` (prev<-cur), so a WITH-gyro
   run of the SAME freeze band advances rotation by ``R_prior.T == R_pc``. The
   no-gyro pose rotation must MATCH that with-gyro pose rotation (both recover
   the planted yaw) -- a transpose error in the fix would flip the world-rotation
   axis sign and fail this.

3. WITH-GYRO PATH BYTE-UNCHANGED (gap=0, unit form). The fix lives strictly in
   the ``R_prior is None`` / gyro-off branch. A WITH-gyro freeze-band run must be
   byte-identical to the legacy composition (``self.pose @ inv(T_pc)`` with
   ``T_pc[:3,:3]=R_prior.T``), and keep the legacy ``reason=low_inliers_frozen``
   -- proving the offline/oracle path (which always replays IMU -> has R_prior)
   is untouched. The full 640-pose oracle gate is
   ``verification/oracle_replay_selftest.py``.

Run:  .venv/bin/python -m vio.tests.low_inlier_rot_selftest
Exit code 0 on success, 1 on any failure.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.math import so3_exp_unit as so3_exp, so3_log           # noqa: E402
from sky.front.odometry import (                                # noqa: E402
    OdometryConfig, RGBDVisualOdometry)


# Freeze-band gate: well above the PnP minimum (8) so the synthetic ~12-inlier
# frame clears min_pnp_points yet sits below this -> the low-inlier freeze fires.
_FREEZE_GATE = 100


def _build_pair(n: int = 14, yaw: float = 0.10):
    """A planar grid seen from two poses with a KNOWN yaw; few-but-clean points.

    Returns ``(K, prev_obs, cur_obs, prev_depth, R_prior, yaw_deg)``. The relative
    motion cur<-prev rotation is a pure yaw; the gyro prior is supplied in the
    prev<-cur convention the odometry expects (so ``R_prior.T`` == the cur<-prev
    rotation vision should also recover). The small point count keeps the PnP
    inlier count in the low-inlier-freeze band (>= min_pnp_points, < _FREEZE_GATE)
    while every correspondence is exact, so PnP still SOLVES (this is the
    blank-wall-with-a-few-corners regime, not a PnP failure).
    """
    rng = np.random.default_rng(7)
    K = np.array([[600.0, 0.0, 320.0],
                  [0.0, 600.0, 200.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    h, w = 400, 640

    X = rng.uniform(-1.0, 1.0, n)
    Y = rng.uniform(-0.6, 0.6, n)
    Z = rng.uniform(1.0, 3.0, n)
    pts3d_prev = np.stack([X, Y, Z], axis=1)

    pu = fx * X / Z + cx
    pv = fy * Y / Z + cy
    inside = (pu > 5) & (pu < w - 5) & (pv > 5) & (pv < h - 5)

    rotvec = np.array([0.0, yaw, 0.0])     # pure yaw, cur<-prev
    R_pc = so3_exp(rotvec)
    yaw_deg = float(np.degrees(np.linalg.norm(rotvec)))
    t_pc = np.array([0.04, -0.02, 0.08])
    pts3d_cur = pts3d_prev @ R_pc.T + t_pc
    cu = fx * pts3d_cur[:, 0] / pts3d_cur[:, 2] + cx
    cv = fy * pts3d_cur[:, 1] / pts3d_cur[:, 2] + cy

    prev_depth = np.zeros((h, w), dtype=np.float32)
    prev_obs: dict[int, np.ndarray] = {}
    cur_obs: dict[int, np.ndarray] = {}
    for i in range(n):
        if not inside[i]:
            continue
        prev_obs[i] = np.array([pu[i], pv[i]], dtype=np.float64)
        cur_obs[i] = np.array([cu[i], cv[i]], dtype=np.float64)
        prev_depth[int(round(pv[i])), int(round(pu[i]))] = np.float32(Z[i])

    # Gyro prior is prev<-cur; the cur<-prev rotation is R_pc, so the prior is
    # R_pc.T (the odometry transposes it back to cur<-prev internally).
    R_prior = R_pc.T
    return K, prev_obs, cur_obs, prev_depth, R_prior, yaw_deg


def _make_odom(K):
    """Odometry with the low-inlier freeze ARMED (live regime), no gyro fusion."""
    cfg = OdometryConfig(min_pnp_points=8, ransac_reproj_px=2.0,
                         min_inliers_for_translation=_FREEZE_GATE)
    return RGBDVisualOdometry(K, cfg=cfg)


def _make_odom_gyro(K):
    """Same freeze gate, gyro fusion ON (the with-gyro freeze branch)."""
    cfg = OdometryConfig(min_pnp_points=8, ransac_reproj_px=2.0,
                         min_inliers_for_translation=_FREEZE_GATE, gyro_fuse=True)
    return RGBDVisualOdometry(K, cfg=cfg)


def test_no_gyro_freeze_tracks_rotation() -> bool:
    print("== 1. no-gyro low-inlier freeze advances rotation by vision ==")
    K, prev_obs, cur_obs, prev_depth, _R_prior, yaw_deg = _build_pair()

    odo = _make_odom(K)
    odo._prev_obs = prev_obs
    odo._prev_depth = prev_depth
    # R_prior=None == the no-IMU-extrinsic regression regime.
    pose = odo.estimate(cur_obs, np.zeros_like(prev_depth), R_prior=None)
    info = odo.last_info

    ok = True

    # The frame must actually land in the freeze band (else the test proves
    # nothing). PnP solved with >= min_pnp_points but < the freeze gate.
    ninl = int(info.get("n_inliers", 0))
    band = (8 <= ninl < _FREEZE_GATE)
    print(f"  n_inliers={ninl} in freeze band [8,{_FREEZE_GATE}) -> "
          f"{'PASS' if band else 'FAIL'}")
    ok &= band

    reason_ok = info.get("reason") == "low_inliers_rot_vision"
    print(f"  reason={info.get('reason')!r} == 'low_inliers_rot_vision' -> "
          f"{'PASS' if reason_ok else 'FAIL'}")
    ok &= reason_ok

    # THE regression assertion: the rotation block is NO LONGER the identity init.
    rot_deg = float(np.degrees(np.linalg.norm(so3_log(pose[:3, :3]))))
    moved = rot_deg > 1.0                 # epsilon: the seed is exactly 0 deg
    print(f"  pose rot angle={rot_deg:.4f} deg > 1.0 (NOT frozen at identity) -> "
          f"{'PASS' if moved else 'FAIL'}")
    ok &= moved

    # ... and it tracks the actual vision rotation (~ the planted yaw).
    tracks = abs(rot_deg - yaw_deg) < 0.5
    print(f"  pose rot {rot_deg:.4f} ~ planted yaw {yaw_deg:.4f} -> "
          f"{'PASS' if tracks else 'FAIL'}")
    ok &= tracks

    # Translation stays FROZEN on the blank wall (rotate, don't translate).
    t_norm = float(np.linalg.norm(pose[:3, 3]))
    frozen = t_norm == 0.0
    print(f"  translation norm={t_norm:.6f} == 0 (frozen) -> "
          f"{'PASS' if frozen else 'FAIL'}")
    ok &= frozen
    return ok


def test_direction_matches_gyro() -> bool:
    print("== 2. no-gyro vision rotation direction == with-gyro (no transpose) ==")
    K, prev_obs, cur_obs, prev_depth, R_prior, _yaw = _build_pair()

    odo_n = _make_odom(K)
    odo_n._prev_obs = dict(prev_obs)
    odo_n._prev_depth = prev_depth.copy()
    pose_n = odo_n.estimate(dict(cur_obs), np.zeros_like(prev_depth), R_prior=None)

    odo_g = _make_odom_gyro(K)
    odo_g._prev_obs = dict(prev_obs)
    odo_g._prev_depth = prev_depth.copy()
    pose_g = odo_g.estimate(dict(cur_obs), np.zeros_like(prev_depth),
                            R_prior=R_prior.copy())

    # Vision R (cur<-prev) should recover the same rotation the gyro prior
    # encodes (R_prior.T), so the two world-rotation blocks must agree closely.
    # A transpose error in the fix would invert the axis -> this fails loudly.
    dR = so3_log(pose_n[:3, :3] @ pose_g[:3, :3].T)
    diff_deg = float(np.degrees(np.linalg.norm(dR)))
    same = diff_deg < 0.5
    print(f"  no-gyro vs with-gyro world rotation diff={diff_deg:.4f} deg < 0.5 "
          f"-> {'PASS' if same else 'FAIL'}")
    return same


def test_with_gyro_byte_unchanged() -> bool:
    print("== 3. with-gyro freeze path byte-unchanged (gap=0, unit form) ==")
    K, prev_obs, cur_obs, prev_depth, R_prior, _yaw = _build_pair()

    odo = _make_odom_gyro(K)
    odo._prev_obs = dict(prev_obs)
    odo._prev_depth = prev_depth.copy()
    pose = odo.estimate(dict(cur_obs), np.zeros_like(prev_depth),
                        R_prior=R_prior.copy())
    info = odo.last_info

    ok = True

    # The with-gyro freeze keeps its LEGACY reason (untouched by the fix).
    reason_ok = info.get("reason") == "low_inliers_frozen"
    print(f"  reason={info.get('reason')!r} == 'low_inliers_frozen' (legacy) -> "
          f"{'PASS' if reason_ok else 'FAIL'}")
    ok &= reason_ok

    # Byte-identical to the legacy composition: gyro owns rotation (R_prior.T),
    # translation frozen. This is the exact code the fix must NOT have perturbed.
    T_pc = np.eye(4)
    T_pc[:3, :3] = np.asarray(R_prior, dtype=np.float64).T
    expected = np.eye(4) @ np.linalg.inv(T_pc)
    identical = np.array_equal(pose, expected)
    print(f"  pose byte-identical to legacy gyro-freeze composition -> "
          f"{'PASS' if identical else 'FAIL'}")
    ok &= identical
    return ok


def main() -> int:
    r1 = test_no_gyro_freeze_tracks_rotation()
    r2 = test_direction_matches_gyro()
    r3 = test_with_gyro_byte_unchanged()
    ok = r1 and r2 and r3
    print()
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
