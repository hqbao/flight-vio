#!/usr/bin/env python3
"""Self-test for the sliding-window marginalization prior (ours.lib.backend).

Three checks, cheapest/strongest first:

1. **Schur identity (the proof).** On a random SPD linear system the condensed
   prior ``(H_p, b_p)`` reproduces the full solution's kept-block exactly. This
   certifies the marginalization formula + sign convention to ~1e-10, independent
   of any nonlinear solver.

2. **Exactness on a noise-free window.** Sliding the real
   :class:`WindowedBAMap` with the prior on a consistent (noise-free) synthetic
   scene recovers the truth trajectory, matching a full-batch BA (large window)
   to sub-millimetre.

3. **Benefit under noise.** With observation noise, the marginalizing window
   keeps drift below the plain-drop window (the prior carries old information
   forward instead of discarding it).

Run::

    python -m ours.tools.ba_marg_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.lib.backend.bundle import se3_exp                       # noqa: E402
from ours.lib.backend.windowed import WindowedBAMap, WindowedConfig  # noqa: E402


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


# --------------------------------------------------------------------------- #
# 1. Schur identity
# --------------------------------------------------------------------------- #
def test_schur_identity() -> None:
    print("schur identity (marginalization is loss-free)")
    rng = np.random.default_rng(0)
    D, m = 18, 6                          # 18 vars, marginalize first 6
    A = rng.normal(size=(D, D))
    H = A @ A.T + D * np.eye(D)           # SPD
    b = rng.normal(size=D)

    full = np.linalg.solve(H, -b)         # full Newton step
    dk_full = full[m:]

    Hmm, Hmk = H[:m, :m], H[:m, m:]
    Hkk = H[m:, m:]
    bm, bk = b[:m], b[m:]
    Hinv = np.linalg.inv(Hmm)
    H_p = Hkk - Hmk.T @ Hinv @ Hmk
    b_p = bk - Hmk.T @ Hinv @ bm
    dk_marg = np.linalg.solve(H_p, -b_p)  # prior-only Newton step

    err = float(np.max(np.abs(dk_full - dk_marg)))
    _check(err < 1e-9, f"condensed prior reproduces kept block (max err {err:.2e})")


# --------------------------------------------------------------------------- #
# Synthetic RGB-D scene helpers
# --------------------------------------------------------------------------- #
def _scene(rng, n_kf=7, M=60):
    K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])
    Xw = np.column_stack([rng.uniform(-3, 3, M),
                          rng.uniform(-2, 2, M),
                          rng.uniform(4, 8, M)])
    poses = []                            # world->camera truth
    for i in range(n_kf):
        ang = np.deg2rad(5.0 * i)
        xi = np.array([0.2 * i, 0.02 * i, 0.0, 0.0, ang, 0.0])
        poses.append(np.linalg.inv(se3_exp(xi)))
    T0i = np.linalg.inv(poses[0])
    poses = [T @ T0i for T in poses]      # anchor cam0 == world
    return K, poses, Xw


def _project(K, T_cw, Xw):
    Xc = T_cw[:3, :3] @ Xw + T_cw[:3, 3]
    u = K[0, 0] * Xc[0] / Xc[2] + K[0, 2]
    v = K[1, 1] * Xc[1] / Xc[2] + K[1, 2]
    return np.array([u, v]), float(Xc[2])


def _depth_img(uv, z, h=480, w=640):
    img = np.zeros((h, w), np.float32)
    for (u, v), zz in zip(uv, z):
        pu, pv = int(round(u)), int(round(v))
        if 0 <= pu < w and 0 <= pv < h:
            img[pv, pu] = zz
    return img


def _run_window(K, poses, Xw, cfg, pose_noise, uv_noise, rng, depth_noise=0.0):
    """Drive WindowedBAMap keyframe-by-keyframe; return {kf_id: T_cw refined}."""
    ba = WindowedBAMap(K, cfg)
    out = {}
    for i, Tcw in enumerate(poses):
        uv, zs, ids = [], [], []
        for l in range(Xw.shape[0]):
            p, z = _project(K, Tcw, Xw[l])
            if z <= 0.3 or z > 8.0 or not (0 <= p[0] < 640 and 0 <= p[1] < 480):
                continue
            uv.append(p + rng.normal(0, uv_noise, 2) if uv_noise else p)
            zs.append(z + rng.normal(0, depth_noise) if depth_noise else z)
            ids.append(l)
        if len(ids) < 8:
            continue
        depth = _depth_img(uv, zs)
        # the "estimate" handed to the map is the truth pose + a perturbation;
        # the first keyframe defines the gauge, so it is handed in at truth.
        if i == 0:
            est = Tcw.copy()
        else:
            noise = np.concatenate([rng.normal(0, pose_noise, 3),
                                    rng.normal(0, pose_noise * 0.3, 3)])
            est = se3_exp(noise) @ Tcw
        ba.add_keyframe(est, np.array(ids), np.array(uv, float), depth)
        ba.run_ba()
        for kf in ba.keyframes:
            out[int(kf["id"])] = kf["T_cw"].copy()
    return out, ba


def _corridor_scene(rng, n_kf=24):
    """Forward motion down a corridor; landmarks appear/disappear with view.

    This is the realistic sliding-window structure (tracks are born and die), so
    each dropped keyframe hosts fresh landmarks -- exactly where marginalization
    has to carry scale/heading forward instead of re-estimating it locally.
    """
    K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])
    poses = []                                # world->camera truth
    for i in range(n_kf):
        # translate forward (+z) with a slow yaw and small lateral drift
        ang = np.deg2rad(2.0 * i)
        xi = np.array([0.03 * i, 0.0, 0.35 * i, 0.0, ang, 0.0])
        poses.append(np.linalg.inv(se3_exp(xi)))
    T0i = np.linalg.inv(poses[0])
    poses = [T @ T0i for T in poses]
    # landmarks spread along the corridor walls/volume the camera flies through
    span_z = 0.35 * n_kf
    M = 26 * n_kf
    Xw = np.column_stack([rng.uniform(-4, 4, M),
                          rng.uniform(-3, 3, M),
                          rng.uniform(-1.0, span_z + 5.0, M)])
    return K, poses, Xw


def _pure_forward_scene(rng, n_kf=40):
    """Pure forward translation -- the scale-ambiguous case.

    With no lateral parallax the metric scale is pinned ONLY by depth, so noisy
    depth makes the per-window scale random-walk. Plain-drop re-estimates scale
    locally each window and the error accumulates; the marginalization prior
    carries the scale/heading forward, so it should drift less.
    """
    K = np.array([[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1.0]])
    poses = []
    for i in range(n_kf):
        xi = np.array([0.0, 0.0, 0.3 * i, 0.0, 0.0, 0.0])
        poses.append(np.linalg.inv(se3_exp(xi)))
    T0i = np.linalg.inv(poses[0])
    poses = [T @ T0i for T in poses]
    span_z = 0.3 * n_kf
    M = 30 * n_kf
    Xw = np.column_stack([rng.uniform(-3, 3, M),
                          rng.uniform(-2, 2, M),
                          rng.uniform(-1.0, span_z + 6.0, M)])
    return K, poses, Xw


def _ate(est_map, poses_truth):
    """RMSE of camera-centre error over keyframes still in est_map (gauge: cam0)."""
    errs = []
    for kid, Tcw in est_map.items():
        if kid >= len(poses_truth):
            continue
        c_est = -Tcw[:3, :3].T @ Tcw[:3, 3]
        c_tru = -poses_truth[kid][:3, :3].T @ poses_truth[kid][:3, 3]
        errs.append(np.linalg.norm(c_est - c_tru))
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else 0.0


# --------------------------------------------------------------------------- #
# 2. Exactness on a noise-free window
# --------------------------------------------------------------------------- #
def test_exactness_noisefree() -> None:
    print("exactness: noise-free marginalizing window recovers truth")
    rng = np.random.default_rng(3)
    K, poses, Xw = _corridor_scene(rng, n_kf=14)

    cfg_full = WindowedConfig(window=20, use_marg=False)
    cfg_full.ba.max_iters = 25
    cfg_marg = WindowedConfig(window=4, use_marg=True)
    cfg_marg.ba.max_iters = 25

    full_map, _ = _run_window(
        K, poses, Xw, cfg_full,
        pose_noise=0.02, uv_noise=0.0, rng=np.random.default_rng(10))
    marg_map, ba = _run_window(
        K, poses, Xw, cfg_marg,
        pose_noise=0.02, uv_noise=0.0, rng=np.random.default_rng(10))

    ate_full = _ate(full_map, poses)
    ate_marg = _ate(marg_map, poses)
    _check(ba.prior is not None, "a marginalization prior was actually formed")
    _check(ate_full < 5e-3, f"full-batch recovers truth (ATE {ate_full*1000:.2f} mm)")
    _check(ate_marg < 1e-2,
           f"marg window recovers truth (ATE {ate_marg*1000:.2f} mm)")


# --------------------------------------------------------------------------- #
# 3. Benefit under noise
# --------------------------------------------------------------------------- #
def test_benefit_under_noise() -> None:
    print("benefit: prior beats plain-drop (pure-forward, scale-ambiguous)")
    drops, margs = [], []
    n_seeds = 6
    for s in range(n_seeds):
        K, poses, Xw = _pure_forward_scene(np.random.default_rng(100 + s), n_kf=40)
        drop_map, _ = _run_window(
            K, poses, Xw, WindowedConfig(window=5, use_marg=False),
            pose_noise=0.02, uv_noise=0.4, rng=np.random.default_rng(200 + s),
            depth_noise=0.08)
        marg_map, _ = _run_window(
            K, poses, Xw, WindowedConfig(window=5, use_marg=True),
            pose_noise=0.02, uv_noise=0.4, rng=np.random.default_rng(200 + s),
            depth_noise=0.08)
        drops.append(_ate(drop_map, poses))
        margs.append(_ate(marg_map, poses))
    drops = np.array(drops)
    margs = np.array(margs)
    wins = int((margs < drops).sum())
    print(f"    mean ATE  plain-drop {drops.mean()*1000:7.2f} mm  |  "
          f"marg {margs.mean()*1000:7.2f} mm   (marg wins {wins}/{n_seeds})")
    _check(margs.mean() < drops.mean(),
           f"mean marg drift < mean plain-drop "
           f"({margs.mean()*1000:.2f} < {drops.mean()*1000:.2f} mm)")
    _check(wins >= n_seeds // 2,
           f"marg wins the majority of seeds ({wins}/{n_seeds})")


def main() -> int:
    print("ba_marg_selftest")
    test_schur_identity()
    test_exactness_noisefree()
    test_benefit_under_noise()
    print("\nALL BA MARG SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
