#!/usr/bin/env python3
"""Dead-branch gap=0 proof + positive assembly check for the ICP factor.

Proves the OPT-IN ICP factor adds LITERALLY NOTHING on the default path, and
that when enabled it scatters ONLY into the pose blocks (no velocity/bias/
landmark coupling):

  1. GAP=0 (the non-negotiable one, at the unit level): with ``icp_factor`` False
     -- OR with ``icp_factor`` True but an empty ``icp_factors`` list -- the
     build_system (H, b) and total_cost are BIT-FOR-BIT identical to the baseline
     VioConfig with the new fields untouched. (The byte-parity oracle replay is
     the end-to-end version of this; this is the fast unit gate.)

  2. POSE-ONLY assembly: with one ICP factor enabled, the H delta vs flags-off is
     non-zero ONLY on the two keyframes' pose blocks -- the velocity, bias and
     landmark blocks are untouched (so no landmark-Schur coupling, the factor
     composes additively).

  3. COST term: the total_cost delta equals the factor-level (Huber) cost of the
     single ICP residual.

Run::

    .venv/bin/python vio/tests/icp_factor_gap0_selftest.py
"""
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.math import se3_from_Rp, se3_inv, so3_exp_unit  # noqa: E402
from sky.vio.imu import preintegrate_imu  # noqa: E402
from sky.vio.window import (  # noqa: E402
    IcpFactor, VioConfig, VioState, _icp_omega, _icp_residual, optimize_vio,
)


def _make_problem(seed: int = 0):
    rng = np.random.default_rng(seed)
    K = np.array([[120.0, 0, 27.0], [0, 120.0, 21.0], [0, 0, 1.0]])
    nC = 2
    st = VioState(
        R=[np.eye(3), so3_exp_unit(np.array([0.0, 0.02, 0.0]))],
        p=[np.zeros(3), np.array([0.10, 0.0, 0.30])],
        v=[np.array([0.5, 0.0, 1.0]), np.array([0.6, -0.1, 1.2])],
        bg=[np.zeros(3), np.zeros(3)],
        ba=[np.zeros(3), np.zeros(3)],
        landmarks=rng.uniform(-1, 1, size=(8, 3)) + np.array([0, 0, 4.0]),
    )
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    obs_cam, obs_lm, obs_uv, obs_depth = [], [], [], []
    for ci in range(nC):
        Rc, pc = st.R[ci], st.p[ci]
        for m in range(st.landmarks.shape[0]):
            Xc = Rc.T @ (st.landmarks[m] - pc)
            if Xc[2] < 0.2:
                continue
            obs_cam.append(ci); obs_lm.append(m)
            obs_uv.append([fx * Xc[0] / Xc[2] + cx, fy * Xc[1] / Xc[2] + cy])
            obs_depth.append(Xc[2])
    ts = np.linspace(0, 1e8, 6).astype(np.int64)
    gyro = np.tile(np.array([0.001, 0.0, 0.0]), (6, 1))
    accel = np.tile(np.array([0.0, 9.81, 0.0]), (6, 1))
    pre = preintegrate_imu(ts, gyro, accel, np.zeros(3), np.zeros(3))
    g_world = np.array([0.0, 9.81, 0.0])
    return (K, st, np.array(obs_cam), np.array(obs_lm),
            np.array(obs_uv, float), np.array(obs_depth, float),
            [(0, 1, pre)], g_world)


def _system(args, cfg, icp_factors=None):
    """Capture (H, b, cost0) at the input state (init_lambda=0 -> A == H).

    Uses the optimiser's ``_solve_probe`` hook, which receives the FULL damped
    system (A, b) on every inner solve regardless of whether the Schur-complement
    fast path is active -- so the captured (A, b) is the full ndim assembled
    system this test asserts on, not the Schur-reduced nav-only matrix.
    """
    captured = {}

    def probe(A, b, _delta):
        if "b" not in captured:
            captured["b"] = np.asarray(b).copy()
            captured["A"] = np.asarray(A).copy()

    cfg0 = replace(cfg, max_iters=1, init_lambda=0.0)
    res = optimize_vio(*args[:6], args[6], args[7], cfg=cfg0, anchor=0,
                       icp_factors=icp_factors, _solve_probe=probe)
    return captured["A"], captured["b"], res.cost0


def _one_factor(st):
    """A single ICP factor KF0<-KF1 with a small offset from the state rel pose."""
    T0 = se3_from_Rp(st.R[0], st.p[0])
    T1 = se3_from_Rp(st.R[1], st.p[1])
    T_rel = se3_inv(T0) @ T1
    off = se3_from_Rp(so3_exp_unit(np.array([0.0, 0.0, 0.01])),
                      np.array([0.02, 0.0, 0.0]))
    A = np.random.default_rng(1).normal(0, 1, (6, 6))
    Lam = A @ A.T + 5.0 * np.eye(6)
    cfg = VioConfig(sigma_rot_icp=0.2, icp_lambda_thresh=0.02,
                    icp_lambda_floor=1.0)
    Omega = _icp_omega(Lam, cfg)
    return IcpFactor(i=0, j=1, T_icp_ij=T_rel @ off, Omega_icp=Omega)


def main() -> int:
    args = _make_problem()
    K, st, obs_cam, obs_lm, obs_uv, obs_depth, imu_factors, g_world = args
    base = VioConfig(lock_tilt=False)
    ok = True

    H_off, b_off, c_off = _system(args, base)

    # ---- 1. gap=0 dead branch ------------------------------------------- #
    # (a) icp_factor False, factors supplied -> ignored entirely
    f = _one_factor(st)
    H_a, b_a, c_a = _system(args, base, icp_factors=[f])
    gap_a = (np.array_equal(H_off, H_a) and np.array_equal(b_off, b_a)
             and c_off == c_a)
    # (b) icp_factor True but empty factor list -> no-op
    cfg_on = replace(base, icp_factor=True)
    H_b, b_b, c_b = _system(args, cfg_on, icp_factors=[])
    gap_b = (np.array_equal(H_off, H_b) and np.array_equal(b_off, b_b)
             and c_off == c_b)
    gap_ok = gap_a and gap_b
    print(f"[{'ok' if gap_ok else 'FAIL'}] gap=0 dead branch: "
          f"flag-off-with-factors identical={gap_a}, "
          f"flag-on-empty-list identical={gap_b}")
    ok = ok and gap_ok

    # ---- 2. pose-only assembly ------------------------------------------ #
    H_on, b_on, c_on = _system(args, cfg_on, icp_factors=[f])
    dH = H_on - H_off
    # column layout (lock_tilt=False, anchor=0): KF0 anchored (no pose cols),
    # KF1 pose at cols [0:6], then v0,bg0,ba0, v1,bg1,ba1, then 8 landmarks.
    pose_dof = 6
    p1 = slice(0, pose_dof)        # KF1 pose block
    # everything OUTSIDE the KF1 pose block must be untouched by the ICP factor
    mask = np.ones(dH.shape[0], bool); mask[p1] = False
    off_block = float(np.max(np.abs(dH[np.ix_(mask, mask)])))
    cross = float(np.max(np.abs(dH[np.ix_(~mask, mask)])))
    pose_block_nonzero = float(np.max(np.abs(dH[p1, p1]))) > 1e-6
    pose_only_ok = off_block < 1e-12 and cross < 1e-12 and pose_block_nonzero
    print(f"[{'ok' if pose_only_ok else 'FAIL'}] pose-only assembly: "
          f"off-pose-block max|dH|={off_block:.2e}, cross={cross:.2e}, "
          f"pose block non-zero={pose_block_nonzero}")
    ok = ok and pose_only_ok

    # ---- 3. cost term --------------------------------------------------- #
    r = _icp_residual(st.R[0], st.p[0], st.R[1], st.p[1],
                      f.T_icp_ij, f.Omega_icp)
    e = float(np.linalg.norm(r))
    thr = 3.0
    exp = 0.5 * float(r @ r) if e <= thr else thr * (e - 0.5 * thr)
    got = c_on - c_off
    cost_ok = abs(got - exp) < 1e-9
    print(f"[{'ok' if cost_ok else 'FAIL'}] cost term: got={got:.6e} "
          f"expect={exp:.6e} err={abs(got - exp):.2e}")
    ok = ok and cost_ok

    print("\n" + ("PASS -- ICP factor gap=0 dead branch + pose-only assembly hold."
                  if ok else "FAIL -- see flagged checks above."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
