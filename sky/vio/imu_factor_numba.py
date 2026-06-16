"""Numba-JIT core for the tight VIO IMU-factor finite-difference Jacobian.

The tight windowed-VIO solve (:func:`sky.vio.window.optimize_vio` with IMU
factors present) spends ~67-72% of its time inside ``build_system`` building the
per-edge IMU-factor Jacobian by finite difference: each of the ~7 keyframe edges
is differentiated against ~31 perturbation columns, every column a fresh pure-
Python ``_imu_eval`` (so3/se3 primitives), across the ~12 LM iterations.

This module reimplements *only that hot loop* as an explicit-array
``@njit(cache=True, fastmath=False)`` kernel: the SAME finite-difference math
(same ``eps``, same so3 ``exp``/``log`` formulas, same residual order
``[rRot; rVel; rPos; rBgRw; rBaRw (; rCv)]``, same 9x9 ``sqrt_info`` whitening),
so the assembled ``H``/``b`` are numerically ~equivalent (not bit-exact) to the
pure-Python build. The kernel returns each edge's ``Ji`` + ``r0i`` as plain
float64 arrays; the NumPy caller scatters them into ``H``/``b`` exactly as the
pure-Python ``np.ix_`` accumulation did, so the dense Schur solve is unchanged.

Why ``fastmath=False`` (NOT ``True`` like ``sky.front.klt_numba``)
------------------------------------------------------------------
The LM loop in ``sky.vio.window.optimize_vio`` branches DISCRETELY on the cost
(``cost_new < cost_prev``) and on the relative-improvement tolerance
(``improved < cfg.rel_tol``). ``fastmath=True`` lets LLVM reassociate floating-
point ops, which perturbs ``H``/``b`` enough to flip an accept/reject or an
early-stop decision -> a different iteration count -> a different converged
trajectory. ``fastmath=False`` keeps the float ops in IEEE order so the kernel
tracks the pure-Python build as closely as possible. **Do not copy
``fastmath=True`` into this module.**

DEFAULT ON, COUPLED TO THE DIVERGENCE GUARD (status, 2026-06-16)
---------------------------------------------------------------
The kernel's ``H``/``b`` match the pure-Python build to the fp ROUND-OFF FLOOR
(relative ~7e-11, see ``verification/imu_factor_njit_equiv.py``). njit-vs-pure
USED to diverge ~600 mm on the high-excitation sessions (push_shake_20s,
lab_straight_20s, quick_motion_15s) because the tight LM solve was round-off-
CHAOTIC: a ~1e-11 relative ``H``/``b`` difference flipped an LM iteration and
walked the converged trajectory. That chaos came ENTIRELY from a DIVERGENT window
the LM accepted -- with the divergence guard (``WindowedVIOConfig.
divergence_guard``, default ON) REJECTING that window DETERMINISTICALLY (regardless
of njit-vs-pure round-off), the chaos source is gone: the njit-vs-pure full-session
ATE is now EQUAL (0.0 mm) on EVERY gold session INCLUDING shake
(``verification/imu_factor_njit_ate.py`` PASS on all gold). So the kernel is now
DEFAULT ON -- but validated ONLY with the guard ON.

The caller's gate (``njit_guard_ok`` in :func:`sky.vio.window.optimize_vio`), in
precedence order:
  * ``SKY_VIO_IMU_NJIT=0`` -> force OFF (explicit kill switch, always honoured);
  * GUARD-COUPLING: if the divergence guard is OFF (``cfg.njit_guard_ok`` False,
    set by ``WindowedVIOMap.run_ba`` from ``divergence_guard``) -> FORCE the kernel
    OFF and log why (safety wins even over an explicit ``=1``: njit determinism is
    validated only WITH the guard, running it guard-off re-opens the round-off
    chaos);
  * otherwise DEFAULT ON (env unset) -- the validated production path.

Numba is an OPTIONAL dependency. :data:`HAVE_NUMBA` is ``False`` when it is
absent (or when the ``@njit`` wrappers are no-ops); the caller
(:func:`sky.vio.window.build_system`) then runs the UNCHANGED pure-Python build.
The kernel is entered ONLY on the live tight config (``imu_info_weight`` set); the
loose / oracle path never reaches it, so the default loose build stays
byte-identical (gap=0).
"""
from __future__ import annotations

import numpy as np

try:
    from numba import njit  # type: ignore
    HAVE_NUMBA = True
except Exception:  # pragma: no cover - exercised only when numba is absent
    HAVE_NUMBA = False

    def njit(*args, **kwargs):  # type: ignore
        """No-op fallback so the module imports without numba installed."""
        def wrap(fn):
            return fn
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]
        return wrap


# --------------------------------------------------------------------------- #
# njit linear-algebra helpers (3x3 only -- NO np.linalg, so scipy-free trivially).
# Defined first so the SO(3) primitives below can call ``_mm3``.
# --------------------------------------------------------------------------- #
@njit(cache=True, fastmath=False)
def _mm3(A, B):
    """3x3 @ 3x3."""
    C = np.empty((3, 3))
    for i in range(3):
        for j in range(3):
            C[i, j] = A[i, 0] * B[0, j] + A[i, 1] * B[1, j] + A[i, 2] * B[2, j]
    return C


@njit(cache=True, fastmath=False)
def _mtv3(A, v):
    """A.T @ v for 3x3 A, 3-vector v."""
    out = np.empty(3)
    for i in range(3):
        out[i] = A[0, i] * v[0] + A[1, i] * v[1] + A[2, i] * v[2]
    return out


@njit(cache=True, fastmath=False)
def _mv3(A, v):
    """A @ v for 3x3 A, 3-vector v."""
    out = np.empty(3)
    for i in range(3):
        out[i] = A[i, 0] * v[0] + A[i, 1] * v[1] + A[i, 2] * v[2]
    return out


# --------------------------------------------------------------------------- #
# SO(3) primitives, njit-ready (pure float64 arrays).
#
# Bit-for-bit the formulas in :mod:`sky.math.so3` (``so3_exp_unit`` / ``so3_log``)
# -- the IMU convention (exact identity at zero) and the standard small-residual
# log. The pure-Python versions stay the single source of truth for the loose
# path; these duplicate ONLY the arithmetic njit can compile.
# --------------------------------------------------------------------------- #
@njit(cache=True, fastmath=False)
def _so3_exp_unit(omega):
    """Rodrigues exp, IMU convention (exact ``I`` for ``||omega|| < 1e-12``).

    Reproduces :func:`sky.math.so3.so3_exp_unit` operation-for-operation:
    ``theta = norm(omega)``; ``K = skew(omega/theta)``;
    ``R = I + sin(theta)*K + (1-cos(theta))*(K @ K)``. The matmul ``K @ K`` is
    formed explicitly (not algebraically pre-expanded) so the float summation
    order matches NumPy's -> the FD column noise stays at the round-off floor.
    """
    theta = np.sqrt(omega[0] * omega[0] + omega[1] * omega[1]
                    + omega[2] * omega[2])
    R = np.empty((3, 3))
    if theta < 1e-12:
        R[0, 0] = 1.0; R[0, 1] = 0.0; R[0, 2] = 0.0
        R[1, 0] = 0.0; R[1, 1] = 1.0; R[1, 2] = 0.0
        R[2, 0] = 0.0; R[2, 1] = 0.0; R[2, 2] = 1.0
        return R
    k0 = omega[0] / theta
    k1 = omega[1] / theta
    k2 = omega[2] / theta
    K = np.empty((3, 3))
    K[0, 0] = 0.0; K[0, 1] = -k2; K[0, 2] = k1
    K[1, 0] = k2; K[1, 1] = 0.0; K[1, 2] = -k0
    K[2, 0] = -k1; K[2, 1] = k0; K[2, 2] = 0.0
    KK = _mm3(K, K)
    s = np.sin(theta)
    c = 1.0 - np.cos(theta)
    for r in range(3):
        for cc in range(3):
            R[r, cc] = (1.0 if r == cc else 0.0) + s * K[r, cc] + c * KK[r, cc]
    return R


@njit(cache=True, fastmath=False)
def _so3_log(R):
    """Standard SO(3) log (matches :func:`sky.math.so3.so3_log`)."""
    cos_t = (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) * 0.5
    if cos_t > 1.0:
        cos_t = 1.0
    elif cos_t < -1.0:
        cos_t = -1.0
    theta = np.arccos(cos_t)
    w = np.empty(3)
    w[0] = R[2, 1] - R[1, 2]
    w[1] = R[0, 2] - R[2, 0]
    w[2] = R[1, 0] - R[0, 1]
    if theta < 1e-8:
        w[0] *= 0.5; w[1] *= 0.5; w[2] *= 0.5
        return w
    scale = theta / (2.0 * np.sin(theta))
    w[0] *= scale; w[1] *= scale; w[2] *= scale
    return w


@njit(cache=True, fastmath=False)
def _corrected(dR, dv, dp, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
               bg_lin, ba_lin, bg_i, ba_i):
    """First-order bias-corrected ``(dR, dv, dp)`` -- inlined ``ImuPreintegration.corrected``.

    ``dbg = bg_i - bg_lin``, ``dba = ba_i - ba_lin`` (the integration
    linearisation bias is ``bg_lin/ba_lin``). Matches
    :meth:`sky.vio.imu.ImuPreintegration.corrected` exactly::

        dR' = dR @ Exp(dR_dbg @ dbg)
        dv' = dv + dv_dbg @ dbg + dv_dba @ dba
        dp' = dp + dp_dbg @ dbg + dp_dba @ dba
    """
    dbg = np.empty(3)
    dba = np.empty(3)
    for k in range(3):
        dbg[k] = bg_i[k] - bg_lin[k]
        dba[k] = ba_i[k] - ba_lin[k]
    dRc = _mm3(dR, _so3_exp_unit(_mv3(dR_dbg, dbg)))
    dvc = np.empty(3)
    dpc = np.empty(3)
    dvb = _mv3(dv_dbg, dbg)
    dva = _mv3(dv_dba, dba)
    dpb = _mv3(dp_dbg, dbg)
    dpa = _mv3(dp_dba, dba)
    for k in range(3):
        dvc[k] = dv[k] + dvb[k] + dva[k]
        dpc[k] = dp[k] + dpb[k] + dpa[k]
    return dRc, dvc, dpc


@njit(cache=True, fastmath=False)
def _imu_eval(R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v_j, bg_j, ba_j,
              dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
              bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
              sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
              sigma_vel_cv, out):
    """Whitened stacked IMU-edge residual into ``out`` (length = ``_imu_rows``).

    Mirrors ``_imu_eval`` in :func:`sky.vio.window.build_system`:
    ``[ _imu_residual (9) ; _bias_rw_residual (6) (; r_cv (3)) ]``.

    The 9-vector IMU residual is ``[rRot; rVel; rPos]`` (the
    :func:`sky.vio.window._imu_residual` order), whitened either by the per-edge
    ``sqrt_info`` (``info_weight=True``) or by the fixed scalar sigmas. The
    bias-random-walk block is ``[(bg_j-bg_i)/sigma_bg_rw; (ba_j-ba_i)/sigma_ba_rw]``.
    The optional constant-velocity prior ``(v_j-v_i)/sigma_vel_cv`` is appended.
    """
    dRc, dvc, dpc = _corrected(dR, dv, dp, dR_dbg, dv_dbg, dv_dba,
                               dp_dbg, dp_dba, bg_lin, ba_lin, bg_i, ba_i)
    # rR = log( dRc.T @ R_i.T @ R_j ). NumPy evaluates ``dR.T @ Ri_T @ R_j``
    # left-to-right, so group as ``(dRc.T @ R_i.T) @ R_j`` to match its float
    # summation order (keeps the FD column noise at the round-off floor).
    rR = _so3_log(_mm3(_mm3(dRc.T, R_i.T), R_j))
    # rv = R_i.T @ (v_j - v_i - g*dt) - dvc
    tv = np.empty(3)
    tp = np.empty(3)
    for k in range(3):
        tv[k] = v_j[k] - v_i[k] - g_world[k] * dt
        tp[k] = p_j[k] - p_i[k] - v_i[k] * dt - 0.5 * g_world[k] * dt * dt
    rv = _mtv3(R_i, tv)
    rp = _mtv3(R_i, tp)
    raw = np.empty(9)
    for k in range(3):
        raw[k] = rR[k]
        raw[3 + k] = rv[k] - dvc[k]
        raw[6 + k] = rp[k] - dpc[k]

    if info_weight:
        # 9x9 whitening: out[0:9] = sqrt_info @ raw
        for i in range(9):
            s = 0.0
            for j in range(9):
                s += sqrt_info[i, j] * raw[j]
            out[i] = s
    else:
        for k in range(3):
            out[k] = raw[k] / sigma_rot
            out[3 + k] = raw[3 + k] / sigma_vel
            out[6 + k] = raw[6 + k] / sigma_pos

    # bias random walk (6)
    for k in range(3):
        out[9 + k] = (bg_j[k] - bg_i[k]) / sigma_bg_rw
        out[12 + k] = (ba_j[k] - ba_i[k]) / sigma_ba_rw
    if vel_cv:
        for k in range(3):
            out[15 + k] = (v_j[k] - v_i[k]) / sigma_vel_cv


@njit(cache=True, fastmath=False)
def _pose_perturb(R, p, d, lock_tilt, up_axis):
    """Perturbed ``(R, p)`` for a pose DoF -- mirrors :func:`sky.vio.window._pose_perturb`.

    ``d`` is the per-pose increment (length 4 if ``lock_tilt`` else 6):
    full 6-DoF -> ``R @ Exp(d[3:6])``, ``p + R @ d[:3]``;
    tilt-locked 4-DoF -> ``Exp(up_axis*d[3]) @ R``, ``p + R @ d[:3]``.
    """
    dp = np.empty(3)
    dp[0] = d[0]; dp[1] = d[1]; dp[2] = d[2]
    Rdp = _mv3(R, dp)
    p_new = np.empty(3)
    for k in range(3):
        p_new[k] = p[k] + Rdp[k]
    if lock_tilt:
        omega = np.empty(3)
        for k in range(3):
            omega[k] = up_axis[k] * d[3]
        R_new = _mm3(_so3_exp_unit(omega), R)
    else:
        omega = np.empty(3)
        omega[0] = d[3]; omega[1] = d[4]; omega[2] = d[5]
        R_new = _mm3(R, _so3_exp_unit(omega))
    return R_new, p_new


@njit(cache=True, fastmath=False)
def imu_factor_jacobian(
        R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v_j, bg_j, ba_j,
        dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
        bg_lin, ba_lin, sqrt_info,
        sigma_rot, sigma_vel, sigma_pos, sigma_bg_rw, sigma_ba_rw,
        g_world, info_weight, vel_cv, sigma_vel_cv,
        eps, lock_tilt, up_axis, pose_dof,
        i_free, j_free, n_cols, nrows):
    """Per-edge IMU-factor FD residual ``r0`` and Jacobian ``Ji``.

    Returns ``(r0, Ji)`` with ``r0`` length ``nrows`` and ``Ji`` shape
    ``(nrows, n_cols)``. The column order mirrors ``build_system``'s ``blocks``
    list EXACTLY: for the i-side then the j-side keyframe,
    ``[pose(pose_dof, only if free)] + vel(3) + bg(3) + ba(3)``. ``i_free`` /
    ``j_free`` flag whether that keyframe's pose is a free (non-anchor) column.

    Finite difference: ``Ji[:, c] = (_imu_eval(perturbed) - r0) / eps`` with the
    same forward step (+eps), same ``_pose_perturb`` retraction, same residual
    stack as the pure-Python build -> numerically ~equivalent ``Ji``.
    """
    r0 = np.empty(nrows)
    _imu_eval(R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v_j, bg_j, ba_j,
              dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
              bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
              sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
              sigma_vel_cv, r0)

    Ji = np.zeros((nrows, n_cols))
    rp = np.empty(nrows)
    col = 0

    # ----- i-side blocks ------------------------------------------------------
    if i_free:
        for d in range(pose_dof):
            dd = np.zeros(pose_dof)
            dd[d] = eps
            Rp, pp = _pose_perturb(R_i, p_i, dd, lock_tilt, up_axis)
            _imu_eval(Rp, pp, v_i, bg_i, ba_i, R_j, p_j, v_j, bg_j, ba_j,
                      dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                      bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                      sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                      sigma_vel_cv, rp)
            for r in range(nrows):
                Ji[r, col] = (rp[r] - r0[r]) / eps
            col += 1
    # vel_i
    for d in range(3):
        v2 = v_i.copy(); v2[d] += eps
        _imu_eval(R_i, p_i, v2, bg_i, ba_i, R_j, p_j, v_j, bg_j, ba_j,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1
    # bg_i
    for d in range(3):
        b2 = bg_i.copy(); b2[d] += eps
        _imu_eval(R_i, p_i, v_i, b2, ba_i, R_j, p_j, v_j, bg_j, ba_j,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1
    # ba_i
    for d in range(3):
        a2 = ba_i.copy(); a2[d] += eps
        _imu_eval(R_i, p_i, v_i, bg_i, a2, R_j, p_j, v_j, bg_j, ba_j,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1

    # ----- j-side blocks ------------------------------------------------------
    if j_free:
        for d in range(pose_dof):
            dd = np.zeros(pose_dof)
            dd[d] = eps
            Rp, pp = _pose_perturb(R_j, p_j, dd, lock_tilt, up_axis)
            _imu_eval(R_i, p_i, v_i, bg_i, ba_i, Rp, pp, v_j, bg_j, ba_j,
                      dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                      bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                      sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                      sigma_vel_cv, rp)
            for r in range(nrows):
                Ji[r, col] = (rp[r] - r0[r]) / eps
            col += 1
    # vel_j
    for d in range(3):
        v2 = v_j.copy(); v2[d] += eps
        _imu_eval(R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v2, bg_j, ba_j,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1
    # bg_j
    for d in range(3):
        b2 = bg_j.copy(); b2[d] += eps
        _imu_eval(R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v_j, b2, ba_j,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1
    # ba_j
    for d in range(3):
        a2 = ba_j.copy(); a2[d] += eps
        _imu_eval(R_i, p_i, v_i, bg_i, ba_i, R_j, p_j, v_j, bg_j, a2,
                  dR, dv, dp, dt, dR_dbg, dv_dbg, dv_dba, dp_dbg, dp_dba,
                  bg_lin, ba_lin, sqrt_info, sigma_rot, sigma_vel, sigma_pos,
                  sigma_bg_rw, sigma_ba_rw, g_world, info_weight, vel_cv,
                  sigma_vel_cv, rp)
        for r in range(nrows):
            Ji[r, col] = (rp[r] - r0[r]) / eps
        col += 1

    return r0, Ji
