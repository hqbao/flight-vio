"""Tight-coupled visual-inertial window optimizer (pure NumPy).

This is the Basalt-style core the loosely-coupled gyro fusion could not be: it
puts the raw visual measurements (reprojection + metric depth) **and** the IMU
preintegration factors (rotation, velocity, position increments) into ONE
non-linear least-squares problem, solving jointly for every keyframe's pose,
**velocity** and **gyro/accel bias**, plus the landmarks. Because the
accelerometer ties consecutive keyframes through ``v`` and ``p``, a pure in-place
rotation -- where the true linear acceleration is ~0 -- can no longer be
explained as a translation by slipped visual tracks: the IMU says "no
acceleration => no net translation", killing the phantom yaw-drift that the
vision-only / loosely-coupled paths leave behind.

Design choices (deliberate, to be correct and verifiable before fast):
  * **Body frame == camera frame** in this core. The IMU<->camera extrinsic is
    handled by the caller, which rotates the raw IMU samples into the camera
    optical frame before preintegrating (see :mod:`sky.vio.imu`). The small
    IMU/camera lever arm is treated as modelling noise (the OAK-D IMU sits ~cm
    from the left camera); a future refinement can add it explicitly.
  * Poses are parametrised as **body->world** ``(R, p)`` with the perturbation
    ``R <- R Exp(dphi)``, ``p <- p + R dp`` (GTSAM Pose3 convention). Velocity
    and biases are plain additive vector states.
  * **Per-factor finite-difference Jacobians.** Each factor differentiates only
    its own local variables (a projection sees 1 pose + 1 landmark; an IMU
    factor sees the 2 adjacent nav states), so FD is cheap AND immune to the
    hand-derivation sign errors that plague analytic VIO Jacobians. The IMU
    residual formulas themselves are the ones validated in
    ``vio/tests/vio_ba_selftest.py``.
  * **Dense Levenberg-Marquardt** over the whole window (no Schur complement).
    Correctness first; the window is small, so a dense solve is fine. Schur is a
    speed optimisation left for later if the live path needs it.

One keyframe (``anchor``, default 0) has its pose held fixed to pin the global
position+yaw gauge (gravity already fixes roll/pitch through the IMU factors).
Its velocity and biases stay free.

Validated end-to-end by ``vio/tests/vio_ba_selftest.py``: a synthetic multi-segment
trajectory (fast yaw + translation under gravity) with consistent IMU + image
measurements is perturbed and recovered to sub-mm / sub-mdeg.
"""
from __future__ import annotations

import os
from collections import Counter
from dataclasses import dataclass, field, replace

import numpy as np

from sky.math import so3_exp_unit as so3_exp
from sky.math import so3_log
from sky.math import se3_from_Rp, se3_inv, se3_log_robust

from sky.front.frontend import FrontendConfig, KLTFrontend
from sky.depth.icp import backproject_depth, icp_p2plane_blend, imu_seed_relpose
from .imu import ImuPreintegration, preintegrate_imu
from .imu_factor_numba import HAVE_NUMBA, imu_factor_jacobian
from sky.front.odometry import OdometryConfig, RGBDVisualOdometry


# --------------------------------------------------------------------------- #
# State + configuration
# --------------------------------------------------------------------------- #
@dataclass
class VioState:
    """Mutable VIO window state. Lists are per-keyframe, indexed alike.

    R, p : body->world rotation (3x3) and position (3,) per keyframe.
    v    : world-frame velocity (3,) per keyframe.
    bg, ba: gyro / accel bias (3,) per keyframe.
    landmarks: (M,3) world points.
    """
    R: list = field(default_factory=list)
    p: list = field(default_factory=list)
    v: list = field(default_factory=list)
    bg: list = field(default_factory=list)
    ba: list = field(default_factory=list)
    landmarks: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))

    def copy(self) -> "VioState":
        return VioState(
            R=[r.copy() for r in self.R],
            p=[x.copy() for x in self.p],
            v=[x.copy() for x in self.v],
            bg=[x.copy() for x in self.bg],
            ba=[x.copy() for x in self.ba],
            landmarks=self.landmarks.copy(),
        )


@dataclass
class VioConfig:
    # measurement sigmas (whiten each residual to be dimensionless)
    sigma_px: float = 1.0           # pixel reprojection sigma
    depth_sigma_coeff: float = 0.02  # sigma_z = coeff * z^2  (metres)
    depth_huber: float = 0.10       # robust threshold on depth residual (m)
    sigma_rot: float = 0.01         # rad, IMU rotation increment
    sigma_vel: float = 0.05         # m/s, IMU velocity increment
    sigma_pos: float = 0.05         # m, IMU position increment
    sigma_bg_rw: float = 1e-3       # gyro-bias random walk
    sigma_ba_rw: float = 1e-2       # accel-bias random walk
    # IMU-factor weighting (Phase 1). When False (default) the IMU residual
    # [dphi; dvel; dpos] is whitened by the FIXED scalar sigmas above -- the
    # original, byte-parity-preserving behaviour the frozen ``vio`` oracle
    # entries depend on. When True the residual is whitened by the per-edge
    # information square root ``Omega_I = pre.sqrt_info`` (``sqrt_info.T @
    # sqrt_info == Sigma_ij^-1``, the Phase-0 covariance), so the IMU factor's
    # weight is the ACTUAL integration uncertainty + noise model rather than a
    # hand-tuned constant. This is the covariance-correct tight-coupling weight;
    # it is OPT-IN so enabling it never moves the loose path or the oracle (which
    # both leave it at the default False). If a factor's ``sqrt_info`` is missing
    # (degenerate / empty interval) the code falls back to the fixed sigmas for
    # that edge.
    imu_info_weight: bool = False
    # --- Phase-4 velocity stabilisation (ALL default OFF -> oracle byte-safe) --
    # The lone IMU factor is rank-6-deficient in velocity: it only ties the
    # DIFFERENCES (v_j - v_i, dp - v_i*dt); it carries ZERO absolute-velocity
    # information. At 54x42 the feature-starved vision cannot pin p_j, so the
    # position residual r_p cannot transfer weight onto v_i, leaving only the
    # difference-tie r_v -- which faithfully copies a drifting velocity seed
    # forward and compounds it (the 0.175 -> 4.96 m/s shake runaway). The two
    # opt-in terms below inject the missing constraint:
    #
    # (A) vel_cv_prior -- a constant-velocity SMOOTHNESS prior per IMU edge i->j:
    #     r_cv = (v_j - v_i) / sigma_vel_cv  (world frame, isotropic). It penalises
    #     velocity CHANGE between keyframes, which is what stops the ramp; sigma is
    #     loose so it only bites the divergent ramp, not real acceleration.
    # (B) vel_zupt -- an excitation-gated zero-velocity prior r_zupt = v_i /
    #     sigma_vel_zupt applied only on KFs whose inbound IMU edge is
    #     LOW-EXCITATION (near rest). This is the ABSOLUTE anchor the CV prior
    #     lacks; gated on excitation (not translation) so shake (high excitation)
    #     turns it OFF and the CV prior carries the window instead.
    #
    # Both paths are guarded by their flag so the OFF path is byte-identical to
    # the frozen oracle. sigma_vel_zupt >> sigma_vel_cv on purpose: ZUPT is a weak
    # absolute pin, the CV prior is the primary ramp-killer.
    vel_cv_prior: bool = False
    sigma_vel_cv: float = 0.15      # m/s, loose -> only bites the divergent ramp
    vel_zupt: bool = False
    sigma_vel_zupt: float = 0.5     # m/s, weak absolute anchor (>> sigma_vel_cv)
    zupt_accel_thresh: float = 0.30  # m/s^2, |dv|/dt below this == low excitation
    zupt_gyro_thresh: float = 0.20  # rad/s, |log(dR)|/dt below this == low excite
    # --- Absolute-velocity GAUGE anchor (the real rank fix; default ON for tight)
    # The lone IMU velocity residual r_v = R_i^T (v_j - v_i - g dt) - dv is a pure
    # DIFFERENCE operator: d r_v / d v_i = -R_i^T, d r_v / d v_j = +R_i^T, so the
    # IMU factor carries ZERO absolute-velocity information -- the NAV information
    # matrix has a rank-3 null space (eigs ~3e-17) in absolute velocity. The LM
    # relative damping ``lam*diag(H)`` floors that null direction only by a
    # machine-eps-scale value, so a ~1e-11 round-off perturbation of H/b divides by
    # ~(lam*diag) ~ 1e-6 and is amplified ~1e6x -> the converged basin flips
    # (round-off CHAOS; njit-vs-pure diverges 603 mm on shake). This anchor pins
    # the genuinely-free absolute-velocity gauge with a WEAK absolute prior on
    # EVERY keyframe -- it restores rank without inventing information (a loose
    # sigma so it does not fight the IMU/vision solution):
    #
    #     r_vabs = (v_i - v_pred_i) / sigma_vabs        (3 rows per KF)
    #
    # where v_pred_i is the IMU forward-prediction of v_i from the PREVIOUS
    # keyframe's velocity along the inbound edge (i-1, i):
    #     v_pred_i = v_{i-1} + g dt + R_{i-1} dv_pre
    # treated as a CONSTANT pseudo-measurement recomputed each iteration from the
    # current neighbour estimate (so the Jacobian is the simple absolute self-term
    # d r_vabs / d v_i = I / sigma_vabs, like ZUPT). The anchor keyframe (no
    # inbound edge) uses its own current estimate as v_pred -> a prior pulling v_0
    # toward its linearisation point. UNLIKE vel_zupt this is NOT excitation-gated:
    # it must be ON during shake (exactly when ZUPT turns OFF). This is the
    # standard VINS/Basalt velocity gauge anchor. Default ON because it is read
    # ONLY on the tight path (gated by imu_info_weight in build_system); the loose
    # / oracle path never assembles it, so the default loose build is unchanged.
    vel_abs_prior: bool = True
    sigma_vabs: float = 1.0         # m/s, LOOSE -> pins the gauge, never fights IMU
    # --- Dense-ICP relative-pose factor (OPT-IN, default OFF -> oracle byte-safe)
    # At 54x42 the sparse frontend FAILS on 5-10% of frames (<6 tracks), leaving
    # the inter-keyframe TRANSLATION unobservable. A dense point-to-plane ICP
    # between two keyframes' depth clouds never fails (it always yields a
    # translation constraint), so an ICP relative-pose factor gives the missing
    # Delta-p a real anchor. The factor is ADDED to (never replaces) the sparse
    # reprojection: it self-balances by information -- full-res reproj dwarfs the
    # ICP term (inert), 54x42 reproj collapses and the ICP carries Delta-p. The
    # measurement information ``Lambda`` (the ICP point-to-plane Hessian) is
    # whitened ONCE per factor into ``Omega_icp`` (see ``_icp_omega``) and held
    # fixed during the solve, mirroring ``pre.sqrt_info``. When ``icp_factor`` is
    # False AND no factors are supplied the build_system/total_cost loops are
    # skipped entirely -> H/b/cost are BIT-IDENTICAL to today.
    icp_factor: bool = False
    sigma_rot_icp: float = 0.2      # rad: LOOSE fixed ICP rotation sigma. The gyro
    #                                 owns rotation (ICP rotation is noisy), so the
    #                                 rotation 3x3 block of Omega_icp is OVERWRITTEN
    #                                 with 1/sigma_rot_icp^2 and the trans-rot cross
    #                                 blocks are ZEROED -- ICP only pins translation.
    icp_lambda_thresh: float = 0.02  # relative eigenvalue gate kappa: an eigen-
    #                                 direction with lambda < kappa*lambda_max is a
    #                                 degenerate (e.g. along-a-wall) direction and is
    #                                 PROJECTED OUT (zero information) so the factor
    #                                 never invents a constraint it cannot observe.
    icp_lambda_floor: float = 1.0   # absolute eigenvalue floor (information units):
    #                                 a direction below this is also projected out,
    #                                 guarding the case where ALL eigenvalues are
    #                                 small (a near-featureless cloud).
    icp_huber: float = 3.0          # factor-level Huber threshold on ||r_icp||.
    huber_px: float = 2.0           # robust threshold on pixel residual
    use_depth: bool = True
    min_view_z: float = 1e-3
    # When True the camera ROLL/PITCH (tilt relative to gravity) is HELD FIXED
    # during optimisation: each pose only has 4 free DoF -- 3 translation + 1 YAW
    # about the world-vertical axis -- instead of the full 6. The accelerometer
    # (via the IMU factor's gravity term, and the f2f gravity-levelling that
    # seeds the input pose) already owns tilt absolutely; locking it stops the
    # reprojection factors from drifting roll/pitch and stops gravity leaking
    # into a horizontal translation. Vision/depth then refine only what the IMU
    # cannot observe: yaw and position. Default False keeps the full 6-DoF
    # solve (and the byte-identical self-test recovery of a perturbed tilt).
    lock_tilt: bool = False
    # LM
    max_iters: int = 30
    init_lambda: float = 1e-3
    # LM damping floor. Raised 1e-9 -> 1e-6 as the SECOND half of the gauge
    # regulariser (paired with ``tau_nav`` + ``vel_abs_prior``): at the old 1e-9
    # the relative damping ``lam*diag(H)`` on the (now-pinned) low-curvature nav
    # directions decayed to a machine-eps-scale floor, so a round-off-scale H/b
    # difference still produced a meaningfully different step on the flat
    # directions. A 1e-6 floor keeps the LM step bounded near convergence without
    # biasing the well-conditioned solution (1e-6 * diag(H) is negligible against
    # the real curvature). ``optimize_vio`` is reached ONLY by the tight VIO
    # ``run_ba`` (the loose / oracle path uses ``sky.backend.windowed`` and never
    # calls this), so the raise is tight-only by construction.
    min_lambda: float = 1e-6
    max_lambda: float = 1e9
    rel_tol: float = 1e-7
    fd_eps: float = 1e-6
    # --- Absolute Tikhonov floor on the NAV block (TIGHT-ONLY, gauge regulariser)
    # The LM damping ``A = H + lam*diag(H)`` floors the absolute-velocity null
    # direction only RELATIVELY (by lam*diag, machine-eps-scale there), so the
    # null-direction step ~ -b_null / (lam*diag) divides round-off by a tiny number
    # and amplifies it ~1e6x. ``tau_nav`` adds a small ABSOLUTE floor on the NAV
    # columns only -- ``A = H + lam*diag(H) + tau_nav * I_nav`` (identity on the
    # first ``nav_dim`` poses+vel+bg+ba columns, ZERO on landmark columns so the
    # block-diagonal landmark Schur structure is untouched). It is added to the
    # FULL damped A BEFORE Schur partitioning, so the exact Schur path consumes it
    # unchanged. With ``vel_abs_prior`` already pinning the velocity gauge, this is
    # a belt-and-braces conditioner on the whole nav block (poses + biases too):
    # it caps the round-off amplification at ~b/tau_nav regardless of lam, killing
    # the LM accept/reject chaos. Applied ONLY when ``imu_info_weight`` is True
    # (the tight marker); the loose / oracle LM solve never adds it (byte-safe).
    # Kept small so it never biases the well-conditioned solution.
    tau_nav: float = 1e-3
    # --- njit IMU-factor kernel <-> divergence-guard SAFETY COUPLING --------- #
    # The njit FD IMU-factor kernel (sky.vio.imu_factor_numba, default ON now)
    # tracks the pure-Python build only to the fp round-off floor (~1e-11). The
    # tight LM solve branches DISCRETELY on cost, so that round-off can flip an
    # iteration and walk a DIVERGENT window elsewhere -- UNLESS the divergence
    # guard rejects that window deterministically (it does: see the njit gate in
    # ``optimize_vio`` and ``imu_factor_njit_ate.py``, ATE-equal on every gold
    # session incl. shake ONLY with the guard ON). So njit-default-ON is validated
    # ONLY when the divergence guard is on. ``WindowedVIOMap.run_ba`` sets this to
    # ``WindowedVIOConfig.divergence_guard``; when it is False the njit gate
    # force-disables the kernel and logs why. Default True so a bare ``VioConfig``
    # (the self-tests / loose path) is not gratuitously degraded -- the loose path
    # never enables njit anyway (``imu_info_weight`` is False there).
    njit_guard_ok: bool = True


@dataclass
class VioResult:
    state: VioState
    iters: int
    cost0: float
    cost1: float
    mean_reproj_px: float


# --------------------------------------------------------------------------- #
# Residual primitives (raw, unwhitened) -- shared by cost eval and FD assembly
# --------------------------------------------------------------------------- #
def _project(R, p, Xw, fx, fy, cx, cy, min_z):
    """Camera-frame point + pixel of a world landmark for a body->world pose."""
    Xc = R.T @ (Xw - p)
    Z = Xc[2]
    Zc = Z if Z > min_z else min_z
    u = fx * Xc[0] / Zc + cx
    v = fy * Xc[1] / Zc + cy
    return Xc, Z, u, v


def _imu_residual(R_i, p_i, v_i, bg_i, ba_i,
                  R_j, p_j, v_j,
                  pre: ImuPreintegration, g_world, cfg) -> np.ndarray:
    """9-vector whitened IMU preintegration residual between KF i and j.

    Order: [rRot(3), rVel(3), rPos(3)].
    Matches the increment convention validated in imu_preint_selftest:
        R_j = R_i dR ; v_j = v_i + g dt + R_i dv ;
        p_j = p_i + v_i dt + 0.5 g dt^2 + R_i dp
    The *corrected* increments use the CURRENT bias estimate at i, which is the
    Forster first-order relinearise (cheap: re-uses the cached deltas + bias
    Jacobians instead of re-integrating the raw samples).

    Whitening (Phase 1)
    -------------------
    The RAW residual ``r = [rR; rv; rp]`` is whitened in one of two ways:

    * ``cfg.imu_info_weight == False`` (default): each block is divided by the
      fixed scalar sigma (``sigma_rot/sigma_vel/sigma_pos``). This is the
      original, byte-identical behaviour the frozen ``vio`` oracle depends on.
    * ``cfg.imu_info_weight == True``: the full 9-vector is whitened by the
      per-edge information square root ``Omega_I = pre.sqrt_info`` (from the
      Phase-0 covariance), i.e. ``sqrt_info @ r``. This makes the factor's weight
      the actual integration uncertainty. If this edge has no ``sqrt_info`` (an
      empty / degenerate interval), it falls back to the fixed sigmas so the
      factor is still well-posed rather than dropped.
    """
    dt = pre.dt
    dR, dv, dp = pre.corrected(bg_i, ba_i)
    Ri_T = R_i.T
    rR = so3_log(dR.T @ Ri_T @ R_j)
    rv = Ri_T @ (v_j - v_i - g_world * dt) - dv
    rp = Ri_T @ (p_j - p_i - v_i * dt - 0.5 * g_world * dt * dt) - dp
    if cfg.imu_info_weight and pre.sqrt_info is not None:
        # Covariance-correct weight: whiten the joint [dphi; dvel; dpos] residual
        # with Omega_I = sqrt_info (sqrt_info.T @ sqrt_info == Sigma_ij^-1). The
        # ordering of r MATCHES the covariance ordering propagated in
        # preintegrate_imu ([dphi; dvel; dpos]).
        return pre.sqrt_info @ np.concatenate([rR, rv, rp])
    return np.concatenate([
        rR / cfg.sigma_rot,
        rv / cfg.sigma_vel,
        rp / cfg.sigma_pos,
    ])


def _bias_rw_residual(bg_i, ba_i, bg_j, ba_j, cfg) -> np.ndarray:
    return np.concatenate([
        (bg_j - bg_i) / cfg.sigma_bg_rw,
        (ba_j - ba_i) / cfg.sigma_ba_rw,
    ])


def _imu_predict_pose(R_i, p_i, v_i, bg_i, ba_i, pre, g_world):
    """IMU forward-prediction (dead-reckon) of the NEXT keyframe's ``(R, p)``.

    Propagates the previous keyframe's nav-state ``(R_i, p_i, v_i)`` through the
    cached inter-keyframe preintegration ``pre`` (bias-corrected about the
    previous keyframe's ``bg_i/ba_i``). This is exactly the increment convention
    of :func:`_imu_residual` -- the pose at which the IMU rotation/position
    residuals are zero::

        R_j = R_i @ dR
        p_j = p_i + v_i dt + 0.5 g dt^2 + R_i @ dp

    Used by the divergence guard as the inertial reference the refined visual
    solve is checked against (and the fallback pose on rejection).
    """
    dt = pre.dt
    dR, _dv, dp = pre.corrected(bg_i, ba_i)
    R_pred = R_i @ dR
    p_pred = p_i + v_i * dt + 0.5 * g_world * dt * dt + R_i @ dp
    return R_pred, p_pred


def _v_pred_inbound(stt, inbound, ci, g_world) -> np.ndarray:
    """IMU forward-prediction of ``v[ci]`` from its inbound edge (i_prev -> ci).

    ``v_pred = v_{i_prev} + g dt + R_{i_prev} dv`` using the CURRENT neighbour
    estimate (the bias-corrected increment about ``bg/ba`` at ``i_prev``). When
    ``ci`` has no inbound edge (window anchor / gap) ``inbound[ci]`` is ``None``
    and the prediction is the keyframe's OWN current velocity -> the absolute
    prior then pulls ``v[ci]`` toward its linearisation point (a gauge pin that
    invents no information). The prediction is treated as a CONSTANT pseudo-
    measurement (recomputed each iteration), so the prior's Jacobian wrt ``v[ci]``
    is the simple absolute self-term ``I / sigma_vabs``.
    """
    edge = inbound[ci]
    if edge is None:
        return stt.v[ci]
    i_prev, pre = edge
    _dR, dv, _dp = pre.corrected(stt.bg[i_prev], stt.ba[i_prev])
    return stt.v[i_prev] + g_world * pre.dt + stt.R[i_prev] @ dv


# --------------------------------------------------------------------------- #
# Dense-ICP relative-pose factor primitives.
#
# An ICP factor links two keyframes (i, j) by a measured relative pose
# ``T_icp_ij`` (cam_i <- cam_j) with measurement information ``Lambda`` (6x6, the
# ICP point-to-plane Hessian in [trans(3); rot(3)] order). The residual compares
# the measured relative pose to the current state relative pose:
#
#     r_se3 = se3_log_robust( se3_inv(T_icp_ij) @ se3_inv(T_i) @ T_j )   ([rho;phi])
#     r_icp = Omega_icp @ r_se3
#
# with ``T_i = se3_from_Rp(R_i, p_i)`` (cam_i->world, body == camera). At the
# measurement (state relative pose == T_icp_ij) the residual is zero.
#
# The whitening ``Omega_icp`` is computed ONCE per factor from ``Lambda`` (like
# ``pre.sqrt_info``) and held fixed through the solve.
# --------------------------------------------------------------------------- #
def _icp_omega(Lambda: np.ndarray, cfg) -> np.ndarray:
    """6x6 whitening matrix ``Omega_icp`` from the ICP information ``Lambda``.

    Pipeline (the crux -- faithfully implements the math-reviewer spec):

    1. **Eigendecompose** ``Lambda`` (symmetric PSD, [trans;rot] order):
       ``Lambda = V diag(lambda) V^T``.
    2. **Degeneracy remap.** A direction with ``lambda < lambda_thr`` carries no
       trustworthy information (e.g. translation ALONG a flat wall), so its
       remapped eigenvalue is set to 0 (projected out). ``lambda_thr`` uses BOTH
       a relative gate ``kappa * lambda_max`` (``cfg.icp_lambda_thresh``) AND an
       absolute floor (``cfg.icp_lambda_floor``) so an all-small spectrum (a
       near-featureless cloud) is projected out entirely rather than amplified.
    3. **Overwrite the rotation 3x3 block** of the remapped information with the
       LOOSE fixed ``1/sigma_rot_icp^2 I`` (the gyro owns rotation -- ICP rotation
       is noisy) and **zero the trans-rot cross blocks**: the ICP factor pins only
       translation, never fighting the IMU on rotation.
    4. **Square-root via eigendecomposition** (NOT Cholesky, which would choke on
       the projected-out zeros): ``Omega = diag(sqrt(mu)) @ U^T`` where
       ``M = U diag(mu) U^T`` is the final block-structured information. Then
       ``Omega.T @ Omega == M`` exactly, and ``Omega @ r`` whitens the residual.
    """
    Lambda = np.asarray(Lambda, np.float64)
    Lambda = 0.5 * (Lambda + Lambda.T)        # symmetrise (guard FP asymmetry)
    evals, V = np.linalg.eigh(Lambda)         # ascending; columns = eigenvectors
    lam_max = float(evals[-1]) if evals.size else 0.0
    lam_thr = max(cfg.icp_lambda_thresh * lam_max, cfg.icp_lambda_floor)
    lam_remap = np.where(evals >= lam_thr, evals, 0.0)
    # remapped full information, then impose the rotation/cross-block structure
    M = V @ np.diag(lam_remap) @ V.T
    inv_var_rot = 1.0 / (cfg.sigma_rot_icp * cfg.sigma_rot_icp)
    M[0:3, 3:6] = 0.0
    M[3:6, 0:3] = 0.0
    M[3:6, 3:6] = np.eye(3) * inv_var_rot
    # sqrt via eigendecomposition of the final symmetric PSD information
    M = 0.5 * (M + M.T)
    mu, U = np.linalg.eigh(M)
    mu = np.clip(mu, 0.0, None)               # numerical floor at 0
    Omega = np.diag(np.sqrt(mu)) @ U.T
    return Omega


def _icp_residual(R_i, p_i, R_j, p_j, T_icp_ij, Omega_icp) -> np.ndarray:
    """Whitened ICP relative-pose residual ``Omega_icp @ r_se3`` ([rho;phi]).

    ``r_se3 = se3_log_robust(se3_inv(T_icp_ij) @ se3_inv(T_i) @ T_j)`` with
    ``T_i = (R_i, p_i)`` etc. (cam->world, body == camera). Zero at the
    measurement.
    """
    T_i = se3_from_Rp(R_i, p_i)
    T_j = se3_from_Rp(R_j, p_j)
    T_err = se3_inv(T_icp_ij) @ se3_inv(T_i) @ T_j
    r_se3 = se3_log_robust(T_err)
    return Omega_icp @ r_se3


@dataclass
class IcpFactor:
    """A dense-ICP relative-pose factor linking window keyframes ``i`` and ``j``.

    ``T_icp_ij`` is the ICP-measured relative pose ``cam_i <- cam_j`` and
    ``Omega_icp`` its precomputed 6x6 whitening (from :func:`_icp_omega`). Built
    once by ``run_ba`` (or a test) and consumed unchanged by :func:`optimize_vio`.
    """
    i: int
    j: int
    T_icp_ij: np.ndarray
    Omega_icp: np.ndarray


# --------------------------------------------------------------------------- #
# Pose perturbation helper.
#   Full 6-DoF (up_axis None):  d = [dp(3), dphi(3)], R <- R Exp(dphi).
#   Tilt-locked 4-DoF (up_axis): d = [dp(3), dyaw(1)], R <- Exp(up_axis*dyaw) R.
# In the tilt-locked case the yaw increment is a rotation about the WORLD
# vertical axis applied on the LEFT, so the gravity direction expressed in the
# body frame (R^T @ down_world) is unchanged -- i.e. roll/pitch stay fixed and
# only the heading (yaw) moves. Translation stays a body-frame perturbation in
# both cases.
# --------------------------------------------------------------------------- #
def _pose_perturb(R, p, d, up_axis=None):
    dp = d[:3]
    if up_axis is None:
        return R @ so3_exp(d[3:6]), p + R @ dp
    return so3_exp(up_axis * d[3]) @ R, p + R @ dp


# --------------------------------------------------------------------------- #
# Schur-complement linear solve.
#
# The damped normal-equations matrix ``A`` (== H + lam*diag, full ``ndim``) is
# dominated by the 3*M landmark columns, yet a dense ``np.linalg.solve`` on the
# whole window scales ~ndim^3. The state is laid out NAV-first / LANDMARK-last
# (see the column layout in :func:`optimize_vio`), and the landmark Hessian
# block ``All`` is BLOCK-DIAGONAL (3x3 per landmark -- each landmark only sees
# its own reprojection/depth rows). Marginalising the landmark block out gives
# the algebraically EXACT same ``delta`` from a solve on the small nav-only
# system:
#
#     [[App, Apl],[Apl.T, All]] [dp; dl] = -[gp; gl]
#       App - Apl All^-1 Apl.T  =: S          (Schur reduced, ~nav_dim)
#       gp  - Apl All^-1 gl     =: rp
#       dp  = solve(S, -rp)                    (small system)
#       dl  = All^-1 (-gl - Apl.T dp)          (back-substitution)
#
# All^-1 is built from the (M,3,3) block stack via batched host-LAPACK
# ``np.linalg.inv`` -- never a dense 3M x 3M inverse. The reduced coupling
# ``Apl All^-1 Apl.T`` is assembled by a per-landmark SCATTER: each landmark ``l``
# couples ONLY to the pose columns of the keyframes that observe it (a small set
# ``k_l`` of the ~nav_dim columns -- captured at build time, see ``lm_nav_cols``),
# so its contribution is a tiny ``(k_l x k_l)`` outer product scattered into ``S``
# rather than a dense ``nav_dim``-wide update. That makes forming ``S`` / ``rp``
# O(sum_l k_l^2) instead of O(nav_dim^2 * M).
#
# The DENSE-einsum form (``_schur_reduce_dense``) is kept as the algebraic
# reference the Schur equivalence gate compares against.
# --------------------------------------------------------------------------- #
def _schur_partition(A: np.ndarray, g: np.ndarray, nav_dim: int, M: int):
    """Split the damped system into nav/landmark blocks + batched All^-1."""
    App = A[:nav_dim, :nav_dim]
    Apl = A[:nav_dim, nav_dim:]                       # (nav_dim, 3M)
    gp = g[:nav_dim]
    gl = g[nav_dim:].reshape(M, 3)                    # (M,3)
    # block-diagonal All -> (M,3,3) stack; batched 3x3 inverse (host LAPACK)
    All_blk = A[nav_dim:, nav_dim:].reshape(M, 3, M, 3)
    All_blk = All_blk[np.arange(M), :, np.arange(M), :]   # (M,3,3) diagonal
    All_inv = np.linalg.inv(All_blk)                  # (M,3,3)
    return App, Apl, gp, gl, All_inv


def _schur_reduce_dense(App, Apl, gp, gl, All_inv, nav_dim, M):
    """Dense reference reduction: S = App - Apl All^-1 Apl.T, rp = gp - ...

    Operates over the full ``nav_dim``-wide ``Apl`` (the original first-cut
    assembly). Retained as the algebraic reference for the equivalence gate.
    """
    Apl_blk = Apl.reshape(nav_dim, M, 3).transpose(1, 0, 2)   # (M,nav_dim,3)
    W = np.einsum('mnk,mkj->mnj', Apl_blk, All_inv)           # (M,nav_dim,3)
    S = App - np.einsum('mnj,mij->ni', W, Apl_blk)
    rp = gp - np.einsum('mnj,mj->n', W, gl)
    return S, rp


def _schur_solve(A: np.ndarray, g: np.ndarray, nav_dim: int, M: int,
                 lm_nav_cols: list[np.ndarray]) -> np.ndarray:
    """Solve ``A @ delta = -g`` via landmark Schur complement (exact).

    ``A`` is the FULL damped system (``ndim x ndim``), NAV block first
    (``[0, nav_dim)``) and ``M`` landmarks last (3 cols each, block-diagonal
    ``All``). ``lm_nav_cols[m]`` is the ascending array of nav columns landmark
    ``m`` couples to (its observers' pose columns). Returns ``delta`` in the
    original (nav-then-landmark) order, equal to ``np.linalg.solve(A, -g)`` up to
    floating-point round-off.
    """
    App, Apl, gp, gl, All_inv = _schur_partition(A, g, nav_dim, M)

    # --- reduced nav system by per-landmark scatter ------------------------ #
    # S = App - sum_l A_pl_l All_inv_l A_pl_l.T ; rp = gp - sum_l A_pl_l All_inv_l gl_l
    # Each landmark touches only its coupling columns k_l, so the update is a
    # tiny (k_l x k_l) outer product scattered with np.add.at (handles columns
    # shared by landmarks that co-observe a keyframe -- accumulation is additive).
    S = App.copy()
    rp = gp.copy()
    for m in range(M):
        k = lm_nav_cols[m]
        if k.size == 0:
            continue                              # landmark seen only by anchor
        A_pl = Apl[k, 3 * m:3 * m + 3]            # (k_l, 3) coupling rows
        WA = A_pl @ All_inv[m]                    # (k_l, 3) = A_pl_l All_inv_l
        np.add.at(S, (k[:, None], k[None, :]), -(WA @ A_pl.T))
        np.add.at(rp, k, -(WA @ gl[m]))

    delta_p = np.linalg.solve(S, -rp)

    # --- back-substitute per landmark -------------------------------------- #
    # dl = All^-1 (-gl - Apl.T dp) ; only the coupling columns enter Apl.T dp.
    delta_l = np.empty((M, 3))
    for m in range(M):
        k = lm_nav_cols[m]
        rhs = -gl[m]
        if k.size:
            rhs = rhs - Apl[k, 3 * m:3 * m + 3].T @ delta_p[k]
        delta_l[m] = All_inv[m] @ rhs

    delta = np.empty(A.shape[0])
    delta[:nav_dim] = delta_p
    delta[nav_dim:] = delta_l.reshape(-1)
    return delta


# --------------------------------------------------------------------------- #
# Main optimiser
# --------------------------------------------------------------------------- #
def optimize_vio(
    K: np.ndarray,
    state: VioState,
    obs_cam: np.ndarray,
    obs_lm: np.ndarray,
    obs_uv: np.ndarray,
    obs_depth: np.ndarray | None,
    imu_factors: list[tuple[int, int, ImuPreintegration]],
    g_world: np.ndarray,
    cfg: VioConfig | None = None,
    anchor: int = 0,
    icp_factors: list[IcpFactor] | None = None,
    _solve_probe=None,
) -> VioResult:
    """Jointly refine poses, velocities, biases and landmarks.

    obs_cam/obs_lm : (N,) int keyframe / landmark index per observation.
    obs_uv         : (N,2) measured pixels.
    obs_depth      : (N,) metric depth (m), <=0 means none, or None to disable.
    imu_factors    : consecutive-keyframe preintegration factors (i, j, pre).
    g_world        : (3,) gravity ACCELERATION vector in the world frame
                     (e.g. optical-down [0, +9.81, 0]).
    icp_factors    : OPT-IN dense-ICP relative-pose factors (:class:`IcpFactor`),
                     each pinning the translation increment between two window
                     keyframes. Default ``None``/empty -> the ICP loops are
                     skipped and H/b/cost are bit-identical to the no-ICP build.
                     Honoured only when ``cfg.icp_factor`` is True (the caller
                     sets both together); an empty list with the flag on is a
                     no-op too.
    _solve_probe   : debug-only hook ``fn(A, b, delta)`` invoked on every inner
                     LM damping-retry solve (used by the Schur equivalence gate
                     to compare against the dense reference). ``None`` in
                     production -> zero cost.
    """
    cfg = cfg or VioConfig()
    # ICP factors are honoured only when the flag is on AND the list is non-empty;
    # either condition false leaves ``icp_list`` empty so every ICP loop below is
    # skipped -> the assembled H/b/cost stay bit-identical to the no-ICP build.
    icp_list = (list(icp_factors) if (cfg.icp_factor and icp_factors) else [])
    st = state.copy()
    fx, fy, cx, cy = (float(K[0, 0]), float(K[1, 1]),
                      float(K[0, 2]), float(K[1, 2]))
    nC = len(st.R)
    M = st.landmarks.shape[0]
    g_world = np.asarray(g_world, np.float64)
    obs_cam = np.asarray(obs_cam, np.int64)
    obs_lm = np.asarray(obs_lm, np.int64)
    obs_uv = np.asarray(obs_uv, np.float64)
    use_depth = bool(cfg.use_depth and obs_depth is not None)
    obs_depth = (np.asarray(obs_depth, np.float64) if use_depth
                 else np.zeros(obs_cam.shape[0]))

    # --- Phase-4 excitation-gated ZUPT pre-pass (opt-in) -------------------- #
    # ZUPT anchors v_i ~= 0 ONLY on a keyframe whose INBOUND IMU edge is
    # low-excitation (near rest). The gate is a property of the edge (raw
    # preintegrated increment), so compute it ONCE here from imu_factors rather
    # than per LM iteration. ``zupt_on[ci]`` is True iff KF ci should be pinned.
    # KF0 (no inbound edge) is never gated on. Gate on EXCITATION, not on the
    # translation seed.
    #
    # Accelerometer excitation must be measured GRAVITY-AWARE: ``pre.dv`` is the
    # preintegrated SPECIFIC FORCE (it still contains gravity), so at true rest
    # ``||pre.dv||/dt`` equals the gravity magnitude |g| (~9.8), NOT zero -- the
    # accelerometer reads +g upward even when motionless. The real linear-
    # acceleration excitation is therefore the DEVIATION of that specific-force
    # magnitude from |g|: ``a_exc = | ||pre.dv||/dt - |g| |``. This is frame-
    # independent (pure magnitudes), so it needs no body->world rotation: at rest
    # a_exc ~= 0, under a real push or shake the specific-force magnitude departs
    # from |g| and a_exc grows. The gyro rate ``w_exc = ||log(pre.dR)||/dt`` is
    # already gravity-free. Both small == rest -> ZUPT on (absolute anchor); shake
    # -> high excitation -> ZUPT off so the CV prior carries the window instead.
    zupt_on = np.zeros(nC, dtype=bool)
    if cfg.vel_zupt:
        g_mag = float(np.linalg.norm(g_world))
        for (_i, j, pre) in imu_factors:
            dt_edge = float(pre.dt)
            if dt_edge <= 1e-9:
                continue
            a_exc = abs(float(np.linalg.norm(pre.dv)) / dt_edge - g_mag)
            w_exc = float(np.linalg.norm(so3_log(pre.dR))) / dt_edge
            if a_exc < cfg.zupt_accel_thresh and w_exc < cfg.zupt_gyro_thresh:
                zupt_on[j] = True

    # --- Absolute-velocity gauge anchor pre-pass (TIGHT-ONLY, un-gated) ------ #
    # The anchor needs each keyframe's inbound IMU edge (i_prev -> ci) so it can
    # forward-predict ``v_pred_ci = v_{i_prev} + g dt + R_{i_prev} dv`` from the
    # CURRENT neighbour estimate every iteration. Capture (i_prev, pre) per KF
    # ONCE here (a function of the edge topology, not the iterate); a KF without an
    # inbound edge (the window anchor / a gap) gets ``None`` and is pinned toward
    # its OWN current velocity instead (a prior at the linearisation point). The
    # prior is read ONLY on the tight path (imu_info_weight) -- the marker the
    # --tight builder sets -- so the loose / oracle build never assembles it and
    # stays byte-identical. (A degenerate edge with ``dt <= 0`` falls back to the
    # self-prediction too.)
    vabs_on = bool(cfg.vel_abs_prior and cfg.imu_info_weight and imu_factors)
    vabs_inbound: list = [None] * nC
    if vabs_on:
        for (i, j, pre) in imu_factors:
            if float(pre.dt) > 1e-9:
                vabs_inbound[j] = (i, pre)

    # --- column layout -----------------------------------------------------
    # tilt-lock: pose has 4 free DoF (3 translation + 1 yaw) instead of 6, with
    # the yaw perturbation about the world-vertical (gravity) axis.
    lock_tilt = bool(cfg.lock_tilt)
    pose_dof = 4 if lock_tilt else 6
    up_axis = None
    if lock_tilt:
        gn = float(np.linalg.norm(g_world))
        up_axis = (g_world / gn) if gn > 1e-9 else np.array([0.0, 1.0, 0.0])

    pose_col = np.full(nC, -1, np.int64)
    vel_col = np.zeros(nC, np.int64)
    bg_col = np.zeros(nC, np.int64)
    ba_col = np.zeros(nC, np.int64)
    n = 0
    for i in range(nC):
        if i != anchor:
            pose_col[i] = n
            n += pose_dof
    for i in range(nC):
        vel_col[i] = n; n += 3
        bg_col[i] = n; n += 3
        ba_col[i] = n; n += 3
    # The NAV block (free poses + vel + bg + ba) is assigned FIRST and is
    # contiguous [0, nav_dim); the LANDMARK block (3*M) is assigned LAST and is
    # contiguous [nav_dim, ndim). The Schur complement (see ``_schur_solve``)
    # relies on exactly this nav-then-landmark partition.
    nav_dim = n
    lm_col = np.zeros(M, np.int64)
    for m in range(M):
        lm_col[m] = n; n += 3
    ndim = n
    eps = cfg.fd_eps
    N = obs_cam.shape[0]
    sigma_px = cfg.sigma_px
    huber_px = cfg.huber_px
    depth_huber = cfg.depth_huber

    # --- njit IMU-factor Jacobian gate (TIGHT-ONLY, DEFAULT ON) ------------- #
    # The finite-difference IMU-factor Jacobian is the tight stage's wall, so it
    # is hoisted into ``imu_factor_jacobian`` (sky.vio.imu_factor_numba). Enter it
    # ONLY on the live tight config -- the defining marker is the covariance-
    # correct IMU weight ``imu_info_weight`` (set by the --tight builder together
    # with lock_tilt + use_imu). The loose / oracle path (imu_info_weight=False)
    # and the no-numba build keep the UNCHANGED pure-Python FD loop.
    #
    # DEFAULT ON (safety-reviewer approved, 2026-06-16), COUPLED TO THE GUARD.
    # The kernel's H/b match the pure-Python build to the fp ROUND-OFF FLOOR (rel
    # ~7e-11). The tight LM solve branches discretely on cost, so that round-off
    # USED to flip an iteration and walk a DIVERGENT window elsewhere (push_shake
    # diverged ~600 mm njit-vs-pure). That chaos came ENTIRELY from the divergent
    # window the LM accepted -- with the divergence guard (``divergence_guard``,
    # WindowedVIOConfig) REJECTING that window DETERMINISTICALLY (regardless of
    # njit-vs-pure round-off), the chaos source is gone: the njit-vs-pure
    # full-session ATE is EQUAL (sub-mm) on EVERY gold session INCLUDING shake
    # (verification/imu_factor_njit_ate.py PASS on all gold). So the kernel is
    # validated ONLY with the guard ON.
    #
    # Gate (in precedence order):
    #   * ``SKY_VIO_IMU_NJIT=0`` -> force OFF (the explicit kill switch, always
    #     honoured: a no-numba / debug / A-B build can pin the pure-Python path).
    #   * GUARD-COUPLING SAFETY: if the divergence guard is OFF
    #     (``cfg.njit_guard_ok`` False, set by ``WindowedVIOMap.run_ba`` from
    #     ``divergence_guard``), force the kernel OFF and log why -- njit
    #     determinism is only validated WITH the guard; running it guard-off
    #     re-opens the round-off-chaos divergence. Safety wins over the default
    #     (and over an explicit ``=1``).
    #   * otherwise DEFAULT ON (env unset) -- the validated production path.
    # ``HAVE_NUMBA`` False (no-numba host) keeps the pure-Python build regardless.
    njit_env = os.environ.get("SKY_VIO_IMU_NJIT")
    njit_force_off = (njit_env == "0")
    njit_guard_ok = bool(cfg.njit_guard_ok)
    if (HAVE_NUMBA and cfg.imu_info_weight and imu_factors
            and not njit_force_off and not njit_guard_ok):
        import sys
        print("[vio] njit IMU-factor kernel DISABLED: divergence_guard is OFF "
              "(njit determinism is only validated with the guard ON; running "
              "the pure-Python FD build instead)", file=sys.stderr)
    use_imu_njit = bool(HAVE_NUMBA and cfg.imu_info_weight
                        and bool(imu_factors)
                        and not njit_force_off and njit_guard_ok)
    # The kernel needs a concrete up-axis array even in full 6-DoF mode (where the
    # pure-Python path passes ``up_axis=None``); a zeros(3) placeholder is never
    # read in that branch (lock_tilt selects the full-DoF retraction).
    up_axis_arr = (np.asarray(up_axis, np.float64) if up_axis is not None
                   else np.zeros(3))

    # --- precomputed constants for the vectorised factor assembly ----------
    # The projection + depth factors used to be a scalar Python loop over every
    # observation, each finite-differencing one column at a time. Run from the
    # background VIO worker that loop held the GIL for the whole ~100ms solve and
    # starved the realtime camera loop (frame drops -> feature loss on fast
    # motion). Here the identical finite-difference math is computed with batched
    # numpy ops (which release the GIL), and the per-observation J^T J / J^T r
    # blocks are scattered with np.add.at. Mirrors the BA/PGO vectorisations.
    depth_mask = ((obs_depth > 0) if use_depth
                  else np.zeros(N, dtype=bool))
    # safe per-observation depth sigma (1.0 where no depth; that row is masked)
    sz_all = cfg.depth_sigma_coeff * np.where(depth_mask, obs_depth, 1.0) ** 2
    lm_base = lm_col[obs_lm]                 # (N,) landmark column base
    pose_base_all = pose_col[obs_cam]        # (N,) pose column base, -1 anchor
    free_obs = pose_base_all >= 0
    pose_base_free = pose_base_all[free_obs]
    lm_base_free = lm_base[free_obs]
    ar_pose = np.arange(pose_dof, dtype=np.int64)
    ar3 = np.arange(3, dtype=np.int64)

    # --- Schur scatter bookkeeping -----------------------------------------
    # Per landmark, the EXACT set of nav columns it couples to in ``Apl``: a
    # landmark only writes the pose-landmark cross block (build_system, lines
    # ~738) for the FREE (non-anchor) keyframes that observe it -- velocity /
    # bias / IMU / ICP columns never touch a landmark column. So ``Apl_blk[m]``
    # is non-zero ONLY on those keyframes' pose columns. We capture, per
    # landmark, the sorted array of those nav-column indices ONCE here (it is a
    # function of the observation topology + column layout, not of the iterate)
    # and hand it to ``_schur_solve`` so the reduced system is assembled by a
    # per-landmark SCATTER over its few coupling columns -- O(M * k^2) -- instead
    # of a dense ``Apl @ All^-1 @ Apl.T`` over the full nav width. The DENSE
    # einsum form stays the algebraic reference (Schur equivalence gate).
    lm_nav_cols: list[np.ndarray] = [
        np.empty(0, np.int64) for _ in range(M)]
    if M and pose_base_free.size:
        # distinct (landmark, pose-base) pairs among free observations
        pair = np.stack([obs_lm[free_obs], pose_base_free], axis=1)
        pair = np.unique(pair, axis=0)                  # sorted by (lm, base)
        # expand each distinct pose base to its pose_dof contiguous columns
        cols = (pair[:, 1, None] + ar_pose[None, :]).reshape(-1)   # (P*dof,)
        lm_of = np.repeat(pair[:, 0], pose_dof)                    # (P*dof,)
        # split the (already lm-sorted) column list per landmark
        order = np.argsort(lm_of, kind="stable")
        lm_of, cols = lm_of[order], cols[order]
        bounds = np.searchsorted(lm_of, np.arange(M + 1))
        for m in range(M):
            seg = cols[bounds[m]:bounds[m + 1]]
            lm_nav_cols[m] = np.sort(seg)               # ascending nav cols
    # Constant finite-difference rotation increments: the same pose DoF
    # perturbation is applied to every observation, so build the Exp(eps) 3x3
    # rotation(s) once (identical to the scalar _pose_perturb step).
    if lock_tilt:
        dR_yaw = so3_exp(up_axis * eps)
    else:
        dR_rot = np.stack([so3_exp(np.eye(3)[a] * eps) for a in range(3)])
    fd_tiny = 1e-300

    # --- batched residual evaluators ---------------------------------------
    def _proj_uvz(Rb, pb, lmb):
        """Vectorised _project over a batch of (R, p, landmark)."""
        d = lmb - pb
        Xc = np.einsum('nki,nk->ni', Rb, d)          # R^T @ (Xw - p)
        Z = Xc[:, 2]
        Zc = np.where(Z > cfg.min_view_z, Z, cfg.min_view_z)
        u = fx * Xc[:, 0] / Zc + cx
        v = fy * Xc[:, 1] / Zc + cy
        return u, v, Z

    def _rows(Rb, pb, lmb):
        """Raw (unweighted) residual rows (N,3); depth row 0 where absent."""
        u, v, Z = _proj_uvz(Rb, pb, lmb)
        r = np.empty((N, 3))
        r[:, 0] = (u - obs_uv[:, 0]) / sigma_px
        r[:, 1] = (v - obs_uv[:, 1]) / sigma_px
        r[:, 2] = np.where(depth_mask, (Z - obs_depth) / sz_all, 0.0)
        return r

    def _gather(stt):
        R_arr = np.array(stt.R) if nC else np.zeros((0, 3, 3))
        p_arr = np.array(stt.p) if nC else np.zeros((0, 3))
        return R_arr[obs_cam], p_arr[obs_cam], stt.landmarks[obs_lm]

    def total_cost(stt) -> tuple[float, float]:
        cost = 0.0
        mean_e = 0.0
        if N:
            R_obs, p_obs, lm_obs = _gather(stt)
            r = _rows(R_obs, p_obs, lm_obs)
            e_px = np.hypot(r[:, 0], r[:, 1]) * sigma_px
            w = np.where(e_px <= huber_px, 1.0,
                         huber_px / np.maximum(e_px, fd_tiny))
            cost += 0.5 * float(np.sum(w * (r[:, 0] ** 2 + r[:, 1] ** 2)))
            az = np.abs(r[:, 2])
            thr = depth_huber / sz_all
            dcost = np.where(az <= thr, 0.5 * r[:, 2] ** 2,
                             thr * (az - 0.5 * thr))
            cost += float(np.sum(np.where(depth_mask, dcost, 0.0)))
            mean_e = float(np.mean(e_px))
        for (i, j, pre) in imu_factors:
            ri = _imu_residual(stt.R[i], stt.p[i], stt.v[i], stt.bg[i], stt.ba[i],
                               stt.R[j], stt.p[j], stt.v[j], pre, g_world, cfg)
            rb = _bias_rw_residual(stt.bg[i], stt.ba[i], stt.bg[j], stt.ba[j], cfg)
            cost += 0.5 * float(ri @ ri + rb @ rb)
            if cfg.vel_cv_prior:
                # constant-velocity smoothness prior on this edge (see VioConfig)
                r_cv = (stt.v[j] - stt.v[i]) / cfg.sigma_vel_cv
                cost += 0.5 * float(r_cv @ r_cv)
        if cfg.vel_zupt:
            # excitation-gated zero-velocity prior (absolute anchor on rest KFs)
            for ci in range(nC):
                if zupt_on[ci]:
                    r_z = stt.v[ci] / cfg.sigma_vel_zupt
                    cost += 0.5 * float(r_z @ r_z)
        if vabs_on:
            # absolute-velocity GAUGE anchor on every KF (un-gated; tight only)
            inv_s = 1.0 / cfg.sigma_vabs
            for ci in range(nC):
                v_pred = _v_pred_inbound(stt, vabs_inbound, ci, g_world)
                r_va = (stt.v[ci] - v_pred) * inv_s
                cost += 0.5 * float(r_va @ r_va)
        # Dense-ICP relative-pose factors (opt-in; empty list -> no contribution).
        # Factor-level Huber on the whitened residual norm so a single bad ICP
        # match cannot dominate the window cost.
        for f in icp_list:
            r_icp = _icp_residual(stt.R[f.i], stt.p[f.i], stt.R[f.j], stt.p[f.j],
                                  f.T_icp_ij, f.Omega_icp)
            e = float(np.linalg.norm(r_icp))
            thr = cfg.icp_huber
            if e <= thr:
                cost += 0.5 * float(r_icp @ r_icp)
            else:
                cost += thr * (e - 0.5 * thr)
        return cost, mean_e

    # --- one Gauss-Newton/LM linear system ---------------------------------
    def build_system(stt):
        H = np.zeros((ndim, ndim))
        b = np.zeros(ndim)

        # projection + depth factors (vectorised over all observations)
        if N:
            R_obs, p_obs, lm_obs = _gather(stt)
            ncol = pose_dof + 3
            r0 = _rows(R_obs, p_obs, lm_obs)
            J = np.empty((N, 3, ncol))
            col = 0
            # pose translation columns: p <- p + R @ (eps e_d) = p + eps R[:,d]
            for dax in range(3):
                pp = p_obs + eps * R_obs[:, :, dax]
                J[:, :, col] = (_rows(R_obs, pp, lm_obs) - r0) / eps
                col += 1
            # pose rotation column(s): R <- R Exp(eps e_a) (or yaw-only locked)
            if lock_tilt:
                Rp = np.einsum('ij,njk->nik', dR_yaw, R_obs)
                J[:, :, col] = (_rows(Rp, p_obs, lm_obs) - r0) / eps
                col += 1
            else:
                for a in range(3):
                    Rp = np.einsum('nij,jk->nik', R_obs, dR_rot[a])
                    J[:, :, col] = (_rows(Rp, p_obs, lm_obs) - r0) / eps
                    col += 1
            # landmark columns
            for dax in range(3):
                lp = lm_obs.copy()
                lp[:, dax] += eps
                J[:, :, col] = (_rows(R_obs, p_obs, lp) - r0) / eps
                col += 1

            # IRLS robust sqrt-weights from the current residual (held fixed
            # across this linearisation), identical to the scalar version.
            r = r0.copy()
            e_px = np.hypot(r0[:, 0], r0[:, 1]) * sigma_px
            sw = np.where(e_px <= huber_px, 1.0,
                          np.sqrt(huber_px / np.maximum(e_px, fd_tiny)))
            r[:, 0] *= sw
            r[:, 1] *= sw
            J[:, 0, :] *= sw[:, None]
            J[:, 1, :] *= sw[:, None]
            az = np.abs(r0[:, 2])
            thr = depth_huber / sz_all
            dw = np.where(az <= thr, 1.0,
                          np.sqrt(thr / np.maximum(az, 1e-12)))
            dw = np.where(depth_mask, dw, 1.0)
            r[:, 2] *= dw
            J[:, 2, :] *= dw[:, None]

            Jp = J[:, :, :pose_dof]
            Jl = J[:, :, pose_dof:]

            # landmark-landmark block + landmark rhs (every observation)
            Hll = np.einsum('nri,nrj->nij', Jl, Jl)
            bl = np.einsum('nri,nr->ni', Jl, r)
            lm_rows = (lm_base[:, None, None] + ar3[None, :, None]
                       + np.zeros((1, 1, 3), np.int64))
            lm_cols = (lm_base[:, None, None] + ar3[None, None, :]
                       + np.zeros((1, 3, 1), np.int64))
            np.add.at(H, (lm_rows.ravel(), lm_cols.ravel()), Hll.ravel())
            np.add.at(b, (lm_base[:, None] + ar3[None, :]).ravel(), bl.ravel())

            # pose blocks (only free, non-anchor poses)
            if pose_base_free.size:
                Jpf = Jp[free_obs]
                Jlf = Jl[free_obs]
                rf = r[free_obs]
                Hpp = np.einsum('nri,nrj->nij', Jpf, Jpf)
                Hpl = np.einsum('nri,nrj->nij', Jpf, Jlf)
                bp = np.einsum('nri,nr->ni', Jpf, rf)
                pp_rows = (pose_base_free[:, None, None]
                           + ar_pose[None, :, None]
                           + np.zeros((1, 1, pose_dof), np.int64))
                pp_cols = (pose_base_free[:, None, None]
                           + ar_pose[None, None, :]
                           + np.zeros((1, pose_dof, 1), np.int64))
                np.add.at(H, (pp_rows.ravel(), pp_cols.ravel()), Hpp.ravel())
                np.add.at(b, (pose_base_free[:, None]
                              + ar_pose[None, :]).ravel(), bp.ravel())
                # pose-landmark cross blocks (+ symmetric transpose)
                pl_rows = (pose_base_free[:, None, None]
                           + ar_pose[None, :, None]
                           + np.zeros((1, 1, 3), np.int64))
                pl_cols = (lm_base_free[:, None, None]
                           + ar3[None, None, :]
                           + np.zeros((1, pose_dof, 1), np.int64))
                # H[pose+a, lm+b] += Hpl[n,a,b]; the symmetric H[lm+b, pose+a]
                # gets the SAME value (Jl^T Jp)[b,a] == Hpl[n,a,b], so reuse the
                # ravel with the row/col index arrays swapped (do NOT transpose
                # the value array -- its [n,a,b] order must match the indices).
                np.add.at(H, (pl_rows.ravel(), pl_cols.ravel()), Hpl.ravel())
                np.add.at(H, (pl_cols.ravel(), pl_rows.ravel()), Hpl.ravel())

        # IMU + bias-rw factors (few factors -> scalar FD, but evaluated on the
        # 10 nav vectors of the two adjacent keyframes directly: no VioState /
        # landmark copies per perturbation column, so the per-column GIL-held
        # Python work is minimal).
        def _imu_eval(pre, Ri, pi, vi, bgi, bai, Rj, pj, vj, bgj, baj):
            ri = _imu_residual(Ri, pi, vi, bgi, bai, Rj, pj, vj,
                               pre, g_world, cfg)
            rb = _bias_rw_residual(bgi, bai, bgj, baj, cfg)
            if cfg.vel_cv_prior:
                # CRITICAL: the constant-velocity prior is appended to the STACKED
                # residual here, NOT folded into _imu_residual -- folding it in
                # would desync the 9x9 pre.sqrt_info whitening (covariance path).
                # As 3 extra rows on the end, the existing per-edge FD-Jacobian
                # loop (which already perturbs the vi/vj velocity blocks) fills the
                # r_cv columns automatically; no new Jacobian code, same H/b
                # assembly. r_cv = (v_j - v_i) / sigma_vel_cv, isotropic.
                r_cv = (vj - vi) / cfg.sigma_vel_cv
                return np.concatenate([ri, rb, r_cv])
            return np.concatenate([ri, rb])

        def _imu_factor_jac_py(pre, i, j, i_free, j_free):
            """Pure-Python per-edge FD residual ``r0i`` + Jacobian ``Ji``.

            The original scalar build, factored out so it serves both the
            non-tight / no-numba fallback AND the degenerate (sqrt_info=None) edge
            on the tight path. Column order matches the ``idx`` assembly above.
            """
            base_vals = [stt.R[i], stt.p[i], stt.v[i], stt.bg[i], stt.ba[i],
                         stt.R[j], stt.p[j], stt.v[j], stt.bg[j], stt.ba[j]]
            r0i = _imu_eval(pre, *base_vals)
            rows = r0i.shape[0]
            blocks = []
            if i_free:
                blocks.append(("pose", i))
            blocks.append(("vel", i))
            blocks.append(("bg", i))
            blocks.append(("ba", i))
            if j_free:
                blocks.append(("pose", j))
            blocks.append(("vel", j))
            blocks.append(("bg", j))
            blocks.append(("ba", j))
            ncol = sum(pose_dof if k == "pose" else 3 for k, _ in blocks)
            Ji = np.zeros((rows, ncol))
            base_slot = {i: 0, j: 5}
            kind_off = {"pose": 0, "vel": 2, "bg": 3, "ba": 4}
            col = 0
            for kind, vk in blocks:
                s0 = base_slot[vk]
                size = pose_dof if kind == "pose" else 3
                for d in range(size):
                    vals = list(base_vals)
                    if kind == "pose":
                        dd = np.zeros(pose_dof); dd[d] = eps
                        vals[s0], vals[s0 + 1] = _pose_perturb(
                            stt.R[vk], stt.p[vk], dd, up_axis)
                    else:
                        si = s0 + kind_off[kind]
                        v = vals[si].copy(); v[d] += eps; vals[si] = v
                    Ji[:, col] = (_imu_eval(pre, *vals) - r0i) / eps
                    col += 1
            return r0i, Ji

        for (i, j, pre) in imu_factors:
            i_free = pose_col[i] >= 0
            j_free = pose_col[j] >= 0
            # Column index set in the SAME order as the FD column order: i-side
            # [pose(if free) + vel + bg + ba] then j-side [pose(if free) + vel +
            # bg + ba]. Identical to the pure-Python ``blocks`` ordering.
            idx = []
            if i_free:
                idx.extend(range(pose_col[i], pose_col[i] + pose_dof))
            idx.extend(range(vel_col[i], vel_col[i] + 3))
            idx.extend(range(bg_col[i], bg_col[i] + 3))
            idx.extend(range(ba_col[i], ba_col[i] + 3))
            if j_free:
                idx.extend(range(pose_col[j], pose_col[j] + pose_dof))
            idx.extend(range(vel_col[j], vel_col[j] + 3))
            idx.extend(range(bg_col[j], bg_col[j] + 3))
            idx.extend(range(ba_col[j], ba_col[j] + 3))
            idx = np.asarray(idx, np.int64)

            if use_imu_njit:
                # TIGHT-ONLY njit FD Jacobian. ``pre.sqrt_info`` is non-None on the
                # info-weighted path (a degenerate edge with sqrt_info=None falls
                # back to the pure-Python build below). Pack raw float64 arrays
                # only -- no Python object crosses into the kernel.
                if pre.sqrt_info is None:
                    r0i, Ji = _imu_factor_jac_py(pre, i, j, i_free, j_free)
                else:
                    nrows = 18 if cfg.vel_cv_prior else 15
                    r0i, Ji = imu_factor_jacobian(
                        np.ascontiguousarray(stt.R[i]), stt.p[i], stt.v[i],
                        stt.bg[i], stt.ba[i],
                        np.ascontiguousarray(stt.R[j]), stt.p[j], stt.v[j],
                        stt.bg[j], stt.ba[j],
                        pre.dR, pre.dv, pre.dp, float(pre.dt),
                        pre.dR_dbg, pre.dv_dbg, pre.dv_dba,
                        pre.dp_dbg, pre.dp_dba, pre.bg, pre.ba, pre.sqrt_info,
                        cfg.sigma_rot, cfg.sigma_vel, cfg.sigma_pos,
                        cfg.sigma_bg_rw, cfg.sigma_ba_rw, g_world,
                        True, bool(cfg.vel_cv_prior), cfg.sigma_vel_cv,
                        eps, lock_tilt, up_axis_arr, pose_dof,
                        i_free, j_free, idx.shape[0], nrows)
            else:
                r0i, Ji = _imu_factor_jac_py(pre, i, j, i_free, j_free)

            H[np.ix_(idx, idx)] += Ji.T @ Ji
            b[idx] += Ji.T @ r0i

        # Phase-4 excitation-gated ZUPT (opt-in): a velocity-only absolute prior
        # r_zupt = v_i / sigma_vel_zupt on each gated KF. The block is ANALYTIC
        # (J = d r_zupt / d v_i = I / sigma_vel_zupt): the Gauss-Newton normal
        # equations contribute J^T J = I / sigma^2 to H[vel_i, vel_i] and
        # J^T r = v_i / sigma^2 to b[vel_i]. Gate is precomputed in zupt_on.
        if cfg.vel_zupt:
            inv_var_z = 1.0 / (cfg.sigma_vel_zupt * cfg.sigma_vel_zupt)
            for ci in range(nC):
                if not zupt_on[ci]:
                    continue
                vc = vel_col[ci]
                vrange = slice(vc, vc + 3)
                H[vrange, vrange] += np.eye(3) * inv_var_z
                b[vc:vc + 3] += stt.v[ci] * inv_var_z

        # Absolute-velocity GAUGE anchor (tight only, un-gated across KFs). The
        # residual r_vabs = (v_ci - v_pred_ci) / sigma_vabs treats v_pred as a
        # CONSTANT pseudo-measurement (recomputed each iteration from the current
        # neighbour estimate), so the Jacobian wrt v_ci is the absolute self-term
        # J = I / sigma_vabs -> the Gauss-Newton normal equations contribute
        # J^T J = I / sigma_vabs^2 to H[vel_ci, vel_ci] and J^T r = (v_ci - v_pred)
        # / sigma_vabs^2 to b[vel_ci]. This is the analytic, exact ZUPT-shaped
        # block; it restores the rank-3 absolute-velocity null space.
        if vabs_on:
            inv_var_va = 1.0 / (cfg.sigma_vabs * cfg.sigma_vabs)
            for ci in range(nC):
                v_pred = _v_pred_inbound(stt, vabs_inbound, ci, g_world)
                vc = vel_col[ci]
                vrange = slice(vc, vc + 3)
                H[vrange, vrange] += np.eye(3) * inv_var_va
                b[vc:vc + 3] += (stt.v[ci] - v_pred) * inv_var_va

        # Dense-ICP relative-pose factors (opt-in; empty list -> nothing added).
        # POSE-ONLY: each factor differentiates only the two keyframe POSE blocks
        # (i, j), so -- mirroring the IMU edge -- it scatters into H[pose,pose] and
        # b[pose] without touching velocity / bias / landmark columns (no
        # landmark-Schur coupling). The Jacobian is finite-differenced through the
        # SAME ``_pose_perturb`` the IMU edge uses, so tilt-lock (pose_dof=4, yaw-
        # about-vertical) is handled identically and automatically. Omega_icp is
        # fixed (computed once per factor), so the residual is differentiable.
        for f in icp_list:
            r0i = _icp_residual(stt.R[f.i], stt.p[f.i], stt.R[f.j], stt.p[f.j],
                                f.T_icp_ij, f.Omega_icp)
            # IRLS Huber sqrt-weight on the whitened residual norm (held fixed
            # across this linearisation, like the reproj/depth robust weights).
            e = float(np.linalg.norm(r0i))
            sw = 1.0 if e <= cfg.icp_huber else np.sqrt(cfg.icp_huber / e)

            blocks = []
            if pose_col[f.i] >= 0:
                blocks.append((f.i, pose_col[f.i]))
            if pose_col[f.j] >= 0:
                blocks.append((f.j, pose_col[f.j]))
            if not blocks:
                continue                      # both poses anchored -> nothing free
            idx = []
            for _, base in blocks:
                idx.extend(range(base, base + pose_dof))
            idx = np.asarray(idx, np.int64)
            Ji = np.zeros((r0i.shape[0], idx.shape[0]))
            col = 0
            for ci, _base in blocks:
                R_c, p_c = stt.R[ci], stt.p[ci]
                for d in range(pose_dof):
                    dd = np.zeros(pose_dof); dd[d] = eps
                    Rp, pp = _pose_perturb(R_c, p_c, dd, up_axis)
                    if ci == f.i:
                        rp = _icp_residual(Rp, pp, stt.R[f.j], stt.p[f.j],
                                           f.T_icp_ij, f.Omega_icp)
                    else:
                        rp = _icp_residual(stt.R[f.i], stt.p[f.i], Rp, pp,
                                           f.T_icp_ij, f.Omega_icp)
                    Ji[:, col] = (rp - r0i) / eps
                    col += 1
            Ji *= sw
            r0w = r0i * sw
            H[np.ix_(idx, idx)] += Ji.T @ Ji
            b[idx] += Ji.T @ r0w

        return H, b

    def retract(stt, delta):
        out = stt.copy()
        for i in range(nC):
            if pose_col[i] >= 0:
                dd = delta[pose_col[i]:pose_col[i] + pose_dof]
                out.R[i], out.p[i] = _pose_perturb(stt.R[i], stt.p[i], dd,
                                                   up_axis)
            out.v[i] = stt.v[i] + delta[vel_col[i]:vel_col[i] + 3]
            out.bg[i] = stt.bg[i] + delta[bg_col[i]:bg_col[i] + 3]
            out.ba[i] = stt.ba[i] + delta[ba_col[i]:ba_col[i] + 3]
        for m in range(M):
            out.landmarks[m] = stt.landmarks[m] + delta[lm_col[m]:lm_col[m] + 3]
        return out

    # --- LM loop -----------------------------------------------------------
    # Schur gate: marginalise the landmark block ONLY on the tight VIO solve
    # (defined by the presence of IMU factors -- the loose/oracle BA path uses
    # ``sky.backend.windowed`` and never calls this with IMU factors). The result
    # is algebraically identical to the dense solve; this branch only changes
    # WHICH matrix ``np.linalg.solve`` factorises (the ~nav_dim Schur reduction
    # vs the full ndim). ``M > 0`` guards the degenerate no-landmark window.
    use_schur = bool(imu_factors) and M > 0
    # Absolute Tikhonov floor on the NAV block (TIGHT-ONLY). ``tau_nav_diag`` is
    # ``tau_nav`` on the first ``nav_dim`` (pose+vel+bg+ba) columns and ZERO on the
    # landmark columns, so adding ``np.diag(tau_nav_diag)`` to the FULL damped A
    # leaves the block-diagonal landmark Schur block ``All`` untouched (the exact
    # Schur path consumes the conditioned A unchanged). Applied only on the tight
    # marker ``imu_info_weight`` (and tau_nav>0): the loose / oracle LM solve adds
    # nothing -> byte-identical. Built once (constant across iterations).
    use_tau_nav = bool(cfg.imu_info_weight and cfg.tau_nav > 0.0 and nav_dim > 0)
    tau_nav_diag = None
    if use_tau_nav:
        tau_nav_diag = np.zeros(ndim)
        tau_nav_diag[:nav_dim] = cfg.tau_nav
    cost0, _ = total_cost(st)
    cost_prev = cost0
    lam = cfg.init_lambda
    it = 0
    for it in range(cfg.max_iters):
        H, b = build_system(st)
        diag = np.clip(np.diag(H).copy(), 1e-12, None)
        solved = False
        for _ in range(12):                      # inner LM damping retries
            A = H + lam * np.diag(diag)
            if use_tau_nav:
                A = A + np.diag(tau_nav_diag)
            try:
                if use_schur:
                    delta = _schur_solve(A, b, nav_dim, M, lm_nav_cols)
                else:
                    delta = np.linalg.solve(A, -b)
            except np.linalg.LinAlgError:
                delta = np.linalg.lstsq(A, -b, rcond=None)[0]
            if _solve_probe is not None:
                _solve_probe(A, b, delta)
            trial = retract(st, delta)
            cost_new, _ = total_cost(trial)
            if cost_new < cost_prev:
                st = trial
                lam = max(cfg.min_lambda, lam * 0.5)
                improved = (cost_prev - cost_new) / max(cost_prev, 1e-15)
                cost_prev = cost_new
                solved = True
                break
            lam = min(cfg.max_lambda, lam * 4.0)
        if not solved:
            break
        if improved < cfg.rel_tol:
            break

    final_cost, mean_px = total_cost(st)
    return VioResult(state=st, iters=it + 1, cost0=cost0, cost1=final_cost,
                     mean_reproj_px=mean_px)


# --------------------------------------------------------------------------- #
# Frame conversions: pipeline T_cw (world->cam) <-> VioState body->world (R,p)
# --------------------------------------------------------------------------- #
def T_cw_to_body_world(T_cw: np.ndarray):
    """World->camera 4x4 -> body->world (R_wb, p_wb), with body == camera."""
    R_cw = T_cw[:3, :3]
    t_cw = T_cw[:3, 3]
    R_wb = R_cw.T
    p_wb = -R_cw.T @ t_cw
    return R_wb, p_wb


def body_world_to_T_cw(R_wb: np.ndarray, p_wb: np.ndarray) -> np.ndarray:
    """Inverse of :func:`T_cw_to_body_world`."""
    T = np.eye(4)
    R_cw = R_wb.T
    T[:3, :3] = R_cw
    T[:3, 3] = -R_cw @ p_wb
    return T


def _imu_segment(ts_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
                 t0: int, t1: int):
    """Clamped IMU slice for the open interval ``(t0, t1]`` with the endpoints
    linearly interpolated, so the preintegrated ``dt`` matches the real frame
    interval exactly (no sub-sample truncation at the keyframe boundaries).

    Returns ``(ts_seg, gyro_seg, accel_seg)`` or ``None`` if the interval has no
    usable samples.
    """
    t0 = int(t0); t1 = int(t1)
    if t1 <= t0 or ts_ns.size < 2:
        return None

    def interp(t):
        j = int(np.searchsorted(ts_ns, t))
        if j <= 0:
            return gyro[0], accel[0]
        if j >= ts_ns.size:
            return gyro[-1], accel[-1]
        ta, tb = int(ts_ns[j - 1]), int(ts_ns[j])
        if tb == ta:
            return gyro[j], accel[j]
        a = (t - ta) / (tb - ta)
        return (gyro[j - 1] * (1 - a) + gyro[j] * a,
                accel[j - 1] * (1 - a) + accel[j] * a)

    # interior samples strictly inside (t0, t1)
    lo = int(np.searchsorted(ts_ns, t0, side="right"))
    hi = int(np.searchsorted(ts_ns, t1, side="left"))
    g0, a0 = interp(t0)
    g1, a1 = interp(t1)
    ts_list = [t0]
    g_list = [g0]
    a_list = [a0]
    for k in range(lo, hi):
        ts_list.append(int(ts_ns[k]))
        g_list.append(gyro[k])
        a_list.append(accel[k])
    ts_list.append(t1)
    g_list.append(g1)
    a_list.append(a1)
    if len(ts_list) < 2:
        return None
    return (np.asarray(ts_list, np.int64),
            np.asarray(g_list, np.float64),
            np.asarray(a_list, np.float64))


# --------------------------------------------------------------------------- #
# Per-edge IMU preintegration cache.
#
# One :class:`_ImuEdge` is the IMU factor linking two consecutive keyframes. It
# OWNS the raw IMU segment for the inter-keyframe interval and the integrated
# :class:`ImuPreintegration` (deltas + bias Jacobians + Sigma_ij/sqrt_info). The
# cache exists so the optimiser does NOT re-integrate the raw samples every solve:
#
#   * The covariance Sigma_ij and the five bias Jacobians depend only on the raw
#     samples and the *linearisation* bias, so they are computed ONCE per edge.
#   * Inside a window solve the per-iteration bias change is absorbed by the
#     first-order ``ImuPreintegration.corrected`` update (which re-uses the cached
#     deltas + Jacobians -- no re-integration), exactly as Forster prescribes.
#   * Only when the keyframe's bias estimate drifts past ``bias_reint_thresh``
#     from the linearisation point does the edge RE-INTEGRATE (``relinearize``),
#     refreshing the deltas, Jacobians and covariance about the new bias. This is
#     the standard "relinearise when the first-order correction is stale" rule.
#
# The cache is template-similar to :class:`GyroPreintegrator` (it owns a raw IMU
# slice and lazily produces a preintegrated increment), but carries the FULL
# 9-state factor (rotation+velocity+position + covariance) rather than gyro-only
# rotation.
# --------------------------------------------------------------------------- #
class _ImuEdge:
    """Cached IMU preintegration factor between two consecutive keyframes.

    Parameters
    ----------
    seg : ``(ts_ns, gyro_cam, accel_cam)`` raw IMU samples spanning the interval,
          already rotated into the camera optical frame.
    bg0, ba0 : the gyro/accel bias linearisation point used for the integration.
    noise : optional :class:`ImuNoise` for the covariance; defaults to the
            preintegrator's own default when None.
    bias_reint_thresh : (gyro, accel) L2 distances of the live bias from the
            linearisation point beyond which the edge re-integrates. Defaults are
            generous -- the first-order ``corrected`` update is accurate well past
            the bias swings a single window solve produces.
    """

    __slots__ = ("seg", "pre", "bg_lin", "ba_lin", "noise", "_thr_g", "_thr_a")

    def __init__(self, seg, bg0, ba0, noise=None,
                 bias_reint_thresh: tuple[float, float] = (5e-3, 5e-2)):
        self.seg = seg
        self.noise = noise
        self.bg_lin = np.asarray(bg0, np.float64).copy()
        self.ba_lin = np.asarray(ba0, np.float64).copy()
        self._thr_g, self._thr_a = bias_reint_thresh
        self.pre = preintegrate_imu(seg[0], seg[1], seg[2],
                                    self.bg_lin, self.ba_lin, noise=self.noise)

    def relinearize(self, bg, ba) -> None:
        """Re-integrate about a new bias linearisation point (refresh Sigma/Jac)."""
        self.bg_lin = np.asarray(bg, np.float64).copy()
        self.ba_lin = np.asarray(ba, np.float64).copy()
        self.pre = preintegrate_imu(self.seg[0], self.seg[1], self.seg[2],
                                    self.bg_lin, self.ba_lin, noise=self.noise)

    def maybe_relinearize(self, bg, ba) -> bool:
        """Relinearise iff the live bias drifted past the threshold. Returns True
        when a re-integration actually happened (the factor changed)."""
        if (np.linalg.norm(np.asarray(bg, np.float64) - self.bg_lin) > self._thr_g
                or np.linalg.norm(np.asarray(ba, np.float64) - self.ba_lin)
                > self._thr_a):
            self.relinearize(bg, ba)
            return True
        return False


# --------------------------------------------------------------------------- #
# Windowed tight-coupled VIO map (Basalt-style sliding window)
# --------------------------------------------------------------------------- #
@dataclass
class WindowedVIOConfig:
    kf_every: int = 4            # insert a keyframe every N frames
    window: int = 8             # keyframes kept in the VIO window
    min_depth_m: float = 0.2
    max_depth_m: float = 8.0
    min_ba_views: int = 2       # landmark needs >= this many KF views
    # gravity ACCELERATION vector in the optical world frame: "down" is +y, and
    # at rest the accelerometer reads +g upward, so g_world points +y.
    g_world: tuple = (0.0, 9.81, 0.0)
    use_imu: bool = True         # set False to A/B the IMU factors (diagnostic)
    # Phase-4 single live --tight knob: when True, ``run_ba`` flips on BOTH
    # velocity-stabilisation terms (vel_cv_prior + vel_zupt) in ``self.cfg.vio``
    # via dataclasses.replace, so a caller need not reach into the nested VioConfig.
    # Default False -> the nested VioConfig flags stay OFF -> oracle byte-safe.
    stabilize_velocity: bool = False
    # Phase-4 dense-ICP relative-pose factor (OPT-IN, sibling of
    # ``stabilize_velocity``). When True, ``run_ba`` backprojects each keyframe's
    # depth into a cached cam-frame cloud and adds a dense point-to-plane ICP
    # relative-pose factor between every adjacent in-window keyframe pair, then
    # flips ``self.cfg.vio.icp_factor`` on for that solve. The factor gives the
    # inter-keyframe TRANSLATION the anchor the feature-starved 54x42 frontend
    # cannot. Default False -> no clouds cached, no factors built, vio_cfg
    # unchanged -> oracle byte-identical. Composes with ``stabilize_velocity``.
    depth_icp: bool = False
    # The windowed VIO solves each pose with roll/pitch LOCKED (lock_tilt): the
    # accelerometer owns tilt absolutely (gravity is an absolute reference), so
    # the joint solve only refines what the IMU cannot observe -- yaw + position
    # + velocity + scale. Locking tilt removes the 2 DoF where gravity used to
    # leak into a horizontal translation (corridor scale drift) and where the
    # reprojection factors could fight the IMU. The IMU vel/pos sigmas are TIGHT
    # so the accelerometer (not the blur-biased RGB-D depth) anchors metric scale.
    vio: VioConfig = field(default_factory=lambda: VioConfig(
        max_iters=12, sigma_rot=0.02, sigma_vel=0.03, sigma_pos=0.03,
        lock_tilt=True))
    # --- Divergence guard (SAFETY, tight-only) ------------------------------ #
    # The LM window solve accepts ANY step that lowers the cost: there is no
    # step-magnitude bound, no reprojection ceiling, no output sanity check. On a
    # bad frame (motion blur / feature loss) it can converge into a DIVERGENT
    # basin -- a one-window pose jump of metres (observed 4.5 m on push_shake
    # seq~95, mean reproj 71-91 px) -- and ``run_ba`` returns that pose
    # unconditionally, poisoning both the published pose AND the live VO
    # frontend (``self.vo.pose``). This guard catches a diverged solve AFTER the
    # LM converges and falls the latest keyframe back to the IMU forward-
    # prediction (dead-reckon from the previous accepted keyframe through the
    # inter-KF IMU), rejecting the whole window mutation so the next solve starts
    # from a clean seed. It is a PURE ADDITION: it fires ONLY on divergence, so a
    # well-conditioned session is bit-identical to the pre-guard solve.
    #
    # WHY THE REPROJECTION ERROR IS THE PRIMARY SIGNAL (tuning evidence)
    # -----------------------------------------------------------------
    # The post-solve mean reprojection error directly measures whether the visual
    # solve FIT its own measurements. On the live --tight path it cleanly
    # separates a divergent solve from every well-conditioned gold session:
    #     push_shake  (DIVERGENT) : up to 41 px (the catastrophic basin; the
    #                               safety report saw 71-91 px on its reduction)
    #     lab_straight (HEALTHY)  : <= 5.6 px
    #     push_fwdback (HEALTHY)  : <= 9.4 px
    #     push_straight/quick     : <= 1.3 px
    # so a single ``max_reproj_px`` ceiling above the healthy band but far below
    # the divergent regime catches shake with ZERO false positives.
    #
    # The IMU forward-prediction (dead-reckon) jump is a USEFUL second signal for
    # a catastrophic jump that LM happened to fit -- BUT on its own it FALSE-
    # POSITIVES: when the velocity/bias estimate drifts (the documented |v| ramp),
    # the IMU PREDICTION itself drifts away from a perfectly good visual solve, so
    # a raw "refined vs predicted" deviation flags a well-fit solve as divergent
    # (measured: push_fwdback fires on a 1.8->10.5 m predicted-vs-refined gap while
    # reproj stays ~1-9 px -- the SOLVE is fine, the PREDICTION drifted). A guard
    # that rejected those would replace a good visual pose with the WORSE dead-
    # reckon. So the jump test is GATED on the solve also being visually
    # questionable (reproj > ``jump_reproj_floor_px``): a large jump only counts
    # as divergence when the solve ALSO failed to fit -- exactly the shake case
    # (high jump AND high reproj together), never a clean-but-drifting prediction.
    #
    # Two trip signals (EITHER one rejects the window):
    #   (a) the post-solve mean reprojection error exceeds ``max_reproj_px``;
    #   (b) the refined latest-KF position deviates from the IMU forward-
    #       prediction by more than ``max_window_jump_m`` PLUS a margin scaled by
    #       the IMU-implied displacement (``jump_margin_frac`` * |p_pred-p_prev|,
    #       so a fast-but-REAL inertial motion is not flagged) AND the solve is
    #       visually questionable (reproj > ``jump_reproj_floor_px``).
    # Default ON: the guard is read ONLY on the tight map's ``run_ba`` (the loose
    # WindowedBAMap / oracle path never constructs a WindowedVIOConfig), so the
    # default loose / oracle build is unchanged regardless of this flag.
    divergence_guard: bool = True
    max_reproj_px: float = 20.0          # post-solve mean reproj ceiling (px):
    #                                      well above the <=9.4 px healthy band,
    #                                      below the divergent 40+ px regime.
    max_window_jump_m: float = 1.0       # absolute slack on |p_refined - p_pred|
    jump_margin_frac: float = 2.0        # + this * |IMU displacement| (scales w/
    #                                      real motion so fast-but-real is allowed)
    jump_reproj_floor_px: float = 15.0   # the jump test only fires when reproj
    #                                      also exceeds this (so a clean-but-
    #                                      drifting prediction never rejects a
    #                                      well-fit visual solve)
    # Physical speed bound that ARBITRATES the rejection FALLBACK pose. A diverged
    # window is often accompanied by a runaway velocity STATE (the documented |v|
    # ramp to 20-25 m/s on shake), so a literal IMU forward-prediction off that
    # velocity would itself jump metres -- the dead-reckon would inherit the very
    # divergence it is meant to escape. On rejection the guard therefore publishes
    # the IMU dead-reckon ONLY when it AGREES with the bounded frontend visual seed
    # to within ``max_deadreckon_speed_mps * dt`` (a physically-plausible handheld/
    # UAV inter-keyframe travel): vision-degraded-but-IMU-trustworthy (covered
    # camera, plausible speed) -> dead-reckon ~ gyro-seeded visual seed -> publish
    # the dead-reckon (intended behaviour); a runaway velocity -> dead-reckon
    # DISAGREES wildly with the bounded frontend -> keep the frontend seed so the
    # published trajectory stays FINITE. Generous (5 m/s) so it never mistakes real
    # motion for divergence -- it is a circuit-breaker, not a motion model.
    max_deadreckon_speed_mps: float = 5.0


class WindowedVIOMap:
    """Sliding-window tight-coupled VIO map (visual + IMU), tracker-agnostic.

    Mirrors :class:`sky.backend.windowed.WindowedBAMap` but feeds the raw visual
    measurements **and** IMU preintegration factors into the joint optimiser
    :func:`optimize_vio`, solving for each keyframe's pose, velocity and
    gyro/accel bias together with the landmarks. The accelerometer ties the
    keyframes through velocity/position, so an in-place rotation (true linear
    acceleration ~0) can no longer be explained away as a translation by slipped
    visual tracks -- the phantom yaw-drift that the vision-only / loosely-coupled
    paths leave behind.

    The caller supplies the full IMU stream **already rotated into the camera
    optical frame** (``gyro_cam``/``accel_cam``) at construction. Each keyframe
    carries its device-clock timestamp; the map preintegrates the IMU between
    consecutive keyframe timestamps internally.
    """

    #: Soft cap on the per-keyframe ICP cloud size (points). A 54x42 ToF frame
    #: (2268 px) is under this so it is taken whole; full-res frames are strided
    #: down to roughly this many points before the salient subset is taken.
    _ICP_CLOUD_CAP = 2500

    def __init__(self, K: np.ndarray, ts_ns: np.ndarray | None = None,
                 gyro_cam: np.ndarray | None = None,
                 accel_cam: np.ndarray | None = None,
                 bg0: np.ndarray | None = None, ba0: np.ndarray | None = None,
                 cfg: WindowedVIOConfig | None = None):
        self.K = np.asarray(K, dtype=np.float64)
        self.cfg = cfg or WindowedVIOConfig()
        # Offline: the full IMU stream is supplied up front and sliced between
        # keyframe timestamps. Live: no stream exists yet, so the caller hands
        # each keyframe's raw IMU segment to ``add_keyframe(imu_seg=...)`` and we
        # leave the stored stream empty.
        if ts_ns is None or len(ts_ns) == 0:
            self.imu_ts = np.zeros(0, np.int64)
            self.imu_gyro = np.zeros((0, 3), np.float64)
            self.imu_accel = np.zeros((0, 3), np.float64)
        else:
            order = np.argsort(ts_ns)
            self.imu_ts = np.asarray(ts_ns, np.int64)[order]
            self.imu_gyro = np.asarray(gyro_cam, np.float64)[order]
            self.imu_accel = np.asarray(accel_cam, np.float64)[order]
        self.bg0 = (np.zeros(3) if bg0 is None
                    else np.asarray(bg0, np.float64).copy())
        self.ba0 = (np.zeros(3) if ba0 is None
                    else np.asarray(ba0, np.float64).copy())
        self.g_world = np.asarray(self.cfg.g_world, np.float64)
        self.landmarks: dict[int, np.ndarray] = {}
        self.keyframes: list[dict] = []
        self.last_info: dict = {}

    def _backproject_world(self, T_cw: np.ndarray, u: float, v: float,
                           z: float) -> np.ndarray:
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        Xc = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
        R, t = T_cw[:3, :3], T_cw[:3, 3]
        return R.T @ (Xc - t)

    def add_keyframe(self, T_cw: np.ndarray, ids: np.ndarray,
                     pts: np.ndarray, depth_m: np.ndarray, ts_ns: int,
                     imu_seg: tuple[np.ndarray, np.ndarray, np.ndarray] | None
                     = None) -> None:
        """Register a keyframe (pose + track snapshot + depth + timestamp).

        ``imu_seg`` (live path) is the raw ``(ts_ns, gyro_cam, accel_cam)`` block
        of IMU samples spanning the interval since the previous keyframe, already
        rotated into the camera frame. When ``None`` (offline path) the segment
        is sliced from the stored full stream by timestamp.
        """
        h, w = depth_m.shape
        obs: dict[int, np.ndarray] = {}
        for tid, px in zip(ids, pts):
            tid = int(tid)
            u, v = float(px[0]), float(px[1])
            pu, pv = int(round(u)), int(round(v))
            if not (0 <= pu < w and 0 <= pv < h):
                continue
            z = float(depth_m[pv, pu])
            z_ok = self.cfg.min_depth_m <= z <= self.cfg.max_depth_m
            if tid not in self.landmarks:
                if not z_ok:
                    continue
                self.landmarks[tid] = self._backproject_world(T_cw, u, v, z)
            obs[tid] = np.array([u, v, z if z_ok else 0.0])

        T_cw = np.asarray(T_cw, float).copy()
        ts_ns = int(ts_ns)
        # Dense-ICP cloud cache (opt-in): backproject this keyframe's depth into
        # its CAMERA frame ONCE here, so ``run_ba`` reuses it every solve instead
        # of re-backprojecting. ``None`` when the ICP factor is OFF (no cost on the
        # default / oracle path). The cloud is in the camera optical frame -- the
        # same body==camera frame the poses live in -- so ``T_i^-1 T_j`` maps the
        # cam_j cloud into cam_i for the relative ICP.
        cloud = None
        if self.cfg.depth_icp:
            # Cap the cloud at ~ICP_CLOUD_CAP points so the brute-force NN stays
            # cheap: a 54x42 ToF frame (2268 px) is taken whole (stride 1), a
            # full-res 640x400 frame is strided down. The salient subset inside
            # ICP further shrinks what is actually matched.
            npx = h * w
            stride = max(1, int(np.ceil(np.sqrt(npx / self._ICP_CLOUD_CAP))))
            cloud = backproject_depth(
                np.asarray(depth_m, np.float64), self.K,
                min_z=self.cfg.min_depth_m, max_z=self.cfg.max_depth_m,
                stride=stride)
        if not self.keyframes:
            # First KF has no inbound edge: ``edge`` is None and ``pre`` (the
            # cached factor reference used by ``run_ba``) stays None.
            kf = {"T_cw": T_cw, "obs": obs, "ts_ns": ts_ns,
                  "edge": None, "pre": None, "cloud": cloud,
                  "v": np.zeros(3), "bg": self.bg0.copy(), "ba": self.ba0.copy()}
        else:
            prev = self.keyframes[-1]
            bg_i, ba_i = prev["bg"], prev["ba"]
            if imu_seg is not None:
                ts_seg = np.asarray(imu_seg[0], np.int64)
                seg = None if ts_seg.size < 2 else (
                    ts_seg, np.asarray(imu_seg[1], np.float64),
                    np.asarray(imu_seg[2], np.float64))
            else:
                seg = _imu_segment(self.imu_ts, self.imu_gyro, self.imu_accel,
                                   prev["ts_ns"], ts_ns)
            edge = None
            pre = None
            v_j = prev["v"].copy()
            if seg is not None:
                # Build the per-edge cache: integrate ONCE here about the previous
                # keyframe's bias (the same single preintegrate_imu call as before,
                # default noise -> byte-identical deltas for the loose/oracle path).
                # ``run_ba`` reuses ``edge.pre`` every solve and only re-integrates
                # if the live bias drifts past threshold (tight path only).
                edge = _ImuEdge(seg, bg_i, ba_i)
                pre = edge.pre
                R_i, _ = T_cw_to_body_world(prev["T_cw"])
                dR, dv, dp = pre.corrected(bg_i, ba_i)
                # predict velocity from the IMU increment (position/rotation are
                # seeded from the visual pose instead, which is metric already).
                v_j = prev["v"] + self.g_world * pre.dt + R_i @ dv
            kf = {"T_cw": T_cw, "obs": obs, "ts_ns": ts_ns,
                  "edge": edge, "pre": pre, "cloud": cloud,
                  "v": v_j, "bg": bg_i.copy(), "ba": ba_i.copy()}
        self.keyframes.append(kf)
        self._marginalize()

    def _marginalize(self) -> None:
        while len(self.keyframes) > self.cfg.window:
            self.keyframes.pop(0)
        live = set()
        for kf in self.keyframes:
            live.update(kf["obs"].keys())
        for tid in list(self.landmarks.keys()):
            if tid not in live:
                del self.landmarks[tid]

    def run_ba(self) -> np.ndarray | None:
        """Optimise the window; return the refined latest ``T_cw`` (or None)."""
        kfs = self.keyframes
        if len(kfs) < 2:
            return None
        cnt = Counter()
        for kf in kfs:
            for tid in kf["obs"]:
                if tid in self.landmarks:
                    cnt[tid] += 1
        ba_tids = [t for t, c in cnt.items() if c >= self.cfg.min_ba_views]
        if len(ba_tids) < 6:
            return None
        lm_index = {t: j for j, t in enumerate(ba_tids)}

        st = VioState(
            R=[], p=[], v=[], bg=[], ba=[],
            landmarks=np.array([self.landmarks[t] for t in ba_tids]),
        )
        for kf in kfs:
            R_wb, p_wb = T_cw_to_body_world(kf["T_cw"])
            st.R.append(R_wb)
            st.p.append(p_wb)
            st.v.append(kf["v"].copy())
            st.bg.append(kf["bg"].copy())
            st.ba.append(kf["ba"].copy())

        obs_cam, obs_lm, obs_uv, obs_depth = [], [], [], []
        for ci, kf in enumerate(kfs):
            for tid, uvz in kf["obs"].items():
                j = lm_index.get(tid)
                if j is None:
                    continue
                obs_cam.append(ci)
                obs_lm.append(j)
                obs_uv.append(uvz[:2])
                obs_depth.append(uvz[2])
        if len(obs_cam) < 12:
            return None

        # IMU factors between consecutive in-window keyframes, read from the
        # per-edge cache. ``kf[ci]["edge"]`` links kf[ci-1] -> kf[ci]; the
        # window's first keyframe's own edge (which linked to a now-dropped
        # keyframe) is simply never referenced.
        #
        # Cache reuse vs relinearisation: the cached ``edge.pre`` (deltas + bias
        # Jacobians + Sigma_ij/sqrt_info) is reused every solve. The optimiser's
        # per-iteration bias change is handled by the first-order
        # ``pre.corrected`` update (no re-integration). Only on the covariance-
        # weighted (tight) path do we refresh an edge whose keyframe bias has
        # drifted materially from its linearisation point, so the covariance
        # weight stays consistent with the current bias. The default / oracle
        # path NEVER relinearises -> its factor is bit-identical to before.
        relinearize = bool(self.cfg.vio.imu_info_weight)
        imu_factors = []
        for ci in range(1, len(kfs)):
            edge = kfs[ci]["edge"]
            if edge is None:
                continue
            if relinearize:
                # use the host keyframe (i = ci-1) bias as the linearisation pt
                edge.maybe_relinearize(kfs[ci - 1]["bg"], kfs[ci - 1]["ba"])
            kfs[ci]["pre"] = edge.pre        # keep the convenience reference fresh
            imu_factors.append((ci - 1, ci, edge.pre))
        if not self.cfg.use_imu:
            imu_factors = []

        # Phase-4: the single ``stabilize_velocity`` knob flips on both velocity
        # priors for this solve (CV smoothness + excitation-gated ZUPT) without
        # the caller editing the nested VioConfig. Default False -> vio_cfg is
        # ``self.cfg.vio`` unchanged -> oracle byte-identical.
        vio_cfg = self.cfg.vio
        if self.cfg.stabilize_velocity:
            vio_cfg = replace(vio_cfg, vel_cv_prior=True, vel_zupt=True)
        # njit IMU-factor kernel <-> divergence-guard SAFETY COUPLING. The njit
        # gate in ``optimize_vio`` reads ``cfg.njit_guard_ok``: thread THIS map's
        # guard state into the solve config so the default-ON kernel runs only
        # when the divergence guard is on (the only config it is validated under;
        # guard off -> the gate force-disables it and logs why). ``replace`` keeps
        # the loose path / oracle ``self.cfg.vio`` untouched (it never reaches
        # here). The guard is ON by default, so this is a no-op for the shipped
        # --tight config; it bites only an explicit divergence_guard=False A/B.
        if vio_cfg.njit_guard_ok != self.cfg.divergence_guard:
            vio_cfg = replace(vio_cfg, njit_guard_ok=bool(self.cfg.divergence_guard))

        # Phase-4 dense-ICP relative-pose factors (opt-in): one per adjacent
        # in-window KF pair whose cached clouds converge. Default OFF -> empty
        # list, ``icp_factor`` stays False -> the optimiser's ICP loops are
        # skipped and the solve is byte-identical to today.
        icp_factors = []
        if self.cfg.depth_icp:
            vio_cfg = replace(vio_cfg, icp_factor=True)
            icp_factors = self._build_icp_factors(kfs, st, vio_cfg)

        # --- Divergence guard: capture the IMU forward-prediction of the latest
        # keyframe pose BEFORE the solve mutates the window. The latest KF's
        # inbound edge (kfs[-1]["edge"], linking kfs[-2] -> kfs[-1]) carries the
        # inter-KF preintegration; the previous keyframe's nav-state is read from
        # the seeded input ``st`` (the solve will overwrite the live kf dicts, so
        # we MUST read the prediction inputs now). The prediction is the inertial
        # reference the refined visual pose is checked against and the fallback
        # on rejection. ``pre_pred`` stays None when there is no inbound edge (no
        # IMU to predict from) -> the jump test is skipped for that keyframe.
        guard = bool(self.cfg.divergence_guard)
        R_pred = p_pred = None
        imu_disp = 0.0
        pred_dt = 0.0
        if guard and len(kfs) >= 2:
            last_edge = kfs[-1]["edge"]
            if last_edge is not None and last_edge.pre is not None:
                i_prev = len(kfs) - 2
                pred_dt = float(last_edge.pre.dt)
                R_pred, p_pred = _imu_predict_pose(
                    st.R[i_prev], st.p[i_prev], st.v[i_prev],
                    st.bg[i_prev], st.ba[i_prev], last_edge.pre, self.g_world)
                imu_disp = float(np.linalg.norm(p_pred - st.p[i_prev]))

        res = optimize_vio(
            self.K, st,
            np.array(obs_cam), np.array(obs_lm), np.array(obs_uv),
            np.array(obs_depth), imu_factors, self.g_world,
            cfg=vio_cfg, anchor=0, icp_factors=icp_factors,
        )
        out = res.state

        # --- Divergence detection (after LM converges, before publishing) ----
        # Two trip signals (EITHER one rejects this window):
        #   (a) the post-solve mean reprojection error exceeds the ceiling -- the
        #       PRIMARY signal (a solve that did not fit its own measurements);
        #   (b) the refined latest-KF position deviates from the IMU forward-
        #       prediction by more than a KINEMATIC bound (absolute slack + a
        #       margin scaled by the IMU-implied displacement, so fast-but-REAL
        #       inertial motion is not flagged) AND the solve is ALSO visually
        #       questionable (reproj > jump_reproj_floor_px). The reproj gate on
        #       (b) is essential: a large refined-vs-predicted gap on a CLEAN
        #       (low-reproj) solve means the IMU PREDICTION drifted, not the
        #       solve -- rejecting it would swap a good pose for a worse dead-
        #       reckon. A genuine divergence trips BOTH together (shake).
        degraded = False
        window_jump_m = 0.0
        if guard:
            if res.mean_reproj_px > self.cfg.max_reproj_px:
                degraded = True
            if (R_pred is not None
                    and res.mean_reproj_px > self.cfg.jump_reproj_floor_px):
                window_jump_m = float(np.linalg.norm(out.p[-1] - p_pred))
                jump_bound = (self.cfg.max_window_jump_m
                              + self.cfg.jump_margin_frac * imu_disp)
                if window_jump_m > jump_bound:
                    degraded = True

        if degraded:
            # REJECT the diverged window. Do NOT write back the solver output to
            # the live keyframes/landmarks -- leave the window at its pre-solve
            # seed so the NEXT solve starts from a clean (non-poisoned) state.
            #
            # FALLBACK POSE: the latest keyframe falls back to the IMU forward-
            # prediction (dead-reckon) ONLY when it AGREES with the bounded
            # frontend visual seed to within a physical inter-keyframe travel
            # bound. This is the safe synthesis the live data demands:
            #   * vision degraded but IMU trustworthy (covered camera, plausible
            #     speed) -> the dead-reckon and the (gyro-seeded) visual seed are
            #     close -> we publish the dead-reckon, the intended behaviour;
            #   * a DIVERGENT solve is typically accompanied by a runaway VELOCITY
            #     state (the documented |v|->20-25 m/s ramp on shake), so a literal
            #     dead-reckon off that velocity jumps metres and DISAGREES with the
            #     bounded frontend -> we keep the frontend visual seed instead, so
            #     the published trajectory stays bounded (the frontend is the
            #     trustworthy estimate here; we deliberately did NOT poison it).
            # ``kfs[-1]["T_cw"]`` on entry is exactly the frontend visual seed
            # (``inv(self.pose)`` at keyframe insertion), so the default action --
            # leave it untouched -- already publishes the bounded frontend pose.
            if R_pred is not None:
                _, p_vis = T_cw_to_body_world(kfs[-1]["T_cw"])
                max_disp = self.cfg.max_deadreckon_speed_mps * pred_dt
                if float(np.linalg.norm(p_pred - p_vis)) <= max_disp:
                    # dead-reckon agrees with vision within a plausible window of
                    # travel -> trust the IMU prediction (the covered-camera case).
                    kfs[-1]["T_cw"] = body_world_to_T_cw(R_pred, p_pred)
                # else: keep the untouched frontend visual seed (bounded fallback).
            self.last_info = {
                "vio_kfs": len(kfs), "vio_lms": len(ba_tids),
                "vio_obs": len(obs_cam), "vio_imu": len(imu_factors),
                "vio_icp": len(icp_factors),
                "vio_iters": res.iters, "vio_reproj_px": res.mean_reproj_px,
                "vio_window_jump_m": window_jump_m,
                "vio_degraded": True,
            }
            return kfs[-1]["T_cw"].copy()

        for ci, kf in enumerate(kfs):
            kf["T_cw"] = body_world_to_T_cw(out.R[ci], out.p[ci])
            kf["v"] = out.v[ci].copy()
            kf["bg"] = out.bg[ci].copy()
            kf["ba"] = out.ba[ci].copy()
        for t, j in lm_index.items():
            self.landmarks[t] = out.landmarks[j]

        self.last_info = {
            "vio_kfs": len(kfs), "vio_lms": len(ba_tids),
            "vio_obs": len(obs_cam), "vio_imu": len(imu_factors),
            "vio_icp": len(icp_factors),
            "vio_iters": res.iters, "vio_reproj_px": res.mean_reproj_px,
            "vio_window_jump_m": window_jump_m,
            "vio_degraded": False,
        }
        return kfs[-1]["T_cw"].copy()

    def _build_icp_factors(self, kfs, st, vio_cfg):
        """Dense-ICP relative-pose factors for adjacent in-window KF pairs.

        For each pair (ci-1, ci) with both cached clouds present, run the IMU-
        seeded point-to-plane ICP (``cam_{ci-1} <- cam_ci``) and -- iff it
        converges with enough correspondences -- emit an :class:`IcpFactor` whose
        whitening ``Omega_icp`` is built once from the ICP information ``Lambda``.
        A non-converged / under-determined ICP yields NO factor (the spec's drop
        rule), so a degenerate frame simply contributes nothing rather than a
        bad constraint.

        Seed: the current window's STATE relative pose ``T_i^-1 T_j`` (visual +
        prior-solve IMU, metric and frame-exact) refined towards the IMU edge's
        rotation increment ``dR`` so a fast rotation still seeds close. This is
        the best available baseline guess and lands ICP within a few iterations.
        """
        factors: list[IcpFactor] = []
        for ci in range(1, len(kfs)):
            cloud_i = kfs[ci - 1].get("cloud")
            cloud_j = kfs[ci].get("cloud")
            if cloud_i is None or cloud_j is None:
                continue
            i, j = ci - 1, ci
            # state relative pose cam_i <- cam_j as the ICP seed
            T_i = se3_from_Rp(st.R[i], st.p[i])
            T_j = se3_from_Rp(st.R[j], st.p[j])
            T_seed = se3_inv(T_i) @ T_j
            edge = kfs[ci].get("edge")
            if edge is not None and edge.pre is not None:
                # blend in the IMU rotation increment (gyro is the trustworthy
                # rotation source) while keeping the metric state translation seed.
                dR, _dv, dp = edge.pre.corrected(kfs[i]["bg"], kfs[i]["ba"])
                T_seed = imu_seed_relpose(dR, T_seed[:3, 3])
            T_icp, Lambda, n_corr, converged = icp_p2plane_blend(
                cloud_i, cloud_j, T_seed)
            if not converged or n_corr < 20:
                continue
            Omega = _icp_omega(Lambda, vio_cfg)
            factors.append(IcpFactor(i=i, j=j, T_icp_ij=T_icp, Omega_icp=Omega))
        return factors


class WindowedVIORGBDOdometry:
    """Frame-to-frame tracking with a tight-coupled sliding-window VIO backend.

    Drop-in sibling of :class:`sky.backend.windowed.WindowedRGBDOdometry`: the same
    KLT/PnP frontend produces a smooth per-frame pose, but every keyframe is
    refined by :class:`WindowedVIOMap` (visual + IMU joint optimisation) instead
    of vision-only bundle adjustment. The caller passes the full IMU stream in
    the camera optical frame at construction; :meth:`process` takes the frame
    timestamp so the map can preintegrate the IMU between keyframes.
    """

    def __init__(self, K: np.ndarray, ts_ns: np.ndarray,
                 gyro_cam: np.ndarray, accel_cam: np.ndarray,
                 bg0: np.ndarray | None = None, ba0: np.ndarray | None = None,
                 cfg: WindowedVIOConfig | None = None,
                 frontend: KLTFrontend | None = None,
                 odom_cfg: OdometryConfig | None = None):
        self.K = np.asarray(K, dtype=np.float64)
        self.cfg = cfg or WindowedVIOConfig()
        fe = frontend or KLTFrontend(FrontendConfig())
        self.vo = RGBDVisualOdometry(self.K, odom_cfg or OdometryConfig(),
                                     frontend=fe)
        self.frontend = self.vo.frontend
        self.map = WindowedVIOMap(self.K, ts_ns, gyro_cam, accel_cam,
                                  bg0, ba0, self.cfg)
        self._frames_since_kf = 0
        self._frame_idx = -1
        self.pose = np.eye(4)
        self.last_info: dict = {}

    def align_to_gravity(self, accel_cam: np.ndarray) -> None:
        self.vo.align_to_gravity(accel_cam)
        self.pose = self.vo.pose.copy()

    @property
    def landmarks(self) -> dict[int, np.ndarray]:
        return self.map.landmarks

    @property
    def keyframes(self) -> list[dict]:
        return self.map.keyframes

    def process(self, gray: np.ndarray, depth_m: np.ndarray, ts_ns: int,
                R_prior: np.ndarray | None = None) -> np.ndarray:
        """Advance one frame; return the current 4x4 world pose (camera->world)."""
        self._frame_idx += 1
        self.pose = self.vo.process(gray, depth_m, R_prior=R_prior).copy()
        self.last_info = dict(self.vo.last_info)
        self._frames_since_kf += 1

        is_kf = (not self.keyframes) or (self._frames_since_kf >= self.cfg.kf_every)
        if is_kf:
            self._frames_since_kf = 0
            state = self.frontend.tracks
            self.map.add_keyframe(np.linalg.inv(self.pose),
                                  state.ids, state.points, depth_m, ts_ns)
            post = self.map.run_ba()
            if post is not None:
                self.pose = np.linalg.inv(post)
                # Divergence guard: a diverged window solve returns the IMU
                # dead-reckon fallback (not the diverged pose), so the PUBLISHED
                # pose stays inertially consistent -- but we must NOT write it
                # back into the live VO frontend (``self.vo.pose``). The frontend
                # tracks frame-to-frame from its own pose; overwriting it with a
                # solve we just rejected as untrustworthy would let a transient
                # fault (one bad blur frame) persistently poison tracking. On a
                # HEALTHY solve the frontend is re-anchored to the refined pose
                # exactly as before (behaviour bit-unchanged).
                if not self.map.last_info.get("vio_degraded", False):
                    self.vo.pose = self.pose.copy()
                self.last_info.update(self.map.last_info)
            self.last_info["is_kf"] = True
        else:
            self.last_info["is_kf"] = False
        return self.pose
