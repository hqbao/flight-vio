"""Dense point-to-plane(+point) ICP relative-pose factor (pure NumPy).

Why this module exists
----------------------
At the real VL53-class ToF target resolution (54x42) the sparse KLT frontend
**starves**: 5-10% of frames drop below ~6 tracks, so the inter-keyframe
translation / velocity becomes unobservable and the tight-VIO window diverges
(Phase 4). Empirically a *dense* point-to-plane ICP between two keyframes' depth
clouds has 0% failure (it always yields a translation constraint) and comparable
translation accuracy (~2.4 vs ~2.8 cm) to the sparse solve -- only its rotation
is noisier. So a dense-ICP **relative-pose factor** between adjacent keyframes
gives the otherwise-missing translation increment a real anchor, composing with
the already-shipped constant-velocity prior / ZUPT.

This file is the geometry half: given two camera-frame point clouds and an
IMU-preintegrated seed for the relative pose, it returns the refined relative
pose AND the point-to-plane normal-equation Hessian at convergence -- which IS
the measurement information ``Lambda`` the VIO factor whitens with. The factor
assembly + whitening + Jacobian live in :mod:`sky.vio.window` (the tight VIO
optimiser); this module stays a pure ``sky.*`` leaf (numpy only).

Algorithm (GenZ-ICP-style blend, deliberately conservative before fast)
-----------------------------------------------------------------------
* **Salient subset.** Only a ~5-10% subset of the source cloud is matched, the
  points with the largest local depth-gradient / normal saliency. Planar
  interiors carry no along-plane information, so matching all of them just costs
  time; the salient edges/corners are what little geometric texture a
  feature-starved ToF view has.
* **Point-to-plane + point-to-point blend.** The residual for a correspondence
  ``(p_src, q_tgt, n_tgt)`` is ``alpha * n^T (T p_src - q) + (1-alpha) * |T p_src
  - q|`` linearised. Pure point-to-plane slides freely along a single plane (the
  flat-wall degeneracy); a SMALL point-to-point term regularises that sliding so
  the SOLVE stays bounded, without destroying the planar-convergence speed.

  NOTE on the blend weight (empirically tuned, deviates from the spec's ~0.8):
  the source subset is the most-SALIENT (edge/corner) points, and point-to-point
  on edge points is biased by nearest-neighbour TANGENTIAL drift -- on a discrete
  depth grid an edge source point latches onto a laterally-offset target, so a
  heavy point-to-point weight (alpha 0.8) converges to a wrong-by-~5cm fixed
  point even on a fully-constrained 3-plane corner (verified). Pure point-to-
  plane recovers that corner EXACTLY; a small point-to-point weight (alpha~0.97)
  keeps the exact recovery AND still bounds the single-plane along-wall slide
  (verified: 0.1 cm, not unbounded). Crucially the reported ``Lambda`` is the
  PURE point-to-plane Hessian regardless of alpha, so the degeneracy null space
  the VIO remap projects out is never masked by the regulariser.
* **t-distribution robust IRLS** (nu~5): each correspondence is reweighted by a
  Student-t kernel ``(nu+1)/(nu + (r/sigma)^2)`` so a handful of wrong matches
  (occlusion, depth holes) cannot drag the solve. Robust to the heavy-tailed
  residuals dense depth produces.
* **IMU-seeded.** ``T_seed`` is the preintegrated relative pose on the IMU edge;
  starting there (instead of identity) means the keyframe-to-keyframe baseline
  converges in a handful of iterations even when the motion is large.

Output information ``Lambda``
-----------------------------
``Lambda = sum_k w_k a_k a_k^T`` with ``a_k = [n_k ; p_k x n_k]`` (translation
part first, to match the VIO factor's ``[rho; phi]`` twist order). This is the
Gauss-Newton Hessian of the point-to-plane cost at convergence, i.e. the
*measurement information* of the recovered relative pose. A single-plane cloud
makes ``a_k`` span only the plane-normal translation + in-plane rotation, so the
two along-plane translation directions get **near-zero eigenvalues** -- exactly
the degeneracy the VIO factor's eigenvalue remap must project out.
"""
from __future__ import annotations

import numpy as np

from sky.math import se3_from_Rp, so3_exp_unit, so3_log


def _skew_batch(v: np.ndarray) -> np.ndarray:
    """Batched skew-symmetric matrices: ``v`` (M,3) -> (M,3,3) with ``S@u = v x u``."""
    M = v.shape[0]
    S = np.zeros((M, 3, 3))
    S[:, 0, 1] = -v[:, 2]
    S[:, 0, 2] = v[:, 1]
    S[:, 1, 0] = v[:, 2]
    S[:, 1, 2] = -v[:, 0]
    S[:, 2, 0] = -v[:, 1]
    S[:, 2, 1] = v[:, 0]
    return S


# --------------------------------------------------------------------------- #
# Cloud helpers
# --------------------------------------------------------------------------- #
def backproject_depth(depth_m: np.ndarray, K: np.ndarray,
                      min_z: float = 0.05, max_z: float = 1e9,
                      stride: int = 1) -> np.ndarray:
    """Backproject a metric depth map into the CAMERA frame point cloud (N,3).

    Only finite, positive, in-range pixels are kept. ``stride`` subsamples the
    pixel grid (a cheap density cap before the saliency subset is taken). The
    returned points are in the camera optical frame (x right, y down, z forward),
    the same frame the VIO ``body == camera`` poses live in.
    """
    depth_m = np.asarray(depth_m, np.float64)
    h, w = depth_m.shape
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    z = depth_m[::stride, ::stride]
    valid = np.isfinite(z) & (z > min_z) & (z < max_z)
    zz = z[valid]
    uu = xs[valid].astype(np.float64)
    vv = ys[valid].astype(np.float64)
    x = (uu - cx) * zz / fx
    y = (vv - cy) * zz / fy
    return np.stack([x, y, zz], axis=1)


def _knn_normals(cloud: np.ndarray, k: int = 12) -> tuple[np.ndarray, np.ndarray]:
    """Per-point unit normal + a scalar saliency for a small cloud (brute-force).

    Returns ``(normals (N,3), saliency (N,))``. The normal is the smallest-
    eigenvector of the local covariance over the ``k`` nearest neighbours (PCA
    plane fit); the saliency is the surface-variation ratio
    ``lambda_min / (lambda_0+lambda_1+lambda_2)`` -- LOW on flat planes, HIGH on
    edges/corners. Brute force is fine here: the clouds are small (a 54x42 ToF
    frame is <=2268 points, full-res is strided/capped before this call).
    """
    n = cloud.shape[0]
    normals = np.zeros((n, 3))
    saliency = np.zeros(n)
    if n == 0:
        return normals, saliency
    k = int(min(k, n))
    # pairwise squared distances (n,n) -- small n, so the n^2 matrix is cheap
    d2 = np.sum((cloud[:, None, :] - cloud[None, :, :]) ** 2, axis=2)
    nn_idx = np.argsort(d2, axis=1)[:, :k]
    for i in range(n):
        nbr = cloud[nn_idx[i]]
        c = nbr - nbr.mean(axis=0)
        cov = c.T @ c
        evals, evecs = np.linalg.eigh(cov)        # ascending
        normals[i] = evecs[:, 0]                  # smallest-eigenvalue direction
        tot = float(evals.sum())
        saliency[i] = (float(evals[0]) / tot) if tot > 1e-12 else 0.0
    # orient normals towards the camera (-z half-space) for a consistent sign
    flip = normals[:, 2] > 0.0
    normals[flip] *= -1.0
    return normals, saliency


def _salient_subset(cloud: np.ndarray, normals: np.ndarray, saliency: np.ndarray,
                    frac: float, min_pts: int) -> np.ndarray:
    """Indices of the ~``frac`` most-salient points (>= ``min_pts`` when possible).

    The flat-plane interior carries no along-plane geometric information, so we
    match only the most salient (edge/corner) points -- exactly where a feature-
    starved ToF view still has texture. Falls back to all points if the cloud is
    already small.
    """
    n = cloud.shape[0]
    if n == 0:
        return np.zeros(0, np.int64)
    k = int(max(min_pts, round(frac * n)))
    k = min(k, n)
    # most salient first
    return np.argsort(saliency)[::-1][:k].astype(np.int64)


# --------------------------------------------------------------------------- #
# The ICP solve
# --------------------------------------------------------------------------- #
def icp_p2plane_blend(
    cloud_i: np.ndarray,
    cloud_j: np.ndarray,
    T_seed: np.ndarray,
    *,
    alpha: float = 0.97,
    nu: float = 5.0,
    salient_frac: float = 0.08,
    min_salient: int = 40,
    # ``max_iters`` / ``max_corr_dist`` were measured on the real 54x42 OAK-D
    # passive-stereo clouds: the default 0.30 m / 12 iters left ~90% of keyframe
    # pairs NON-converged (the inter-KF motion + depth noise exceeded the gate),
    # whereas 0.6 m / 25 iters converges the large majority while still rejecting
    # gross mismatches. Tuned, not arbitrary.
    max_iters: int = 25,
    max_corr_dist: float = 0.60,
    min_corr: int = 20,
    knn: int = 12,
    conv_rot: float = 1e-4,
    conv_trans: float = 1e-4,
) -> tuple[np.ndarray, np.ndarray, int, bool]:
    """IMU-seeded point-to-plane(+point) ICP between two camera-frame clouds.

    Aligns ``cloud_j`` (expressed in cam_j) onto ``cloud_i`` (cam_i): finds the
    relative pose ``T_icp_ij`` (4x4, ``cam_i <- cam_j``) such that
    ``T_icp_ij @ p_j`` matches a point of ``cloud_i``. Seeded from ``T_seed`` (the
    IMU-preintegrated relative pose, same ``cam_i <- cam_j`` convention).

    Parameters
    ----------
    cloud_i, cloud_j : (Ni,3) / (Nj,3) camera-frame points (metres).
    T_seed           : (4,4) initial ``cam_i <- cam_j`` relative pose.
    alpha            : point-to-plane vs point-to-point blend (1 = pure p2plane).
    nu               : Student-t degrees of freedom for the robust IRLS weight.
    salient_frac     : fraction of the SOURCE cloud (j) matched (most salient).
    max_corr_dist    : reject correspondences farther than this (metres).
    min_corr         : drop the factor if fewer correspondences survive.

    Returns
    -------
    T_icp_ij : (4,4) refined relative pose ``cam_i <- cam_j``.
    Lambda   : (6,6) point-to-plane normal-equation Hessian at convergence in
               ``[trans(3); rot(3)]`` order = the measurement information. For a
               degenerate (single-plane) cloud the along-plane translation
               directions have near-zero eigenvalues here -- that is the signal
               the caller's eigenvalue remap projects out.
    n_corr   : number of inlier correspondences at convergence.
    converged: whether the iteration met the rotation+translation tolerance.

    The caller MUST drop the factor when ``not converged`` or ``n_corr <
    min_corr``: a non-converged ICP carries no trustworthy constraint.
    """
    cloud_i = np.asarray(cloud_i, np.float64)
    cloud_j = np.asarray(cloud_j, np.float64)
    T = np.asarray(T_seed, np.float64).copy()
    Lambda = np.zeros((6, 6))
    if cloud_i.shape[0] < min_corr or cloud_j.shape[0] < min_corr:
        return T, Lambda, 0, False

    # Target (cam_i) normals + a salient subset of the SOURCE (cam_j). The target
    # normals define the point-to-plane geometry; the source subset is what we
    # actually push onto the target.
    n_i, _ = _knn_normals(cloud_i, k=knn)
    _, sal_j = _knn_normals(cloud_j, k=knn)
    sub = _salient_subset(cloud_j, None, sal_j, salient_frac, min_salient)
    src = cloud_j[sub]                                     # (S,3) in cam_j
    if src.shape[0] < min_corr:
        return T, Lambda, 0, False

    max_corr2 = max_corr_dist * max_corr_dist
    R = T[:3, :3]
    t = T[:3, 3]
    n_corr = 0
    converged = False

    for _ in range(max_iters):
        # transform the source subset into cam_i with the current estimate
        ps = (R @ src.T).T + t                             # (S,3) in cam_i

        # nearest target for each transformed source point (brute force; small)
        d2 = np.sum((ps[:, None, :] - cloud_i[None, :, :]) ** 2, axis=2)
        nn = np.argmin(d2, axis=1)
        dmin2 = d2[np.arange(ps.shape[0]), nn]
        inlier = dmin2 < max_corr2
        if int(inlier.sum()) < min_corr:
            return T, Lambda, int(inlier.sum()), False

        p = ps[inlier]                                     # source (in cam_i)
        q = cloud_i[nn[inlier]]                            # target
        nn_n = n_i[nn[inlier]]                             # target normals
        diff = p - q                                       # (M,3)

        # point-to-plane scalar residual  r_pl = n . (p - q)
        r_pl = np.einsum('mi,mi->m', nn_n, diff)
        # point-to-point residual magnitude (for the blend weight + robust scale)
        r_pt = np.linalg.norm(diff, axis=1)

        # t-distribution robust IRLS weight on the blended residual scale. The
        # MAD-based sigma keeps the kernel scale-adaptive across clips.
        r_blend = alpha * r_pl + (1.0 - alpha) * r_pt
        sigma = 1.4826 * np.median(np.abs(r_blend)) + 1e-6
        w = (nu + 1.0) / (nu + (r_blend / sigma) ** 2)     # (M,)

        # Linearised point-to-plane Jacobian row a = [n ; (p x n)] (trans-first).
        # The increment is a LEFT perturbation of the cam_i<-cam_j pose:
        #   p' = Exp([dphi]) (R p_src + t) + dt  ==>  d(n.(p-q)) = a . [dt; dphi]
        # with a_trans = n, a_rot = p x n. Assembling the weighted normal eqs:
        #   H_pl dx = -g_pl,  H_pl = sum w a a^T,  g_pl = sum w r_pl a.
        cross = np.cross(p, nn_n)                          # (M,3) = p x n
        a = np.concatenate([nn_n, cross], axis=1)          # (M,6) [trans; rot]

        # POINT-TO-PLANE normal equations. ``Lambda_pl`` is built from these and
        # ONLY these: it is the measurement information the spec defines
        # (``Lambda = sum w a a^T``), so a degenerate single-plane cloud keeps its
        # near-zero along-plane eigenvalues (the point-to-point regulariser below
        # must NOT mask them). The robust weight ``w`` and the blend weight
        # ``alpha`` scale the point-to-plane contribution.
        wa = (w * alpha)[:, None] * a                      # (M,6) = w*alpha*a
        H_pl = a.T @ wa                                    # (6,6) = sum w*alpha a a^T
        g_pl = (wa * r_pl[:, None]).sum(axis=0)            # (6,)  = sum w*alpha r_pl a

        # POINT-TO-POINT regulariser (only for the SOLVE step, NOT for Lambda):
        # a small isotropic term that stops a single plane sliding freely along
        # itself. Jacobian of the i-th point-to-point residual wrt the same left
        # twist is J_pt = [I3, -skew(p_i)]; its normal eqs are assembled in a
        # vectorised batch (no Python per-point loop).
        H = H_pl.copy()
        g = g_pl.copy()
        if alpha < 1.0:
            wp = w * (1.0 - alpha)                          # (M,)
            sk = _skew_batch(p)                             # (M,3,3) = skew(p)
            J_pt = np.zeros((p.shape[0], 3, 6))
            J_pt[:, :, :3] = np.eye(3)
            J_pt[:, :, 3:] = -sk
            # H_pt = sum_m wp_m J_pt_m^T J_pt_m ;  g_pt = sum_m wp_m J_pt_m^T diff_m
            H += np.einsum('m,mri,mrj->ij', wp, J_pt, J_pt)
            g += np.einsum('m,mri,mr->i', wp, J_pt, diff)

        try:
            dx = np.linalg.solve(H, -g)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(H, -g, rcond=None)[0]

        dt = dx[:3]
        dphi = dx[3:]
        dR = so3_exp_unit(dphi)
        # LEFT update on cam_i<-cam_j:  T <- Exp(dx) @ T
        R = dR @ R
        t = dR @ t + dt
        T[:3, :3] = R
        T[:3, 3] = t
        n_corr = int(inlier.sum())

        # ``Lambda`` is the PURE point-to-plane Hessian at the current (about to
        # be converged) linearisation: the measurement information. Refreshed each
        # iteration so on convergence it is evaluated at the final pose.
        Lambda = H_pl

        if (np.linalg.norm(dphi) < conv_rot
                and np.linalg.norm(dt) < conv_trans):
            converged = True
            break

    return T, Lambda, n_corr, converged


def imu_seed_relpose(dR: np.ndarray, dp: np.ndarray) -> np.ndarray:
    """Build the ``cam_i <- cam_j`` seed pose from IMU preintegrated increments.

    The preintegration gives ``R_j = R_i dR`` and ``p_j = p_i + ... + R_i dp``
    (body == camera). The relative pose ``T_ij = inv(T_i) @ T_j`` (cam_i <- cam_j)
    therefore has rotation ``dR`` and translation ``dp`` (the position increment
    expressed in cam_i). Velocity / gravity terms in the full ``dp`` formula are
    folded into ``dp`` by the caller (it passes the position part of the relative
    transform it already trusts), so here we just pack ``(dR, dp)``.
    """
    return se3_from_Rp(np.asarray(dR, np.float64), np.asarray(dp, np.float64))


# Convenience re-export so the factor code can build a seed log without importing
# se3 twice; keeps the leaf surface small.
__all__ = ["backproject_depth", "icp_p2plane_blend", "imu_seed_relpose",
           "so3_log"]
