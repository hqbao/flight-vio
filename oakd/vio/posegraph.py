"""SE(3) pose-graph optimisation — pure NumPy.

This is the backend optimiser for loop closure. Visual odometry (or windowed BA)
gives accurate *relative* motion between keyframes but accumulates *global*
drift, so after a long trajectory the start and end no longer line up even when
the camera physically returned to the same spot. A loop closure provides one
extra relative constraint ("keyframe 117 is right back at keyframe 3"), and
pose-graph optimisation (PGO) distributes the accumulated error over the whole
graph so the whole trajectory becomes globally consistent.

Formulation
-----------
- A node is a keyframe pose ``X_i = T_wc`` (camera->world, 4x4 SE3).
- An edge ``(i, j, Z_ij, Omega)`` carries a *measured* relative transform
  ``Z_ij = T_ci_cj`` (pose of cam j expressed in cam i, i.e. ``X_i^{-1} X_j``
  in the noise-free case) and a 6x6 information matrix ``Omega``.
- The error of an edge is ``e_ij = Log( Z_ij^{-1} · (X_i^{-1} X_j) )`` (a
  6-vector ``[rho; phi]``, translation part first to match :func:`se3_exp`).
- We minimise ``sum_ij e_ij^T Omega_ij e_ij`` by Gauss-Newton with a right
  perturbation ``X <- X · Exp(delta)``. The first node is pinned (gauge) with a
  strong prior so the global frame stays fixed.

Jacobians use the standard small-residual approximation ``J_r^{-1}(e) ~= I``
(exact at convergence; the relative measurements are accurate so every *edge*
residual stays small even when the *global* drift is large). With that,

    de/ddelta_i = -Ad( X_j^{-1} X_i )      de/ddelta_j = +I

which is the well-known Grisetti pose-graph linearisation. Levenberg-Marquardt
damping keeps it stable from a poor initial guess.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .bundle import se3_exp, skew


# --------------------------------------------------------------------------- #
# SE(3) log + adjoint (se3_exp / so3_exp / skew live in bundle.py)
# --------------------------------------------------------------------------- #
def so3_log(R: np.ndarray) -> np.ndarray:
    """SO(3) -> so3 (rotation matrix to rotation vector)."""
    c = (np.trace(R) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    theta = float(np.arccos(c))
    if theta < 1e-9:
        # Near identity: vee of the skew-symmetric part (first-order).
        return 0.5 * np.array([R[2, 1] - R[1, 2],
                               R[0, 2] - R[2, 0],
                               R[1, 0] - R[0, 1]])
    if np.pi - theta < 1e-6:
        # Near pi: recover axis from the symmetric part (sign-robust).
        A = (R + np.eye(3)) * 0.5
        axis = np.sqrt(np.clip(np.diag(A), 0.0, None))
        # fix signs from off-diagonals
        if axis[0] > 1e-6:
            axis[1] = np.copysign(axis[1], A[0, 1])
            axis[2] = np.copysign(axis[2], A[0, 2])
        elif axis[1] > 1e-6:
            axis[2] = np.copysign(axis[2], A[1, 2])
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return theta * axis
    w = theta / (2.0 * np.sin(theta))
    return w * np.array([R[2, 1] - R[1, 2],
                         R[0, 2] - R[2, 0],
                         R[1, 0] - R[0, 1]])


def se3_log(T: np.ndarray) -> np.ndarray:
    """SE(3) -> se3. Returns xi = [rho(3); phi(3)] (translation part first)."""
    R = T[:3, :3]
    t = T[:3, 3]
    phi = so3_log(R)
    theta = float(np.linalg.norm(phi))
    if theta < 1e-9:
        Vinv = np.eye(3) - 0.5 * skew(phi)
    else:
        K = skew(phi)
        a = 1.0 / (theta * theta)
        b = (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
        Vinv = np.eye(3) - 0.5 * K + (a - b) * (K @ K)
    rho = Vinv @ t
    return np.concatenate([rho, phi])


def se3_adjoint(T: np.ndarray) -> np.ndarray:
    """6x6 adjoint Ad(T) for the [rho; phi] (translation-first) twist order."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[:3, 3:] = skew(t) @ R
    Ad[3:, 3:] = R
    return Ad


def se3_inv(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


# --------------------------------------------------------------------------- #
# Pose graph
# --------------------------------------------------------------------------- #
@dataclass
class Edge:
    i: int
    j: int
    Z: np.ndarray            # measured relative T_ci_cj (4x4)
    Omega: np.ndarray        # 6x6 information
    loop: bool = False


class PoseGraph:
    """Keyframe pose graph with SE(3) Gauss-Newton optimisation.

    Nodes are ``T_wc`` (camera->world). Add nodes in any order by integer id,
    add relative edges, then call :meth:`optimize`. Node 0 (or the lowest id) is
    pinned as the gauge anchor.
    """

    def __init__(self) -> None:
        self.nodes: dict[int, np.ndarray] = {}
        self.edges: list[Edge] = []

    def add_node(self, i: int, T_wc: np.ndarray) -> None:
        self.nodes[i] = np.asarray(T_wc, float).copy()

    def add_edge(self, i: int, j: int, Z: np.ndarray,
                 omega: np.ndarray | float = 1.0, loop: bool = False) -> None:
        if np.isscalar(omega):
            Om = np.eye(6) * float(omega)
        else:
            Om = np.asarray(omega, float)
        self.edges.append(Edge(i, j, np.asarray(Z, float).copy(), Om, loop))

    def total_error(self) -> float:
        e2 = 0.0
        for e in self.edges:
            Xi, Xj = self.nodes[e.i], self.nodes[e.j]
            E = se3_inv(e.Z) @ (se3_inv(Xi) @ Xj)
            r = se3_log(E)
            e2 += float(r @ e.Omega @ r)
        return e2

    def optimize(self, iters: int = 30, anchor: int | None = None,
                 rel_tol: float = 1e-6, huber_delta: float = 0.5,
                 verbose: bool = False) -> dict:
        """Gauss-Newton (with LM damping). Mutates node poses in place.

        ``huber_delta`` applies a Huber robust kernel to **loop** edges only
        (odometry edges are trusted): a loop whose residual exceeds the threshold
        is down-weighted, so a few surviving false loop closures (perceptual
        aliasing) cannot drag the whole graph. Set to 0 to disable.
        """
        ids = sorted(self.nodes.keys())
        idx = {nid: k for k, nid in enumerate(ids)}
        N = len(ids)
        if anchor is None:
            anchor = ids[0]
        a = idx[anchor]

        def _huber_w(e: np.ndarray, Om: np.ndarray) -> float:
            if huber_delta <= 0.0:
                return 1.0
            chi = float(np.sqrt(max(e @ Om @ e, 0.0)))
            return 1.0 if chi <= huber_delta else huber_delta / chi

        lam = 1e-6
        cost_prev = self.total_error()
        cost0 = cost_prev
        it = 0
        for it in range(iters):
            H = np.zeros((6 * N, 6 * N))
            b = np.zeros(6 * N)
            for e in self.edges:
                ci, cj = idx[e.i], idx[e.j]
                Xi, Xj = self.nodes[e.i], self.nodes[e.j]
                Xi_inv = se3_inv(Xi)
                E = se3_inv(e.Z) @ (Xi_inv @ Xj)
                r = se3_log(E)
                # J_r^{-1} ~= I  ->  Ji = -Ad(Xj^{-1} Xi), Jj = +I
                Ad = se3_adjoint(se3_inv(Xj) @ Xi)
                Ji = -Ad
                Jj = np.eye(6)
                Om = e.Omega
                if e.loop:
                    Om = Om * _huber_w(r, Om)
                si, sj = slice(6 * ci, 6 * ci + 6), slice(6 * cj, 6 * cj + 6)
                H[si, si] += Ji.T @ Om @ Ji
                H[sj, sj] += Jj.T @ Om @ Jj
                H[si, sj] += Ji.T @ Om @ Jj
                H[sj, si] += Jj.T @ Om @ Ji
                b[si] += Ji.T @ Om @ r
                b[sj] += Jj.T @ Om @ r

            # Pin the anchor node with a strong prior (gauge freedom removal).
            sa = slice(6 * a, 6 * a + 6)
            H[sa, sa] += np.eye(6) * 1e12

            # LM damping.
            H[np.diag_indices_from(H)] += lam * np.clip(np.diag(H), 1e-9, None)

            try:
                dx = np.linalg.solve(H, -b)
            except np.linalg.LinAlgError:
                dx = np.linalg.lstsq(H, -b, rcond=None)[0]

            # Trial update on a copy.
            trial = {}
            for nid in ids:
                k = idx[nid]
                trial[nid] = self.nodes[nid] @ se3_exp(dx[6 * k:6 * k + 6])
            saved = self.nodes
            self.nodes = trial
            cost_new = self.total_error()
            if cost_new < cost_prev:
                lam = max(lam * 0.5, 1e-9)
                improved = (cost_prev - cost_new) / max(cost_prev, 1e-12)
                cost_prev = cost_new
                if verbose:
                    print(f"  pgo it{it:02d} cost={cost_new:.6g} lam={lam:.1e}")
                if improved < rel_tol:
                    break
            else:
                self.nodes = saved      # reject
                lam = min(lam * 8.0, 1e9)

        return {"iters": it + 1, "cost0": cost0, "cost1": cost_prev,
                "nodes": N, "edges": len(self.edges)}
