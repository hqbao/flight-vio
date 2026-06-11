"""Sliding-window marginalization prior for the keyframe bundle adjustment.

When the BA window slides and the oldest keyframe ``KF_0`` is dropped, a plain
drop throws away everything ``KF_0`` knew, so each window re-anchors on a fresh
estimate and the absolute gauge (yaw + scale especially) drifts between windows.
Instead we *marginalize* ``KF_0``: we Schur-complement its pose (and the
landmarks it hosts) out of the local linear system, condensing their information
into a small **linear-Gaussian prior over the surviving keyframe poses**. That
prior is then added back into :func:`vio.mathlib.backend.bundle.optimize` (camera
only, First-Estimate Jacobians), so the dropped information keeps constraining
the window.

Scope (matches the validated design)
-------------------------------------
- Landmarks here are **global Euclidean** points, so there is no inverse-depth
  re-anchoring: a landmark just carries a lightweight "host" tag = the id of the
  first keyframe that observed it.
- On a slide we marginalize ``mu = {KF_0 pose} + {landmarks hosted by KF_0 with
  >= 2 views}`` plus the **previous prior** (folded in as a factor, which is how
  ``KF_0`` is also removed from the carried-forward prior). ``KF_0``'s
  observations of landmarks hosted by *later* keyframes are simply dropped (the
  standard sliding-window approximation) — those landmarks stay live.
- The result is a prior over ``kappa`` = the keyframes that co-observed the
  marginalized landmarks (or appeared in the previous prior), excluding ``KF_0``.

The math (left SE3 ``T <- Exp(xi) @ T``, ``xi = [rho; phi]``, ``b = J^T r``,
matching :mod:`vio.mathlib.backend.bundle`)::

    H_p = H_kk - H_km @ inv(H_mm) @ H_mk
    b_p = b_k  - H_km @ inv(H_mm) @ b_m

evaluated once at the converged window estimate; the stored linearization poses
let later windows fold this prior in with frozen (FEJ) Jacobians.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sky.backend.bundle import BAConfig, se3_log, skew


@dataclass
class MargPrior:
    """A linear-Gaussian prior over keyframe poses from marginalization.

    ``kf_ids`` is the block order; ``H`` (6k,6k) and ``b0`` (6k,) are the
    condensed information and gradient at the linearization point; ``lin_Tcw``
    maps each ``kf_id`` to its ``T_cw`` at marginalization time (the FEJ
    linearization point).
    """

    kf_ids: list[int]
    H: np.ndarray
    b0: np.ndarray
    lin_Tcw: dict[int, np.ndarray]

    def resolve(self, id_to_index: dict[int, int]):
        """Map to ``optimize`` arguments for the current window, or ``None``.

        ``id_to_index`` maps a keyframe id to its index in the window's ``poses``
        list. Returns ``(prior_cams, prior_H, prior_b0, prior_lin)`` keeping only
        blocks whose keyframe is still in the window (all of them, normally).
        """
        keep = [a for a, kid in enumerate(self.kf_ids) if kid in id_to_index]
        if not keep:
            return None
        cams = [id_to_index[self.kf_ids[a]] for a in keep]
        sel = np.array(keep, dtype=np.int64)
        idx = np.concatenate([np.arange(6 * a, 6 * a + 6) for a in keep])
        H = self.H[np.ix_(idx, idx)]
        b0 = self.b0[idx]
        lin = np.stack([self.lin_Tcw[self.kf_ids[a]] for a in keep])
        return cams, H, b0, lin


def marginalize_keyframe(
    K: np.ndarray,
    cfg: BAConfig,
    keyframes: list[dict],
    landmarks: dict[int, np.ndarray],
    lm_host: dict[int, int],
    drop_id: int,
    prev_prior: MargPrior | None,
    drop_fixed: bool = False,
    grav_world: np.ndarray | None = None,
) -> tuple[MargPrior | None, list[int]]:
    """Marginalize keyframe ``drop_id`` -> a pose prior over the survivors.

    ``keyframes`` must still INCLUDE the keyframe being dropped. Each keyframe is
    a dict with ``id`` (int), ``T_cw`` (4,4), ``obs`` (``{tid: [u,v,z]}``) and an
    optional ``accel`` (camera-frame gravity, only used when ``cfg.use_gravity``).

    ``drop_fixed`` marks the bootstrap case where the dropped keyframe was the
    hard-fixed gauge anchor: its pose has no DoF, so it is NOT a marginalized
    variable -- its observations still constrain the (absolutely-anchored)
    landmarks, and only those landmarks are Schur-eliminated, transferring the
    fixed keyframe's absolute information into the prior over the survivors. For
    a free (prior-anchored) drop the pose IS marginalized with the landmarks.

    Returns ``(prior, marg_lm_ids)``: the new :class:`MargPrior` (or ``None`` when
    there is nothing to carry forward) and the list of landmark ids that were
    folded into the prior. The caller MUST delete those landmarks and their
    observations from the live map -- their information now lives in the prior,
    so re-using them in the next window would double-count it.
    """
    K = np.asarray(K, np.float64)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    kf_by_id = {int(kf["id"]): kf for kf in keyframes}
    if drop_id not in kf_by_id:
        return None, []

    # Hosted landmarks of the dropped keyframe with >= 2 views in the window.
    hosted = []
    for tid, host in lm_host.items():
        if host != drop_id or tid not in landmarks:
            continue
        views = sum(1 for kf in keyframes if tid in kf["obs"])
        if views >= 2:
            hosted.append(int(tid))
    hosted.sort()

    prev_ids = list(prev_prior.kf_ids) if prev_prior is not None else []

    # Pose variables: every kf observing a hosted landmark + prev-prior
    # keyframes. The dropped kf is a pose variable only when it is free (a fixed
    # gauge anchor contributes its observations but carries no pose DoF).
    pose_ids = set()
    for tid in hosted:
        for kf in keyframes:
            if tid in kf["obs"]:
                pose_ids.add(int(kf["id"]))
    for kid in prev_ids:
        if kid in kf_by_id:
            pose_ids.add(int(kid))
    kept_ids = sorted(pid for pid in pose_ids if pid != drop_id)
    if not kept_ids:
        return None, []
    if not hosted and prev_prior is None:
        return None, []    # nothing to carry forward -> caller does a plain drop

    # Variable layout: [(drop pose) | kept poses... | hosted landmarks...].
    # A fixed drop is excluded from the pose variables (no DoF to marginalize).
    pose_order = (kept_ids if drop_fixed else [drop_id] + kept_ids)
    pose_slot = {pid: i for i, pid in enumerate(pose_order)}
    nP = len(pose_order)
    lm_slot = {tid: j for j, tid in enumerate(hosted)}
    nL = len(hosted)
    D = 6 * nP + 3 * nL
    H = np.zeros((D, D))
    b = np.zeros(D)

    def pose_sl(pid: int):
        if pid not in pose_slot:           # fixed drop -> no pose block
            return None
        s = 6 * pose_slot[pid]
        return slice(s, s + 6)

    def lm_sl(tid: int) -> slice:
        s = 6 * nP + 3 * lm_slot[tid]
        return slice(s, s + 3)

    use_depth = bool(cfg.use_depth)
    # --- visual reprojection + metric depth factors on hosted landmarks ------
    for tid in hosted:
        Xw = np.asarray(landmarks[tid], np.float64)
        ps_lm = lm_sl(tid)
        for kf in keyframes:
            uvz = kf["obs"].get(tid)
            if uvz is None:
                continue
            pid = int(kf["id"])
            T = np.asarray(kf["T_cw"], np.float64)
            R, t = T[:3, :3], T[:3, 3]
            Xc = R @ Xw + t
            Z = float(Xc[2])
            if Z <= cfg.min_view_z:
                continue
            invZ = 1.0 / Z
            u = fx * Xc[0] * invZ + cx
            v = fy * Xc[1] * invZ + cy
            r = np.array([u - uvz[0], v - uvz[1]])
            e = float(np.hypot(r[0], r[1]))
            w = 1.0 if e <= cfg.huber_px else np.sqrt(cfg.huber_px / max(e, 1e-12))
            Jp = np.array([[fx * invZ, 0.0, -fx * Xc[0] * invZ * invZ],
                           [0.0, fy * invZ, -fy * Xc[1] * invZ * invZ]])
            Jl = Jp @ R                                   # (2,3)
            A = np.zeros((3, 6))
            A[0, 0] = A[1, 1] = A[2, 2] = 1.0
            A[0, 4], A[0, 5] = Xc[2], -Xc[1]
            A[1, 3], A[1, 5] = -Xc[2], Xc[0]
            A[2, 3], A[2, 4] = Xc[1], -Xc[0]
            Jc = Jp @ A                                   # (2,6)
            Jl_w, Jc_w, r_w = w * Jl, w * Jc, w * r
            ps = pose_sl(pid)
            if ps is not None:                            # free observing camera
                H[ps, ps] += Jc_w.T @ Jc_w
                H[ps, ps_lm] += Jc_w.T @ Jl_w
                H[ps_lm, ps] += Jl_w.T @ Jc_w
                b[ps] += Jc_w.T @ r_w
            H[ps_lm, ps_lm] += Jl_w.T @ Jl_w
            b[ps_lm] += Jl_w.T @ r_w

            if use_depth and uvz[2] > 0:
                z = float(uvz[2])
                sig = cfg.depth_sigma_coeff * z * z
                rz = (Z - z) / sig
                thr = cfg.depth_huber / sig
                wz = 1.0 if abs(rz) <= thr else np.sqrt(thr / max(abs(rz), 1e-12))
                Jcz = np.zeros(6)
                Jcz[2] = 1.0 / sig
                Jcz[3] = Xc[1] / sig
                Jcz[4] = -Xc[0] / sig
                Jlz = R[2, :] / sig                       # (3,)
                Jcz_w, Jlz_w, rz_w = wz * Jcz, wz * Jlz, wz * rz
                if ps is not None:
                    H[ps, ps] += np.outer(Jcz_w, Jcz_w)
                    H[ps, ps_lm] += np.outer(Jcz_w, Jlz_w)
                    H[ps_lm, ps] += np.outer(Jlz_w, Jcz_w)
                    b[ps] += Jcz_w * rz_w
                H[ps_lm, ps_lm] += np.outer(Jlz_w, Jlz_w)
                b[ps_lm] += Jlz_w * rz_w

    # --- dropped keyframe's gravity factor (camera-only, rotation block) ------
    # Only when the dropped keyframe is a free pose variable (a fixed gauge
    # anchor has no pose DoF to constrain).
    ps_drop = pose_sl(drop_id)
    if cfg.use_gravity and ps_drop is not None:
        gw = (np.asarray(grav_world, np.float64)
              if grav_world is not None else np.array([0.0, 1.0, 0.0]))
        gw = gw / max(float(np.linalg.norm(gw)), 1e-12)
        a = kf_by_id[drop_id].get("accel")
        if a is not None:
            a = np.asarray(a, np.float64)
            an = float(np.linalg.norm(a))
            if np.isfinite(an) and an > 1e-6:
                R = np.asarray(kf_by_id[drop_id]["T_cw"], np.float64)[:3, :3]
                gwc = R @ gw
                rg = gwc + a / an              # down_meas = -a/|a|; r = gwc - down
                g_sig = max(cfg.gravity_sigma_rad, 1e-6)
                g_thr = cfg.gravity_huber / g_sig
                e_rho = float(np.linalg.norm(rg)) / g_sig
                wh = 1.0 if e_rho <= g_thr else np.sqrt(g_thr / max(e_rho, 1e-12))
                c = (wh / g_sig) ** 2
                Sg = skew(gwc)
                rot = slice(ps_drop.start + 3, ps_drop.start + 6)
                H[rot, rot] += c * (Sg.T @ Sg)
                b[rot] += c * (Sg @ rg)

    # --- fold the previous prior (FEJ at its own linearization points) --------
    if prev_prior is not None:
        d_prev = np.empty(6 * len(prev_ids))
        for a, kid in enumerate(prev_ids):
            T_cur = np.asarray(kf_by_id[kid]["T_cw"], np.float64)
            T_lin_inv = np.linalg.inv(np.asarray(prev_prior.lin_Tcw[kid], np.float64))
            d_prev[6 * a:6 * a + 6] = se3_log(T_cur @ T_lin_inv)
        g_prev = prev_prior.H @ d_prev + prev_prior.b0
        for a, kid in enumerate(prev_ids):
            ra = pose_sl(kid)
            b[ra] += g_prev[6 * a:6 * a + 6]
            for bb, kid2 in enumerate(prev_ids):
                cb = pose_sl(kid2)
                H[ra, cb] += prev_prior.H[6 * a:6 * a + 6, 6 * bb:6 * bb + 6]

    H = 0.5 * (H + H.T)

    # --- Schur-complement out mu, leaving the prior over the kept poses -------
    # mu = hosted landmarks (always) + the dropped pose (only when it is free).
    # kappa = the kept keyframe poses. With a fixed drop the pose block is absent
    # from the layout, so all pose columns are kept.
    if drop_fixed:
        mu = list(range(6 * nP, D))              # hosted landmarks only
        kappa = list(range(0, 6 * nP))           # all (kept) poses
    else:
        mu = list(range(0, 6)) + list(range(6 * nP, D))   # drop pose + landmarks
        kappa = list(range(6, 6 * nP))           # kept poses
    mu = np.array(mu, dtype=np.int64)
    kappa = np.array(kappa, dtype=np.int64)

    Hmm = H[np.ix_(mu, mu)].copy()
    Hmm[np.diag_indices_from(Hmm)] += 1e-9       # ridge for invertibility
    Hmk = H[np.ix_(mu, kappa)]
    Hkk = H[np.ix_(kappa, kappa)]
    bm = b[mu]
    bk = b[kappa]

    # X = inv(Hmm) @ [Hmk | bm]
    try:
        X = np.linalg.solve(Hmm, np.column_stack([Hmk, bm]))
    except np.linalg.LinAlgError:
        X = np.linalg.lstsq(Hmm, np.column_stack([Hmk, bm]), rcond=None)[0]
    Xmk = X[:, :-1]
    Xbm = X[:, -1]
    H_p = Hkk - Hmk.T @ Xmk
    b_p = bk - Hmk.T @ Xbm
    H_p = 0.5 * (H_p + H_p.T)

    lin_Tcw = {kid: np.asarray(kf_by_id[kid]["T_cw"], np.float64).copy()
               for kid in kept_ids}
    return MargPrior(kf_ids=kept_ids, H=H_p, b0=b_p, lin_Tcw=lin_Tcw), hosted
