"""``sky.front.direct`` -- dense DIRECT RGB-D visual odometry (Stage-1 prototype).

WHY THIS EXISTS
---------------
At the 54x42 VL53-class ToF target the SPARSE corner/KLT VIO front-end suffers
**scale collapse** (measured Sim3 scale 0.23-0.63 against Basalt) and feature
dropouts (okfrac 0.54-0.81), giving ATE 50-98 cm vs 10-18 cm at full-res. The
root cause is feature starvation: only ~2300 px to find corners in. There simply
are not enough trackable corners for triangulation to fix the metric scale, so
the windowed BA / VO-translation-prior path under-estimates motion.

The research lever (Steinbrucker 2011; Kerl/Sturm/Cremers ICRA'13 "Robust
Odometry Estimation for RGB-D Cameras"; Whelan ICRA'13 "Robust Real-Time Visual
Odometry for Dense RGB-D Mapping") is **dense direct photometric alignment**:
align EVERY pixel that has a gradient (not just corners) using the ACCURATE
per-pixel ToF depth. Because the depth is given (metric), the pose is a pure
6-DoF SE(3) and the **scale is OBSERVED from the depth** rather than estimated
from feature triangulation -- which is exactly what should kill the scale
collapse, while using all gradient pixels kills the starvation.

This module is the from-scratch implementation of that estimator. It is a
research prototype: it estimates a single frame-to-keyframe relative pose, it
does NOT touch the frozen loose/tight live path, and it is exercised only by the
offline harness ``verification/direct_vo_bench.py``.

THE FORMULATION (so it can be checked by review)
------------------------------------------------
Unknown: ``T_cur_ref`` in SE(3) -- the rigid transform mapping a 3D point
expressed in the REFERENCE camera frame into the CURRENT camera frame.

For each reference pixel ``p = (u, v)`` that has a valid depth ``Z = depth_ref(p)``:

1. back-project:  ``P_ref = Z * K^{-1} [u, v, 1]^T``                  (ref frame)
2. transform:     ``P_cur = T_cur_ref @ P_ref``  (homogeneous)        (cur frame)
3. project:       ``w = pi(P_cur) = (fx X/Zc + cx, fy Y/Zc + cy)``    (cur pixels)
4. residual:      ``r(p) = I_cur(w) - I_ref(p)``   (bilinear-sampled intensities)

We minimise ``sum_p rho( w_p * r(p)^2 )`` over the SE(3) twist using
Gauss-Newton with a LEFT perturbation on the current estimate:

    ``T_cur_ref  <-  Exp(dxi) @ T_cur_ref``,   ``dxi = [rho(3); phi(3)]``.

The per-pixel Jacobian of the residual w.r.t. ``dxi`` (evaluated at ``dxi = 0``)
is the chain rule

    ``J_p = g_p^T  *  J_pi(P_cur)  *  J_warp``

where
  * ``g_p = [I_cur_x(w), I_cur_y(w)]`` -- image gradient of the CURRENT image at
    the warped location (px / px), obtained by Sobel + bilinear sampling.
  * ``J_pi`` -- the 2x3 Jacobian of the pinhole projection ``pi`` w.r.t. the 3D
    point ``P_cur = (X, Y, Z)``:
        ``[[fx/Z,    0,   -fx X / Z^2],
          [  0,   fy/Z,   -fy Y / Z^2]]``
  * ``J_warp`` -- the 3x6 Jacobian of ``P_cur`` w.r.t. the LEFT twist ``dxi``.
    For a left perturbation ``Exp(dxi) @ T`` acting on a point already at
    ``P_cur`` this is ``d(Exp(dxi) P_cur)/d dxi |_0 = [ I_3 | -skew(P_cur) ]``
    (translation-first twist order, matching :mod:`sky.math`). The minus sign on
    the rotation block is the derivative of ``skew(phi) @ P = -skew(P) @ phi``.

So ``J_p = g_p^T @ J_pi @ [I_3 | -skew(P_cur)]`` is a 1x6 row. The Gauss-Newton
normal equations accumulate ``H = sum_p w_p J_p^T J_p`` (6x6) and
``b = -sum_p w_p J_p r(p)`` (6x1); the update is ``dxi = solve(H, b)`` (with a
small Levenberg-Marquardt diagonal damping for conditioning), applied on the
left. We iterate to convergence per pyramid level, COARSE -> FINE.

ROBUST WEIGHTING (Kerl)
-----------------------
The photometric residual is heavy-tailed (occlusion, ToF depth holes, moving
edges). We use the iteratively-reweighted **Student-t** weight Kerl prescribes:
``w(r) = (nu + 1) / (nu + (r / sigma)^2)`` with ``nu = 5`` and ``sigma``
re-estimated each iteration from the residuals (the t-distribution scale). A
Huber weight is also provided as a fallback. Per-pixel depth validity (and the
warp falling inside the current image, with positive projected depth) gates which
pixels contribute at all.

LEAF / PORT RULES
-----------------
Keeps ``sky.*`` a leaf: imports only ``numpy``, :mod:`sky.math`, and -- lazily,
inside the functions that need it, mirroring :mod:`sky.front.odometry` -- ``cv2``
(for Sobel gradients + the image pyramid resize). ``sky.assert_import_clean()``
passes. No process / comms / io module is reachable. Maps onto the C
``libskyfront`` layer alongside the KLT/PnP front-end.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sky.math import se3_exp

__all__ = [
    "DirectConfig",
    "estimate_pose_direct",
    "build_pyramid",
]


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
@dataclass
class DirectConfig:
    """Tunables for :func:`estimate_pose_direct` (all with research defaults)."""

    levels: int = 3
    """Number of image/depth pyramid levels (level 0 = full resolution)."""

    max_iters: int = 30
    """Max Gauss-Newton iterations PER pyramid level."""

    min_grad: float = 4.0
    """Reference pixels whose CURRENT-image gradient magnitude (at the warp) is
    below this are kept but contribute little; we instead pre-select reference
    pixels by their gradient to focus the solve on informative pixels. This is
    the |grad| threshold (intensity units / px) for that pre-selection."""

    huber_delta: float = 4.0
    """Huber transition (intensity units). Only used when ``robust='huber'``."""

    t_dof: float = 5.0
    """Student-t degrees of freedom ``nu`` (Kerl uses 5). Only for ``robust='t'``."""

    robust: str = "t"
    """Robust weight: ``'t'`` (Student-t, Kerl -- default) or ``'huber'``."""

    convergence_eps: float = 1e-6
    """Stop a level early once ``||dxi||^2`` drops below this."""

    lm_damping: float = 1e-3
    """Levenberg-Marquardt diagonal damping factor ``lambda`` added as
    ``lambda * diag(H)`` for numerical conditioning of the 6x6 solve."""

    min_valid_frac: float = 0.02
    """If fewer than this fraction of selected pixels remain usable at a level
    (after validity + in-bounds gating), skip the level (too little signal)."""

    depth_min: float = 0.05
    """Minimum valid depth (m). Depths <= this are treated as invalid holes."""

    depth_max: float = 30.0
    """Maximum valid depth (m). Depths >= this are treated as invalid."""

    max_pixels: int = 6000
    """Cap on selected reference pixels per level (sub-sample the highest-gradient
    ones beyond this) to bound the per-iteration cost. 0 == no cap."""

    # Convenience: how many top-gradient pixels to *select* per level as a
    # fraction of the valid set (1.0 == use all valid). Kept conservative so the
    # solve is driven by textured regions, matching the direct-VO literature.
    grad_select_frac: float = 1.0


# --------------------------------------------------------------------------- #
# Pyramid construction (valid-aware depth reduction)
# --------------------------------------------------------------------------- #
def build_pyramid(
    gray: np.ndarray,
    depth: np.ndarray | None,
    K: np.ndarray,
    levels: int,
    *,
    depth_min: float = 0.05,
    depth_max: float = 30.0,
) -> list[tuple[np.ndarray, np.ndarray | None, np.ndarray]]:
    """Build a ``levels``-level (gray, depth, K) pyramid, coarse handled by caller.

    Returned list is ordered FINE -> COARSE: index 0 is full resolution, index
    ``levels-1`` is the coarsest. Each entry is ``(gray_f32, depth_or_None, K_l)``.

    * GRAY is reduced with ``cv2.pyrDown`` (Gaussian + 2x decimation), the
      standard intensity reduction, returned as float32.
    * DEPTH is reduced VALID-AWARE: a 2x2 block reduces to the MEDIAN of its
      valid (in ``[depth_min, depth_max]``) members, or 0 (invalid) if the whole
      block is holes. This never blends a real depth with a 0-hole or across a
      depth discontinuity -- a naive blur/pyrDown would, corrupting the metric
      back-projection that gives this method its scale.
    * K is scaled per level: ``fx, fy, cx, cy`` are halved each downscale (the
      standard pinhole-under-decimation rule, with the +0.5/-0.5 pixel-centre
      convention folded in: ``c' = (c + 0.5)/2 - 0.5``).

    ``depth`` may be None (the CURRENT frame needs only intensity, not depth);
    then every level's depth entry is None.
    """
    import cv2  # lazy (leaf rule -- mirrors sky.front.odometry)

    g0 = np.ascontiguousarray(gray, dtype=np.float32)
    d0 = None if depth is None else np.ascontiguousarray(depth, dtype=np.float32)
    K0 = np.asarray(K, dtype=np.float64).copy()

    pyr: list[tuple[np.ndarray, np.ndarray | None, np.ndarray]] = [(g0, d0, K0)]
    for _ in range(1, levels):
        g_prev, d_prev, K_prev = pyr[-1]
        # cv2.pyrDown DEFINES the canonical level size ((W+1)//2 x (H+1)//2 for
        # odd dims). We reduce depth valid-aware to THAT EXACT shape so the gray
        # and depth grids never disagree (the 54x42 odd-dim trap).
        g_next = cv2.pyrDown(g_prev)
        nh, nw = g_next.shape[:2]
        d_next = (None if d_prev is None
                  else _downsample_depth_valid(d_prev, nh, nw, depth_min, depth_max))
        # Pinhole K under 2x decimation with the pixel-centre convention.
        K_next = K_prev.copy()
        K_next[0, 0] *= 0.5                       # fx
        K_next[1, 1] *= 0.5                       # fy
        K_next[0, 2] = (K_prev[0, 2] + 0.5) * 0.5 - 0.5   # cx
        K_next[1, 2] = (K_prev[1, 2] + 0.5) * 0.5 - 0.5   # cy
        pyr.append((g_next, d_next, K_next))
    return pyr


def _downsample_depth_valid(
    depth: np.ndarray, out_h: int, out_w: int, dmin: float, dmax: float
) -> np.ndarray:
    """Valid-aware 2x depth reduction to an EXACT ``(out_h, out_w)`` target shape.

    Each output cell takes the MEDIAN of its source 2x2 block's VALID members
    (``dmin < d < dmax``); a block with no valid member maps to 0 (invalid). The
    target shape is taken from the matching ``cv2.pyrDown`` gray level so the two
    grids stay aligned even at odd dims (``out = (in+1)//2``): we pad the source
    by one row/col when needed so the 2x2 blocks tile exactly ``out_h x out_w``.
    This never blends a real depth with a 0-hole or across a depth edge -- a naive
    blur/pyrDown would, corrupting the metric back-projection.
    """
    h, w = depth.shape[:2]
    need_h, need_w = out_h * 2, out_w * 2
    d = depth.astype(np.float32)
    # Pad (edge-replicate) up to an exact 2x tiling of the target shape.
    if need_h > h or need_w > w:
        d = np.pad(d, ((0, max(0, need_h - h)), (0, max(0, need_w - w))),
                   mode="edge")
    d = d[:need_h, :need_w]
    valid = (d > dmin) & (d < dmax)
    # Stack the four 2x2-block members -> (out_h, out_w, 4).
    blocks = np.stack(
        [d[0::2, 0::2], d[0::2, 1::2], d[1::2, 0::2], d[1::2, 1::2]], axis=-1
    )
    vblocks = np.stack(
        [valid[0::2, 0::2], valid[0::2, 1::2],
         valid[1::2, 0::2], valid[1::2, 1::2]], axis=-1
    )
    # Median of the VALID members per block, computed warning-free (no nanmedian,
    # which emits an "All-NaN slice" RuntimeWarning on the all-holes blocks the ToF
    # depth is full of). Trick: push invalid members to +inf and SORT each block;
    # the valid values then occupy the first `cnt` slots in ascending order, so the
    # median is the average of the two middle valid slots (selected by `cnt`).
    cnt = vblocks.sum(axis=-1)                       # (out_h, out_w) valid count 0..4
    srt = np.sort(np.where(vblocks, blocks, np.inf), axis=-1)  # valids first, asc
    # Lower/upper middle indices of the valid run (clamped; only used where cnt>0).
    lo = np.clip((cnt - 1) // 2, 0, 3)
    hi = np.clip(cnt // 2, 0, 3)
    ii, jj = np.indices(cnt.shape)
    med = 0.5 * (srt[ii, jj, lo] + srt[ii, jj, hi])
    return np.where(cnt > 0, med, 0.0).astype(np.float32)


# --------------------------------------------------------------------------- #
# Bilinear sampling + gradients
# --------------------------------------------------------------------------- #
def _bilinear_sample(img: np.ndarray, u: np.ndarray, v: np.ndarray):
    """Bilinear-sample ``img`` (HxW float) at floating (u, v); vectorised.

    Returns ``(values, valid)`` where ``valid`` is the boolean mask of samples
    whose 2x2 support lies fully inside the image. Out-of-bounds samples return 0
    (and ``valid=False``) so the caller can drop them.
    """
    h, w = img.shape[:2]
    u0 = np.floor(u).astype(np.int64)
    v0 = np.floor(v).astype(np.int64)
    u1 = u0 + 1
    v1 = v0 + 1

    valid = (u0 >= 0) & (v0 >= 0) & (u1 <= w - 1) & (v1 <= h - 1)
    # Clamp indices so the gather is always in-range; invalid entries are masked
    # out by the caller via `valid`, so their clamped value is irrelevant.
    u0c = np.clip(u0, 0, w - 1)
    u1c = np.clip(u1, 0, w - 1)
    v0c = np.clip(v0, 0, h - 1)
    v1c = np.clip(v1, 0, h - 1)

    wu = u - u0
    wv = v - v0
    Ia = img[v0c, u0c]
    Ib = img[v0c, u1c]
    Ic = img[v1c, u0c]
    Id = img[v1c, u1c]
    top = Ia * (1.0 - wu) + Ib * wu
    bot = Ic * (1.0 - wu) + Id * wu
    val = top * (1.0 - wv) + bot * wv
    return val, valid


def _image_gradients(img: np.ndarray):
    """Central-difference image gradients ``(gx, gy)`` (intensity / px), via Sobel.

    Uses a normalised Sobel (``cv2.Sobel`` / 8) so the result matches a central
    difference in magnitude. ``img`` is float32; returns two float32 arrays.
    """
    import cv2  # lazy (leaf rule)

    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3) / 8.0
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3) / 8.0
    return gx, gy


# --------------------------------------------------------------------------- #
# Robust weights (Kerl Student-t, or Huber)
# --------------------------------------------------------------------------- #
def _t_scale_and_weights(r: np.ndarray, nu: float, n_iters: int = 5):
    """Student-t IRLS: estimate scale ``sigma`` then per-residual weights.

    Implements Kerl ICRA'13 eq. (7)-(9): the t-distribution variance is solved by
    the fixed-point iteration
        ``sigma^2 <- mean( r^2 * (nu + 1) / (nu + (r/sigma)^2) )``
    seeded from the residual MAD, then the weights are
        ``w(r) = (nu + 1) / (nu + (r/sigma)^2)``.
    Returns ``(weights, sigma)``.
    """
    if r.size == 0:
        return np.ones_like(r), 1.0
    # Robust seed for sigma (MAD -> Gaussian-consistent stdev).
    sigma2 = max(np.median(np.abs(r)) * 1.4826, 1e-3) ** 2
    for _ in range(n_iters):
        w = (nu + 1.0) / (nu + (r * r) / sigma2)
        new = float(np.mean(w * r * r))
        if new <= 1e-12:
            break
        if abs(new - sigma2) / max(sigma2, 1e-12) < 1e-3:
            sigma2 = new
            break
        sigma2 = new
    sigma = float(np.sqrt(max(sigma2, 1e-12)))
    w = (nu + 1.0) / (nu + (r * r) / (sigma * sigma))
    return w, sigma


def _huber_weights(r: np.ndarray, delta: float):
    """Huber IRLS weights ``w = 1`` for ``|r| <= delta`` else ``delta / |r|``."""
    a = np.abs(r)
    w = np.ones_like(r)
    big = a > delta
    w[big] = delta / np.maximum(a[big], 1e-12)
    return w


# --------------------------------------------------------------------------- #
# The estimator
# --------------------------------------------------------------------------- #
@dataclass
class _LevelCache:
    """Pre-computed, pose-independent reference quantities for one pyramid level."""

    P_ref: np.ndarray   # (M, 3) back-projected reference points (ref frame)
    I_ref: np.ndarray   # (M,)   reference intensities at the selected pixels
    K: np.ndarray       # (3, 3) this level's intrinsics
    shape: tuple        # (H, W) of the current image at this level


def estimate_pose_direct(
    gray_ref: np.ndarray,
    depth_ref: np.ndarray,
    gray_cur: np.ndarray,
    K: np.ndarray,
    *,
    init_T: np.ndarray | None = None,
    levels: int = 3,
    max_iters: int = 30,
    cfg: DirectConfig | None = None,
) -> tuple[np.ndarray, dict]:
    """Dense direct RGB-D odometry: estimate ``T_cur_ref`` (4x4 SE(3)).

    Aligns the CURRENT image to the REFERENCE frame's geometry by minimising the
    photometric residual ``I_cur(warp(p)) - I_ref(p)`` over every selected
    reference pixel with valid depth, by coarse-to-fine Gauss-Newton on the SE(3)
    left twist (see the module docstring for the full formulation).

    Parameters
    ----------
    gray_ref, gray_cur : (H, W) arrays (uint8 or float) -- reference & current
        intensity images, SAME resolution.
    depth_ref : (H, W) float -- metric depth (m) for the REFERENCE frame; 0 (or
        outside ``[cfg.depth_min, cfg.depth_max]``) marks invalid pixels. The
        current frame needs NO depth (that is the point: pose only).
    K : (3, 3) pinhole intrinsics for this resolution.
    init_T : optional 4x4 SE(3) seed for ``T_cur_ref`` (e.g. an IMU/gyro prior);
        identity if None.
    levels, max_iters : pyramid depth and per-level GN iteration cap. If ``cfg``
        is given its ``levels`` / ``max_iters`` take precedence only when the
        positional args are left at the defaults; otherwise the explicit args win.
    cfg : optional :class:`DirectConfig` for the robust weight / thresholds.

    Returns
    -------
    (T_cur_ref, info) where ``info`` carries convergence diagnostics:
        ``converged`` (bool), ``final_rmse`` (intensity units, finest level),
        ``iters`` (total GN steps over all levels), ``valid_frac`` (fraction of
        selected finest-level pixels that stayed usable), ``n_pixels`` (finest
        level), ``per_level`` (list of dicts), ``sigma`` (final t-scale).
    """
    cfg = cfg or DirectConfig()
    # Explicit positional args win over cfg defaults when the caller set them.
    if levels != 3:
        cfg = _with(cfg, levels=levels)
    if max_iters != 30:
        cfg = _with(cfg, max_iters=max_iters)
    L = int(cfg.levels)

    gref = np.ascontiguousarray(gray_ref, dtype=np.float32)
    gcur = np.ascontiguousarray(gray_cur, dtype=np.float32)
    dref = np.ascontiguousarray(depth_ref, dtype=np.float32)

    pyr_ref = build_pyramid(gref, dref, K, L,
                            depth_min=cfg.depth_min, depth_max=cfg.depth_max)
    pyr_cur = build_pyramid(gcur, None, K, L)

    # Working estimate of T_cur_ref (left-perturbed during the solve).
    T = np.eye(4) if init_T is None else np.asarray(init_T, dtype=np.float64).copy()

    total_iters = 0
    per_level: list[dict] = []
    final_rmse = float("nan")
    final_valid_frac = 0.0
    final_n = 0
    final_sigma = float("nan")
    any_converged = False

    # COARSE -> FINE: iterate from the coarsest level (last in the FINE->COARSE
    # pyramid list) down to the finest (index 0).
    for lvl in range(L - 1, -1, -1):
        g_ref_l, d_ref_l, K_l = pyr_ref[lvl]
        g_cur_l, _, _ = pyr_cur[lvl]
        gx_l, gy_l = _image_gradients(g_cur_l)

        cache = _select_reference_pixels(g_ref_l, d_ref_l, K_l, cfg)
        if cache is None:
            per_level.append({"level": lvl, "skipped": True, "reason": "no_valid_px"})
            continue

        T, lvl_info = _solve_level(
            T, cache, g_cur_l, gx_l, gy_l, K_l, cfg)
        lvl_info["level"] = lvl
        per_level.append(lvl_info)
        total_iters += lvl_info["iters"]
        any_converged = any_converged or lvl_info["converged"]
        if lvl == 0:
            final_rmse = lvl_info["rmse"]
            final_valid_frac = lvl_info["valid_frac"]
            final_n = lvl_info["n_pixels"]
            final_sigma = lvl_info["sigma"]

    info = {
        "converged": bool(any_converged),
        "final_rmse": final_rmse,
        "iters": total_iters,
        "valid_frac": final_valid_frac,
        "n_pixels": final_n,
        "sigma": final_sigma,
        "per_level": per_level,
    }
    return T, info


def _with(cfg: DirectConfig, **kw) -> DirectConfig:
    """Return a copy of ``cfg`` with the given fields overridden (dataclass replace)."""
    from dataclasses import replace
    return replace(cfg, **kw)


def _select_reference_pixels(
    g_ref: np.ndarray, d_ref: np.ndarray, K: np.ndarray, cfg: DirectConfig
) -> _LevelCache | None:
    """Pick informative reference pixels (valid depth + texture) and back-project.

    Returns a :class:`_LevelCache` with the back-projected 3D points (ref frame)
    and the reference intensities, or None if too few valid pixels exist. Pixel
    SELECTION is driven by the REFERENCE-image gradient (texture there is what
    makes the photometric error informative) and by depth validity.
    """
    h, w = g_ref.shape[:2]
    if d_ref is None:
        return None

    gx, gy = _image_gradients(g_ref)
    gmag = np.sqrt(gx * gx + gy * gy)

    valid_depth = (d_ref > cfg.depth_min) & (d_ref < cfg.depth_max)
    textured = gmag >= cfg.min_grad
    mask = valid_depth & textured
    n_valid = int(mask.sum())
    if n_valid < max(8, int(cfg.min_valid_frac * h * w)):
        # Fall back to ALL valid-depth pixels (low-texture scene): better a noisy
        # solve than no solve, and the robust weight will down-weight flat ones.
        mask = valid_depth
        n_valid = int(mask.sum())
        if n_valid < 8:
            return None

    vs, us = np.nonzero(mask)
    Z = d_ref[vs, us].astype(np.float64)
    I = g_ref[vs, us].astype(np.float64)

    # Optional: keep only the highest-gradient subset, and/or cap the count.
    gsel = gmag[vs, us]
    if 0.0 < cfg.grad_select_frac < 1.0 and us.size > 16:
        keep = max(16, int(cfg.grad_select_frac * us.size))
        idx = np.argpartition(gsel, -keep)[-keep:]
        us, vs, Z, I = us[idx], vs[idx], Z[idx], I[idx]
        gsel = gsel[idx]
    if cfg.max_pixels and us.size > cfg.max_pixels:
        idx = np.argpartition(gsel, -cfg.max_pixels)[-cfg.max_pixels:]
        us, vs, Z, I = us[idx], vs[idx], Z[idx], I[idx]

    # Back-project: P = Z * K^{-1} [u, v, 1]^T (reference camera frame).
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    X = (us.astype(np.float64) - cx) * Z / fx
    Y = (vs.astype(np.float64) - cy) * Z / fy
    P_ref = np.stack([X, Y, Z], axis=1)  # (M, 3)

    return _LevelCache(P_ref=P_ref, I_ref=I, K=np.asarray(K, np.float64),
                       shape=(h, w))


def _solve_level(
    T: np.ndarray,
    cache: _LevelCache,
    g_cur: np.ndarray,
    gx_cur: np.ndarray,
    gy_cur: np.ndarray,
    K: np.ndarray,
    cfg: DirectConfig,
) -> tuple[np.ndarray, dict]:
    """One pyramid level of robust Gauss-Newton. Returns (updated T, level info)."""
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    P_ref = cache.P_ref          # (M, 3) reference-frame points
    I_ref = cache.I_ref          # (M,)
    M = P_ref.shape[0]

    converged = False
    iters_done = 0
    rmse = float("nan")
    valid_frac = 0.0
    sigma = float("nan")

    for it in range(cfg.max_iters):
        iters_done = it + 1
        R = T[:3, :3]
        t = T[:3, 3]
        # Transform reference points into the CURRENT camera frame.
        P_cur = (R @ P_ref.T).T + t      # (M, 3)
        Xc, Yc, Zc = P_cur[:, 0], P_cur[:, 1], P_cur[:, 2]

        in_front = Zc > cfg.depth_min
        # Project (guard the divide; out-of-front pixels are masked out anyway).
        Zc_safe = np.where(in_front, Zc, 1.0)
        u = fx * Xc / Zc_safe + cx
        v = fy * Yc / Zc_safe + cy

        I_cur, in_img = _bilinear_sample(g_cur, u, v)
        gxs, _ = _bilinear_sample(gx_cur, u, v)
        gys, _ = _bilinear_sample(gy_cur, u, v)

        usable = in_front & in_img
        n_use = int(usable.sum())
        valid_frac = n_use / max(M, 1)
        if n_use < 8:
            break

        # Photometric residual on the usable subset.
        r = (I_cur - I_ref)[usable]

        # Robust weights (Kerl Student-t or Huber).
        if cfg.robust == "huber":
            wts = _huber_weights(r, cfg.huber_delta)
            sigma = float(np.median(np.abs(r)) * 1.4826)
        else:
            wts, sigma = _t_scale_and_weights(r, cfg.t_dof)

        rmse = float(np.sqrt(np.mean(r * r)))

        # ---- Per-pixel 1x6 Jacobians (left twist, translation-first) -------- #
        # J = g^T @ J_pi @ [I3 | -skew(P_cur)]
        # J_pi (2x3) per pixel:
        #   [[fx/Z, 0, -fx X/Z^2],
        #    [0, fy/Z, -fy Y/Z^2]]
        Xu = Xc[usable]
        Yu = Yc[usable]
        Zu = Zc[usable]
        invZ = 1.0 / Zu
        invZ2 = invZ * invZ
        gxu = gxs[usable]
        gyu = gys[usable]

        # row = g^T @ J_pi  (1x3): the image-gradient-weighted projection Jacobian.
        # a = d(residual)/dX, b = d/dY, c = d/dZ  (in current camera coords).
        a = gxu * (fx * invZ)
        b = gyu * (fy * invZ)
        c = -(gxu * fx * Xu + gyu * fy * Yu) * invZ2
        # Now J_warp = [I3 | -skew(P_cur)]; multiply row=[a,b,c] by it:
        #   translation block (I3):           [a, b, c]
        #   rotation block (-skew(P_cur)):
        #     -skew(P) = [[0, Z, -Y], [-Z, 0, X], [Y, -X, 0]]
        #     row @ (-skew(P)) = [ -b*Z + c*Y,  a*Z - c*X,  -a*Y + b*X ]
        J = np.empty((n_use, 6), dtype=np.float64)
        J[:, 0] = a
        J[:, 1] = b
        J[:, 2] = c
        J[:, 3] = -b * Zu + c * Yu
        J[:, 4] = a * Zu - c * Xu
        J[:, 5] = -a * Yu + b * Xu

        # Weighted normal equations: H = J^T W J, g = -J^T W r.
        WJ = J * wts[:, None]
        H = J.T @ WJ                      # (6, 6)
        grad = -(J.T @ (wts * r))         # (6,)

        # Levenberg-Marquardt diagonal damping for conditioning.
        H[np.diag_indices(6)] += cfg.lm_damping * np.diag(H)
        # Tiny absolute floor so an all-zero column (degenerate DoF) is solvable.
        H[np.diag_indices(6)] += 1e-9

        try:
            dxi = np.linalg.solve(H, grad)
        except np.linalg.LinAlgError:
            break

        # Apply on the LEFT: T <- Exp(dxi) @ T  (dxi = [rho; phi]).
        T = se3_exp(dxi) @ T

        if float(dxi @ dxi) < cfg.convergence_eps:
            converged = True
            break

    return T, {
        "iters": iters_done,
        "converged": converged,
        "rmse": rmse,
        "valid_frac": valid_frac,
        "n_pixels": M,
        "sigma": sigma,
    }
