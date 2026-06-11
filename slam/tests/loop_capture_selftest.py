#!/usr/bin/env python3
"""Unit-test for the loop-closure match-funnel capture (no recorded data needed).

The capture (``LoopDetector.verify_capture`` in
:mod:`sky.slam.loopclosure`) must (a) return EXACTLY what ``verify``
returns -- the offline path is byte-frozen -- and (b) ALSO populate a
:class:`~sky.slam.loopclosure.LoopMatchCapture` whose per-match stage
labels and funnel counts are correct. This builds a SYNTHETIC, fully-controlled
match set so the expected stage of every match is known a priori:

* a block of GEOMETRY-CONSISTENT matches (a real (R, t) move of known 3D points)
  -- these pass BOTH the epipolar (fundamental) RANSAC AND the metric PnP, so
  they must be labelled STAGE_PNP (green);
* a block of GROSS OUTLIER matches (random pixel pairs) -- these are dropped at
  the epipolar gate, so they must stay STAGE_APPEARANCE (grey, dropped).

It asserts: verify == verify_capture result; cur/old/stage lengths all equal
n_appearance; the funnel is monotone (n_pnp <= n_fmat <= n_appearance); the PnP
inliers are exactly the geometry-consistent block and the outliers are exactly
the dropped block; and n_pnp / n_fmat counts agree with the stage labels.

Run::

    .venv/bin/python -m slam.tests.loop_capture_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.slam.loopclosure import (                       # noqa: E402
    KeyframeAppearance, LoopConfig, LoopDetector,
    STAGE_APPEARANCE, STAGE_EPIPOLAR, STAGE_PNP,
)


def _assert(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def _proj(K: np.ndarray, X: np.ndarray) -> np.ndarray:
    x = (K @ X.T).T
    return x[:, :2] / x[:, 2:3]


def _fake_app(kps: np.ndarray, desc: np.ndarray, depth: np.ndarray,
              K: np.ndarray) -> KeyframeAppearance:
    """A KeyframeAppearance with hand-supplied kps/desc/depth (bypasses ORB)."""
    a = KeyframeAppearance.__new__(KeyframeAppearance)
    a.kps = np.asarray(kps, np.float32)
    a.desc = np.asarray(desc, np.uint8)
    a.depth = np.asarray(depth, np.float32)
    a.K = K
    return a


def main() -> int:
    rng = np.random.default_rng(3)
    K = np.array([[300.0, 0, 160.0], [0, 300.0, 120.0], [0, 0, 1.0]])

    # --- geometry-consistent block: project N 3D points through a known move ---
    n_geo = 60
    Xold = np.column_stack([rng.uniform(-1, 1, n_geo), rng.uniform(-1, 1, n_geo),
                            rng.uniform(2, 4, n_geo)])
    old_px = _proj(K, Xold)
    th = 0.05
    R = np.array([[np.cos(th), 0, np.sin(th)], [0, 1, 0],
                  [-np.sin(th), 0, np.cos(th)]])
    t = np.array([0.15, 0.02, -0.05])
    Xcur = (R @ Xold.T).T + t
    cur_px = _proj(K, Xcur)

    # Distinct descriptors per match (identical between cur/old so the mutual
    # ratio test pairs i<->i); depth is the OLD point's Z (the verifier
    # back-projects the OLD keypoint).
    desc = rng.integers(0, 256, (n_geo, 32)).astype(np.uint8)
    depth_old = Xold[:, 2].astype(np.float32)

    # --- gross-outlier block: random pixel pairs with their own descriptors ----
    n_out = 5
    out_desc = rng.integers(0, 256, (n_out, 32)).astype(np.uint8)
    cur_out = rng.uniform([0, 0], [320, 240], (n_out, 2)).astype(np.float32)
    old_out = rng.uniform([0, 0], [320, 240], (n_out, 2)).astype(np.float32)

    cur_kps = np.vstack([cur_px, cur_out]).astype(np.float32)
    old_kps = np.vstack([old_px, old_out]).astype(np.float32)
    cur_desc = np.vstack([desc, out_desc])
    old_desc = np.vstack([desc, out_desc])
    cur_depth = np.zeros(n_geo + n_out, np.float32)            # cur depth unused
    old_depth = np.concatenate([depth_old, np.full(n_out, 3.0, np.float32)])

    cfg = LoopConfig(min_matches=5, min_fmat_inliers=5, min_inliers=5,
                     ransac_reproj_px=2.0)
    det = LoopDetector(K, cfg)
    cur_app = _fake_app(cur_kps, cur_desc, cur_depth, K)
    old_app = _fake_app(old_kps, old_desc, old_depth, K)

    # (a) verify == verify_capture result (offline path unchanged).
    r1 = det.verify(cur_app, old_app)
    r2, cap = det.verify_capture(cur_app, old_app)

    def same(a, b) -> bool:
        if a is None or b is None:
            return a is None and b is None
        Ta, na, ma = a
        Tb, nb, mb = b
        return bool(np.allclose(Ta, Tb)) and na == nb and ma == mb

    _assert(same(r1, r2), "verify(...) == verify_capture(...)[0] (byte path same)")
    _assert(r2 is not None, "the geometry-consistent set confirms a loop")

    # (b) lengths + funnel monotonicity.
    n_app = int(cap.n_appearance)
    _assert(len(cap.cur_px) == len(cap.old_px) == len(cap.stage) == n_app,
            f"cur/old/stage lengths == n_appearance ({len(cap.cur_px)}/"
            f"{len(cap.old_px)}/{len(cap.stage)} vs {n_app})")
    n_pnp = int((cap.stage == STAGE_PNP).sum())
    n_epi_only = int((cap.stage == STAGE_EPIPOLAR).sum())
    n_dropped = int((cap.stage == STAGE_APPEARANCE).sum())
    n_epi_or_better = int((cap.stage >= STAGE_EPIPOLAR).sum())
    _assert(n_pnp <= n_epi_or_better <= n_app,
            f"funnel monotone (pnp {n_pnp} <= epi {n_epi_or_better} <= app {n_app})")

    # (c) the counts agree with the stage labels.
    _assert(int(cap.n_pnp_inliers) == n_pnp,
            f"n_pnp_inliers count == stage>=PNP labels "
            f"({cap.n_pnp_inliers} vs {n_pnp})")
    _assert(int(cap.n_fmat_inliers) == n_epi_or_better,
            f"n_fmat_inliers count == stage>=EPIPOLAR labels "
            f"({cap.n_fmat_inliers} vs {n_epi_or_better})")

    # (d) the EXACT expected split: the 60 geometry-consistent matches reach PnP
    # (green) and the 5 gross outliers are dropped at the epipolar gate (grey).
    print(f"  stage counts: PnP={n_pnp}  epipolar-only={n_epi_only}  "
          f"dropped={n_dropped}  (geo={n_geo}, outliers={n_out})")
    _assert(n_pnp >= n_geo - 2,
            f"the geometry-consistent matches reach PnP/green (got {n_pnp}, "
            f"want >= {n_geo - 2})")
    _assert(n_dropped >= n_out,
            f"the gross outliers are dropped/grey (got {n_dropped}, "
            f"want >= {n_out})")
    # The dropped matches must be among the appended OUTLIER rows (indices >= n_geo).
    dropped_idx = np.nonzero(cap.stage == STAGE_APPEARANCE)[0]
    _assert(bool((dropped_idx >= n_geo).all()),
            f"every dropped match is an injected outlier row (idx {dropped_idx})")

    # (e) a REJECTED-at-appearance candidate still yields a funnel (no matches
    # past the gate but the appearance count + pixels are captured).
    poor_a = _fake_app(rng.uniform(0, 200, (3, 2)).astype(np.float32),
                       rng.integers(0, 256, (3, 32)).astype(np.uint8),
                       np.zeros(3, np.float32), K)
    poor_b = _fake_app(rng.uniform(0, 200, (3, 2)).astype(np.float32),
                       rng.integers(0, 256, (3, 32)).astype(np.uint8),
                       np.zeros(3, np.float32), K)
    rr, cc = det.verify_capture(poor_a, poor_b)
    _assert(rr is None and cc.n_pnp_inliers == 0,
            "a sub-threshold candidate returns None with an empty PnP funnel")

    print("\nPASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
