#!/usr/bin/env python3
"""Schur-complement equivalence gate: scatter reduction == dense reference.

The tight VIO solve (``sky.vio.window.optimize_vio`` with IMU factors present)
marginalises the block-diagonal landmark Hessian out of the damped normal
equations and solves the small nav-only Schur system. The reduced system
``S = App - Apl All^-1 Apl.T`` / ``rp = gp - Apl All^-1 gl`` is assembled by a
per-landmark SCATTER (``_schur_solve``): each landmark contributes only a tiny
``(k_l x k_l)`` outer product on its observers' pose columns. This must be
**algebraically exact** -- identical (to floating-point round-off) to the dense
reference ``_schur_reduce_dense`` (which forms the same reduction over the full
nav width) AND to the full dense ``np.linalg.solve(A, -b)``.

The gate drives the optimiser on the synthetic VI window from
``vio_ba_selftest`` across the 4 tight configs (loose-sigma / tilt-lock /
covariance-weighted / tilt+infowt+ZUPT+CV) and, on EVERY inner LM damping-retry
solve (via the ``_solve_probe`` hook), recomputes both references from the same
``(A, b)`` the production scatter used and compares all three deltas. PASS iff
the worst gap over all configs / all inner solves is below ``1e-9``.

Run::

    .venv/bin/python -m vio.tests.schur_equiv_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from vio.tests.vio_ba_selftest import (                            # noqa: E402
    make_world, build_factors, G, K)
from sky.vio.window import (                                        # noqa: E402
    VioState, VioConfig, optimize_vio,
    _schur_partition, _schur_reduce_dense)
from sky.math import so3_exp_unit as so3_exp                        # noqa: E402

GATE = 1e-9


def _dense_delta(A, b, nav_dim, M):
    """Reference: dense Schur reduction + solve + back-substitution."""
    App, Apl, gp, gl, All_inv = _schur_partition(A, b, nav_dim, M)
    S, rp = _schur_reduce_dense(App, Apl, gp, gl, All_inv, nav_dim, M)
    dp = np.linalg.solve(S, -rp)
    rhs = -gl - np.einsum('nmk,n->mk', Apl.reshape(nav_dim, M, 3), dp)
    dl = np.einsum('mkj,mj->mk', All_inv, rhs).reshape(-1)
    out = np.empty(A.shape[0])
    out[:nav_dim] = dp
    out[nav_dim:] = dl
    return out


def _perturbed(lock_tilt):
    """Perturbed initial guess for the synthetic VI window (anchor KF0 fixed)."""
    w = make_world()
    nKF = len(w["kf_R"])
    rng = np.random.default_rng(7)
    gt = VioState(R=[r.copy() for r in w["kf_R"]],
                  p=[x.copy() for x in w["kf_p"]],
                  v=[x.copy() for x in w["kf_v"]],
                  bg=[np.zeros(3) for _ in range(nKF)],
                  ba=[np.zeros(3) for _ in range(nKF)],
                  landmarks=w["lms"].copy())
    up = -G / np.linalg.norm(G)
    st = gt.copy()
    for i in range(1, nKF):
        if lock_tilt:
            st.R[i] = so3_exp(up * rng.normal(0, np.radians(3.0))) @ st.R[i]
        else:
            st.R[i] = st.R[i] @ so3_exp(rng.normal(0, np.radians(3.0), 3))
        st.p[i] = st.p[i] + rng.normal(0, 0.05, 3)
        st.v[i] = st.v[i] + rng.normal(0, 0.1, 3)
    st.v[0] = gt.v[0] + rng.normal(0, 0.1, 3)
    st.landmarks = st.landmarks + rng.normal(0, 0.03, st.landmarks.shape)
    factors = build_factors(w["imu_raw"], np.zeros(3), np.zeros(3))
    return w, st, factors


CONFIGS = [
    ("A loose-sigma",     dict(lock_tilt=False, imu_info_weight=False)),
    ("B lock_tilt",       dict(lock_tilt=True,  imu_info_weight=False)),
    ("C imu_info_weight", dict(lock_tilt=False, imu_info_weight=True)),
    ("D tilt+infowt+zupt+cv",
     dict(lock_tilt=True, imu_info_weight=True, vel_zupt=True,
          vel_cv_prior=True)),
]


def main() -> int:
    M = make_world()["lms"].shape[0]
    print("=== Schur scatter-vs-dense equivalence gate "
          f"(M={M} landmarks, gate {GATE:.0e}) ===")
    global_max = 0.0
    total_solves = 0
    for name, kw in CONFIGS:
        w, st, factors = _perturbed(kw.get("lock_tilt", False))
        cfg = VioConfig(max_iters=40, **kw)
        worst = {"d": 0.0, "n": 0}

        def probe(A, b, delta, _w=worst):
            A = np.asarray(A)
            b = np.asarray(b)
            nav_dim = A.shape[0] - 3 * M
            ref = _dense_delta(A, b, nav_dim, M)        # dense Schur reference
            full = np.linalg.solve(A, -b)               # full dense reference
            d = max(float(np.max(np.abs(np.asarray(delta) - ref))),
                    float(np.max(np.abs(np.asarray(delta) - full))))
            _w["d"] = max(_w["d"], d)
            _w["n"] += 1

        optimize_vio(K, st, w["obs_cam"], w["obs_lm"], w["obs_uv"],
                     w["obs_d"], factors, G, cfg, anchor=0, _solve_probe=probe)
        global_max = max(global_max, worst["d"])
        total_solves += worst["n"]
        print(f"  {name:24s}  solves={worst['n']:4d}  "
              f"max|d_scatter - d_ref| = {worst['d']:.3e}")

    print(f"\nGLOBAL max|d_scatter - d_ref| = {global_max:.3e} "
          f"over {total_solves} inner solves / {len(CONFIGS)} tight configs")
    ok = global_max < GATE
    print("PASS" if ok else f"FAIL (gate {GATE:.0e})")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
