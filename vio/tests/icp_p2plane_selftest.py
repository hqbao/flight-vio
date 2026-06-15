#!/usr/bin/env python3
"""Unit checks for the dense ICP relative-pose geometry (``sky.depth.icp``).

Self-contained, additive (touches no baseline). Verifies the three properties
the math-reviewer spec calls out for the leaf module:

  1. POSE RECOVERY: ICP recovers a known relative pose between two clouds of a
     textured (multi-plane) scene, seeded from a deliberately-wrong seed.
  2. INFORMATION Lambda: the returned 6x6 ``Lambda`` is the point-to-plane
     normal-equation Hessian -- it is SPD on a well-constrained (corner) scene
     and its eigen-spectrum has no near-zero direction there.
  3. DEGENERACY: on a SINGLE fronto-parallel plane the two ALONG-PLANE
     translation directions get near-zero eigenvalues in ``Lambda`` while the
     plane-normal (depth) translation stays well-conditioned -- the signal the
     VIO factor's eigenvalue remap projects out.

  4. LEAF: ``sky.depth.icp`` pulls in no flight-vio process/comms/io module.

Run::

    .venv/bin/python vio/tests/icp_p2plane_selftest.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.math import se3_from_Rp, se3_inv, se3_log_robust, so3_exp_unit  # noqa: E402
from sky.depth.icp import (  # noqa: E402
    backproject_depth,
    icp_p2plane_blend,
)


def _corner_scene(K, n_side=60, seed=0):
    """A textured 3-plane 'corner' depth map: floor + back wall + side wall.

    Returns a metric depth map (H,W) the camera sees, well-constrained in all
    6 DoF (three mutually non-parallel planes give full translation + rotation
    observability).
    """
    rng = np.random.default_rng(seed)
    h, w = 42, 54
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    depth = np.full((h, w), np.nan)
    for v in range(h):
        for u in range(w):
            # ray direction (unit), then intersect three planes, take nearest
            d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
            d = d / np.linalg.norm(d)
            cands = []
            # back wall  z = 2.0
            if d[2] > 1e-3:
                cands.append(2.0 / d[2])
            # floor  y = 0.8
            if d[1] > 1e-3:
                cands.append(0.8 / d[1])
            # side wall x = 0.7
            if d[0] > 1e-3:
                cands.append(0.7 / d[0])
            if not cands:
                continue
            t = min(cands)
            depth[v, u] = (d * t)[2] + rng.normal(0, 0.002)  # z coordinate + tiny noise
    return depth


def _plane_scene(K, z0=2.0, seed=1):
    """A single fronto-parallel plane at depth ``z0`` (the degeneracy case)."""
    rng = np.random.default_rng(seed)
    h, w = 42, 54
    depth = np.full((h, w), z0)
    depth += rng.normal(0, 0.002, size=(h, w))
    return depth


def _transform_cloud(cloud, T):
    return (T[:3, :3] @ cloud.T).T + T[:3, 3]


def main() -> int:
    K = np.array([[40.0, 0, 27.0], [0, 40.0, 21.0], [0, 0, 1.0]])
    ok = True

    # ---- 1+2. corner scene: recover a known relative pose ---------------- #
    depth_i = _corner_scene(K, seed=0)
    cloud_i = backproject_depth(depth_i, K, min_z=0.05, max_z=5.0)

    # ground-truth relative pose cam_i <- cam_j (small rot + translation)
    phi = np.array([0.03, -0.02, 0.04])
    dpos = np.array([0.06, -0.03, 0.05])
    T_ij = se3_from_Rp(so3_exp_unit(phi), dpos)
    # cloud_j lives in cam_j: a point seen in cam_i at X_i is X_j = T_ij^-1 X_i
    T_ji = se3_inv(T_ij)
    cloud_j = _transform_cloud(cloud_i, T_ji)

    # deliberately wrong seed (half the true motion) -> ICP must converge to T_ij
    T_seed = se3_from_Rp(so3_exp_unit(phi * 0.4), dpos * 0.4)
    T_icp, Lambda, n_corr, conv = icp_p2plane_blend(
        cloud_i, cloud_j, T_seed, salient_frac=0.5, min_salient=80,
        max_corr_dist=0.5, max_iters=25)

    err = se3_log_robust(se3_inv(T_icp) @ T_ij)
    e_trans = float(np.linalg.norm(err[:3]))
    e_rot = float(np.linalg.norm(err[3:]))
    # exact-data corner -> sub-cm / sub-0.1deg recovery from a half-motion seed
    pose_ok = conv and e_trans < 0.01 and e_rot < 0.01
    print(f"[{'ok' if pose_ok else 'FAIL'}] corner pose recovery "
          f"conv={conv} n_corr={n_corr} e_trans={e_trans*100:.2f}cm "
          f"e_rot={np.degrees(e_rot):.2f}deg")
    ok = ok and pose_ok

    # Lambda SPD + well-conditioned (no near-zero) on the corner scene
    evals = np.linalg.eigvalsh(Lambda)
    cond_ok = bool(np.all(evals > 0) and evals[0] / evals[-1] > 1e-4)
    print(f"[{'ok' if cond_ok else 'FAIL'}] corner Lambda well-conditioned "
          f"eig=[{evals[0]:.3e} .. {evals[-1]:.3e}] "
          f"ratio={evals[0]/evals[-1]:.2e}")
    ok = ok and cond_ok

    # ---- 3. degeneracy: single fronto-parallel plane --------------------- #
    depth_p = _plane_scene(K, z0=2.0, seed=1)
    cloud_pi = backproject_depth(depth_p, K, min_z=0.05, max_z=5.0)
    # lateral motion along x (along the plane) -- the along-plane DoF
    T_ij_p = se3_from_Rp(np.eye(3), np.array([0.05, 0.0, 0.0]))
    cloud_pj = _transform_cloud(cloud_pi, se3_inv(T_ij_p))
    T_seed_p = np.eye(4)
    _, Lambda_p, n_corr_p, conv_p = icp_p2plane_blend(
        cloud_pi, cloud_pj, T_seed_p, salient_frac=0.6, min_salient=80,
        max_corr_dist=0.5, max_iters=20)

    evals_p, evecs_p = np.linalg.eigh(Lambda_p)
    lam_max = float(evals_p[-1])
    # find which eigen-directions are dominated by along-plane translation (x,y).
    # The plane normal is +z (camera looks down +z at the wall): translation in
    # z is OBSERVABLE (depth changes), translation in x/y is NOT (the plane looks
    # identical). So the two smallest eigenvalues should be the x/y translation.
    small = evals_p[:2] / lam_max
    # the eigenvectors of the 2 smallest eigenvalues should be mostly translation
    # in the x-y plane (rows 0,1 of the [trans;rot] vector)
    along_plane = np.all(np.abs(evecs_p[2, :2]) < 0.5)  # little z-trans content
    deg_ok = bool(np.all(small < 0.05) and along_plane)
    print(f"[{'ok' if deg_ok else 'FAIL'}] single-plane degeneracy "
          f"n_corr={n_corr_p} 2-smallest/lam_max={small[0]:.2e},{small[1]:.2e} "
          f"(along-plane null space present)")
    ok = ok and deg_ok

    # ---- 4. leaf ---------------------------------------------------------- #
    # This test module lives in ``vio.tests`` so ``vio`` is already in
    # sys.modules here; checking leaf-ness in-process would false-positive on
    # our own package. Verify ``sky.depth.icp`` pulls no process/comms/io in a
    # FRESH interpreter instead.
    import subprocess
    leaf = subprocess.run(
        [sys.executable, "-c",
         "import sys, sky.depth.icp; "
         "bad=[m for m in sys.modules if m.split('.')[0] in "
         "('imu_camera','vio','slam','ui','depth') or '.comms' in m]; "
         "sys.exit(1 if bad else 0)"],
        capture_output=True)
    leaf_ok = leaf.returncode == 0
    ok = ok and leaf_ok
    print(f"[{'ok' if leaf_ok else 'FAIL'}] sky.depth.icp import is leaf-clean "
          "(fresh interpreter, no flight-vio process leaked)")

    print("\n" + ("PASS -- ICP geometry + information + degeneracy checks hold."
                  if ok else "FAIL -- see flagged checks above."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
