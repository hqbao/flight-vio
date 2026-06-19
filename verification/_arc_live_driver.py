#!/usr/bin/env python3
"""Deterministic LIVE driver for the arc-fast snap regression (single-process, no drops).

Reuses vio.tests.tight_live_pose_selftest._run_module, which drives the REAL
OdometryModule (frontend KLT/PnP + propagate_imu -> pose.odom, the green VIO line
the UI shows) over a session via a LocalPubSub in FIFO (latest_only=False) order
-- so EVERY frame is processed, NO real-time frame drops, and the result is
seq-keyed {seq: position} that aligns 1:1 to the Basalt reference by seq.

A clean, deterministic metric (vs the confounded multiprocess capture) -- the arc
regression behind Phase 4(k) (the "snap-back" fix).
  .venv/bin/python -m verification._arc_live_driver [--session DIR]
"""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np

from imu_camera.io.reader import SessionReader
from verification.oracle_replay import load_basalt_positions, umeyama
from vio.tests.tight_live_pose_selftest import _run_module


def run(session: Path) -> dict:
    reader = SessionReader(session)
    n = len(reader)
    captured = _run_module(session, n, tight=True)   # {seq: pos}, deterministic
    basalt = load_basalt_positions(session)          # {seq: pos}
    common = sorted(set(captured) & set(basalt))
    ours = np.array([captured[s] for s in common])
    bas = np.array([basalt[s] for s in common])
    steps = np.linalg.norm(np.diff(ours, axis=0), axis=1) * 100.0
    bsteps = np.linalg.norm(np.diff(bas, axis=0), axis=1) * 100.0
    R, t, s = umeyama(ours, bas, True)
    aligned = (s * (R @ ours.T).T + t)
    ate = float(np.sqrt(np.mean(np.sum((aligned - bas) ** 2, axis=1))) * 100.0)
    out = {
        "n_ours": len(captured), "n_basalt": len(basalt), "n_common": len(common),
        "ours_maxstep_cm": float(steps.max()), "basalt_maxstep_cm": float(bsteps.max()),
        "scale": float(s), "ate_cm": ate,
    }
    print(f"frames: ours={out['n_ours']} basalt={out['n_basalt']} common={out['n_common']}")
    print(f"  ours max single-frame step: {out['ours_maxstep_cm']:.1f} cm")
    print(f"  Basalt max single-frame step: {out['basalt_maxstep_cm']:.1f} cm  (clean ref)")
    print(f"  Sim3 scale vs Basalt: {out['scale']:.3f}  (1.0 = perfect)")
    print(f"  ATE (rigid-aligned): {out['ate_cm']:.1f} cm")
    order = np.argsort(steps)[::-1][:8]
    print("  top-8 ours steps (cm @ seq):",
          [f"{steps[i]:.0f}@{common[i+1]}" for i in order])
    # per-step: is ours' big step matched by a Basalt step at the same seq? (real motion vs snap)
    print("  at those seqs, BASALT step (cm):",
          [f"{bsteps[i]:.0f}@{common[i+1]}" for i in order])
    # spans + rigid (no-scale) ATE -- scale=21 may be an alignment artefact
    print(f"  ours  span(m): x[{ours[:,0].min():.2f},{ours[:,0].max():.2f}] "
          f"y[{ours[:,1].min():.2f},{ours[:,1].max():.2f}] z[{ours[:,2].min():.2f},{ours[:,2].max():.2f}]")
    print(f"  basalt span(m): x[{bas[:,0].min():.2f},{bas[:,0].max():.2f}] "
          f"y[{bas[:,1].min():.2f},{bas[:,1].max():.2f}] z[{bas[:,2].min():.2f},{bas[:,2].max():.2f}]")
    Rr, tr, _ = umeyama(ours, bas, False)   # rigid only (no scale)
    arig = (Rr @ ours.T).T + tr
    ate_rigid = float(np.sqrt(np.mean(np.sum((arig - bas) ** 2, axis=1))) * 100.0)
    print(f"  ATE (RIGID, no scale): {ate_rigid:.1f} cm")
    plen = float(np.sum(np.linalg.norm(np.diff(ours, axis=0), axis=1)))
    maxd = float(np.linalg.norm(ours - ours[0], axis=1).max())
    jitter = plen / max(maxd, 1e-9)
    out["path_len_m"] = plen
    out["jitter_ratio"] = jitter
    print(f"  ours path length: {plen:.2f} m ; max dist from start: {maxd:.2f} m ;"
          f"  JITTER ratio = {jitter:.1f}  (clean ~1-3, spiky >>)")
    np.savez("/tmp/arc_traj.npz", ours=ours, basalt=bas, seqs=np.array(common))
    print("  saved /tmp/arc_traj.npz (ours, basalt, seqs)")
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", default="sessions/arc_fast_15s")
    a = ap.parse_args()
    run(Path(a.session))
