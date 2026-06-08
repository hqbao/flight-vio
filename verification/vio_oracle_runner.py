#!/usr/bin/env python3
"""CLI mirror of ``ours/tools/vio_run.py`` that drives the IN-PROCESS split oracle.

Same flags (``--session`` / ``--backend`` / ``--max-frames`` / ``--all``), same
printed ATE block -- but every algorithm class comes from the SPLIT projects
(imu_camera / vio / slam) via :mod:`verification.oracle_replay`, not ``ours.lib``.

Run head-to-head against the reference::

    .venv/bin/python ours/tools/vio_run.py        --session sessions/gold/lab_loop_30s --backend vio --max-frames 20
    .venv/bin/python verification/vio_oracle_runner.py --session sessions/gold/lab_loop_30s --backend vio --max-frames 20

Both must print the SAME ATE block (75.9 mm for that case).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from verification.oracle_replay import (  # noqa: E402
    load_basalt_positions,
    score_session_oracle,
)


# A recorded Basalt trajectory is only a valid reference if it didn't diverge.
# (Verbatim from ours/tools/vio_run.py so --all picks the same valid sessions.)
_MAX_VALID_STEP_M = 1.0


def basalt_ref_is_broken(positions: dict[int, np.ndarray]) -> bool:
    if len(positions) < 2:
        return True
    pos = np.array([positions[s] for s in sorted(positions)])
    steps = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    return bool(steps.max() > _MAX_VALID_STEP_M)


def run_all(use_imu: bool = True, backend: str = "f2f",
            slam_kf_every: int = 5, slam_radius_m: float = 0.0,
            slam_kf_min_trans: float = 0.0, slam_kf_min_rot: float = 0.0,
            slam_max_kf: int = 0, use_gyro: bool = True,
            depth_source: str = "chip", depth_fast: bool = False,
            marg: bool = False, vo_trans_sigma: float = 0.0) -> int:
    gold = Path("sessions/gold")
    rows = []
    for d in sorted(gold.iterdir()):
        if not (d / "basalt" / "vio_pose.jsonl").exists():
            continue
        broken = basalt_ref_is_broken(load_basalt_positions(d))
        note = "broken Basalt ref" if broken else ""
        res = None if broken else score_session_oracle(
            d, 0, False, quiet=True, use_imu=use_imu, backend=backend,
            slam_kf_every=slam_kf_every, slam_radius_m=slam_radius_m,
            slam_kf_min_trans=slam_kf_min_trans, slam_kf_min_rot=slam_kf_min_rot,
            slam_max_kf=slam_max_kf, use_gyro=use_gyro, depth_source=depth_source,
            depth_fast=depth_fast, marg=marg, vo_trans_sigma=vo_trans_sigma)
        rows.append((d.name, res, note))
        print(f"  {d.name:18s} done")

    print()
    print(f"backend: {backend}  (IN-PROCESS SPLIT ORACLE)")
    print(f"{'session':18s} {'path(m)':>8s} {'ATE RMSE':>10s} {'%path':>7s} {'scale':>6s}")
    print("-" * 54)
    for name, res, note in rows:
        if res is None:
            tag = f"  <- {note}" if note else "  (too short / no overlap)"
            print(f"{name:18s} {'--':>8s} {'--':>10s} {'--':>7s} {'--':>6s}{tag}")
            continue
        print(f"{name:18s} {res['path']:8.2f} {res['rmse']*1000:8.1f}mm "
              f"{100*res['rmse']/res['path']:6.2f}% {res['scale']:6.3f}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    ap.add_argument("--all", action="store_true",
                    help="run every gold session and print a summary table")
    ap.add_argument("--backend", choices=("f2f", "ba", "slam", "vio"),
                    default="f2f")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--slam-kf-every", type=int, default=5, dest="slam_kf_every")
    ap.add_argument("--slam-radius", type=float, default=0.0)
    ap.add_argument("--slam-kf-min-trans", type=float, default=0.0,
                    dest="slam_kf_min_trans")
    ap.add_argument("--slam-kf-min-rot", type=float, default=0.0,
                    dest="slam_kf_min_rot")
    ap.add_argument("--slam-max-kf", type=int, default=0, dest="slam_max_kf")
    ap.add_argument("--no-gyro", action="store_true")
    ap.add_argument("--depth", choices=("chip", "ours"), default="chip")
    ap.add_argument("--depth-fast", action="store_true")
    ap.add_argument("--marg", action="store_true")
    ap.add_argument("--vo-trans-sigma", type=float, default=0.0,
                    dest="vo_trans_sigma")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    use_imu = not args.no_imu
    use_gyro = not args.no_gyro
    if args.all:
        return run_all(use_imu, backend=args.backend,
                       slam_kf_every=args.slam_kf_every,
                       slam_radius_m=args.slam_radius,
                       slam_kf_min_trans=args.slam_kf_min_trans,
                       slam_kf_min_rot=args.slam_kf_min_rot,
                       slam_max_kf=args.slam_max_kf, use_gyro=use_gyro,
                       depth_source=args.depth, depth_fast=args.depth_fast,
                       marg=args.marg, vo_trans_sigma=args.vo_trans_sigma)

    score_session_oracle(Path(args.session), args.max_frames, args.verbose,
                         quiet=False, use_imu=use_imu, backend=args.backend,
                         slam_kf_every=args.slam_kf_every,
                         slam_radius_m=args.slam_radius,
                         slam_kf_min_trans=args.slam_kf_min_trans,
                         slam_kf_min_rot=args.slam_kf_min_rot,
                         slam_max_kf=args.slam_max_kf, use_gyro=use_gyro,
                         depth_source=args.depth, depth_fast=args.depth_fast,
                         marg=args.marg, vo_trans_sigma=args.vo_trans_sigma)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
