#!/usr/bin/env python3
"""Falsification harness for the dense-ICP relative-pose factor (Phase-4).

Runs the tight VIO over the target clips at BOTH resolutions and prints, per
clip, the metric for four configs so the ICP factor can be judged HONESTLY:

  OFF        : tight baseline (imu_info_weight, the live --tight weight)
  vel        : + velocity prior only (stabilize_velocity)            [reference]
  icp        : + dense-ICP factor only
  icp+vel    : + both

Honest verdict criteria (from the spec):
  * 54x42: icp should improve trans/ATE BEYOND vel-prior-only; rotation must NOT
    regress; scale should move towards 1.
  * full-res: NO regression (the ICP term must be inert when vision is rich).

Run (slow -- ICP is ~4x the tight solve cost):

    .venv/bin/python verification/icp_factor_bench.py [--max-frames N] [--only clip ...]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from verification.loose_vs_tight_bench import run_session  # noqa: E402

GOLD = Path(__file__).resolve().parents[1] / "sessions" / "gold"

TARGETS = ["push_straight_fast_15s", "push_shake_20s", "lab_straight_20s"]

CONFIGS = [
    ("OFF", dict()),
    ("vel", dict(stabilize_velocity=True)),
    ("icp", dict(depth_icp=True)),
    ("icp+vel", dict(stabilize_velocity=True, depth_icp=True)),
]


def _row(r):
    if r is None:
        return f"{'--':>8} {'--':>7} {'--':>9} {'--':>7} {'--':>7}"
    return (f"{r['ate_cm']:>8.2f} {r['scale']:>7.3f} {r['max_step_cm']:>9.2f} "
            f"{r['phantom_cm']:>7.2f} {r['ba_ok_frac']:>7.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--only", nargs="*", default=None)
    args = ap.parse_args()

    targets = args.only if args.only else TARGETS
    hdr = (f"{'config':>9} | {'ATEcm':>8} {'scale':>7} {'maxstepcm':>9} "
           f"{'phantcm':>7} {'baok':>7}")

    for res, res_label in (("tof54", "54x42 ToF (VL53 target)"),
                           ("full", "FULL-RES (chip depth)")):
        print(f"\n{'='*72}\n  RESOLUTION: {res_label}\n{'='*72}")
        for clip in targets:
            sd = GOLD / clip
            if not sd.exists():
                print(f"\n[{clip}] MISSING -> skip")
                continue
            print(f"\n[{clip}]  ({res})")
            print(hdr)
            for tag, kw in CONFIGS:
                t = time.perf_counter()
                r = run_session(sd, backend="vio", resolution=res,
                                imu_info_weight=True,
                                min_ba_views=(1 if res == "tof54" else None),
                                max_frames=args.max_frames, **kw)
                dt = time.perf_counter() - t
                print(f"{tag:>9} | {_row(r)}   ({dt:.0f}s)")
                sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
