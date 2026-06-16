#!/usr/bin/env python3
"""Divergence-guard SIGNAL probe: per-keyframe window-jump m + mean reproj px.

READ-ONLY / ADDITIVE. Drives the EXACT live ``--tight`` path
(``WindowedVIORGBDOdometry`` with ``imu_info_weight=True``) over a gold session
and reads the guard signals the map exposes in ``last_info`` after each keyframe
solve (``vio_window_jump_m``, ``vio_reproj_px``, ``vio_degraded``). It reports the
per-session guard-fire count and the worst-case jump / reproj, so the guard
thresholds can be tuned to REJECT the push_shake divergent window while NEVER
firing on the well-conditioned gold sessions (zero false positives).

It does NOT modify the solve, the loose path, comms, or any frozen baseline.

Run::

    .venv/bin/python -m verification.divergence_guard_diag \
        --sessions push_shake_20s push_straight_fast_15s push_fwdback_20s \
                   lab_straight_20s quick_motion_15s
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from imu_camera.io.reader import SessionReader                        # noqa: E402
from sky.vio.imu import GyroPreintegrator                            # noqa: E402
from sky.front.odometry import OdometryConfig                         # noqa: E402
from sky.vio.window import (                                          # noqa: E402
    WindowedVIOConfig, WindowedVIORGBDOdometry)

GOLD_DIR = Path("sessions/gold")


def run_session(session_dir: Path, *, max_frames: int, guard: bool) -> dict | None:
    reader = SessionReader(session_dir)
    n = len(reader) if max_frames <= 0 else min(max_frames, len(reader))
    if not reader.calib.has_imu_extrinsics:
        return None
    imu = reader.load_imu()
    if imu["ts_ns"].size <= 1:
        return None

    R_imu_cam = reader.calib.T_imu_left[:3, :3]
    gyro_cam = (R_imu_cam @ imu["gyro"].T).T
    accel_cam = (R_imu_cam @ imu["accel"].T).T
    t0 = imu["ts_ns"][0]
    win = imu["ts_ns"] <= t0 + int(0.3 * 1e9)
    bg0 = gyro_cam[win].mean(axis=0) if win.any() else np.zeros(3)

    wcfg = WindowedVIOConfig()
    wcfg.vio.imu_info_weight = True            # the live --tight marker
    wcfg.divergence_guard = bool(guard)

    odom_cfg = OdometryConfig(gyro_fuse=True,
                              use_own_pnp=os.environ.get("OAKD_OWN_PNP", "1") != "0")
    vo = WindowedVIORGBDOdometry(
        reader.K, imu["ts_ns"], gyro_cam, accel_cam,
        bg0=bg0, ba0=np.zeros(3), cfg=wcfg, odom_cfg=odom_cfg)

    pre = GyroPreintegrator(imu["ts_ns"], imu["gyro"], reader.calib.T_imu_left)
    accel_imu = imu["accel"][win].mean(axis=0)
    vo.align_to_gravity(R_imu_cam @ accel_imu)

    rows = []
    prev_ts = None
    for i in range(n):
        f = reader.load_frame(i)
        R_prior = pre.delta_rotation(prev_ts, f.ts_ns) if prev_ts is not None else None
        vo.process(f.gray_left, f.depth_m, f.ts_ns, R_prior=R_prior)
        prev_ts = f.ts_ns
        info = vo.last_info
        if not info.get("is_kf"):
            continue
        if "vio_reproj_px" not in info:        # run_ba returned None (starved)
            continue
        last = vo.map.keyframes[-1] if vo.map.keyframes else None
        v_norm = float(np.linalg.norm(last["v"])) if last is not None else float("nan")
        rows.append({
            "seq": int(f.seq),
            "jump_m": float(info.get("vio_window_jump_m", float("nan"))),
            "reproj_px": float(info.get("vio_reproj_px", float("nan"))),
            "degraded": bool(info.get("vio_degraded", False)),
            "v_norm": v_norm,
        })
    return {"name": session_dir.name, "rows": rows}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sessions", nargs="*", default=[
        "push_shake_20s", "push_straight_fast_15s", "push_fwdback_20s",
        "lab_straight_20s", "quick_motion_15s"])
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-guard", action="store_true",
                    help="signal probe with the guard OFF (measure raw signals)")
    ap.add_argument("--detail", action="store_true",
                    help="dump per-fire (seq, jump_m, reproj_px, |v|) rows")
    args = ap.parse_args()

    print("=== Divergence-guard signal probe (live --tight) ===")
    print(f"{'session':<26}{'#kf':>5}{'fires':>7}{'max_jump_m':>12}"
          f"{'max_rpx':>9}{'fire_seqs'}")
    for name in args.sessions:
        sd = GOLD_DIR / name
        if not sd.exists():
            print(f"  !! missing {name}")
            continue
        d = run_session(sd, max_frames=args.max_frames, guard=not args.no_guard)
        if d is None:
            print(f"  {name:<24} (no IMU extrinsics)")
            continue
        rows = d["rows"]
        fires = [r for r in rows if r["degraded"]]
        jumps = [r["jump_m"] for r in rows if np.isfinite(r["jump_m"])]
        rpxs = [r["reproj_px"] for r in rows if np.isfinite(r["reproj_px"])]
        max_jump = max(jumps) if jumps else float("nan")
        max_rpx = max(rpxs) if rpxs else float("nan")
        fire_seqs = ",".join(str(r["seq"]) for r in fires[:12])
        if len(fires) > 12:
            fire_seqs += ",..."
        print(f"{name:<26}{len(rows):>5}{len(fires):>7}{max_jump:>12.3f}"
              f"{max_rpx:>9.2f}  {fire_seqs}")
        if args.detail and fires:
            for r in fires:
                print(f"    fire seq={r['seq']:>4} jump_m={r['jump_m']:>10.3f} "
                      f"reproj_px={r['reproj_px']:>8.2f} |v|={r['v_norm']:>7.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
