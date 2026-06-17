#!/usr/bin/env python3
"""FULL-SESSION ATE-equal gate: njit IMU-factor FD vs pure-Python trajectory.

THE binding correctness gate. Numerically-equivalent ``H``/``b`` (rel ~1e-10) do
NOT by themselves guarantee the same converged pose -- the LM loop in
``optimize_vio`` branches DISCRETELY on cost (accept/reject) and on the relative-
improvement early-stop, so a round-off-scale ``H``/``b`` perturbation could flip
an iteration and walk the trajectory elsewhere. This gate runs the REAL live
``--tight`` engine over a full gold session TWICE -- once with the njit FD kernel
on, once with it forced off (the unchanged pure-Python build) -- on byte-identical
inputs, and compares the two output trajectories. PASS iff the per-keyframe
position diff is within fp noise (sub-mm) and the rotation diff sub-mdeg.

If the trajectories diverge anywhere the gold session converged in pure-Python,
this FAILS -> the njit kernel must NOT ship (it changed ATE). This mirrors the
``--tight`` wiring in ``tight_smoke_selftest`` exactly (the live 6-tuple snapshot
through ``make_vi_engine``).

Run::

    .venv/bin/python -m verification.imu_factor_njit_ate \
        --session sessions/gold/push_straight_fast_15s
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sky.vio.window as W                                            # noqa: E402
from imu_camera.io.reader import SessionReader                        # noqa: E402
from sky.vio.imu import GyroPreintegrator                            # noqa: E402
from sky.front.odometry import OdometryConfig, RGBDVisualOdometry    # noqa: E402
from sky.vio.window import WindowedVIOConfig                          # noqa: E402
from ba.engine import make_vi_engine                                  # noqa: E402
from sky.math import so3_log                                          # noqa: E402

POS_GATE_MM = 1.0       # sub-mm per-keyframe position agreement
ROT_GATE_MDEG = 1.0     # sub-mdeg per-keyframe rotation agreement


def _slice_imu_seg(ts_all, gyro_cam, accel_cam, t0, t1):
    if t1 <= t0:
        return None
    m = (ts_all > t0) & (ts_all <= t1)
    if int(m.sum()) < 2:
        return None
    return (ts_all[m].astype(np.int64), gyro_cam[m].astype(np.float64),
            accel_cam[m].astype(np.float64))


def _run_tight(reader, n, kf_every, force_njit):
    """Run the live --tight engine over the session; return {seq: T_world_cam}."""
    saved = W.HAVE_NUMBA
    saved_env = os.environ.get("SKY_VIO_IMU_NJIT")
    W.HAVE_NUMBA = bool(force_njit)
    # Production gate is default-OFF behind SKY_VIO_IMU_NJIT; flip on to exercise.
    os.environ["SKY_VIO_IMU_NJIT"] = "1" if force_njit else "0"
    try:
        R_imu_cam = reader.calib.T_imu_left[:3, :3]
        imu = reader.load_imu()
        ts_all = imu["ts_ns"].astype(np.int64)
        gyro_cam = (R_imu_cam @ imu["gyro"].T).T
        accel_cam = (R_imu_cam @ imu["accel"].T).T
        t0 = int(ts_all[0])
        win = ts_all <= t0 + int(0.3 * 1e9)

        odom_cfg = OdometryConfig(gyro_fuse=True)
        tight_cfg = WindowedVIOConfig()
        tight_cfg.vio.imu_info_weight = True       # the live --tight marker
        engine = make_vi_engine(reader.K, tight_cfg, worker=False)
        tight_fe = RGBDVisualOdometry(reader.K, odom_cfg)
        pre = GyroPreintegrator(imu["ts_ns"], imu["gyro"], reader.calib.T_imu_left)
        accel_imu0 = imu["accel"][win].mean(axis=0)
        tight_fe.align_to_gravity(R_imu_cam @ accel_imu0)

        poses: dict[int, np.ndarray] = {}
        tight_pose = tight_fe.pose.copy()
        prev_ts = None
        frames_since_kf = 0
        last_kf_ts = None
        n_kf = 0
        for i in range(n):
            f = reader.load_frame(i)
            R_prior = (pre.delta_rotation(prev_ts, f.ts_ns)
                       if prev_ts is not None else None)
            tight_pose = tight_fe.process(f.gray_left, f.depth_m,
                                          R_prior=R_prior).copy()
            frames_since_kf += 1
            is_kf = (n_kf == 0) or (frames_since_kf >= kf_every)
            if is_kf:
                frames_since_kf = 0
                n_kf += 1
                tr = tight_fe.frontend.tracks
                ids = tr.ids.copy() if tr is not None else None
                px = tr.points.copy() if tr is not None else None
                imu_seg = (None if last_kf_ts is None else
                           _slice_imu_seg(ts_all, gyro_cam, accel_cam,
                                          last_kf_ts, int(f.ts_ns)))
                last_kf_ts = int(f.ts_ns)
                T_cw = np.linalg.inv(tight_pose)
                engine.submit((T_cw, ids, px, f.depth_m, int(f.ts_ns), imu_seg))
                post = engine.poll()
                if isinstance(post, tuple):       # tight: vio_step returns (T_cw, health, bias)
                    post, _health, *_ = post       # *_ tolerates the new bias element
                if post is not None:
                    tight_pose = np.linalg.inv(post)
                    tight_fe.pose = tight_pose.copy()
            poses[f.seq] = tight_pose.copy()
            prev_ts = f.ts_ns
        engine.close()
        return poses
    finally:
        W.HAVE_NUMBA = saved
        if saved_env is None:
            os.environ.pop("SKY_VIO_IMU_NJIT", None)
        else:
            os.environ["SKY_VIO_IMU_NJIT"] = saved_env


def run(session: Path, max_frames: int, kf_every: int) -> int:
    reader = SessionReader(session)
    n = len(reader) if max_frames <= 0 else min(max_frames, len(reader))
    if not reader.calib.has_imu_extrinsics:
        print(f"SKIP: {session} has no IMU extrinsics (tight needs IMU).")
        return 0
    print("=== FULL-SESSION ATE-equal: njit vs pure-Python tight solve ===")
    print(f"session={session.name}  frames={n}  kf_every={kf_every}  "
          f"(HAVE_NUMBA={W.HAVE_NUMBA})")

    py = _run_tight(reader, n, kf_every, force_njit=False)
    nj = _run_tight(reader, n, kf_every, force_njit=True)

    seqs = sorted(set(py) & set(nj))
    pos_diffs = []
    rot_diffs = []
    for s in seqs:
        Tp, Tn = py[s], nj[s]
        pos_diffs.append(float(np.linalg.norm(Tp[:3, 3] - Tn[:3, 3])))
        dR = Tp[:3, :3].T @ Tn[:3, :3]
        rot_diffs.append(float(np.degrees(np.linalg.norm(so3_log(dR)))))
    pos_diffs = np.asarray(pos_diffs)
    rot_diffs = np.asarray(rot_diffs)
    max_pos_mm = float(pos_diffs.max() * 1e3) if pos_diffs.size else 0.0
    max_rot_mdeg = float(rot_diffs.max() * 1e3) if rot_diffs.size else 0.0
    # final-pose drift (the accumulated ATE endpoint divergence)
    end = seqs[-1]
    end_pos_mm = float(np.linalg.norm(py[end][:3, 3] - nj[end][:3, 3]) * 1e3)

    print(f"compared {len(seqs)} keyframe-resolved poses")
    print(f"  max per-pose position diff = {max_pos_mm:.4f} mm "
          f"(gate {POS_GATE_MM} mm)")
    print(f"  max per-pose rotation diff = {max_rot_mdeg:.4f} mdeg "
          f"(gate {ROT_GATE_MDEG} mdeg)")
    print(f"  endpoint position diff     = {end_pos_mm:.4f} mm")

    ok = max_pos_mm < POS_GATE_MM and max_rot_mdeg < ROT_GATE_MDEG
    print("\nPASS -- njit tight trajectory == pure-Python within fp noise."
          if ok else
          "\nFAIL -- njit tight trajectory DIVERGED from pure-Python (do NOT ship)")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session",
                    default="sessions/gold/push_straight_fast_15s")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--kf-every", type=int, default=5)
    args = ap.parse_args()
    return run(Path(args.session), args.max_frames, args.kf_every)


if __name__ == "__main__":
    raise SystemExit(main())
