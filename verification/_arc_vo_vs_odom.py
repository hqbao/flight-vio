#!/usr/bin/env python3
"""Arc-fast diagnostic: pose.vo (pure vision) vs pose.odom (IMU-fused).

Settles whether the arc-fast jitter/spikes are in the VISION signal (pure
frame-to-frame PnP) or introduced by the IMU dead-reckon + complementary
correction. Drives the real OdometryModule with publish_vo=True, retain_imu=True
(--tight), captures BOTH lines, plots X vs frame, prints jitter for each. This is
the decomposition that pinned Phase 4(k) (the "snap-back" root cause).
"""
from __future__ import annotations
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from imu_camera.io.reader import SessionReader
from vio.comms import LocalPubSub, topics
from vio.comms.messages import DepthFrame, END, ImuCamPacket
from vio.modules import OdometryModule
from sky.front.odometry import OdometryConfig
from vio.tests.tight_live_pose_selftest import _per_frame_imu


def _jit(pos):
    plen = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))
    maxd = float(np.linalg.norm(pos - pos[0], axis=1).max())
    return plen, maxd, plen / max(maxd, 1e-9)


def main(session="sessions/arc_fast_15s"):
    sd = Path(session)
    reader = SessionReader(sd)
    n = len(reader)
    R_imu_cam = reader.calib.T_imu_left[:3, :3]
    imu = reader.load_imu()
    ts_all = imu["ts_ns"].astype(np.int64)
    gyro = imu["gyro"].astype(np.float64)
    accel = imu["accel"].astype(np.float64)
    t0 = int(ts_all[0]); win = ts_all <= t0 + int(0.3 * 1e9)
    accel_align = R_imu_cam @ accel[win].mean(axis=0)

    bus = LocalPubSub()
    odom = OdometryModule(
        bus, reader.K, R_imu_cam=R_imu_cam, accel_align=accel_align,
        odom_cfg=OdometryConfig(gyro_fuse=True), kf_every=5, use_gyro=True,
        latest_only=False, level_tilt=True, publish_vo=True, retain_imu=True)

    vo, od = {}, {}
    bus.subscribe(topics.POSE_VO, lambda m: vo.__setitem__(m.seq, m.T_world_cam[:3,3].copy()) if m is not END else None)
    bus.subscribe(topics.POSE_ODOM, lambda m: od.__setitem__(m.seq, m.T_world_cam[:3,3].copy()) if m is not END else None)
    odom.start()
    prev_ts = None
    for i in range(n):
        f = reader.load_frame(i)
        its, ig, ia = _per_frame_imu(ts_all, gyro, accel, prev_ts, int(f.ts_ns))
        bus.publish(topics.IMUCAM_SAMPLE, ImuCamPacket(f.seq, int(f.ts_ns), f.gray_left, None, its, ig, ia))
        bus.publish(topics.FRAME_DEPTH, DepthFrame(f.seq, int(f.ts_ns), f.gray_left, f.depth_m))
        prev_ts = int(f.ts_ns)
    bus.publish(topics.IMUCAM_SAMPLE, END); bus.publish(topics.FRAME_DEPTH, END)
    odom.done.wait(timeout=120.0); odom.stop()

    common = sorted(set(vo) & set(od))
    vp = np.array([vo[s] for s in common]); op = np.array([od[s] for s in common])
    pv, mv, jv = _jit(vp); po, mo, jo = _jit(op)
    print(f"frames common: {len(common)}")
    print(f"  pose.vo  (pure vision): path {pv:.1f}m  maxdist {mv:.2f}m  JITTER {jv:.1f}")
    print(f"  pose.odom (IMU-fused):  path {po:.1f}m  maxdist {mo:.2f}m  JITTER {jo:.1f}")
    print(f"  => jitter source: {'VISION (pose.vo spiky too)' if jv > 6 else 'IMU/fusion (vo is smooth, odom spikes)'}")
    # max per-frame step each
    print(f"  pose.vo  max step: {np.linalg.norm(np.diff(vp,axis=0),axis=1).max()*100:.1f} cm")
    print(f"  pose.odom max step: {np.linalg.norm(np.diff(op,axis=0),axis=1).max()*100:.1f} cm")

    fig, ax = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    ax[0].plot(vp[:,0], label='vo x', color='tab:orange'); ax[0].plot(op[:,0], label='odom x', color='tab:blue', alpha=0.8)
    ax[0].set_title("X vs frame: pose.vo (pure vision) vs pose.odom (IMU-fused)"); ax[0].legend(); ax[0].grid(alpha=0.3); ax[0].set_ylabel("x [m]")
    ax[1].plot(vp[:,2], label='vo z', color='tab:orange'); ax[1].plot(op[:,2], label='odom z', color='tab:green', alpha=0.8)
    ax[1].legend(); ax[1].grid(alpha=0.3); ax[1].set_ylabel("z [m]"); ax[1].set_xlabel("frame")
    plt.tight_layout(); plt.savefig("/tmp/arc_vo_vs_odom.png", dpi=90)
    print("  saved /tmp/arc_vo_vs_odom.png")


if __name__ == "__main__":
    main()
