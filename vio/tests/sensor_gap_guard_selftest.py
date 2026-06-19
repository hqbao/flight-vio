#!/usr/bin/env python3
"""Self-test: the LIVE ``--tight`` pose does NOT jump across a SENSOR DROPOUT.

The OAK USB-crashes and re-enumerates mid-flight (the Pi shows it cycling
Luxonis-device -> Movidius-ROM -> reconnect, seconds of NO camera and NO IMU).
On reconnect the next IMU block's first sample is SECONDS after the last one the
live propagator integrated. The gap-free prepend (``prev_tail``) would then make
:func:`~sky.vio.imu.predict_state` dead-reckon ``v*dt + 0.5*a*dt^2`` over the
WHOLE blackout in one step -> a metres-large pose JUMP the instant the stream
returns (the user's "sometimes still, sometimes moving / jumps too far" -- dangerous for the FC).

This drives the REAL :class:`~vio.modules.pipeline.OdometryModule`
(``retain_imu=True``, the live ``--tight`` front-end) over a gold session but
DROPS a window of frames to fake the blackout: the frames resume with their
NATURAL (small) per-frame IMU blocks, so the only large interval is the stale
``prev_tail`` boundary -- exactly the reconnect signature. It asserts:

* with the guard DISABLED (``_SENSOR_GAP_S`` huge) the resume frame JUMPS
  (the bug -- the blackout is integrated in one step), and no gap is flagged;
* with the guard at its DEFAULT the resume jump is bounded to roughly a normal
  frame's motion, the velocity was reset, and the frame is flagged
  ``sensor_gap_s`` for the UI/FC.

Run::

    .venv/bin/python -m vio.tests.sensor_gap_guard_selftest
    .venv/bin/python -m vio.tests.sensor_gap_guard_selftest --session sessions/gold/push_straight_fast_15s
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from imu_camera.io.reader import SessionReader                       # noqa: E402
from vio.comms import LocalPubSub, topics                            # noqa: E402
from vio.comms.messages import DepthFrame, END, ImuCamPacket         # noqa: E402
from vio.modules import OdometryModule                               # noqa: E402
from vio.modules import propagate_imu                                # noqa: E402
from sky.front.odometry import OdometryConfig                        # noqa: E402


def _per_frame_imu(ts_all, gyro, accel, prev_ts, ts):
    """IMU samples in the NATURAL interval ``(prev_ts, ts]`` for one frame."""
    if prev_ts is None:
        m = ts_all <= ts
    else:
        m = (ts_all > prev_ts) & (ts_all <= ts)
    return (ts_all[m].astype(np.int64), gyro[m].astype(np.float64),
            accel[m].astype(np.float64))


def _run_with_drop(session_dir: Path, n: int, drop: tuple[int, int],
                   gap_s: float) -> dict[int, tuple[np.ndarray, dict]]:
    """Run the real tight OdometryModule, DROPPING frames in ``[drop)``.

    ``gap_s`` monkeypatches ``propagate_imu._SENSOR_GAP_S`` (huge = guard off).
    Each PUBLISHED frame carries its NATURAL per-frame IMU block (cut against its
    own predecessor index), so the dropped window leaves the module's
    ``prev_tail`` stale by the window's wall-time -- the reconnect signature.
    Returns ``{seq: (position, info)}``.
    """
    propagate_imu._SENSOR_GAP_S = gap_s

    reader = SessionReader(session_dir)
    R_imu_cam = reader.calib.T_imu_left[:3, :3]
    imu = reader.load_imu()
    ts_all = imu["ts_ns"].astype(np.int64)
    gyro = imu["gyro"].astype(np.float64)
    accel = imu["accel"].astype(np.float64)
    t0 = int(ts_all[0])
    win = ts_all <= t0 + int(0.3 * 1e9)
    accel_align = R_imu_cam @ accel[win].mean(axis=0)

    bus = LocalPubSub()
    odom = OdometryModule(
        bus, reader.K, R_imu_cam=R_imu_cam, accel_align=accel_align,
        odom_cfg=OdometryConfig(gyro_fuse=True), kf_every=5, use_gyro=True,
        latest_only=False, level_tilt=True, publish_vo=False, retain_imu=True)

    captured: dict[int, tuple[np.ndarray, dict]] = {}

    def _grab(m):
        if m is not END:
            captured[m.seq] = (m.T_world_cam[:3, 3].copy(), dict(m.info or {}))
    bus.subscribe(topics.POSE_ODOM, _grab)
    odom.start()

    ts_frame = [int(reader.load_frame(i).ts_ns) for i in range(n)]
    for i in range(n):
        if drop[0] <= i < drop[1]:
            continue                                    # blackout: publish nothing
        f = reader.load_frame(i)
        nat_prev = ts_frame[i - 1] if i > 0 else None   # NATURAL block boundary
        its, ig, ia = _per_frame_imu(ts_all, gyro, accel, nat_prev, ts_frame[i])
        bus.publish(topics.IMUCAM_SAMPLE,
                    ImuCamPacket(f.seq, int(f.ts_ns), f.gray_left, None, its, ig, ia))
        bus.publish(topics.FRAME_DEPTH,
                    DepthFrame(f.seq, int(f.ts_ns), f.gray_left, f.depth_m))

    bus.publish(topics.IMUCAM_SAMPLE, END)
    bus.publish(topics.FRAME_DEPTH, END)
    if not odom.done.wait(timeout=120.0):
        odom.stop()
        raise RuntimeError("odometry module did not drain")
    odom.stop()
    return captured


def _resume_jump(cap: dict[int, tuple[np.ndarray, dict]],
                 drop: tuple[int, int]) -> tuple[int, int, float, dict]:
    """Jump from the last pre-drop pose to the first post-drop pose."""
    pre = [s for s in cap if s < drop[0]]
    post = [s for s in cap if s >= drop[1]]
    last_pre, first_post = max(pre), min(post)
    jump = float(np.linalg.norm(cap[first_post][0] - cap[last_pre][0]))
    return last_pre, first_post, jump, cap[first_post][1]


def run(session_dir: Path, max_frames: int) -> int:
    reader = SessionReader(session_dir)
    n = len(reader) if max_frames <= 0 else min(max_frames, len(reader))
    # Drop ~1.5s of frames mid-session (>> the 0.25s guard, a real reconnect span).
    g = n // 2
    k = 30
    drop = (g, min(g + k, n - 2))
    default_gap = propagate_imu._SENSOR_GAP_S
    print(f"session: {session_dir.name}  frames={n}  drop={drop} (~{k} frames)\n")

    fails: list[str] = []
    try:
        # warm import / numba, then the two measured runs.
        _run_with_drop(session_dir, n, drop, gap_s=1e9)
        off = _run_with_drop(session_dir, n, drop, gap_s=1e9)      # guard DISABLED
        on = _run_with_drop(session_dir, n, drop, gap_s=default_gap)  # guard DEFAULT
    finally:
        propagate_imu._SENSOR_GAP_S = default_gap

    lp0, fp0, jump_off, info_off = _resume_jump(off, drop)
    lp1, fp1, jump_on, info_on = _resume_jump(on, drop)
    print(f"[guard OFF] resume seq {lp0}->{fp0}  jump = {jump_off*100:8.1f} cm  "
          f"sensor_gap_s={info_off.get('sensor_gap_s')}")
    print(f"[guard ON ] resume seq {lp1}->{fp1}  jump = {jump_on*100:8.1f} cm  "
          f"sensor_gap_s={info_on.get('sensor_gap_s')}")

    for nm, c in (("OFF", off), ("ON", on)):
        ps = np.array([c[s][0] for s in sorted(c)])
        if not np.all(np.isfinite(ps)):
            fails.append(f"guard {nm}: pose.odom has NaN/Inf")

    # (1) guard OFF reproduces the dangerous jump (blackout integrated in one step).
    if jump_off < 0.20:
        fails.append(f"guard OFF jump only {jump_off*100:.1f} cm -- bug not "
                     f"reproduced (session too slow? raise k / pick a moving session)")
    if "sensor_gap_s" in info_off:
        fails.append("guard OFF should NOT flag sensor_gap_s")

    # (2) guard ON bounds the jump AND flags it.
    if "sensor_gap_s" not in info_on:
        fails.append("guard ON did NOT flag sensor_gap_s on the resume frame")
    else:
        gs = float(info_on["sensor_gap_s"])
        if gs < 0.25:
            fails.append(f"flagged gap {gs:.3f}s below the threshold -- mis-fire")
    if not info_on.get("inertial_dr", False):
        fails.append("guard ON did not mark the resume frame inertial_dr")
    if jump_on > 0.5 * max(jump_off, 1e-6):
        fails.append(f"guard ON jump {jump_on*100:.1f} cm not materially smaller "
                     f"than OFF {jump_off*100:.1f} cm -- guard ineffective")
    # The bounded resume motion should be near a normal frame step, not metres.
    if jump_on > 0.30:
        fails.append(f"guard ON resume jump {jump_on*100:.1f} cm still too large")

    if fails:
        print("\nFAIL:")
        for f_ in fails:
            print(f"  - {f_}")
        return 1
    ratio = jump_off / max(jump_on, 1e-6)
    print(f"\nPASS -- the sensor-gap guard cut the reconnect jump "
          f"{jump_off*100:.1f} -> {jump_on*100:.1f} cm ({ratio:.1f}x), "
          f"reset velocity, and flagged the dropout.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=160)
    args = ap.parse_args()
    return run(Path(args.session), args.max_frames)


if __name__ == "__main__":
    raise SystemExit(main())
