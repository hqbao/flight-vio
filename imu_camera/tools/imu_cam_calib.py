#!/usr/bin/env python3
"""Interactive per-device IMU->camera rotation calibration wizard.

WHY: the OAK EEPROM's ``getImuToCameraExtrinsics`` is wrong on some devices --
notably the OAK-D Lite (BMI270), whose nominal value points startup gravity at the
optical +y axis and FLIPS the gravity-aligned initial attitude ~180deg in roll.
This wizard measures the true IMU->camera rotation from the accelerometer in a few
KNOWN poses (see :mod:`sky.sensors.imu_cam_extrinsic`) and stores it per device, so
:func:`imu_camera.device.live_calib.read_live_calibration` uses it instead of the
bad EEPROM value on every subsequent run.

It is INTERACTIVE on purpose: the operator presses Enter to capture each pose, so
the camera is held still in the right orientation at capture time (a timed,
non-interactive capture can't guarantee that). Run it with the camera free (no
other process holding the device):

    .venv/bin/python -m imu_camera.tools.imu_cam_calib
    .venv/bin/python -m imu_camera.tools.imu_cam_calib --model lite   # pick device

Hardware-only (opens the OAK-D); the maths it calls is offline-tested
(``imu_camera.tests.imu_cam_extrinsic_selftest``).
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.sensors.calib_store import save_imu_cam_rotation          # noqa: E402
from sky.sensors.imu_cam_extrinsic import (                        # noqa: E402
    POSES, residual_deg, solve_imu_cam_rotation)

# Accept a pose only when the camera is genuinely at rest: the accel magnitude is
# within this band of g, and its sample std is below the jitter bound.
_G = 9.81
_MAG_TOL = 0.8          # |mean accel| within g +/- this (m/s^2)
_STILL_STD = 0.25       # per-axis std below this over the window (m/s^2)
_WINDOW_S = 1.2
_TIMEOUT_S = 8.0


def _collect_still(q) -> np.ndarray | None:
    """Collect a still ~``_WINDOW_S`` accel window; return mean or ``None`` if it
    never settles (kept moving / not at rest) within ``_TIMEOUT_S``."""
    buf: list[list[float]] = []
    t_start = time.monotonic()
    win_start: float | None = None
    while time.monotonic() - t_start < _TIMEOUT_S:
        msg = q.tryGet()
        if msg is None:
            time.sleep(0.004)
            continue
        for pk in msg.packets:
            a = pk.acceleroMeter
            buf.append([a.x, a.y, a.z])
        if len(buf) < 40:
            continue
        recent = np.asarray(buf[-120:], dtype=np.float64)
        mean = recent.mean(axis=0)
        if (abs(np.linalg.norm(mean) - _G) <= _MAG_TOL
                and float(recent.std(axis=0).max()) <= _STILL_STD):
            if win_start is None:
                win_start = time.monotonic()
            elif time.monotonic() - win_start >= _WINDOW_S:
                return mean
        else:
            win_start = None        # moved -> restart the stillness window
    return None


def run_wizard(model: str | None) -> int:
    import depthai as dai
    from imu_camera.device.probe import probe_capabilities, select_device

    print("=== IMU<->camera rotation calibration ===")
    try:
        device, _seen = select_device(model)
    except RuntimeError as e:
        print(f"[wizard] {e}", file=sys.stderr)
        return 1
    caps = probe_capabilities(device)
    print(f"device: {caps.name}  IMU={caps.imu_type}  id={caps.device_id}")
    if not caps.has_imu:
        print("[wizard] device has no IMU -- nothing to calibrate.",
              file=sys.stderr)
        device.close()
        return 1
    try:
        R_eeprom = np.array(device.readCalibration().getImuToCameraExtrinsics(
            dai.CameraBoardSocket.CAM_B), dtype=np.float64)[:3, :3]
        print("EEPROM IMU->cam rotation (what we are replacing):")
        print(np.array2string(R_eeprom, precision=3, suppress_small=True))
    except Exception:                                              # noqa: BLE001
        print("[wizard] (could not read EEPROM extrinsic for comparison)")

    p = dai.Pipeline(device)
    imu = p.create(dai.node.IMU)
    imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW], 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(20)
    q = imu.out.createOutputQueue(maxSize=100, blocking=False)
    p.start()

    meas: list[np.ndarray] = []
    ups: list[np.ndarray] = []
    try:
        for i, pose in enumerate(POSES, 1):
            print(f"\n--- Pose {i}/{len(POSES)}: {pose.instruction}")
            while True:
                input("    Hold still in the correct pose then press ENTER to capture... ")
                print("    measuring, hold still ...", flush=True)
                m = _collect_still(q)
                if m is None:
                    print("    !! not at rest / wrong gravity magnitude -- "
                          "hold steadier and try again.")
                    continue
                print(f"    ✔ captured  accel={np.array2string(m, precision=2)}")
                meas.append(m)
                ups.append(pose.up_cam)
                break
    finally:
        p.stop()
        device.close()

    R = solve_imu_cam_rotation(np.array(meas), np.array(ups))
    res = residual_deg(R, np.array(meas), np.array(ups))
    print("\n=== RESULT ===")
    print("R_imu_cam (measured):")
    print(np.array2string(R, precision=4, suppress_small=True))
    print(f"residual per pose (deg): "
          f"{np.array2string(res, precision=2)}  max={res.max():.2f}")
    if res.max() > 5.0:
        print(f"[wizard] ⚠️ large residual ({res.max():.1f} deg) -- most likely "
              f"the poses were held off-axis. NOT saving; rerun and hold the correct orientation.",
              file=sys.stderr)
        return 2
    path = save_imu_cam_rotation(caps.device_id, R, n_poses=len(POSES),
                                 residual_deg=float(res.max()))
    print(f"\n✅ Saved for device {caps.device_id} -> {path}")
    print("   The next live run will use this value instead of the EEPROM "
          "(initial roll/pitch will be correct).")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", default=None,
                    help="select which OAK device by product-name substring "
                         "(e.g. 'lite') or deviceId, when several are connected")
    args = ap.parse_args()
    try:
        return run_wizard(args.model)
    except KeyboardInterrupt:
        print("\n[wizard] cancelled.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
