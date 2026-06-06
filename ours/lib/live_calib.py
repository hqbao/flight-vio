"""Read the live OAK-D calibration + startup IMU references for the VIO graph.

The unified front-end (``cam`` + ``imu_cam`` off ONE
:class:`~ours.lib.oak_live.SharedLiveDevice`) needs the same boot-time facts the
old monolithic capture flow read in ``LiveCaptureFlow.open()``:

* camera intrinsics ``K`` and the stereo :class:`~ours.lib.io.reader.StereoCalib`
  (so the SGM matcher can rectify the raw cameras),
* the IMU->camera rotation ``R_imu_cam`` (gyro prior conjugation),
* a per-device gyro bias (cached, calibrated once -- only that first calibration
  needs the device held still) and a startup gravity-align accelerometer seed
  (measured each boot; once the bias is cached this is a quick non-gated read, since
  the odometry flow's continuous ``CorrectTilt`` re-levels roll/pitch at rest).

This module is the single place that turns an acquired shared device into those
references, so the live graph and any future tool apply identical maths. It is
hardware-only: validated on the bench, never in the offline test harness (the
offline path never imports depthai). The still-gate + cache logic mirrors the
proven ``flows/capture/live.py`` so behaviour is unchanged by the front-end split.
"""
from __future__ import annotations

import sys
import time
from dataclasses import dataclass

import numpy as np

from .config.resolution import ResolutionProfile
from .imu.accel_calib import AccelCalibration
from .imu.calib_store import load_accel_calib, load_gyro_bias, save_gyro_bias
from .imu.decode import decode_imu_packets
from .imu.imu_calib import ImuCalibration
from .io.reader import StereoCalib
from .oak_live import SharedLiveDevice

# Startup stillness gates (identical to the legacy capture path): the gravity
# level and any measured gyro bias are means, so ANY motion during the window
# poisons them -> reject the sample and restart the window when moving.
_STILL_GYRO = 0.15      # rad/s
_STILL_ACCEL = 0.6      # m/s^2 deviation from the window mean


@dataclass(frozen=True)
class LiveFrontEndCalib:
    """Everything the live VIO graph needs, read once from the shared device."""

    K: np.ndarray
    calib: StereoCalib
    R_imu_cam: np.ndarray | None
    sgm_cfg: object
    res: ResolutionProfile
    accel_align: np.ndarray | None
    imu_calibration: ImuCalibration | None


def _read_stereo_calib(ch, width: int, height: int):
    """Read ``(K, StereoCalib, R_imu_cam)`` from a depthai calibration handler."""
    import depthai as dai

    left_socket = dai.CameraBoardSocket.CAM_B
    right_socket = dai.CameraBoardSocket.CAM_C

    K = np.array(ch.getCameraIntrinsics(left_socket, width, height),
                 dtype=np.float64)

    def _intr(sock):
        Ki = np.array(ch.getCameraIntrinsics(sock, width, height),
                      dtype=np.float64)
        dist = list(ch.getDistortionCoefficients(sock))
        return {"fx": float(Ki[0, 0]), "fy": float(Ki[1, 1]),
                "cx": float(Ki[0, 2]), "cy": float(Ki[1, 2]),
                "dist": [float(x) for x in dist],
                "width": int(width), "height": int(height)}

    T_lr = np.array(ch.getCameraExtrinsics(left_socket, right_socket),
                    dtype=np.float64).reshape(4, 4)
    calib = StereoCalib.from_json({
        "intrinsics_left": _intr(left_socket),
        "intrinsics_right": _intr(right_socket),
        "T_left_right": T_lr.tolist(),
    })

    try:
        R_imu_cam = np.array(ch.getImuToCameraExtrinsics(left_socket),
                             dtype=np.float64)[:3, :3]
    except Exception:
        R_imu_cam = None
    return K, calib, R_imu_cam


def _collect_startup(device: SharedLiveDevice, R_imu_cam, accel_cal,
                     *, estimate_bias: bool, gate: bool = True,
                     window_s: float = 0.4, timeout_s: float = 6.0):
    """Mean startup accel (gravity-align, cam frame). Returns ``(accel_align, gyro_bias)``.

    Two modes:

    * ``gate=True`` (used when the per-device gyro **bias** must be measured -- the
      first-ever run or ``--recalibrate-bias``): a sample is accepted only while the
      device is at rest (``|gyro| < _STILL_GYRO`` and accel within ``_STILL_ACCEL``
      of the window mean); any motion clears the buffer and restarts the window, so
      the bias mean is a true still-window. This is the only path that asks the
      operator to hold still, and it runs at most once per device.
    * ``gate=False`` (bias already cached -- the normal Start): collect over
      ``window_s`` with NO motion gate, a quick *rough* gravity seed. It need not be
      accurate because the odometry flow's continuous ``CorrectTilt`` re-levels
      roll/pitch on any at-rest frame -- so the operator does not hold still at Start.
    """
    q = device.q_imu
    accel: list[np.ndarray] = []
    gyro: list[np.ndarray] = []
    win_start: float | None = None
    t_start = time.monotonic()

    def _level(samples):
        a = np.mean(samples, axis=0)
        a = accel_cal.apply(a) if accel_cal is not None else a
        return (R_imu_cam @ a) if R_imu_cam is not None else a

    while time.monotonic() - t_start < timeout_s:
        msg = q.tryGet() if q is not None else None
        if msg is None:
            time.sleep(0.005)
            continue
        for w, v, _ in decode_imu_packets(msg):
            if not (np.all(np.isfinite(v)) and np.all(np.isfinite(w))):
                continue
            if gate:
                moving = float(np.linalg.norm(w)) > _STILL_GYRO
                if accel and float(np.linalg.norm(
                        v - np.mean(accel, axis=0))) > _STILL_ACCEL:
                    moving = True
                if moving:
                    accel.clear()
                    gyro.clear()
                    win_start = None
                    continue
            accel.append(v)
            gyro.append(w)
            now = time.monotonic()
            if win_start is None:
                win_start = now
            elif now - win_start >= window_s and len(gyro) >= 10:
                bias = np.mean(gyro, axis=0) if estimate_bias else None
                return _level(accel), bias
    # Never settled: rough level from whatever we saw, no measured bias.
    if estimate_bias:
        print("[live] WARNING: camera kept moving during startup calibration; "
              "gyro bias not estimated. Hold still and restart.", file=sys.stderr)
    return (_level(accel) if accel else None), None


def read_live_calibration(device: SharedLiveDevice, *, width: int, height: int,
                          use_gyro: bool, depth_fast: bool,
                          recalibrate_bias: bool = False) -> LiveFrontEndCalib:
    """Acquire the shared device and read all VIO boot references.

    The device is :meth:`~ours.lib.oak_live.SharedLiveDevice.acquire`-d here and
    kept open (the caller releases it when the run ends); the camera/IMU sources
    attach to the same reference-counted device when they start.
    """
    device.acquire()
    ch = device.read_calibration()
    K, calib, R_imu_cam = _read_stereo_calib(ch, width, height)
    res = ResolutionProfile.for_resolution(width, height)
    sgm_cfg = res.sgm(fast=depth_fast)

    accel_align = None
    imu_calibration = None
    if use_gyro:
        dev_id = device.device_id
        cached = None if recalibrate_bias else load_gyro_bias(dev_id)
        accel_cal: AccelCalibration | None = load_accel_calib(dev_id)
        # Hold-still ONLY when the gyro bias must be measured (first run /
        # --recalibrate-bias). Once cached, take a quick non-gated gravity seed --
        # the continuous CorrectTilt in the odometry flow re-levels at rest, so the
        # operator never has to hold the camera still at Start.
        need_bias = cached is None
        accel_align, measured = _collect_startup(
            device, R_imu_cam, accel_cal, estimate_bias=need_bias,
            gate=need_bias, window_s=0.4 if need_bias else 0.2)
        gyro_bias = cached if cached is not None else measured
        if measured is not None and cached is None:
            try:
                p = save_gyro_bias(dev_id, measured, 0)
                print(f"[live] gyro bias calibrated {measured.round(5).tolist()} "
                      f"rad/s -> saved to {p}", file=sys.stderr)
            except OSError as e:
                print(f"[live] WARNING: could not save gyro bias: {e}",
                      file=sys.stderr)
        if gyro_bias is not None or accel_cal is not None:
            imu_calibration = ImuCalibration(gyro_bias=gyro_bias, accel=accel_cal)

    return LiveFrontEndCalib(K=K, calib=calib, R_imu_cam=R_imu_cam,
                             sgm_cfg=sgm_cfg, res=res, accel_align=accel_align,
                             imu_calibration=imu_calibration)
