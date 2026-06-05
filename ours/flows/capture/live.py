"""Live capture: stream the OAK-D device onto the bus.

This is the on-device counterpart of :class:`ReplayCaptureFlow`. It taps BOTH raw
cameras (no VPU StereoDepth) plus the IMU and publishes the SAME topics, so the
depth / odometry / backend / slam / ui flows are byte-for-byte identical live or
offline:

* one :class:`~ours.lib.messages.ImuInit` (startup gravity-align accel), then
* per matched stereo pair, one :class:`~ours.lib.messages.ImuPrior` (the gyro
  rotation prior integrated since the previous frame) followed by one
  :class:`~ours.lib.messages.RawFrame` carrying the RAW left + RAW right frames
  (the depth flow rectifies both).

Only the *capture* concern lives here -- depth, odometry, BA and SLAM are other
flows. The IMU->prior fusion (gyro integration + at-rest accel) mirrors the
validated legacy ``OakOursVioSource._run`` so the rotation prior convention
matches the offline gyro preintegrator.

Wiring is two-phase because the device calibration is only known once the device
is open: call :meth:`open` (opens the device, returns :class:`LiveCalib`), build
the downstream flows from that calibration, then ``start()`` the flow to stream.

NOTE: this path can only be exercised on real hardware; it is not part of the
offline test harness.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from ...lib import topics
from ...lib.config.resolution import ResolutionProfile
from ...lib.flow import SourceFlow
from ...lib.imu.imu import so3_exp
from ...lib.io.reader import StereoCalib
from ...lib.messages import ImuInit, ImuPrior, RawFrame
from ...lib.pubsub import Bus
from ...lib.task import Task

# Accel leveling only runs when the camera is at rest, detected from the residual
# of the raw accelerometer against its EMA (recent motion energy, m/s^2). Mirrors
# the legacy live path so the at-rest gate behaves identically.
_REST_MOTION_THRESH = 0.35


@dataclass(frozen=True)
class LiveCalib:
    """Device calibration the downstream flows need (read once at ``open``)."""

    K: np.ndarray
    calib: StereoCalib
    sgm_cfg: object
    res: ResolutionProfile
    accel_align: np.ndarray | None


class _PublishCapture(Task):
    """Route a produced message to its topic by type."""

    name = "publish_capture"

    def run(self, ctx, msg):
        if isinstance(msg, (ImuInit, ImuPrior)):
            ctx.bus.publish(topics.IMU_SAMPLE, msg)
        elif isinstance(msg, RawFrame):
            ctx.bus.publish(topics.FRAME_RAW, msg)
        return None


class LiveCaptureFlow(SourceFlow):
    def __init__(self, bus: Bus, width: int = 640, height: int = 400,
                 fps: int = 20, depth_fast: bool = True,
                 use_gyro: bool = True) -> None:
        super().__init__("capture", bus, [_PublishCapture()])
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.depth_fast = bool(depth_fast)
        self.use_gyro = bool(use_gyro)
        self.res = ResolutionProfile.for_resolution(self.width, self.height)
        self.forwards_to(topics.FRAME_RAW)

        # Filled by open(); the device + queues stay alive until produce() ends.
        self._dai = None
        self._pipeline = None
        self._q_left = None
        self._q_right = None
        self._q_imu = None
        self._R_imu_cam = np.eye(3)
        self._gyro_bias = np.zeros(3)
        self._accel_align: np.ndarray | None = None

    # ------------------------------------------------------------------ #
    def open(self) -> LiveCalib:
        """Open the device, read calibration, collect the startup accel/bias."""
        import depthai as dai

        self._dai = dai
        left_socket = dai.CameraBoardSocket.CAM_B
        right_socket = dai.CameraBoardSocket.CAM_C

        p = dai.Pipeline()
        left = p.create(dai.node.Camera).build(left_socket, sensorFps=self.fps)
        right = p.create(dai.node.Camera).build(right_socket, sensorFps=self.fps)
        imu = p.create(dai.node.IMU)
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,
                             dai.IMUSensor.GYROSCOPE_RAW], 200)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)

        left_out = left.requestOutput((self.width, self.height))
        right_out = right.requestOutput((self.width, self.height))
        self._q_left = left_out.createOutputQueue(maxSize=4, blocking=False)
        self._q_right = right_out.createOutputQueue(maxSize=4, blocking=False)
        self._q_imu = imu.out.createOutputQueue(maxSize=50, blocking=False)

        p.start()
        self._pipeline = p

        ch = p.getDefaultDevice().readCalibration()
        K = np.array(ch.getCameraIntrinsics(left_socket, self.width,
                                            self.height), dtype=np.float64)

        def _intr(sock):
            Ki = np.array(ch.getCameraIntrinsics(sock, self.width, self.height),
                          dtype=np.float64)
            dist = list(ch.getDistortionCoefficients(sock))
            return {"fx": float(Ki[0, 0]), "fy": float(Ki[1, 1]),
                    "cx": float(Ki[0, 2]), "cy": float(Ki[1, 2]),
                    "dist": [float(x) for x in dist],
                    "width": int(self.width), "height": int(self.height)}

        T_lr = np.array(ch.getCameraExtrinsics(left_socket, right_socket),
                        dtype=np.float64).reshape(4, 4)
        calib = StereoCalib.from_json({
            "intrinsics_left": _intr(left_socket),
            "intrinsics_right": _intr(right_socket),
            "T_left_right": T_lr.tolist(),
        })

        try:
            self._R_imu_cam = np.array(
                ch.getImuToCameraExtrinsics(left_socket),
                dtype=np.float64)[:3, :3]
        except Exception:
            self._R_imu_cam = np.eye(3)

        if self.use_gyro:
            self._accel_align = self._collect_startup()

        sgm_cfg = self.res.sgm(fast=self.depth_fast)
        return LiveCalib(K=K, calib=calib, sgm_cfg=sgm_cfg, res=self.res,
                         accel_align=self._accel_align)

    def _collect_startup(self, window_s: float = 0.4,
                         timeout_s: float = 2.0) -> np.ndarray | None:
        """Mean startup accel (cam frame) + gyro bias over a static window."""
        accel: list[np.ndarray] = []
        gyro: list[np.ndarray] = []
        t_start = time.monotonic()
        t_first: float | None = None
        while time.monotonic() - t_start < timeout_s:
            msg = self._q_imu.tryGet()
            if msg is None:
                time.sleep(0.005)
                continue
            for pkt in msg.packets:
                a = pkt.acceleroMeter
                v = np.array([a.x, a.y, a.z], dtype=np.float64)
                if np.all(np.isfinite(v)):
                    accel.append(v)
                g = pkt.gyroscope
                w = np.array([g.x, g.y, g.z], dtype=np.float64)
                if np.all(np.isfinite(w)):
                    gyro.append(w)
            if t_first is None:
                t_first = time.monotonic()
            elif time.monotonic() - t_first >= window_s:
                break
        if gyro:
            self._gyro_bias = np.mean(gyro, axis=0)
        if not accel:
            return None
        return self._R_imu_cam @ np.mean(accel, axis=0)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _seq(msg) -> int:
        try:
            return int(msg.getSequenceNum())
        except Exception:
            return -1

    @staticmethod
    def _gray(frame) -> np.ndarray:
        g = frame.getCvFrame()
        if g.ndim == 3:                                  # BGR -> luminance (601)
            g = (g[..., 0] * 0.114 + g[..., 1] * 0.587
                 + g[..., 2] * 0.299).astype(np.uint8)
        return g

    def produce(self):
        if self._pipeline is None:
            self.open()
        p = self._pipeline
        ql, qr, qi = self._q_left, self._q_right, self._q_imu
        R_imu_cam = self._R_imu_cam
        gyro_bias = self._gyro_bias

        yield ImuInit(self._accel_align)

        pend_l: dict[int, object] = {}
        pend_r: dict[int, object] = {}
        gyro_last_ts: float | None = None
        accel_ema: np.ndarray | None = None
        try:
            while not self._stop.is_set() and p.isRunning():
                # Drain left to newest; stash by sequence so SGM always gets a
                # true same-instant stereo pair (cameras are hardware synced).
                ld = ql.tryGet()
                while True:
                    nxt = ql.tryGet()
                    if nxt is None:
                        break
                    ld = nxt
                if ld is not None:
                    pend_l[self._seq(ld)] = ld
                while True:
                    nxt = qr.tryGet()
                    if nxt is None:
                        break
                    pend_r[self._seq(nxt)] = nxt
                common = pend_l.keys() & pend_r.keys()
                if not common:
                    for buf in (pend_l, pend_r):
                        if len(buf) > 8:
                            for k in sorted(buf)[:-8]:
                                buf.pop(k, None)
                    time.sleep(0.002)
                    continue
                seq = max(common)
                ld = pend_l.pop(seq)
                rd = pend_r.pop(seq)
                for k in [k for k in pend_l if k < seq]:
                    pend_l.pop(k, None)
                for k in [k for k in pend_r if k < seq]:
                    pend_r.pop(k, None)

                # Integrate the gyro since the previous frame + average accel.
                R_imu_accum = np.eye(3)
                gyro_cnt = 0
                acc_sum = np.zeros(3)
                acc_cnt = 0
                imsg = qi.tryGet()
                while imsg is not None:
                    for pkt in imsg.packets:
                        a = pkt.acceleroMeter
                        v = (a.x, a.y, a.z)
                        if np.all(np.isfinite(v)):
                            acc_sum += v
                            acc_cnt += 1
                        g = pkt.gyroscope
                        w = np.array([g.x, g.y, g.z], dtype=np.float64)
                        if np.all(np.isfinite(w)):
                            try:
                                ts = g.getTimestampDevice().total_seconds()
                            except Exception:
                                ts = None
                            if ts is not None:
                                if gyro_last_ts is not None:
                                    dt = ts - gyro_last_ts
                                    if 0.0 < dt < 0.1:
                                        R_imu_accum = R_imu_accum @ so3_exp(
                                            (w - gyro_bias) * dt)
                                        gyro_cnt += 1
                                gyro_last_ts = ts
                    imsg = qi.tryGet()

                R_prior = (R_imu_cam @ R_imu_accum @ R_imu_cam.T
                           if gyro_cnt > 0 else None)

                accel_cam = None
                at_rest = False
                if acc_cnt > 0:
                    accel_raw = R_imu_cam @ (acc_sum / acc_cnt)
                    if accel_ema is None:
                        accel_ema = accel_raw.copy()
                    else:
                        accel_ema += 0.2 * (accel_raw - accel_ema)
                    accel_cam = accel_ema
                    at_rest = float(np.linalg.norm(accel_raw - accel_ema)) \
                        < _REST_MOTION_THRESH

                try:
                    ts_ns = int(ld.getTimestampDevice().total_seconds() * 1e9)
                except Exception:
                    ts_ns = time.monotonic_ns()

                yield ImuPrior(seq, R_prior, accel_cam, at_rest)
                yield RawFrame(seq, ts_ns, self._gray(ld), self._gray(rd))
        finally:
            try:
                p.stop()
            except Exception:
                pass
