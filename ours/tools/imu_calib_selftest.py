#!/usr/bin/env python3
"""Offline test: imu-reader publishes RAW IMU and CALIBRATED IMU per frame.

The split front-end must, for every camera trigger, emit two messages from the
same drained interval:

* ``topics.IMU_RAW`` -- an :class:`~ours.lib.flow.messages.ImuRaw` carrying the
  uncalibrated samples (exactly what the sensor reported), and
* ``topics.IMUCAM_SAMPLE`` -- an :class:`~ours.lib.flow.messages.ImuCamPacket`
  whose ``gyro`` / ``accel`` are the CALIBRATED samples when a per-device
  calibration exists.

This drives the REAL :class:`~ours.flows.imu_reader.ImuReaderFlow` over a real
bus with a recorded session (no device) and a planted
:class:`~ours.lib.imu.imu_calib.ImuCalibration`, and checks both directly. It
also unit-checks the calibration maths and the no-calibration pass-through.

Run::

    python -m ours.tools.imu_calib_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.flows.cam_reader import CamReaderFlow                   # noqa: E402
from ours.flows.cam_reader.sources import ReplayCamSource         # noqa: E402
from ours.flows.imu_reader import ImuReaderFlow                   # noqa: E402
from ours.flows.imu_reader.sources import ReplayImuSource         # noqa: E402
from ours.lib.flow import Bus, Flow, topics                       # noqa: E402
from ours.lib.imu.accel_calib import AccelCalibration             # noqa: E402
from ours.lib.imu.imu_calib import ImuCalibration                 # noqa: E402
from ours.lib.io.reader import SessionReader                      # noqa: E402

_SESSION = "sessions/gold/lab_straight_20s"
_MAX_FRAMES = 30


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


class _Collector(Flow):
    """Sink: gather raw (imu.raw) and calibrated (imucam.sample) per topic."""

    def __init__(self, bus: Bus) -> None:
        super().__init__("collector", bus)
        self.raw: list = []
        self.cal: list = []
        self.on(topics.IMU_RAW, [self._grab(self.raw)])
        self.on(topics.IMUCAM_SAMPLE, [self._grab(self.cal)])
        self.expected_ends = 2          # both topics carry an END

    def _grab(self, bucket):
        class _Grab:
            name = "grab"

            def run(self, ctx, msg):
                bucket.append(msg)
                return None
        return _Grab()


def _planted_calibration() -> ImuCalibration:
    """A non-trivial gyro bias + affine accel correction to detect."""
    bias = np.array([0.013, -0.006, 0.009])
    T = np.array([[1.02, 0.01, -0.008],
                  [0.005, 0.98, 0.012],
                  [-0.009, 0.007, 1.01]])
    b = np.array([0.18, -0.12, 0.07])
    return ImuCalibration(gyro_bias=bias, accel=AccelCalibration(T, b))


def _run(session: str, calibration: ImuCalibration | None):
    reader = SessionReader(Path(session))
    bus = Bus()
    imu_flow = ImuReaderFlow(bus, ReplayImuSource(reader),
                             calibration=calibration)
    cam_flow = CamReaderFlow(
        bus, ReplayCamSource(reader, max_frames=_MAX_FRAMES), fps=20)
    sink = _Collector(bus)
    sink.start()
    imu_flow.start()
    cam_flow.start()
    cam_flow.join()
    finished = sink.done.wait(timeout=60.0)
    for f in (imu_flow, sink):
        f.stop()
    if not finished:
        raise SystemExit("graph did not drain within timeout")
    return sink.raw, sink.cal


def test_unit_apply() -> None:
    print(" ImuCalibration.apply maths")
    cal = _planted_calibration()
    rng = np.random.default_rng(0)
    gyro = rng.normal(0, 0.5, (7, 3))
    accel = rng.normal(0, 2.0, (7, 3)) + np.array([0, 0, 9.81])
    g_cal, a_cal = cal.apply(gyro, accel)
    _check(np.allclose(g_cal, gyro - cal.gyro_bias),
           "gyro = raw - bias")
    _check(np.allclose(a_cal, (accel - cal.accel.bias) @ cal.accel.T.T),
           "accel = T (raw - b)")
    # Empty interval must not blow up.
    ge, ae = cal.apply(np.zeros((0, 3)), np.zeros((0, 3)))
    _check(ge.shape == (0, 3) and ae.shape == (0, 3), "empty interval safe")
    # Identity passes through untouched.
    ident = ImuCalibration()
    _check(ident.is_identity, "empty calibration is identity")
    gi, ai = ident.apply(gyro, accel)
    _check(np.array_equal(gi, gyro) and np.array_equal(ai, accel),
           "identity passes raw through unchanged")


def test_flow_calibrated() -> None:
    print(" flow with calibration: raw stays raw, packet is calibrated")
    cal = _planted_calibration()
    raw, packets = _run(_SESSION, cal)
    _check(len(raw) == len(packets) and len(packets) > 0,
           f"one raw + one packet per frame ({len(raw)}/{len(packets)})")

    raw_by_seq = {r.seq: r for r in raw}
    n_with_imu = 0
    for p in packets:
        r = raw_by_seq[p.seq]
        # Same interval feeds both: identical timestamps.
        _check(np.array_equal(p.imu_ts, r.imu_ts),
               f"seq {p.seq}: raw and packet cover the same interval")
        if p.imu_ts.size == 0:
            _check(np.array_equal(p.gyro, r.gyro),
                   f"seq {p.seq}: empty interval identical")
            continue
        n_with_imu += 1
        exp_g, exp_a = cal.apply(r.gyro, r.accel)
        _check(np.allclose(p.gyro, exp_g) and np.allclose(p.accel, exp_a),
               f"seq {p.seq}: packet IMU equals calibration applied to raw")
        _check(not np.allclose(p.gyro, r.gyro),
               f"seq {p.seq}: calibrated gyro actually differs from raw")
    _check(n_with_imu >= 1, f"at least one frame carried IMU ({n_with_imu})")


def test_flow_no_calibration() -> None:
    print(" flow without calibration: packet IMU equals raw (pass-through)")
    raw, packets = _run(_SESSION, None)
    raw_by_seq = {r.seq: r for r in raw}
    for p in packets:
        r = raw_by_seq[p.seq]
        _check(np.array_equal(p.gyro, r.gyro) and np.array_equal(p.accel, r.accel),
               f"seq {p.seq}: no calibration -> packet IMU == raw")


def main() -> int:
    print("imu_calib_selftest")
    test_unit_apply()
    test_flow_calibrated()
    test_flow_no_calibration()
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
