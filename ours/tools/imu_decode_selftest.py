"""Offline self-test for the shared depthai IMU packet decoder.

``decode_imu_packets`` is duck-typed over the depthai message, so we drive it
with tiny fakes (no hardware): correct field extraction across multiple packets,
float64 output, and the no-timestamp fallback (``t_s`` is ``None`` so the caller
picks its own clock).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.lib.imu.decode import decode_imu_packets  # noqa: E402


class _V:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class _TS:
    def __init__(self, s):
        self._s = s

    def total_seconds(self):
        return self._s


class _Gyro(_V):
    def __init__(self, x, y, z, ts=None):
        super().__init__(x, y, z)
        self._ts = ts

    def getTimestampDevice(self):
        if self._ts is None:
            raise RuntimeError("no device timestamp")
        return _TS(self._ts)


class _Pkt:
    def __init__(self, accel, gyro):
        self.acceleroMeter = accel
        self.gyroscope = gyro


class _Msg:
    def __init__(self, packets):
        self.packets = packets


def main() -> int:
    ok = True

    msg = _Msg([
        _Pkt(_V(0.1, 0.2, 9.8), _Gyro(0.01, -0.02, 0.03, ts=1.500)),
        _Pkt(_V(0.0, 0.0, 9.81), _Gyro(0.0, 0.0, 0.0, ts=1.505)),
    ])
    out = decode_imu_packets(msg)

    ok_n = len(out) == 2
    print(f"packet count: {'OK' if ok_n else 'FAIL'}")
    ok &= ok_n

    g0, a0, t0 = out[0]
    ok_fields = (
        np.allclose(g0, [0.01, -0.02, 0.03])
        and np.allclose(a0, [0.1, 0.2, 9.8])
        and t0 == 1.500
        and g0.dtype == np.float64 and a0.dtype == np.float64
    )
    print(f"field extraction + dtype: {'OK' if ok_fields else 'FAIL'}")
    ok &= ok_fields

    _, _, t1 = out[1]
    ok_t1 = t1 == 1.505
    print(f"second packet timestamp: {'OK' if ok_t1 else 'FAIL'}")
    ok &= ok_t1

    # A packet without a device timestamp must yield t_s=None (caller decides).
    msg2 = _Msg([_Pkt(_V(1, 2, 3), _Gyro(4, 5, 6, ts=None))])
    _, _, t = decode_imu_packets(msg2)[0]
    ok_none = t is None
    print(f"missing timestamp -> None: {'OK' if ok_none else 'FAIL'}")
    ok &= ok_none

    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
