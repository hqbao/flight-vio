#!/usr/bin/env python3
"""Self-test for the OAK device capability probe + multi-device selector.

Fully OFFLINE (no depthai, no hardware): drives
:func:`imu_camera.device.probe.probe_capabilities` and
:func:`~imu_camera.device.probe.select_device` with fake device handles and a fake
enumerator/opener, so the whole capability-detect + ``--model`` selection tree is
exercised on CI.

Covers:
  probe_capabilities --
    (a) IMU present ("BMI270")     -> has_imu True, fields populated,
    (b) IMU absent  ("" / "NONE")  -> has_imu False,
    (c) mono_max read from getConnectedCameraFeatures() configs,
    (d) a bare handle (missing methods) -> safe defaults, no raise.
  select_device --
    (e) zero devices                -> RuntimeError,
    (f) single device, no model     -> auto-opened,
    (g) multiple devices, no model   -> RuntimeError listing them, all closed,
    (h) select by exact deviceId    -> opens just that one (no full scan),
    (i) select by product-name substring -> first match kept, others CLOSED,
    (j) no match                    -> RuntimeError, every opened handle CLOSED.

Run::

    .venv/bin/python -m imu_camera.tests.probe_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from imu_camera.device import probe                              # noqa: E402

# The probe matches the CAM_B socket against the real depthai enum when depthai
# is importable, else against a plain marker (its offline branch). Use whichever
# the running env exposes so the fake feature's socket compares equal either way.
try:
    import depthai as _dai
    _CAM_B = _dai.CameraBoardSocket.CAM_B
except Exception:                                                 # noqa: BLE001
    _CAM_B = "CAM_B"


# --------------------------------------------------------------------------- #
# Fakes
# --------------------------------------------------------------------------- #
class _Cfg:
    def __init__(self, w, h):
        self.width, self.height = w, h


class _Feat:
    """A fake getConnectedCameraFeatures() entry for one socket."""
    def __init__(self, socket, configs):
        self.socket = socket
        self.configs = configs


class _FakeDevice:
    """A fake open depthai device exposing only what the probe reads."""
    def __init__(self, *, device_id="id0", name="OAK-D", imu="BMI270",
                 mono=(1280, 800), cam_b=_CAM_B):
        self._id, self._name, self._imu = device_id, name, imu
        self._mono, self._cam_b = mono, cam_b
        self.closed = False

    def getDeviceId(self):
        return self._id

    def getDeviceName(self):
        return self._name

    def getConnectedIMU(self):
        return self._imu

    def getConnectedCameraFeatures(self):
        if self._mono is None:
            raise RuntimeError("no features")
        return [_Feat(self._cam_b, [_Cfg(*self._mono)])]

    def close(self):
        self.closed = True


class _BareDevice:
    """A handle missing most methods -> the probe must not raise."""
    pass


class _Info:
    """A fake pre-connect DeviceInfo (only the id is known before opening)."""
    def __init__(self, device_id):
        self.deviceId = device_id


def _opener_from(devices):
    """Return an opener(info) that hands back the fake device for info.deviceId,
    so each DeviceInfo maps to a distinct, inspectable _FakeDevice."""
    by_id = {d._id: d for d in devices}

    def _open(info):
        return by_id[info.deviceId]
    return _open


def _enum_from(devices):
    return lambda: [_Info(d._id) for d in devices]


# --------------------------------------------------------------------------- #
def test_probe() -> None:
    # (a) IMU present.
    caps = probe.probe_capabilities(
        _FakeDevice(name="OAK-D-W", imu="BNO086", mono=(1280, 800)))
    assert caps.has_imu and caps.imu_type == "BNO086", caps
    assert caps.name == "OAK-D-W" and caps.device_id == "id0", caps
    assert caps.mono_max == (1280, 800), caps
    print("[a] probe: IMU present   -> has_imu True, fields populated           OK")

    # (b) IMU absent -- both the empty string and the "NONE" sentinel.
    for imu in ("", "NONE", "none", "  None  "):
        caps = probe.probe_capabilities(_FakeDevice(imu=imu))
        assert not caps.has_imu, (imu, caps)
    print("[b] probe: IMU absent ('', 'NONE') -> has_imu False                  OK")

    # (c) mono_max from the OV7251 Lite sensor.
    caps = probe.probe_capabilities(_FakeDevice(name="OAK-D-LITE", mono=(640, 480)))
    assert caps.mono_max == (640, 480), caps
    print("[c] probe: mono_max read from camera features                        OK")

    # (d) a bare handle -> safe defaults, no raise.
    caps = probe.probe_capabilities(_BareDevice())
    assert caps.has_imu is False and caps.mono_max == (640, 400), caps
    assert caps.device_id == "default", caps
    print("[d] probe: bare handle -> safe defaults, no raise                    OK")


def test_select() -> None:
    # (e) zero devices.
    try:
        probe.select_device(None, enumerator=lambda: [], opener=lambda i: None)
        raise AssertionError("expected RuntimeError on zero devices")
    except RuntimeError as e:
        assert "no OAK device" in str(e), e
    print("[e] select: zero devices -> RuntimeError                             OK")

    # (f) single device, no model -> auto.
    d0 = _FakeDevice(device_id="aaa", name="OAK-D-W")
    dev, seen = probe.select_device(
        None, enumerator=_enum_from([d0]), opener=_opener_from([d0]))
    assert dev is d0 and not d0.closed, (dev, seen)
    assert seen == [{"device_id": "aaa", "name": "OAK-D-W"}], seen
    print("[f] select: single device, no model -> auto-opened                   OK")

    # (g) multiple devices, no model -> RuntimeError; every probe handle closed.
    da = _FakeDevice(device_id="aaa", name="OAK-D-W")
    db = _FakeDevice(device_id="bbb", name="OAK-D-LITE")
    try:
        probe.select_device(None, enumerator=_enum_from([da, db]),
                            opener=_opener_from([da, db]))
        raise AssertionError("expected RuntimeError on ambiguous multi-device")
    except RuntimeError as e:
        assert "2 OAK devices" in str(e) and "--model" in str(e), e
    assert da.closed and db.closed, "ambiguous probe must close every handle"
    print("[g] select: multi, no model -> RuntimeError, all closed              OK")

    # (h) select by exact deviceId -> opens only that one (others never opened).
    da = _FakeDevice(device_id="aaa", name="OAK-D-W")
    db = _FakeDevice(device_id="bbb", name="OAK-D-LITE")
    opened: list[str] = []

    def _track_opener(info):
        opened.append(info.deviceId)
        return {"aaa": da, "bbb": db}[info.deviceId]

    dev, _ = probe.select_device(
        "BBB", enumerator=_enum_from([da, db]), opener=_track_opener)
    assert dev is db and not db.closed, dev
    assert opened == ["bbb"], ("deviceId fast path must open only the match", opened)
    print("[h] select: by exact deviceId -> opens only the match                OK")

    # (i) select by product-name substring -> first match kept, the rest CLOSED.
    da = _FakeDevice(device_id="aaa", name="OAK-D-W")
    db = _FakeDevice(device_id="bbb", name="OAK-D-LITE")
    dev, seen = probe.select_device(
        "lite", enumerator=_enum_from([da, db]), opener=_opener_from([da, db]))
    assert dev is db and not db.closed, dev
    assert da.closed, "the non-matching device must be closed"
    print("[i] select: by name substring -> match kept, non-match closed         OK")

    # (j) no match -> RuntimeError; every opened handle closed.
    da = _FakeDevice(device_id="aaa", name="OAK-D-W")
    db = _FakeDevice(device_id="bbb", name="OAK-D-PRO")
    try:
        probe.select_device("lite", enumerator=_enum_from([da, db]),
                            opener=_opener_from([da, db]))
        raise AssertionError("expected RuntimeError on no name match")
    except RuntimeError as e:
        assert "lite" in str(e), e
    assert da.closed and db.closed, "a no-match scan must close every handle"
    print("[j] select: no match -> RuntimeError, all closed                     OK")


def main() -> int:
    test_probe()
    test_select()
    print("\nALL probe + select_device CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
