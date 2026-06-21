#!/usr/bin/env python3
"""SIL (software-in-the-loop) selftest for the ``fc`` UART sender (dblink).

Drives the real :class:`fc.main.UartSender` against a pty pair (``os.openpty``)
-- the slave side stands in for the FC's serial port -- and the real
:class:`fc.main.LatestPose` holder, exercising the flight-safety floors WITHOUT a
device:

  (a) WIRE FRAMES -- the bytes on the pty parse as well-formed ``dblink`` frames
      (``'db'`` magic, CMD == DB_CMD_VIO_POSE 0x0C, correct LEN + checksum) and
      the pos_n/e/d carry the expected NED position for a known optical-world pose.
  (b) AGE -- ``age_us`` is small + plausible for a just-captured pose, and tracks
      the capture->send elapsed (a pose with an OLD device ts reports a larger age).
  (c) RESET EDGE -- reset_counter bumps EXACTLY ONCE across a multi-frame
      ``sensor_gap_s`` re-lock (the rising edge), and once on an fc-local jump.
  (d) STALE -- a pose older than the staleness window is NOT sent.
  (e) DEGRADED FLOOR -- a ``vio_degraded`` frame sends the INFLATED sigma and the
      degraded flag bit; a clean frame sends the real sigma and clears it.
  (f) LATEST-WINS -- a SLOW pty reader (the FC side draining slowly) does NOT
      stall the IPC callback: the callback's store-and-return stays sub-millisecond
      even while the sender thread is blocked on a full pty write buffer, and the
      holder keeps only the freshest pose.
  (g) NON-FINITE SURVIVAL -- an exploding (1e300) / NaN pose does NOT kill the UART
      daemon thread (it must never silently starve the FC of pose) and goes out as
      an explicitly INVALID, degraded frame (pos_valid=0, degraded=1, position
      zeroed, no NaN/inf on the wire) -- never as a real fix.

  .venv/bin/python -m fc.tests.fc_sil_selftest
"""
from __future__ import annotations

import os
import struct
import sys
import threading
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fc.main import (                                            # noqa: E402
    LatestPose, UartSender, _SIGMA_DEGRADED, _STALE_S,
)
from sky.fc.dblink import (                                      # noqa: E402
    DB_CMD_VIO_POSE, VIO_LEN,
)
from sky.fc.fc_earth_pose import earth_pose_from_T_world_cam    # noqa: E402

#: The dblink VIO-pose payload layout (mirrors sky.fc.dblink._PAYLOAD_STRUCT).
_POSE_STRUCT = struct.Struct("<8fIBB")
_FLAG_DEGRADED = 1 << 2


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


@dataclass
class _FakePose:
    """Minimal stand-in for WirePoseMsg (the sender reads T + info + ts_ns)."""
    T_world_cam: np.ndarray
    info: dict[str, Any] = field(default_factory=dict)
    ts_ns: int = 0
    seq: int = 0


class _FdSerial:
    """A serial-like object that writes to a file descriptor (the pty master)."""

    def __init__(self, fd: int) -> None:
        self._fd = fd

    def write(self, data: bytes) -> int:
        return os.write(self._fd, data)


def _open_raw_pty():
    """openpty() with BOTH ends in RAW mode. A default pty is in COOKED mode, so
    its output line discipline (OPOST/ONLCR) translates a 0x0A byte to CR-LF --
    inserting a 0x0D into the BINARY dblink frame and desyncing the stream (the
    age/checksum fields routinely contain 0x0A). A real UART (pyserial) is raw and
    never does this, so the corruption is a pty-stand-in artifact only."""
    master, slave = os.openpty()
    tty.setraw(master)
    tty.setraw(slave)
    return master, slave


def _T(p=(0.0, 0.0, 0.0), R=None) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    if R is not None:
        T[:3, :3] = R
    T[:3, 3] = p
    return T


def _checksum(cmd_id: int, payload: bytes) -> int:
    """Independent recompute of the dblink checksum (class fixed 0x00)."""
    length = len(payload)
    return (cmd_id + 0x00 + (length & 0xFF) + ((length >> 8) & 0xFF)
            + sum(payload)) & 0xFFFF


def _parse_db_stream(buf):
    """VERBATIM copy of the FC's ``parse_db_stream`` (dblink_test.py:105).

    Returns ``(frames, tail)`` where each frame is ``(msg_id, payload_bytes)``. The
    test verifies the checksum SEPARATELY (the FC routes on the CMD byte and does
    not check this checksum, but a well-formed frame must still carry the right one).
    """
    out = []
    i = 0
    while True:
        j = buf.find(b"db", i)
        if j < 0:
            tail = buf[-1:] if buf.endswith(b"d") else b""
            return out, tail
        if len(buf) - j < 6:
            return out, bytes(buf[j:])
        length = int.from_bytes(buf[j + 4:j + 6], "little")
        if length > 1024:
            i = j + 2
            continue
        frame_total = 6 + length + 2
        if len(buf) - j < frame_total:
            return out, bytes(buf[j:])
        msg_id = buf[j + 2]
        payload = bytes(buf[j + 6:j + 6 + length])
        cksum = buf[j + 6 + length] | (buf[j + 7 + length] << 8)
        out.append((msg_id, payload, cksum))
        i = j + frame_total


def _drain(fd: int, sink: list, stop: threading.Event,
           per_read_sleep: float = 0.0) -> None:
    """Read the pty master into ``sink`` until stopped (optionally slow)."""
    os.set_blocking(fd, False)
    while not stop.is_set():
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                sink.append(chunk)
        except BlockingIOError:
            pass
        time.sleep(max(per_read_sleep, 0.002))


# --------------------------------------------------------------------------- #
def test_wire_and_degraded() -> bool:
    print("[a/e] well-formed dblink frames + inflated sigma/flag when degraded")
    master, slave = _open_raw_pty()
    sink: list = []
    stop = threading.Event()
    rdr = threading.Thread(target=_drain, args=(master, sink, stop), daemon=True)
    rdr.start()

    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)

    # A known optical-world pose: +Z forward 2.5 m -> NED North +2.5. Clean frame
    # with pos_sigma_m -> the real sigma; ts_ns set so age uses the device clock.
    pos_opt = (0.0, 0.0, 2.5)
    now_ns = time.time_ns()
    clean = _FakePose(_T(p=pos_opt), info={"ok": True, "pos_sigma_m": 0.07},
                      ts_ns=now_ns)
    latest.set(clean, time.monotonic())
    _check(sender.send_once() is True, "clean frame sent")
    # A degraded frame -> inflated sigma + degraded flag bit.
    degraded = _FakePose(_T(p=pos_opt),
                         info={"ok": True, "pos_sigma_m": 0.07,
                               "vio_degraded": True}, ts_ns=time.time_ns())
    latest.set(degraded, time.monotonic())
    _check(sender.send_once() is True, "degraded frame sent")

    time.sleep(0.1)
    stop.set()
    rdr.join(timeout=1.0)
    frames, tail = _parse_db_stream(b"".join(sink))
    _check(len(frames) >= 2, f"two dblink frames on the wire (got {len(frames)})")

    # Framing + checksum + CMD on every frame.
    ok = True
    for msg_id, payload, cksum in frames[:2]:
        ok &= (msg_id == DB_CMD_VIO_POSE == 0x0C)
        ok &= (len(payload) == VIO_LEN == 38)
        ok &= (cksum == _checksum(msg_id, payload))
    _check(ok, "CMD==0x0C, LEN==38, checksum correct on both frames")

    pos_ned_expected, _, _ = earth_pose_from_T_world_cam(_T(p=pos_opt))
    g0 = _POSE_STRUCT.unpack(frames[0][1])
    g1 = _POSE_STRUCT.unpack(frames[1][1])
    pos_ok = (abs(g0[0] - pos_ned_expected[0]) < 1e-3
              and abs(g0[1] - pos_ned_expected[1]) < 1e-3
              and abs(g0[2] - pos_ned_expected[2]) < 1e-3)
    _check(pos_ok, "pos_n/e/d match the known NED pose (North +2.5)")

    # Quaternion: identity optical pose -> a fixed unit body->NED quaternion. Just
    # assert it is unit-norm (the SSOT is exercised exhaustively in its own test).
    qn = float(np.linalg.norm(g0[3:7]))
    _check(abs(qn - 1.0) < 1e-5, f"quaternion is unit-norm on the wire (|q|={qn:.6f})")

    # sigma + flags: clean -> real sigma + degraded bit CLEAR; degraded -> inflated
    # sigma + degraded bit SET.
    sig0 = struct.unpack("<f", struct.pack("<f", 0.07))[0]
    _check(abs(g0[7] - sig0) < 1e-9 and not (g0[10] & _FLAG_DEGRADED),
           "clean frame -> real pos_sigma_m, degraded bit clear")
    _check(g1[7] >= _SIGMA_DEGRADED and bool(g1[10] & _FLAG_DEGRADED),
           f"degraded frame -> inflated sigma ({g1[7]:.1f}m) + degraded flag set")
    # NEVER NaN on the wire.
    _check(not (np.isnan(g0[7]) or np.isnan(g1[7])), "pos_sigma_m is never NaN")

    os.close(master)
    os.close(slave)
    return True


def test_age() -> bool:
    print("[b] age_us is small for a fresh pose + larger for an old capture ts")
    master, slave = _open_raw_pty()
    sink: list = []
    stop = threading.Event()
    rdr = threading.Thread(target=_drain, args=(master, sink, stop), daemon=True)
    rdr.start()

    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)

    # A "fresh" pose: device ts ~ now, received now. The recovered offset makes
    # age ~ the (tiny) host elapsed between recv and send.
    base = _T(p=(0.0, 0.0, 1.0))
    for _ in range(5):
        latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1},
                             ts_ns=time.time_ns()), time.monotonic())
        sender.send_once()
        time.sleep(0.005)

    # Now a pose whose CAPTURE was 40 ms before "now" (device ts older) while it is
    # received now -> age should jump by ~the 40 ms, NOT collapse to ~0.
    old_capture_ns = time.time_ns() - 40_000_000     # 40 ms ago
    latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1},
                         ts_ns=old_capture_ns), time.monotonic())
    sender.send_once()

    time.sleep(0.1)
    stop.set()
    rdr.join(timeout=1.0)
    frames, _ = _parse_db_stream(b"".join(sink))
    ages_us = [_POSE_STRUCT.unpack(p)[8] for _, p, _ in frames]
    _check(len(ages_us) >= 6, f"got the age samples (n={len(ages_us)})")
    # The fresh-pose ages are small (well under the 250 ms staleness window).
    fresh_max = max(ages_us[:5])
    _check(fresh_max < 50_000, f"fresh-pose age stays small (<50ms, max {fresh_max}us)")
    # The old-capture frame reports a markedly larger age (~40 ms more).
    _check(ages_us[-1] > fresh_max + 25_000,
           f"old-capture pose reports a larger age ({ages_us[-1]}us > "
           f"{fresh_max}us + 25ms)")

    os.close(master)
    os.close(slave)
    return True


def test_age_fallback_no_ts() -> bool:
    print("[b'] age fallback: ts_ns==0 -> age from queue time only (never crashes)")
    master, slave = _open_raw_pty()
    os.set_blocking(master, False)
    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)
    # ts_ns unset (loose path): set received 30 ms ago -> age ~ 30 ms, still fresh.
    latest.set(_FakePose(_T(p=(0.0, 0.0, 1.0)), info={"pos_sigma_m": 0.1}, ts_ns=0),
               time.monotonic() - 0.03)
    _check(sender.send_once() is True, "ts_ns==0 frame still sends (fallback age)")
    os.close(master)
    os.close(slave)
    return True


def test_reset_edge() -> bool:
    print("[c] reset_counter bumps ONCE per gap re-lock + once on an fc-local jump")
    master, slave = _open_raw_pty()
    os.set_blocking(master, False)
    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)

    base = _T(p=(0.0, 0.0, 1.0))
    # 3 clean frames (establish prev_pos, no gap) -> no bump.
    for _ in range(3):
        latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1}),
                   time.monotonic())
        sender.send_once()
    _check(sender.reset_counter == 0, "no bump on clean frames (reset_counter still 0)")

    # A multi-frame sensor_gap re-lock: sensor_gap_s present on 3 CONSECUTIVE
    # frames -> the rising edge fires ONCE, not 3x.
    for _ in range(3):
        latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1,
                                         "sensor_gap_s": 0.9,
                                         "inertial_dr": True}),
                   time.monotonic())
        sender.send_once()
    _check(sender.reset_counter == 1,
           f"gap rising edge bumped reset_counter ONCE (got {sender.reset_counter})")

    # Gap clears, then a SECOND gap event -> a second single bump.
    for _ in range(2):
        latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1}),
                   time.monotonic())
        sender.send_once()
    for _ in range(2):
        latest.set(_FakePose(base, info={"ok": True, "pos_sigma_m": 0.1,
                                         "sensor_gap_s": 1.2}),
                   time.monotonic())
        sender.send_once()
    _check(sender.reset_counter == 2,
           f"second gap event bumped once more (got {sender.reset_counter})")

    # An fc-local POSITION JUMP (no gap): a 3 m optical-Z jump in one frame (>> the
    # 0.5 m floor) bumps once.
    latest.set(_FakePose(_T(p=(0.0, 0.0, 1.0)),
                         info={"ok": True, "pos_sigma_m": 0.1}), time.monotonic())
    sender.send_once()
    rc_pre_jump = sender.reset_counter
    latest.set(_FakePose(_T(p=(0.0, 0.0, 4.0)),     # +3 m jump
                         info={"ok": True, "pos_sigma_m": 0.1}), time.monotonic())
    sender.send_once()
    _check(sender.reset_counter == rc_pre_jump + 1,
           f"position jump bumped reset_counter once (got {sender.reset_counter})")

    os.close(master)
    os.close(slave)
    return True


def test_stale_not_sent() -> bool:
    print("[d] a pose older than the staleness window is NOT sent")
    master, slave = _open_raw_pty()
    os.set_blocking(master, False)
    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)
    # Stamp the pose as already older than the stale window.
    latest.set(_FakePose(_T(p=(0.0, 0.0, 1.0)), info={"pos_sigma_m": 0.1}),
               time.monotonic() - (_STALE_S + 0.2))
    sent = sender.send_once()
    _check(sent is False and sender.n_stale == 1,
           "stale pose skipped (not sent, n_stale incremented)")
    os.close(master)
    os.close(slave)
    return True


def test_latest_wins_under_load() -> bool:
    print("[f] latest-wins: a SLOW pty reader does NOT stall the IPC callback")
    # A tiny pty buffer that a slow reader lets fill: the sender thread will block
    # in write(), but the callback (LatestPose.set, the real IPC path) must stay
    # instant. We measure the worst-case callback latency while the sender runs.
    master, slave = _open_raw_pty()
    sink: list = []
    stop = threading.Event()
    # DELIBERATELY slow reader (50 ms between reads) so the pty write buffer backs
    # up and the sender thread blocks on os.write.
    rdr = threading.Thread(target=_drain, args=(master, sink, stop, 0.05),
                           daemon=True)
    rdr.start()

    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)
    sender.start()

    # Hammer the callback (the IPC recv path) and record the max set() latency.
    max_cb_s = 0.0
    for i in range(400):
        msg = _FakePose(_T(p=(0.0, 0.0, 1.0 + i * 0.001)),
                        info={"ok": True, "pos_sigma_m": 0.1}, ts_ns=time.time_ns())
        t0 = time.monotonic()
        latest.set(msg, time.monotonic())   # <- exactly what _on_pose does
        max_cb_s = max(max_cb_s, time.monotonic() - t0)
        time.sleep(0.001)
    sender.stop()
    stop.set()
    sender.join(timeout=2.0)
    rdr.join(timeout=1.0)
    os.close(master)
    os.close(slave)

    # The callback never does I/O, so even with the sender thread blocked on a full
    # pty it must return in well under a millisecond. Generous ceiling = 5 ms (GC /
    # scheduler jitter); the real cost is ~microseconds.
    _check(max_cb_s < 5e-3,
           f"callback store-and-return stayed fast under load "
           f"(max {max_cb_s * 1e3:.3f} ms < 5 ms)")
    # And the latest stored pose is the LAST one (latest-wins, not a backlog).
    wm, _ = latest.get()
    _check(abs(wm.T_world_cam[2, 3] - (1.0 + 399 * 0.001)) < 1e-9,
           "holder kept the FRESHEST pose (latest-wins, no queue)")
    return True


def test_nonfinite_pose_survives() -> bool:
    print("[g] a NON-FINITE pose -> INVALID+degraded frame; thread STAYS ALIVE")
    # This codebase genuinely produces exploding / NaN poses (--tight on shake,
    # --direct divergence). Such a pose must NOT kill the UART thread (which would
    # silently starve the FC of pose) and must go out advertised INVALID, never as
    # a real fix.
    _FLAG_POS_VALID = 1 << 0
    master, slave = _open_raw_pty()
    sink: list = []
    stop = threading.Event()
    rdr = threading.Thread(target=_drain, args=(master, sink, stop), daemon=True)
    rdr.start()

    latest = LatestPose()
    sender = UartSender(latest, _FdSerial(slave), rate_hz=50.0)
    sender.start()                       # run the REAL thread loop, not just send_once

    # Feed an EXPLODING translation (1e300, out of f32 range) then a NaN pose,
    # interleaved with a clean pose, across several cadence cycles.
    for k in range(8):
        if k % 3 == 0:
            T = _T(p=(1e300, 1e300, 1e300))
        elif k % 3 == 1:
            T = _T(p=(float("nan"), 0.0, 1.0))
        else:
            T = _T(p=(0.0, 0.0, 2.5))     # clean
        latest.set(_FakePose(T, info={"ok": True, "pos_sigma_m": 0.1},
                             ts_ns=time.time_ns()), time.monotonic())
        time.sleep(0.02)

    time.sleep(0.1)
    # THE point of the test: the daemon thread must not have died on the bad poses.
    _check(sender.is_alive(),
           "UART sender thread is STILL ALIVE after non-finite poses")
    sender.stop()
    stop.set()
    sender.join(timeout=2.0)
    rdr.join(timeout=1.0)

    frames, _ = _parse_db_stream(b"".join(sink))
    _check(len(frames) >= 2, f"frames still emitted across the bad poses (n={len(frames)})")

    # Every emitted frame must be well-formed AND carry only finite floats; the
    # non-finite ones must advertise pos_valid=0 + degraded=1.
    n_invalid = 0
    for msg_id, payload, cksum in frames:
        _check(msg_id == DB_CMD_VIO_POSE and len(payload) == VIO_LEN
               and cksum == _checksum(msg_id, payload),
               "frame well-formed (CMD/LEN/checksum) even on a bad-pose cycle")
        g = _POSE_STRUCT.unpack(payload)
        _check(all(not (np.isnan(v) or np.isinf(v)) for v in g[0:8]),
               "no NaN/inf in any of the 8 float payload fields")
        flags = g[10]
        if not (flags & _FLAG_POS_VALID):
            n_invalid += 1
            _check(bool(flags & _FLAG_DEGRADED),
                   "an INVALID-position frame also has the degraded bit set")
            # The packer zeroed the broken position.
            _check(g[0] == 0.0 and g[1] == 0.0 and g[2] == 0.0,
                   "INVALID frame position is zeroed (not the exploded value)")
    _check(n_invalid >= 1, f"at least one frame advertised pos_valid=0 (got {n_invalid})")
    _check(sender.n_nonfinite >= 1,
           f"sender counted the non-finite frames (n_nonfinite={sender.n_nonfinite})")

    os.close(master)
    os.close(slave)
    return True


def main() -> int:
    print("fc_sil_selftest -- pty-loopback SIL for the dblink UART sender")
    results = {
        "wire + degraded floor": test_wire_and_degraded(),
        "age":                   test_age(),
        "age fallback (no ts)":  test_age_fallback_no_ts(),
        "reset edge":            test_reset_edge(),
        "stale not sent":        test_stale_not_sent(),
        "latest-wins/load":      test_latest_wins_under_load(),
        "non-finite survives":   test_nonfinite_pose_survives(),
    }
    print("\n" + "=" * 64)
    all_ok = all(results.values())
    for name, ok in results.items():
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if all_ok:
        print("\nPASS -- fc UART sender: well-formed dblink frames, age tracking, "
              "reset-counter edges, inflated-sigma floor, non-blocking latest-wins.")
        return 0
    print("\nFAIL -- see the [FAIL] lines above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
