#!/usr/bin/env python3
"""Mock-sensor selftest for the ``lidar`` project (no I2C hardware).

Exercises the read -> gate -> publish path WITHOUT a device, plus the reviewer-fix
guards, several ways:

  (a) GATE -- :func:`lidar.io.vl53l1x_reader.gate_reading` is the pure validity
      rule: ``valid iff (range_status & 0x1F) == 0x09 AND min_mm <= dist_mm <=
      max_mm`` (the masked RESULT__RANGE_STATUS code 9 = "range valid"; bits 7..5
      of the raw byte are unrelated flags the ST ULD strips first). Both reject
      paths (a non-OK masked status; an out-of-band distance) MUST yield valid=0.
  (b) CALIB STORE -- :mod:`lidar.io.lidar_calib_store` save->load round-trips the
      per-sensor crosstalk + offset; an absent/corrupt file or a sensor-id mismatch
      loads as ``None`` (the reader then runs uncalibrated) and NEVER raises.
  (c) MOCK READER -- :class:`MockRangeReader` returns a scripted sequence and
      produces :class:`RangeSample`s with range_m in METRES, 0.0 on a reject, and
      the range_status carried through. A masked-non-OK status sample is invalid.
  (d) PUBLISH -- ``lidar.main.run_lidar(mock=True)`` reads + publishes WireRange on
      a real IPC server; a client on the endpoint receives them and the
      valid/invalid readings round-trip (range_m metres, valid 0/1) exactly as the
      ``fc`` sender will consume them.
  (e) CALIB-STORE MAGNITUDE GUARDS (FIX 1) -- load() rejects the WHOLE entry (-> None,
      run uncalibrated) when ``offset_mm`` is out of +/-2000 mm or ``xtalk`` is not a
      uint16, even though the TYPE is a valid int; a sane entry still loads.
  (f) BENCH CAL GUARDS (FIX 2) -- ``calibrate_xtalk`` raises RuntimeError (NOT
      ZeroDivisionError) when the effective SPAD average is 0, and ValueError on
      ``target_mm <= 0``; ``calibrate_offset`` likewise on ``target_mm <= 0``.
  (g) READ FAIL-CLOSED (FIX 3) -- on a data-ready timeout ``read()`` returns an
      INVALID sample (valid=0, range_m 0), NEVER the previous frame's distance read
      from the (stale) result registers.

  .venv/bin/python -m lidar.tests.lidar_mock_selftest
"""
from __future__ import annotations

import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lidar.comms import IPCPubSub, topics                          # noqa: E402
from lidar.comms.wire import WireRange                             # noqa: E402
from lidar.io import lidar_calib_store                             # noqa: E402
from lidar.io.vl53l1x_reader import (                              # noqa: E402
    LIDAR_MAX_MM, LIDAR_MIN_MM, MockRangeReader, RANGE_STATUS_OK,
    RangeSample, VL53L1XReader, gate_reading,
)
from lidar.main import run_lidar                                   # noqa: E402


# --------------------------------------------------------------------------- #
# A hardware-free VL53L1XReader for the bench-cal-guard + read-fail-closed tests
# (FIX 2 / FIX 3). It bypasses __init__ (which would import smbus2 + probe a real
# chip) via object.__new__ and overrides ONLY the I2C primitives + _data_ready, so
# the REAL read()/calibrate_* logic runs against scripted register bytes -- no bus.
# --------------------------------------------------------------------------- #
class _FakeBusReader(VL53L1XReader):
    """A ``VL53L1XReader`` whose I2C is faked so the real flight/bench code paths run
    on the host. ``data_ready`` toggles whether :meth:`_data_ready` reports a fresh
    frame; ``reg_reads`` maps a 16-bit register -> the bytes a read of it returns;
    ``read_log`` records every register actually read (so a test can prove the stale
    result regs are NEVER touched on a fail-closed read)."""

    def __init__(self, *, data_ready: bool, reg_reads: dict | None = None,
                 sensor_id: str = "default", min_mm: int = LIDAR_MIN_MM,
                 max_mm: int = LIDAR_MAX_MM) -> None:
        # Deliberately DO NOT call super().__init__ (no smbus2, no hardware).
        self._a = 0x29
        self._sensor_id = str(sensor_id)
        self._min_mm = int(min_mm)
        self._max_mm = int(max_mm)
        self._data_ready_flag = bool(data_ready)
        self._reg_reads = dict(reg_reads or {})
        self.read_log: list[int] = []
        self.write_log: list[int] = []
        #: every write as (reg, data_bytes) -- so a test can assert the EXACT payload
        #: the reader packs into a calibration register (offset two's-complement etc.).
        self.writes: list[tuple[int, bytes]] = []

    def _data_ready(self) -> bool:
        return self._data_ready_flag

    def _rd(self, reg: int, n: int = 1) -> bytes:
        self.read_log.append(reg)
        return self._reg_reads.get(reg, b"\x00" * n)

    def _wr(self, reg: int, data: bytes) -> None:
        self.write_log.append(reg)
        self.writes.append((int(reg), bytes(data)))

    def _start_ranging(self) -> None:                # no-op (no bus)
        pass

    def _stop_ranging(self) -> None:                 # no-op (no bus)
        pass

    def _wait_data_ready(self, timeout_s: float = 1.0) -> None:
        # Bench loops: pretend a frame is always ready so the cal arithmetic runs.
        return None


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


def test_gate() -> bool:
    print("[a] gate_reading: (status & 0x1F)==0x09 + in-band -> valid; else reject")
    _check(gate_reading(842, RANGE_STATUS_OK), "0.842 m, status 0x09 -> VALID")
    # The MASK is the fix: a raw byte with high bits set (0x29 = bit5 | 0x09) must
    # still mask to the valid code 9 and pass -- this was the bulk of the lost frames.
    _check(gate_reading(842, 0x29),
           "raw 0x29 -> (0x29 & 0x1F)==9 -> VALID (mask strips bits 7..5)")
    # A different high-bit pattern (bit7|bit5 set) over the SAME low-5 code 9 -> VALID.
    _check(gate_reading(842, 0xA9),
           "raw 0xA9 -> (0xA9 & 0x1F)==9 -> VALID (bits 7..5 stripped)")
    # The complementary trap: a high bit set OVER low-5 code 8 must still REJECT (the
    # mask must not turn a clipped/8 frame valid just because high bits clear to 9-ish).
    _check(not gate_reading(842, 0x28),
           "raw 0x28 -> (0x28 & 0x1F)==8 (MIN_RANGE_CLIPPED) -> REJECT")
    _check(LIDAR_MIN_MM == 40, "LIDAR_MIN_MM is the 4cm datasheet floor (40)")
    _check(gate_reading(LIDAR_MIN_MM, RANGE_STATUS_OK), "exactly the min (40) -> VALID")
    _check(gate_reading(LIDAR_MAX_MM, RANGE_STATUS_OK), "exactly the max -> VALID")
    # A masked-non-OK code rejects even a sane distance.
    _check(not gate_reading(842, 6), "status 6 (sigma fail) -> REJECT")
    _check(not gate_reading(842, 4), "status 4 (signal fail) -> REJECT")
    # 8 == MIN_RANGE_CLIPPED must NOT pass as a valid height (only code 9 does) --
    # at BOTH a near-floor distance and a mid-range one, so the reject is driven by the
    # STATUS, not the distance band (a clipped reading can report any plausible mm).
    _check(not gate_reading(842, 8), "status 8 (MIN_RANGE_CLIPPED) -> REJECT")
    _check(not gate_reading(35, 8),
           "(dist=35, status=8) MIN_RANGE_CLIPPED -> REJECT (status, not band)")
    _check(not gate_reading(200, 8),
           "(dist=200, status=8) MIN_RANGE_CLIPPED -> REJECT (in-band but clipped)")
    # out-of-band distance rejects even with a valid status (explicit boundaries).
    _check(not gate_reading(39, RANGE_STATUS_OK), "39 mm (below the floor) -> REJECT")
    _check(gate_reading(40, RANGE_STATUS_OK), "40 mm (the floor) -> VALID")
    _check(gate_reading(4000, RANGE_STATUS_OK), "4000 mm (the ceiling) -> VALID")
    _check(not gate_reading(4001, RANGE_STATUS_OK), "4001 mm (above the ceiling) -> REJECT")
    _check(not gate_reading(0, RANGE_STATUS_OK), "0 mm (spurious zero) -> REJECT")
    return True


def test_calib_store() -> bool:
    print("[b] lidar_calib_store: save->load round-trip; corrupt/mismatch -> None")
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "lidar_calib.json"
        # save -> load round-trip (the raw uint16 xtalk + signed offset survive).
        lidar_calib_store.save("sensorA", xtalk=12345, offset_mm=-7,
                               distance_mode=1, min_mm=40,
                               timing_budget_us=50_000, n=50, path=p)
        got = lidar_calib_store.load("sensorA", path=p)
        _check(got is not None, "round-trip: load returns a dict after save")
        assert got is not None  # for type-narrowing below
        _check(got["xtalk"] == 12345 and got["offset_mm"] == -7,
               f"round-trip: xtalk/offset preserved (got {got['xtalk']}, "
               f"{got['offset_mm']})")
        _check(got["min_mm"] == 40 and got["timing_budget_us"] == 50_000,
               "round-trip: min_mm + timing_budget preserved")
        # an unknown sensor id in a valid file -> None (no entry, no raise).
        _check(lidar_calib_store.load("sensorB", path=p) is None,
               "sensor-id mismatch (no such entry) -> None")
        # a corrupt JSON file -> None (NEVER raises on the live path).
        corrupt = Path(d) / "corrupt.json"
        corrupt.write_text("{ this is not valid json ]")
        _check(lidar_calib_store.load("sensorA", path=corrupt) is None,
               "corrupt file -> None (no raise)")
        # an absent file -> None.
        _check(lidar_calib_store.load("sensorA", path=Path(d) / "nope.json") is None,
               "absent file -> None")
        # an entry missing a required int field -> None (invalid -> uncalibrated).
        bad = Path(d) / "bad.json"
        bad.write_text('{"sensorA": {"xtalk": 1, "offset_mm": 2}}')
        _check(lidar_calib_store.load("sensorA", path=bad) is None,
               "entry missing required fields -> None")
    return True


def test_calib_store_magnitude_guards() -> bool:
    print("[e] lidar_calib_store FIX1: magnitude/range guards reject corrupt entries")
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "lidar_calib.json"

        def _write_entry(sensor: str, **fields) -> None:
            # Write a single-entry file directly (bypass save()'s int() coercion) so
            # the load()-side guards are what's under test.
            import json
            base = {"xtalk": 100, "offset_mm": 0, "distance_mode": 1,
                    "min_mm": 40, "timing_budget_us": 50_000, "n": 50}
            base.update(fields)
            p.write_text(json.dumps({sensor: base}))

        # offset_mm magnitude: +100000 mm and -5000 mm are valid INTS but absurd for a
        # 4cm-4m sensor (and would overflow the int16 mm x 4 pack) -> reject -> None.
        _write_entry("s", offset_mm=100000)
        _check(lidar_calib_store.load("s", path=p) is None,
               "offset_mm=100000 (>2000) -> None (corrupt magnitude rejected)")
        _write_entry("s", offset_mm=-5000)
        _check(lidar_calib_store.load("s", path=p) is None,
               "offset_mm=-5000 (<-2000) -> None (corrupt magnitude rejected)")
        # xtalk range: 200000 is a valid int but not a uint16 -> reject -> None.
        _write_entry("s", xtalk=200000)
        _check(lidar_calib_store.load("s", path=p) is None,
               "xtalk=200000 (>0xFFFF) -> None (corrupt range rejected)")
        # xtalk negative: -1 is a valid int but below the uint16 floor -> reject -> None
        # (a signed xtalk could not have come from the bench solve, which clamps >=0).
        _write_entry("s", xtalk=-1)
        _check(lidar_calib_store.load("s", path=p) is None,
               "xtalk=-1 (<0) -> None (corrupt range rejected)")
        # Boundary: offset exactly at the bound + xtalk at the uint16 ceiling LOAD.
        _write_entry("s", offset_mm=2000, xtalk=0xFFFF)
        ok_edge = lidar_calib_store.load("s", path=p)
        _check(ok_edge is not None and ok_edge["offset_mm"] == 2000
               and ok_edge["xtalk"] == 0xFFFF,
               "offset_mm=2000 + xtalk=0xFFFF (on the bounds) -> still LOADS")
        # A plainly-sane entry still loads after all the rejections.
        _write_entry("s", offset_mm=-12, xtalk=345)
        ok = lidar_calib_store.load("s", path=p)
        _check(ok is not None and ok["offset_mm"] == -12 and ok["xtalk"] == 345,
               "a valid in-range entry still loads (guards don't over-reject)")
    return True


def test_apply_calibration() -> bool:
    print("[h] _apply_calibration: offset two's-complement pack, saturating xtalk, "
          "uncalibrated warn, sensor-id mismatch never applies another sensor's cal")
    import logging
    R_OFFSET = VL53L1XReader._R_PART_TO_PART_OFFSET
    R_XTALK = VL53L1XReader._R_XTALK_PLANE_OFFSET
    R_XTALK_X = VL53L1XReader._R_XTALK_X_PLANE
    R_XTALK_Y = VL53L1XReader._R_XTALK_Y_PLANE
    R_MM_IN = VL53L1XReader._R_MM_INNER_OFFSET
    R_MM_OUT = VL53L1XReader._R_MM_OUTER_OFFSET

    def _last_write(reader, reg):
        # The LAST payload written to ``reg`` ( _apply_calibration writes each once).
        hits = [data for r, data in reader.writes if r == reg]
        return hits[-1] if hits else None

    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "lidar_calib.json"

        # ---- (1) Offset packing: a NEGATIVE offset round-trips through the store and
        # is written as int16 (mm x 4) two's-complement. -20 mm -> -80 -> 0xFFB0.
        lidar_calib_store.save("A", xtalk=345, offset_mm=-20, distance_mode=1,
                               min_mm=40, timing_budget_us=50_000, n=50, path=p)
        rdr = _FakeBusReader(data_ready=True, sensor_id="A")
        # Point the reader's store load at the temp file (load() takes an optional path).
        import lidar.io.vl53l1x_reader as vr
        orig_load = vr.lidar_calib_store.load
        vr.lidar_calib_store.load = lambda sid, path=p: orig_load(sid, path=p)
        try:
            rdr._apply_calibration()
        finally:
            vr.lidar_calib_store.load = orig_load
        _check(_last_write(rdr, R_OFFSET) == b"\xff\xb0",
               "offset -20mm -> int16 mm*4 two's-complement = ff b0")
        _check(_last_write(rdr, R_MM_IN) == b"\x00\x00"
               and _last_write(rdr, R_MM_OUT) == b"\x00\x00",
               "inner/outer MM offsets zeroed alongside the part-to-part offset")
        # xtalk 345 -> (345<<9)//1000 = 176 -> 0x00B0; X/Y plane gradients zeroed.
        _check(_last_write(rdr, R_XTALK) == b"\x00\xb0",
               "xtalk_raw=345 -> (raw<<9)//1000=176 -> plane offset 00 b0")
        _check(_last_write(rdr, R_XTALK_X) == b"\x00\x00"
               and _last_write(rdr, R_XTALK_Y) == b"\x00\x00",
               "xtalk X/Y plane gradients zeroed")

        # ---- (2) Saturating xtalk apply (defense-in-depth, FIX4): a raw value whose
        # <<9 scale EXCEEDS 0xFFFF must CLAMP to ff ff, never wrap to a small kcps. The
        # store's load() guard would reject raw>0xFFFF, so inject the cal dict directly
        # to prove the reader's own min(.,0xFFFF) saturates.
        sat = _FakeBusReader(data_ready=True, sensor_id="A")
        vr.lidar_calib_store.load = lambda sid, path=None: {
            "offset_mm": 0, "xtalk": 200000, "distance_mode": 1,
            "min_mm": 40, "timing_budget_us": 50_000, "n": 50,
        }
        try:
            sat._apply_calibration()
        finally:
            vr.lidar_calib_store.load = orig_load
        _check(_last_write(sat, R_XTALK) == b"\xff\xff",
               "xtalk scale overflow ((200000<<9)//1000=102400) SATURATES to ff ff "
               "(not wrap to a wrong small kcps)")

        # ---- (3) Uncalibrated path: NO entry for this sensor -> _apply_calibration
        # writes NOTHING to the cal registers AND logs exactly ONE loud WARNING (so
        # ranging still proceeds honest+uncalibrated, never refused).
        empty = _FakeBusReader(data_ready=True, sensor_id="ghost")
        vr.lidar_calib_store.load = lambda sid, path=None: None
        logger = logging.getLogger("lidar.io.vl53l1x")
        warnings: list[str] = []

        class _Capture(logging.Handler):
            def emit(self, rec):
                if rec.levelno >= logging.WARNING:
                    warnings.append(rec.getMessage())
        h = _Capture()
        logger.addHandler(h)
        try:
            empty._apply_calibration()
        finally:
            logger.removeHandler(h)
            vr.lidar_calib_store.load = orig_load
        _check(_last_write(empty, R_OFFSET) is None
               and _last_write(empty, R_XTALK) is None,
               "no cal on disk -> NO write to offset/xtalk regs (runs uncalibrated)")
        _check(len(warnings) == 1 and "UNCALIBRATED" in warnings[0],
               f"no cal on disk -> exactly ONE loud UNCALIBRATED warning (got "
               f"{len(warnings)})")

        # ---- (4) Sensor-id mismatch: store holds ONLY "A"; a reader for "B" must load
        # None (no entry) and apply NOTHING -- B must NEVER inherit A's offset/xtalk.
        # (Store already has "A" from step 1.) Use the REAL load (path=p), sensor "B".
        mism = _FakeBusReader(data_ready=True, sensor_id="B")
        vr.lidar_calib_store.load = lambda sid, path=p: orig_load(sid, path=p)
        try:
            mism._apply_calibration()
        finally:
            vr.lidar_calib_store.load = orig_load
        _check(_last_write(mism, R_OFFSET) is None
               and _last_write(mism, R_XTALK) is None,
               "store has 'A', reader is 'B' -> NO cal applied (never inherits A's)")
        # And prove "A" really is in the store (so the mismatch test isn't a no-op
        # because the file was empty): re-loading "A" returns A's offset.
        a_cal = orig_load("A", path=p)
        _check(a_cal is not None and a_cal["offset_mm"] == -20,
               "control: 'A' IS in the store (mismatch test exercised a real entry)")
    return True


def test_bench_cal_guards() -> bool:
    print("[f] calibrate_* FIX2: zero-SPAD -> RuntimeError; target_mm<=0 -> ValueError")
    # calibrate_xtalk with EVERY SpadNb read == 0 -> avg_spad == 0. The guard must
    # raise RuntimeError BEFORE the divide (NOT a bare ZeroDivisionError, and NOT a
    # silent 0 that would persist as a bogus "calibrated").
    dark = _FakeBusReader(data_ready=True, reg_reads={
        VL53L1XReader._R_SIGNAL_RATE: b"\x00\x10",   # some signal
        VL53L1XReader._R_RANGE_MM: b"\x02\x58",      # 600 mm
        VL53L1XReader._R_SPAD_NB: b"\x00\x00",       # >>8 == 0 -> avg_spad == 0
    })
    raised = None
    try:
        dark.calibrate_xtalk(600, n=4)
    except RuntimeError as e:
        raised = e
    except ZeroDivisionError as e:                   # explicitly the WRONG outcome
        _check(False, f"calibrate_xtalk leaked ZeroDivisionError ({e})")
    _check(raised is not None and "SPAD" in str(raised),
           "calibrate_xtalk zero-SPAD -> RuntimeError (not ZeroDivisionError)")

    # target_mm <= 0 on BOTH bench routines -> ValueError at entry (guards the
    # avg_dist/target + round(target-mean) paths). Use a reader that would otherwise
    # range fine; the guard must trip before any divide.
    good = _FakeBusReader(data_ready=True, reg_reads={
        VL53L1XReader._R_RANGE_MM: b"\x02\x58", VL53L1XReader._R_SIGNAL_RATE: b"\x00\x10",
        VL53L1XReader._R_SPAD_NB: b"\x0A\x00",       # >>8 == 10
    })
    for name, fn in (("calibrate_xtalk", good.calibrate_xtalk),
                     ("calibrate_offset", good.calibrate_offset)):
        ve = None
        try:
            fn(0, n=4)
        except ValueError as e:
            ve = e
        _check(ve is not None, f"{name}(target_mm=0) -> ValueError")
    # n < 1 likewise -> ValueError on BOTH bench routines (guards the /n divide).
    for name, fn in (("calibrate_offset", good.calibrate_offset),
                     ("calibrate_xtalk", good.calibrate_xtalk)):
        ne = None
        try:
            fn(140, n=0)
        except ValueError as e:
            ne = e
        _check(ne is not None, f"{name}(n=0) -> ValueError (guards /n)")
    return True


def test_read_fail_closed_on_timeout() -> bool:
    print("[g] read() FIX3: data-ready timeout -> INVALID sample, NOT a stale frame")
    # The result registers are SCRIPTED to a plausible prior frame (842 mm, status 9).
    # If read() fell through to read them on a timeout it would return THAT stale,
    # "valid"-looking distance -- the exact bug. With _data_ready always False, read()
    # must instead fail closed (valid=0, range_m 0, dist_mm 0) and never touch them.
    stale = _FakeBusReader(data_ready=False, reg_reads={
        VL53L1XReader._R_RANGE_STATUS: bytes([RANGE_STATUS_OK]),
        VL53L1XReader._R_RANGE_MM: b"\x03\x4A",      # 842 mm (the would-be stale frame)
    })
    # Shrink the poll budget so the test doesn't sit for 0.12 s.
    stale._READ_READY_TIMEOUT_S = 0.01
    s = stale.read()
    _check(isinstance(s, RangeSample), "read() returned a RangeSample (never raised)")
    _check(not s.valid and s.range_m == 0.0 and s.dist_mm == 0,
           "timeout -> valid=0, range_m=0.0, dist_mm=0 (fail closed)")
    _check(s.range_status == -1, "timeout sample carries the sentinel status -1")
    _check(s.dist_mm != 842 and s.range_m != 0.842,
           "the stale 842 mm prior frame is NOT returned")
    _check(VL53L1XReader._R_RANGE_MM not in stale.read_log
           and VL53L1XReader._R_RANGE_STATUS not in stale.read_log,
           "the result registers were NEVER read on the fail-closed path")

    # Control: with data ready, the SAME reader DOES read the result regs and gates
    # the (now legitimately fresh) frame valid -- proving the guard, not a dead path.
    fresh = _FakeBusReader(data_ready=True, reg_reads={
        VL53L1XReader._R_RANGE_STATUS: bytes([RANGE_STATUS_OK]),
        VL53L1XReader._R_RANGE_MM: b"\x03\x4A",      # 842 mm
    })
    sf = fresh.read()
    _check(sf.valid and abs(sf.range_m - 0.842) < 1e-9,
           "data ready -> result regs read, fresh 842 mm gated VALID (0.842 m)")
    return True


def test_mock_reader() -> bool:
    print("[c] MockRangeReader: scripted read -> gated RangeSample (mm -> m)")
    reader = MockRangeReader(script=[
        (842, RANGE_STATUS_OK),    # valid -> 0.842 m
        (840, 4),                  # status fail -> reject
        (5000, RANGE_STATUS_OK),   # out of band -> reject
    ])
    s0 = reader.read()
    _check(s0.valid and abs(s0.range_m - 0.842) < 1e-9,
           f"reading 0: VALID, range_m={s0.range_m:.3f} m (mm/1000)")
    _check(s0.range_status == RANGE_STATUS_OK, "reading 0: status carried (0x09)")
    s1 = reader.read()
    _check(not s1.valid and s1.range_m == 0.0,
           "reading 1: range_status != 0x09 -> INVALID, range_m forced 0.0")
    _check(s1.range_status == 4, "reading 1: the failing status is carried (4)")
    s2 = reader.read()
    _check(not s2.valid and s2.range_m == 0.0,
           "reading 2: out-of-band distance -> INVALID, range_m 0.0")
    # The script cycles, so the 4th read is the valid one again (deterministic).
    s3 = reader.read()
    _check(s3.valid and abs(s3.range_m - 0.842) < 1e-9,
           "reading 3: script cycles back to the valid reading")
    reader.close()
    return True


def test_publish_roundtrip() -> bool:
    print("[d] run_lidar(mock) publishes WireRange; a client receives valid+invalid")
    endpoint = "oak.lidar.test_mock"
    received: list[WireRange] = []
    got_enough = threading.Event()

    def _on_range(wm) -> None:
        if isinstance(wm, WireRange):
            received.append(wm)
            if len(received) >= 5:
                got_enough.set()

    # Run the lidar producer in a thread, INJECTING a reader with a KNOWN valid+invalid
    # mix (the live --lidar-mock default is now an all-valid smooth sweep, so we inject
    # a reject-bearing script here to exercise the publish path for BOTH). A MODERATE
    # rate + a generous max_reads keeps the server up for ~3 s so the client has time to
    # connect. This drives the REAL run_lidar publish path, not a hand-rolled server.
    rc_box: list[int] = []
    mix = [(300, RANGE_STATUS_OK), (600, RANGE_STATUS_OK), (5000, RANGE_STATUS_OK),
           (900, RANGE_STATUS_OK), (840, 4), (450, RANGE_STATUS_OK)]  # 4 valid, 2 reject

    def _run() -> None:
        rc_box.append(run_lidar(endpoint=endpoint, rate_hz=30.0,
                                reader=MockRangeReader(script=mix),
                                max_reads=90))     # ~3 s at 30 Hz

    producer = threading.Thread(target=_run, name="lidar.producer", daemon=True)
    producer.start()
    # Give the server a beat to bind, then connect the client (it also retries).
    time.sleep(0.3)
    client = IPCPubSub(endpoint, role="client", connect_timeout_s=10.0)
    client.subscribe(topics.LIDAR_RANGE, _on_range)
    client.start()

    got_enough.wait(timeout=5.0)
    producer.join(timeout=8.0)
    client.stop()

    _check(rc_box and rc_box[0] == 0, "run_lidar(mock) returned 0")
    _check(len(received) >= 3,
           f"client received WireRange messages (got {len(received)})")
    # The injected reader yields a mix of valid + invalid; both must arrive and
    # round-trip the contract the fc sender reads (range_m metres, valid 0/1).
    valids = [m for m in received if m.valid == 1]
    invalids = [m for m in received if m.valid == 0]
    _check(len(valids) >= 1, f"at least one VALID reading arrived (n={len(valids)})")
    _check(len(invalids) >= 1,
           f"at least one INVALID reading arrived (n={len(invalids)})")
    _check(all(0.0 < m.range_m < 5.0 for m in valids),
           "valid readings carry a plausible metres range")
    _check(all(m.range_m == 0.0 for m in invalids),
           "invalid readings carry range_m == 0.0 (gated)")
    # seq is monotone (drop detection) + ts_ns is set.
    seqs = [m.seq for m in received]
    _check(seqs == sorted(seqs), "seq is monotone non-decreasing")
    _check(all(m.ts_ns > 0 for m in received), "every reading carries a ts_ns")
    return True


def main() -> int:
    print("lidar_mock_selftest -- mock-sensor read -> gate -> publish (no I2C)")
    results = {
        "gate":                       test_gate(),
        "calib store":                test_calib_store(),
        "calib store magnitude (F1)": test_calib_store_magnitude_guards(),
        "apply calibration (F4/id)":  test_apply_calibration(),
        "bench cal guards (F2)":      test_bench_cal_guards(),
        "read fail-closed (F3)":      test_read_fail_closed_on_timeout(),
        "mock reader":                test_mock_reader(),
        "publish round-trip":         test_publish_roundtrip(),
    }
    print("\n" + "=" * 64)
    all_ok = all(results.values())
    for name, ok in results.items():
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if all_ok:
        print("\nPASS -- lidar mock: gate (incl. range_status != 0x09 -> valid=0), "
              "mm->m conversion, and run_lidar publishes WireRange round-tripping "
              "the lidar.range contract.")
        return 0
    print("\nFAIL -- see the [FAIL] lines above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
