"""VL53L1X downward-rangefinder I2C reader (real device + a host MOCK).

The flight Pi carries a downward-facing VL53L1X time-of-flight rangefinder (a
bare VL53L1X breakout) read over I2C via a pure-``smbus2`` register-level driver.
This module reads distance + ``range_status`` over I2C and gates each reading
into a (range_m, valid) pair the ``lidar`` process publishes on ``lidar.range``
and the ``fc`` sender bundles into the dblink VIO-pose frame.

SWAPPABLE device interface
--------------------------
The reader is behind a tiny :class:`RangeReader` interface with two
implementations:

* :class:`VL53L1XReader` -- the real reader: a bare VL53L1X at 0x29 driven with
  ``smbus2`` ONLY (register-level), imported LAZILY (only the flight Pi installs
  it; a dev host / CI never does). Init writes the 91-byte ST/Adafruit default
  config block (which boots LONG), applies the requested distance mode + timing
  (SHORT @ 50 ms by default), then -- if a per-sensor calibration is on disk --
  applies the stored crosstalk + offset before starting continuous ranging. Any
  I2C / device error -> a ``valid=0`` sample, NEVER an exception: a flaky sensor
  must not crash the flight process.
* :class:`MockRangeReader` -- a deterministic, hardware-free reader for host
  tests: it returns a scripted sequence of ``(dist_mm, range_status)`` so the
  read -> gate -> publish path (including the ``range_status != RANGE_STATUS_OK``
  reject) is exercised with no I2C bus.

The GATE (:func:`gate_reading`) is a pure function so it is unit-testable in
isolation: ``valid = ((range_status & 0x1F) == RANGE_STATUS_OK) and (min_mm <=
dist_mm <= max_mm)``. The chip's RESULT__RANGE_STATUS (reg 0x0089) carries the
device range-status code in bits 4..0; bits 7..5 are unrelated flags that the ST
ULD masks off (``& 0x1F``) before comparing -- without the mask, perfectly valid
frames are rejected nondeterministically whenever a high bit happens to be set.
``RANGE_STATUS_OK == 9`` is the device code for a completed range; only 9 passes
(8 == MIN_RANGE_CLIPPED, a near-floor clipped reading, must NOT pass as a valid
height). Any other masked code (sigma fail, signal fail, wrap-around, ...) rejects.

Units: the chip reports MILLIMETRES; the published / wire value is METRES.
"""
from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Protocol

from lidar.io import lidar_calib_store

LOG = logging.getLogger("lidar.io.vl53l1x")

# --------------------------------------------------------------------------- #
# Default I2C wiring + gate bounds. The bare VL53L1X answers at its default
# address 0x29 (verified on-device) -- both are overridable on the reader.
# --------------------------------------------------------------------------- #
#: VL53L1X default 7-bit I2C address (0x29) -- the bare breakout's factory
#: address, verified on-device. Pass ``i2c_address`` to override if re-strapped.
DEFAULT_I2C_ADDRESS = 0x29
#: Default Linux I2C bus number on the Pi (``/dev/i2c-1`` on the 40-pin header).
DEFAULT_I2C_BUS = 1

#: VL53L1X distance-mode codes (ST convention). SHORT = up to ~1.3 m, best
#: ambient-light immunity + tightest near-floor accuracy; LONG = up to ~4 m. The
#: downward height reference runs SHORT @ 50 ms (set by
#: :meth:`VL53L1XReader._init_sensor` via the vcsel/phase + macro-period registers);
#: ``distance_mode`` + ``timing_budget_us`` are now honored (not just accepted).
DIST_MODE_SHORT = 1
DIST_MODE_MEDIUM = 2
DIST_MODE_LONG = 3

#: Inter-measurement timing budget (microseconds). 50 ms is the bench-proven budget
#: (SHORT @ 50 ms); 33 ms is also supported. The value selects the SHORT macro-period
#: pair in :meth:`VL53L1XReader._init_sensor` -- it must match a supported entry.
DEFAULT_TIMING_BUDGET_US = 50_000

#: Gate bounds (MILLIMETRES). A reading outside this band is rejected (valid=0)
#: even if the range-status is OK: below the floor is below the sensor's optical
#: limit, above the ceiling is beyond trusted range. The FC owns the flight-relevant
#: ground/disarm thresholds; this is only the sensor sanity band. These are the GATE
#: DEFAULTS -- the reader carries its own instance ``min_mm``/``max_mm`` (so SIL /
#: MockRangeReader are unaffected by any per-reader tuning); do NOT mutate them.
#: 4 cm = the VL53L1X minimum ranging distance (ST datasheet): below it the sensor
#: still detects but is inaccurate / admits crosstalk garbage, so it is gated out.
LIDAR_MIN_MM = 40
LIDAR_MAX_MM = 4000

#: VL53L1X RESULT__RANGE_STATUS device code for a completed/valid range. This is the
#: value AFTER masking the raw reg-0x0089 byte with ``& 0x1F`` (bits 7..5 are
#: unrelated flags the ST ULD strips first). A good measurement masks to 9; any other
#: masked code is a reject (8 == MIN_RANGE_CLIPPED is deliberately NOT accepted).
RANGE_STATUS_OK = 0x09
#: Mask applied to the raw RESULT__RANGE_STATUS byte before the validity compare
#: (ST ULD: ``status & 0x1F``). Without it, bits 7..5 reject valid frames at random.
RANGE_STATUS_MASK = 0x1F


@dataclass(frozen=True)
class RangeSample:
    """One gated rangefinder reading.

    ``range_m`` is METRES and is meaningful ONLY when ``valid`` is True; on a
    rejected reading it is 0.0. ``range_status`` / ``signal`` / ``dist_mm`` are the
    raw sensor fields, kept for the ``--characterize`` tool + logs (they do NOT go
    on the wire). ``range_status`` is the chip status code; ``signal`` is the return
    signal rate (Mcps) when the driver exposes it, else ``None``.
    """

    range_m: float
    valid: bool
    dist_mm: int
    range_status: int
    signal: float | None = None


def gate_reading(dist_mm: int, range_status: int, *,
                 min_mm: int = LIDAR_MIN_MM, max_mm: int = LIDAR_MAX_MM) -> bool:
    """Sensor-side validity gate (pure -> unit-testable).

    ``valid`` iff the chip's MASKED range-status code is the "range valid" code AND
    the distance is inside the sane band ``[min_mm, max_mm]``. ``range_status`` is
    the RAW reg-0x0089 byte: bits 7..5 are unrelated flags the ST ULD strips, so we
    compare ``(range_status & RANGE_STATUS_MASK) == RANGE_STATUS_OK`` (mask = 0x1F).
    Without the mask a high bit rejects an otherwise-valid frame at random -- this
    was the bulk of the lost readings. Only device code 9 passes; any other masked
    code (sigma/signal fail, wrap-around, 8 == MIN_RANGE_CLIPPED, ...) or an
    out-of-band distance rejects, and the caller forces the published ``range_m`` to
    0.0. ``min_mm``/``max_mm`` default to the module gate bounds; the real reader
    passes its own instance bounds (so per-reader tuning never touches a global).
    """
    return ((range_status & RANGE_STATUS_MASK) == RANGE_STATUS_OK
            and min_mm <= dist_mm <= max_mm)


class RangeReader(Protocol):
    """The swappable rangefinder interface the ``lidar`` process depends on.

    Two methods: :meth:`read` returns one gated :class:`RangeSample` (and must
    NEVER raise -- an I2C error becomes an invalid sample), and :meth:`close`
    releases the device. Both the real I2C reader and the host mock implement it.
    """

    def read(self) -> RangeSample:
        ...

    def close(self) -> None:
        ...


# --------------------------------------------------------------------------- #
# Real I2C reader -- a BARE VL53L1X at 0x29 driven with smbus2 ONLY (no Blinka /
# lgpio / RPi.GPIO; those don't pip-build cleanly on the Pi5 / py3.13). The
# register-level init + read are the ST/Adafruit-verified VL53L1X sequence,
# confirmed on-device (model id EA CC 10, status 0x09, live distance).
# --------------------------------------------------------------------------- #
class VL53L1XReader:
    """Real VL53L1X reader over I2C using ONLY ``smbus2``.

    Talks to a bare VL53L1X breakout at its default 7-bit address 0x29. Init writes
    the 91-byte ST/Adafruit default config block to reg 0x002D (which boots LONG),
    applies the requested distance mode + timing (SHORT @ 50 ms by default), then --
    if ``.cache/lidar_calib.json`` holds a valid entry for this ``sensor_id`` --
    applies the stored crosstalk + offset, and finally starts continuous ranging.
    ``smbus2`` is imported INSIDE ``__init__`` so importing this module on a dev host
    / CI costs nothing and the ``cv2-free`` flight image stays clean.

    ``__init__`` raises if the sensor is absent / not a VL53L1X (the caller --
    ``lidar.main`` -- decides whether that is fatal). Once ranging, a per-read I2C
    error is swallowed into a ``valid=0`` sample so a flaky sensor never crashes the
    flight loop. ``distance_mode`` + ``timing_budget_us`` are HONORED (they select the
    register writes in :meth:`_init_sensor`); a missing/corrupt calibration logs a
    loud warning and runs UNCALIBRATED (an honest valid reading is safe -- the FC
    gates on ``valid`` -- so we never refuse to range).
    """

    # VL53L1X registers (16-bit addresses, big-endian on the wire).
    _R_MODEL_ID = 0x010F
    # Offset / crosstalk calibration result registers (applied from the cache).
    _R_XTALK_PLANE_OFFSET = 0x0016   # ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS
    _R_XTALK_X_PLANE = 0x0018        # ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT
    _R_XTALK_Y_PLANE = 0x001A        # ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT
    _R_PART_TO_PART_OFFSET = 0x001E  # ALGO__PART_TO_PART_RANGE_OFFSET_MM (mm x 4)
    _R_MM_INNER_OFFSET = 0x0020      # MM_CONFIG__INNER_OFFSET_MM
    _R_MM_OUTER_OFFSET = 0x0022      # MM_CONFIG__OUTER_OFFSET_MM
    _R_GPIO_HV_MUX_CTRL = 0x0030
    _R_GPIO_TIO_HV_STATUS = 0x0031
    _R_TB_MACROP_A_HI = 0x005E
    _R_TB_MACROP_B_HI = 0x0061
    _R_INTERRUPT_CLEAR = 0x0086
    _R_MODE_START = 0x0087
    _R_RANGE_STATUS = 0x0089
    _R_SPAD_NB = 0x008C              # RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0
    _R_RANGE_MM = 0x0096
    _R_SIGNAL_RATE = 0x0098          # RESULT__PEAK_SIGNAL_COUNT_RATE_XTALK_CORR_SD0

    # ST/Adafruit VL53L1X default configuration block (regs 0x002D..0x0087).
    _INIT_SEQ = bytes([
        0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,
        0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,
        0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
        0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,
        0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
        0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
        0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x00,
    ])

    # SHORT distance-mode register block (1 byte each) applied AFTER the default
    # config block (which boots LONG): vcsel period + phase / valid-phase-high
    # tuning that the ST ULD writes for SHORT. Same for every SHORT timing budget.
    # 0x004B = PHASECAL_CONFIG__TIMEOUT_MACROP: the ULD SetDistanceMode(SHORT) also
    # writes this (= 0x14); omitting it left the SHORT sequence ULD-incomplete.
    # Bench-verify alongside the SHORT macro-period constants in ``_MACROP``.
    _SHORT_MODE_WRITES = (
        (0x004B, 0x14),
        (0x0060, 0x07), (0x0063, 0x05), (0x0069, 0x38),
        (0x0078, 0x07), (0x0079, 0x05), (0x007A, 0x06), (0x007B, 0x06),
    )

    # Timing macro-period pair (RANGE_CONFIG__TIMEOUT_MACROP_A/B_HI, both 16-bit BE)
    # per (distance_mode, timing_budget_us). The LONG values are wrong for SHORT and
    # vice-versa, so they MUST be set to match the mode. SHORT constants are bench-
    # verified by reading back the measured cadence (researcher MEDIUM confidence).
    _MACROP: dict = {
        (DIST_MODE_SHORT, 50_000): (0x01AE, 0x01E8),
        (DIST_MODE_SHORT, 33_000): (0x00D6, 0x006E),
        (DIST_MODE_LONG, 50_000): (0x00AD, 0x00C6),
    }

    def __init__(self, *, i2c_bus: int = DEFAULT_I2C_BUS,
                 i2c_address: int = DEFAULT_I2C_ADDRESS,
                 distance_mode: int = DIST_MODE_SHORT,
                 timing_budget_us: int = DEFAULT_TIMING_BUDGET_US,
                 sensor_id: str = "default",
                 min_mm: int = LIDAR_MIN_MM,
                 max_mm: int = LIDAR_MAX_MM) -> None:
        if (int(distance_mode), int(timing_budget_us)) not in self._MACROP:
            raise ValueError(
                "unsupported (distance_mode=%r, timing_budget_us=%r); supported: %s"
                % (distance_mode, timing_budget_us, sorted(self._MACROP)))
        import smbus2  # lazy: only the flight Pi installs it
        self._smbus2 = smbus2
        self._a = int(i2c_address)
        self._distance_mode = int(distance_mode)
        self._timing_budget_us = int(timing_budget_us)
        self._sensor_id = str(sensor_id)
        self._min_mm = int(min_mm)
        self._max_mm = int(max_mm)
        self._bus = smbus2.SMBus(int(i2c_bus))
        mid = self._rd(self._R_MODEL_ID, 3)
        if mid != b"\xEA\xCC\x10":
            self.close()
            raise RuntimeError(
                "no VL53L1X at 0x%02X (model id %s, expected EA CC 10)"
                % (self._a, mid.hex(" ")))
        self._init_sensor()
        LOG.info("lidar: VL53L1X (smbus2) ranging on i2c bus=%d addr=0x%02X "
                 "(mode=%d, %dus, sensor_id=%r)", int(i2c_bus), self._a,
                 self._distance_mode, self._timing_budget_us, self._sensor_id)

    def _wr(self, reg: int, data: bytes) -> None:
        msg = self._smbus2.i2c_msg.write(
            self._a, [(reg >> 8) & 0xFF, reg & 0xFF] + list(data))
        self._bus.i2c_rdwr(msg)

    def _rd(self, reg: int, n: int = 1) -> bytes:
        w = self._smbus2.i2c_msg.write(self._a, [(reg >> 8) & 0xFF, reg & 0xFF])
        r = self._smbus2.i2c_msg.read(self._a, n)
        self._bus.i2c_rdwr(w, r)
        return bytes(r)

    def _interrupt_polarity(self) -> int:
        return 0 if ((self._rd(self._R_GPIO_HV_MUX_CTRL)[0] >> 4) & 0x01) else 1

    def _data_ready(self) -> bool:
        return (self._rd(self._R_GPIO_TIO_HV_STATUS)[0] & 0x01) \
            == self._interrupt_polarity()

    def _start_ranging(self) -> None:
        """Begin continuous ranging (MODE_START = 0x40)."""
        self._wr(self._R_MODE_START, b"\x40")

    def _stop_ranging(self) -> None:
        """Stop ranging (MODE_START = 0x00)."""
        self._wr(self._R_MODE_START, b"\x00")

    def _apply_mode_timing(self) -> None:
        """Apply distance mode + timing macro-periods over the default (LONG) block.

        The 91-byte default block boots the sensor in LONG mode; for SHORT we write
        the vcsel/phase tuning block first. THEN -- mandatory for any mode -- we set
        the timing macro-period pair (the LONG defaults are invalid in SHORT and
        vice-versa). Both macro-period values are 16-bit big-endian.
        """
        if self._distance_mode == DIST_MODE_SHORT:
            for reg, val in self._SHORT_MODE_WRITES:
                self._wr(reg, bytes([val]))
        macrop_a, macrop_b = self._MACROP[(self._distance_mode,
                                           self._timing_budget_us)]
        self._wr(self._R_TB_MACROP_A_HI, struct.pack(">H", macrop_a))
        self._wr(self._R_TB_MACROP_B_HI, struct.pack(">H", macrop_b))

    def _apply_calibration(self) -> None:
        """Load + apply this sensor's stored offset + crosstalk (if any).

        A missing / corrupt / sensor-id-mismatched entry logs LOUD and runs
        UNCALIBRATED -- an honest valid reading is safe (the FC gates on ``valid``),
        so we never refuse to range. When present, the offset is written first (ST
        order), then the crosstalk; the stored ``xtalk`` is the RAW uint16 the ULD
        CalibrateXtalk formula returns and is scaled to the plane-offset register
        exactly as the ULD SetXtalk does (``(xtalk_raw << 9) // 1000``).
        """
        cal = lidar_calib_store.load(self._sensor_id)
        if cal is None:
            LOG.warning("lidar: NO calibration for sensor_id=%r (.cache/"
                        "lidar_calib.json) -- running UNCALIBRATED (honest valid "
                        "readings; FC gates on valid). Run characterize --calibrate "
                        "to remove near-range bias + crosstalk.", self._sensor_id)
            return
        offset_mm = int(cal["offset_mm"])
        xtalk_raw = int(cal["xtalk"])
        # Offset: ALGO__PART_TO_PART_RANGE_OFFSET_MM is signed mm x 4 (16-bit BE);
        # zero the inner/outer MM offsets the ULD pairs with it.
        self._wr(self._R_PART_TO_PART_OFFSET,
                 struct.pack(">h", offset_mm * 4))
        self._wr(self._R_MM_INNER_OFFSET, b"\x00\x00")
        self._wr(self._R_MM_OUTER_OFFSET, b"\x00\x00")
        # Crosstalk: zero the X/Y plane gradients, then the plane offset (kcps),
        # scaled from the raw value EXACTLY as the ULD SetXtalk (truncating //1000, no
        # +500). SATURATE to uint16 (not wrap with & 0xFFFF): the stored xtalk is
        # already bounded by lidar_calib_store.load, but defense-in-depth -- a large
        # value must clamp to the max kcps, never wrap to a wrong small kcps.
        self._wr(self._R_XTALK_X_PLANE, b"\x00\x00")
        self._wr(self._R_XTALK_Y_PLANE, b"\x00\x00")
        self._wr(self._R_XTALK_PLANE_OFFSET,
                 struct.pack(">H", min((xtalk_raw << 9) // 1000, 0xFFFF)))
        LOG.info("lidar: applied calibration sensor_id=%r (offset=%dmm, xtalk_raw=%d)",
                 self._sensor_id, offset_mm, xtalk_raw)

    def _init_sensor(self) -> None:
        self._wr(0x002D, self._INIT_SEQ)
        self._start_ranging()
        t0 = time.monotonic()
        while not self._data_ready():
            if time.monotonic() - t0 > 0.5:
                break
            time.sleep(0.005)
        self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
        self._stop_ranging()
        self._wr(0x0008, b"\x09")
        self._wr(0x000B, b"\x00")
        self._apply_mode_timing()        # honor distance_mode + timing_budget_us
        self._apply_calibration()        # offset + crosstalk from .cache (if any)
        self._start_ranging()            # continuous

    #: Per-read data-ready poll budget (seconds). One SHORT-@50ms frame is ~20-50 ms;
    #: 0.12 s is a generous ceiling. On expiry the read FAILS CLOSED (invalid sample)
    #: rather than returning a stale frame.
    _READ_READY_TIMEOUT_S = 0.12

    def _poll_data_ready(self, timeout_s: float) -> bool:
        """Bounded poll for a fresh measurement; return True if ready, else False.

        Best-effort (flight path): on expiry it returns False so :meth:`read` can FAIL
        CLOSED. (The bench loops use :meth:`_wait_data_ready`, which raises instead.)
        """
        t0 = time.monotonic()
        while not self._data_ready():
            if time.monotonic() - t0 > timeout_s:
                return False
            time.sleep(0.003)
        return True

    @staticmethod
    def _invalid_sample() -> RangeSample:
        """The fail-closed sample (I2C error OR data-ready timeout): valid=0, range 0."""
        return RangeSample(range_m=0.0, valid=False, dist_mm=0,
                           range_status=-1, signal=None)

    def read(self) -> RangeSample:
        """One gated reading -- NEVER raises (I2C error -> invalid sample).

        FAIL CLOSED on a data-ready timeout: if no fresh frame arrives within
        ``_READ_READY_TIMEOUT_S`` we return an INVALID sample instead of reading the
        result registers, which would otherwise return the PREVIOUS frame's distance
        with a stale-but-OK status -- a stuck-but-"valid" height the FC gate cannot
        catch. An invalid sample (range_m 0, valid=0) IS catchable.
        """
        try:
            if not self._poll_data_ready(self._READ_READY_TIMEOUT_S):
                # No fresh frame -> do NOT read stale result regs; fail closed.
                LOG.debug("lidar: VL53L1X data-ready timeout -> invalid sample")
                return self._invalid_sample()
            status = self._rd(self._R_RANGE_STATUS)[0]
            dist_mm = struct.unpack(">H", self._rd(self._R_RANGE_MM, 2))[0]
            self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
        except Exception as e:                                     # noqa: BLE001
            LOG.debug("lidar: VL53L1X read error (%s) -> invalid sample", e)
            return self._invalid_sample()
        # Pass the RAW status byte (the gate masks & 0x1F) + this reader's bounds.
        valid = gate_reading(dist_mm, status,
                             min_mm=self._min_mm, max_mm=self._max_mm)
        return RangeSample(range_m=(dist_mm * 1e-3) if valid else 0.0,
                           valid=valid, dist_mm=dist_mm,
                           range_status=status, signal=None)

    def _wait_data_ready(self, timeout_s: float = 1.0) -> None:
        """Block until a fresh measurement is ready (bounded; raises on timeout).

        Used only by the BENCH calibration loops (never the flight read path, which
        is best-effort + never raises). A timeout here is a real fault the operator
        must see, so it propagates rather than silently averaging stale frames.
        """
        t0 = time.monotonic()
        while not self._data_ready():
            if time.monotonic() - t0 > timeout_s:
                raise RuntimeError("VL53L1X data-ready timeout during calibration")
            time.sleep(0.003)

    def calibrate_offset(self, target_mm: int = 140, *, n: int = 50) -> int:
        """BENCH: solve the part-to-part range offset against a known target.

        Operator places a target at ``target_mm`` (ST recommends a 17% grey card at
        ~140 mm). Zeroes the offset registers, ranges ``n`` valid frames, and returns
        ``offset_mm = round(target_mm - mean_measured)``. RUN BEFORE :meth:`calibrate_xtalk`
        (ST order). The returned value is persisted by the characterize tool and later
        written to ALGO__PART_TO_PART_RANGE_OFFSET_MM (as mm x 4) by :meth:`_apply_calibration`.

        Bench-path faults RAISE (operator-visible), consistent with
        :meth:`_wait_data_ready`: ``target_mm <= 0`` -> ValueError (guards the
        ``round(target - mean)`` path); ``n < 1`` -> ValueError (guards /n).
        """
        if target_mm <= 0:
            raise ValueError("offset cal: target_mm must be > 0 (got %r)" % target_mm)
        if n < 1:
            raise ValueError("offset cal: n must be >= 1 (got %r)" % n)
        # Zero the offset registers so we measure the raw (uncorrected) distance.
        self._wr(self._R_PART_TO_PART_OFFSET, b"\x00\x00")
        self._wr(self._R_MM_INNER_OFFSET, b"\x00\x00")
        self._wr(self._R_MM_OUTER_OFFSET, b"\x00\x00")
        self._start_ranging()
        try:
            total = 0
            for _ in range(n):
                self._wait_data_ready()
                total += struct.unpack(">H", self._rd(self._R_RANGE_MM, 2))[0]
                self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
        finally:
            self._stop_ranging()
        mean_dist = total / float(n)
        return int(round(target_mm - mean_dist))

    def calibrate_xtalk(self, target_mm: int = 600, *, n: int = 50) -> int:
        """BENCH: solve the crosstalk compensation against a target IN THE DARK.

        Operator places a 17% grey target at ~``target_mm`` with NO ambient IR (dark
        room / no other reflector). Mirrors the ST ULD ``CalibrateXtalk`` loop EXACTLY:
        pre-zero the plane-offset register, then per frame read SignalRate (x8),
        Distance, clear the interrupt, read SpadNb (>>8), accumulate as floats; finally
        ``xtalk_raw = int(512 * avgSignal * (1 - avgDist/target) / avgSpad)`` clamped to
        uint16. NO frame-0 discard. Returns the RAW uint16 (persisted as-is; scaled at
        apply time). SignalRate is RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED
        (@0x0098), correct here precisely BECAUSE the plane offset was zeroed first.

        Bench-path faults RAISE (operator-visible), consistent with
        :meth:`_wait_data_ready`: ``target_mm <= 0`` -> ValueError (guards
        ``1 - avgDist/target``); ``n < 1`` -> ValueError (guards /n); zero effective
        SPADs -> RuntimeError BEFORE the divide (a sensor-dark / no-valid-frames
        condition -- raising is correct; silently returning 0 would persist a bogus
        "calibrated" value).
        """
        if target_mm <= 0:
            raise ValueError("xtalk cal: target_mm must be > 0 (got %r)" % target_mm)
        if n < 1:
            raise ValueError("xtalk cal: n must be >= 1 (got %r)" % n)
        # Pre: zero the crosstalk plane offset (kcps) so SignalRate is uncompensated.
        self._wr(self._R_XTALK_PLANE_OFFSET, b"\x00\x00")
        self._start_ranging()
        try:
            sum_signal = 0.0
            sum_dist = 0.0
            sum_spad = 0.0
            for _ in range(n):
                self._wait_data_ready()
                sum_signal += struct.unpack(
                    ">H", self._rd(self._R_SIGNAL_RATE, 2))[0] * 8.0
                sum_dist += float(
                    struct.unpack(">H", self._rd(self._R_RANGE_MM, 2))[0])
                self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
                sum_spad += struct.unpack(
                    ">H", self._rd(self._R_SPAD_NB, 2))[0] >> 8
        finally:
            self._stop_ranging()
        avg_signal = sum_signal / n
        avg_dist = sum_dist / n
        avg_spad = sum_spad / n
        # Guard BEFORE the divide: zero effective SPADs => sensor dark / no valid
        # frames. Raise (operator-visible); do NOT silently return 0 (it would persist
        # as a bogus "calibrated").
        if avg_spad <= 0:
            raise RuntimeError(
                "xtalk cal: zero effective SPADs (sensor dark / no valid frames)")
        xtalk_raw = int(512 * avg_signal * (1.0 - avg_dist / target_mm) / avg_spad)
        return max(0, min(xtalk_raw, 0xFFFF))

    def close(self) -> None:
        """Stop ranging + close the bus (idempotent, never raises)."""
        try:
            self._stop_ranging()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            self._bus.close()
        except Exception:                                          # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Host MOCK (no hardware) -- for selftests + a deviceless dry-run.
# --------------------------------------------------------------------------- #
class MockRangeReader:
    """Deterministic, hardware-free :class:`RangeReader` for host tests + the
    ``--lidar-mock`` live dry-run.

    Returns a scripted sequence of ``(dist_mm, range_status)`` pairs (cycling once
    exhausted), each passed through the SAME :func:`gate_reading` as the real reader
    -- so the read -> gate -> publish path runs WITHOUT an I2C bus.

    DEFAULT (live ``--lidar-mock``): a smooth ~0.20 m <-> 1.20 m sine sweep, ALL
    valid + in-band, so the FC UI shows a clean MOVING range (obviously "working")
    rather than a confusing static value. The validity-gate reject path is covered by
    the unit selftest, which passes its OWN reject-heavy script via ``script=``.
    """

    @staticmethod
    def _default_sweep():
        """Smooth LIVE-demo sweep: distance glides ~0.20 m <-> 1.20 m and back (an
        object waved under the sensor / a drone bobbing), ALL valid + in-band -- a
        clean moving range on the FC UI. ~160 steps -> a few-second period at the
        read rate."""
        import math
        n = 160
        return tuple(
            (int(200 + 500.0 * (1.0 - math.cos(2.0 * math.pi * i / n))),
             RANGE_STATUS_OK)
            for i in range(n)
        )

    def __init__(self, script=None) -> None:
        self._script = tuple(script) if script else self._default_sweep()
        if not self._script:
            raise ValueError("MockRangeReader needs a non-empty script")
        self._i = 0
        self._closed = False

    def read(self) -> RangeSample:
        dist_mm, range_status = self._script[self._i % len(self._script)]
        self._i += 1
        valid = gate_reading(int(dist_mm), int(range_status))
        return RangeSample(range_m=(dist_mm * 1e-3) if valid else 0.0,
                           valid=valid, dist_mm=int(dist_mm),
                           range_status=int(range_status), signal=None)

    def close(self) -> None:
        self._closed = True
