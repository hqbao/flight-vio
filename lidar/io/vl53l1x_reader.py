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
  config block and selects long distance mode @ 50 ms (proven on the bench). Any
  I2C / device error -> a ``valid=0`` sample, NEVER an exception: a flaky sensor
  must not crash the flight process.
* :class:`MockRangeReader` -- a deterministic, hardware-free reader for host
  tests: it returns a scripted sequence of ``(dist_mm, range_status)`` so the
  read -> gate -> publish path (including the ``range_status != RANGE_STATUS_OK``
  reject) is exercised with no I2C bus.

The GATE (:func:`gate_reading`) is a pure function so it is unit-testable in
isolation: ``valid = (range_status == RANGE_STATUS_OK) and (LIDAR_MIN_MM <=
dist_mm <= LIDAR_MAX_MM)``. ``RANGE_STATUS_OK == 0x09`` is the VL53L1X
RESULT__RANGE_STATUS code for a completed range (verified on-device); any other
status (sigma fail, signal fail, wrap-around, out-of-bounds, ...) rejects the
reading.

Units: the chip reports MILLIMETRES; the published / wire value is METRES.
"""
from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Protocol

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
#: ambient-light immunity; LONG = up to ~4 m. The bench-proven config (written by
#: :meth:`VL53L1XReader._init_sensor` via the macro-period registers) is LONG @
#: 50 ms; ``distance_mode`` is accepted for interface compatibility (SHORT is a
#: future tuning).
DIST_MODE_SHORT = 1
DIST_MODE_MEDIUM = 2
DIST_MODE_LONG = 3

#: Inter-measurement timing budget (microseconds) -> ~50 Hz read cadence. 20 ms is
#: comfortably above the VL53L1X short-mode minimum (~8 ms) and bounds the sensor's
#: own integration so a read never blocks the loop unexpectedly.
DEFAULT_TIMING_BUDGET_US = 20_000

#: Gate bounds (MILLIMETRES). A reading outside this band is rejected (valid=0)
#: even if ``range_status == RANGE_STATUS_OK``: below the floor is the sensor's near
#: dead-zone / a spurious zero, above the ceiling is beyond trusted range. The FC owns
#: the flight-relevant ground/disarm thresholds; this is only the sensor sanity band.
LIDAR_MIN_MM = 30
LIDAR_MAX_MM = 4000

#: VL53L1X RESULT__RANGE_STATUS (reg 0x0089) value for a completed/valid range.
#: Verified on-device: a good measurement reads 0x09; any other value is a reject
#: (sigma/signal fail, wrap-around, out-of-bounds, ...).
RANGE_STATUS_OK = 0x09


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


def gate_reading(dist_mm: int, range_status: int) -> bool:
    """Sensor-side validity gate (pure -> unit-testable).

    ``valid`` iff the chip reported the "range valid" status AND the distance is
    inside the sane band ``[LIDAR_MIN_MM, LIDAR_MAX_MM]``. A non-zero
    ``range_status`` (sigma/signal fail, wrap-around, out-of-bounds, ...) or an
    out-of-band distance rejects the reading. The published ``range_m`` is forced
    to 0.0 on a reject by the caller.
    """
    return (range_status == RANGE_STATUS_OK
            and LIDAR_MIN_MM <= dist_mm <= LIDAR_MAX_MM)


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
    the 91-byte ST/Adafruit default config block to reg 0x002D, then selects long
    distance mode @ 50 ms timing budget (proven on the bench; short mode is a future
    tuning). ``smbus2`` is imported INSIDE ``__init__`` so importing this module on a
    dev host / CI costs nothing and the ``cv2-free`` flight image stays clean.

    ``__init__`` raises if the sensor is absent / not a VL53L1X (the caller --
    ``lidar.main`` -- decides whether that is fatal). Once ranging, a per-read I2C
    error is swallowed into a ``valid=0`` sample so a flaky sensor never crashes the
    flight loop. ``distance_mode``/``timing_budget_us`` are accepted for interface
    compatibility; the proven long-50 ms config is used.
    """

    # VL53L1X registers (16-bit addresses, big-endian on the wire).
    _R_MODEL_ID = 0x010F
    _R_GPIO_HV_MUX_CTRL = 0x0030
    _R_GPIO_TIO_HV_STATUS = 0x0031
    _R_TB_MACROP_A_HI = 0x005E
    _R_TB_MACROP_B_HI = 0x0061
    _R_INTERRUPT_CLEAR = 0x0086
    _R_MODE_START = 0x0087
    _R_RANGE_STATUS = 0x0089
    _R_RANGE_MM = 0x0096

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

    def __init__(self, *, i2c_bus: int = DEFAULT_I2C_BUS,
                 i2c_address: int = DEFAULT_I2C_ADDRESS,
                 distance_mode: int = DIST_MODE_LONG,
                 timing_budget_us: int = DEFAULT_TIMING_BUDGET_US) -> None:
        import smbus2  # lazy: only the flight Pi installs it
        self._smbus2 = smbus2
        self._a = int(i2c_address)
        self._bus = smbus2.SMBus(int(i2c_bus))
        mid = self._rd(self._R_MODEL_ID, 3)
        if mid != b"\xEA\xCC\x10":
            self.close()
            raise RuntimeError(
                "no VL53L1X at 0x%02X (model id %s, expected EA CC 10)"
                % (self._a, mid.hex(" ")))
        self._init_sensor()
        LOG.info("lidar: VL53L1X (smbus2) ranging on i2c bus=%d addr=0x%02X "
                 "(long mode, 50ms)", int(i2c_bus), self._a)

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

    def _init_sensor(self) -> None:
        self._wr(0x002D, self._INIT_SEQ)
        self._wr(self._R_MODE_START, b"\x40")            # start
        t0 = time.monotonic()
        while not self._data_ready():
            if time.monotonic() - t0 > 0.5:
                break
            time.sleep(0.005)
        self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
        self._wr(self._R_MODE_START, b"\x00")            # stop
        self._wr(0x0008, b"\x09")
        self._wr(0x000B, b"\x00")
        self._wr(self._R_TB_MACROP_A_HI, b"\x00\xAD")    # long mode, 50 ms
        self._wr(self._R_TB_MACROP_B_HI, b"\x00\xC6")
        self._wr(self._R_MODE_START, b"\x40")            # start continuous

    def read(self) -> RangeSample:
        """One gated reading -- NEVER raises (I2C error -> invalid sample)."""
        try:
            t0 = time.monotonic()
            while not self._data_ready():
                if time.monotonic() - t0 > 0.12:
                    break
                time.sleep(0.003)
            status = self._rd(self._R_RANGE_STATUS)[0]
            dist_mm = struct.unpack(">H", self._rd(self._R_RANGE_MM, 2))[0]
            self._wr(self._R_INTERRUPT_CLEAR, b"\x01")
        except Exception as e:                                     # noqa: BLE001
            LOG.debug("lidar: VL53L1X read error (%s) -> invalid sample", e)
            return RangeSample(range_m=0.0, valid=False, dist_mm=0,
                               range_status=-1, signal=None)
        valid = gate_reading(dist_mm, status)
        return RangeSample(range_m=(dist_mm * 1e-3) if valid else 0.0,
                           valid=valid, dist_mm=dist_mm,
                           range_status=status, signal=None)

    def close(self) -> None:
        """Stop ranging + close the bus (idempotent, never raises)."""
        try:
            self._wr(self._R_MODE_START, b"\x00")
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
