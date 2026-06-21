"""VL53L1X downward-rangefinder I2C reader (real device + a host MOCK).

The flight Pi carries a downward-facing VL53L1X time-of-flight rangefinder (a
TOF400F breakout, which is a generic VL53L1X exposing the chip's I2C register
map). This module reads distance + ``range_status`` over I2C and gates each
reading into a (range_m, valid) pair the ``lidar`` process publishes on
``lidar.range`` and the ``fc`` sender bundles into the dblink VIO-pose frame.

SWAPPABLE device interface
--------------------------
The TOF400F's exact I2C address / mode is HIL-UNKNOWN until the rig is on the
bench (the breakout may strap a non-default address, and which distance/timing
mode the optics want is a bring-up call). So the reader is behind a tiny
:class:`RangeReader` interface with two implementations:

* :class:`VL53L1XReader` -- the real ``pimoroni-vl53l1x`` + ``smbus2`` reader,
  imported LAZILY (only the flight Pi installs them; a dev host / CI never does).
  Short distance mode (~1.3 m, matching a low-altitude AGL sensor) + a ~50 Hz
  timing budget. Any I2C / device error -> a ``valid=0`` sample, NEVER an
  exception: a flaky sensor must not crash the flight process.
* :class:`MockRangeReader` -- a deterministic, hardware-free reader for host
  tests: it returns a scripted sequence of ``(dist_mm, range_status)`` so the
  read -> gate -> publish path (including the ``range_status != 0`` reject) is
  exercised with no I2C bus.

The GATE (:func:`gate_reading`) is a pure function so it is unit-testable in
isolation: ``valid = (range_status == 0) and (LIDAR_MIN_MM <= dist_mm <=
LIDAR_MAX_MM)``. ``range_status == 0`` is the VL53L1X "range valid" code; any
other status (sigma fail, signal fail, wrap-around, out-of-bounds, ...) rejects
the reading.

Units: the chip reports MILLIMETRES; the published / wire value is METRES.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Protocol

LOG = logging.getLogger("lidar.io.vl53l1x")

# --------------------------------------------------------------------------- #
# Default I2C wiring + gate bounds. The address/bus are HIL-unknown for the
# TOF400F (it may strap a non-default address) -- overridable on the reader.
# --------------------------------------------------------------------------- #
#: VL53L1X default 7-bit I2C address (0x29). The TOF400F may strap a different
#: one -- pass ``i2c_address`` to override once the bench address is known.
DEFAULT_I2C_ADDRESS = 0x29
#: Default Linux I2C bus number on the Pi (``/dev/i2c-1`` on the 40-pin header).
DEFAULT_I2C_BUS = 1

#: VL53L1X distance-mode codes (pimoroni-vl53l1x ``start_ranging`` argument).
#: SHORT = up to ~1.3 m, the best ambient-light immunity -- correct for a
#: low-altitude AGL rangefinder.
DIST_MODE_SHORT = 1
DIST_MODE_MEDIUM = 2
DIST_MODE_LONG = 3

#: Inter-measurement timing budget (microseconds) -> ~50 Hz read cadence. 20 ms is
#: comfortably above the VL53L1X short-mode minimum (~8 ms) and bounds the sensor's
#: own integration so a read never blocks the loop unexpectedly.
DEFAULT_TIMING_BUDGET_US = 20_000

#: Gate bounds (MILLIMETRES). A reading outside this band is rejected (valid=0)
#: even if ``range_status == 0``: below the floor is the sensor's near dead-zone /
#: a spurious zero, above the ceiling is beyond short-mode trust. The FC owns the
#: flight-relevant ground/disarm thresholds; this is only the sensor sanity band.
LIDAR_MIN_MM = 30
LIDAR_MAX_MM = 4000

#: VL53L1X "range valid" status code. Any other value is a rejected reading.
RANGE_STATUS_OK = 0


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
# Real I2C reader (pimoroni-vl53l1x + smbus2), imported lazily.
# --------------------------------------------------------------------------- #
class VL53L1XReader:
    """Real VL53L1X reader over I2C via ``pimoroni-vl53l1x`` + ``smbus2``.

    Constructed with the device opened + ranging started (short distance mode by
    default). The heavy deps are imported INSIDE ``__init__`` so importing this
    module on a dev host / CI (which never installs them) costs nothing and the
    ``cv2-free`` flight image stays clean.

    Any failure to open / start the device raises from ``__init__`` (the caller --
    ``lidar.main`` -- decides whether that is fatal). But once ranging, a per-read
    I2C error is swallowed into a ``valid=0`` sample so a flaky sensor never
    crashes the flight loop.
    """

    def __init__(self, *, i2c_bus: int = DEFAULT_I2C_BUS,
                 i2c_address: int = DEFAULT_I2C_ADDRESS,
                 distance_mode: int = DIST_MODE_SHORT,
                 timing_budget_us: int = DEFAULT_TIMING_BUDGET_US) -> None:
        # Lazy import: only the flight Pi installs these. ``VL53L1X`` is the
        # pimoroni-vl53l1x module; it uses smbus2 under the hood.
        import VL53L1X  # noqa: N814  (third-party module name)

        self._i2c_address = int(i2c_address)
        self._distance_mode = int(distance_mode)
        self._tof = VL53L1X.VL53L1X(i2c_bus=int(i2c_bus),
                                    i2c_address=int(i2c_address))
        self._tof.open()
        # Some breakouts need the timing budget set before ranging; guard it so a
        # driver build without the setter still starts.
        try:
            self._tof.set_timing(int(timing_budget_us),
                                 int(timing_budget_us // 1000))
        except Exception:                                          # noqa: BLE001
            pass
        self._tof.start_ranging(int(distance_mode))
        LOG.info("lidar: VL53L1X open on i2c bus=%d addr=0x%02X mode=%d "
                 "budget=%dus", int(i2c_bus), int(i2c_address),
                 int(distance_mode), int(timing_budget_us))

    def read(self) -> RangeSample:
        """One gated reading -- NEVER raises (I2C error -> invalid sample)."""
        try:
            dist_mm = int(self._tof.get_distance())
            # range_status / signal are exposed via the driver's last-measurement
            # accessors when present; default to "valid status, no signal" if the
            # driver build doesn't surface them (the distance gate still applies).
            range_status = int(getattr(self._tof, "get_range_status",
                                       lambda: RANGE_STATUS_OK)())
            signal = getattr(self._tof, "get_signal_rate", None)
            signal_v = float(signal()) if callable(signal) else None
        except Exception as e:                                     # noqa: BLE001
            # I2C / device hiccup: a flaky sensor must not crash the flight loop.
            LOG.debug("lidar: VL53L1X read error (%s) -> invalid sample", e)
            return RangeSample(range_m=0.0, valid=False, dist_mm=0,
                               range_status=-1, signal=None)
        valid = gate_reading(dist_mm, range_status)
        return RangeSample(range_m=(dist_mm * 1e-3) if valid else 0.0,
                           valid=valid, dist_mm=dist_mm,
                           range_status=range_status, signal=signal_v)

    def close(self) -> None:
        """Stop ranging + close the device (idempotent, never raises)."""
        try:
            self._tof.stop_ranging()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            self._tof.close()
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
