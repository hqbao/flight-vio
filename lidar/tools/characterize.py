"""``--characterize``: bench the VL53L1X over I2C + print the FC ``disarm_range``.

A bring-up tool for the downward rangefinder. It opens the VL53L1X over I2C,
streams each reading's raw fields (distance, range_status, signal), and -- because
the rig is sat ON THE GROUND while you run it -- accumulates the VALID ground
readings into a stable floor estimate and prints the recommended FC
``disarm_range`` = ground floor + margin.

Why this exists: the FC arms / disarms partly on the downward range (it must know
"this is the ground" to refuse a takeoff or to cut at touchdown). The exact ground
floor is a property of THIS rig (sensor mounting height above the skids, the
sensor's near bias), so it has to be MEASURED, not guessed. This tool turns a few
seconds of on-the-ground readings into the single number the FC wants.

It replaces the old UART-probe characterize tool (the rangefinder is now on the
Pi's I2C bus, not a UART). For a deviceless dry-run / to see the OUTPUT FORMAT,
pass ``--mock`` (the hardware-free reader).

Run (on the Pi, rig on the ground)::

    python -m lidar.tools.characterize --seconds 5
    python -m lidar.tools.characterize --mock          # no hardware (format demo)
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lidar.io.vl53l1x_reader import (                             # noqa: E402
    DEFAULT_I2C_ADDRESS, DEFAULT_I2C_BUS, DIST_MODE_SHORT,
    MockRangeReader, RangeSample, VL53L1XReader,
)

#: Default safety margin (metres) added to the measured ground floor to get the
#: recommended FC ``disarm_range``. Generous so sensor noise + a slightly uneven
#: floor never reads as "airborne" while sat on the ground.
DEFAULT_MARGIN_M = 0.10


@dataclass(frozen=True)
class GroundStats:
    """Summary of the valid ground readings collected during a characterize run."""

    n_total: int
    n_valid: int
    floor_m: float          # robust ground-floor estimate (median of valid range)
    min_m: float
    max_m: float
    std_m: float
    disarm_range_m: float   # recommended FC disarm_range = floor + margin


def summarize_ground(valid_ranges_m, *, margin_m: float = DEFAULT_MARGIN_M,
                     n_total: int = 0) -> GroundStats | None:
    """Reduce the valid ground ranges to a :class:`GroundStats` (pure -> testable).

    The floor is the MEDIAN of the valid readings (robust to the odd spurious
    short/long sample); the recommended ``disarm_range`` is ``floor + margin``.
    Returns ``None`` if there were no valid readings (nothing to recommend).
    """
    vals = [float(v) for v in valid_ranges_m]
    if not vals:
        return None
    floor = float(statistics.median(vals))
    std = float(statistics.pstdev(vals)) if len(vals) > 1 else 0.0
    return GroundStats(
        n_total=int(n_total) if n_total else len(vals),
        n_valid=len(vals),
        floor_m=floor,
        min_m=min(vals),
        max_m=max(vals),
        std_m=std,
        disarm_range_m=floor + float(margin_m),
    )


def _fmt_sample(seq: int, s: RangeSample) -> str:
    """One streamed reading line: raw dist + status + signal + the gate verdict."""
    sig = f"{s.signal:6.2f}" if s.signal is not None else "  n/a"
    flag = "VALID" if s.valid else "rej  "
    return (f"  [{seq:5d}] dist={s.dist_mm:5d}mm  status={s.range_status:3d}  "
            f"signal={sig}  -> {flag}  range_m={s.range_m:6.3f}")


def run_characterize(*, seconds: float, rate_hz: float, margin_m: float,
                     mock: bool, i2c_bus: int, i2c_address: int,
                     quiet: bool = False) -> int:
    """Stream readings for ``seconds`` then print the recommended FC disarm_range."""
    print("lidar characterize -- VL53L1X over I2C (rig must be ON THE GROUND)")
    print(f"  reader={'MOCK' if mock else 'I2C'}  bus={i2c_bus}  "
          f"addr=0x{i2c_address:02X}  seconds={seconds:g}  rate={rate_hz:g}Hz  "
          f"margin={margin_m:g}m\n")
    try:
        reader = (MockRangeReader() if mock else
                  VL53L1XReader(i2c_bus=i2c_bus, i2c_address=i2c_address,
                                distance_mode=DIST_MODE_SHORT))
    except Exception as e:                                          # noqa: BLE001
        print(f"  ERROR: could not open the rangefinder ({e})")
        return 1

    valid_ranges: list[float] = []
    seq = 0
    period = 1.0 / max(rate_hz, 1.0)
    deadline = time.monotonic() + float(seconds)
    try:
        while time.monotonic() < deadline:
            s = reader.read()
            if not quiet:
                print(_fmt_sample(seq, s))
            if s.valid:
                valid_ranges.append(s.range_m)
            seq += 1
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n  (interrupted)")
    finally:
        reader.close()

    stats = summarize_ground(valid_ranges, margin_m=margin_m, n_total=seq)
    print("\n" + "=" * 64)
    if stats is None:
        print("  NO valid ground readings -- check the sensor mounting / I2C "
              "address; cannot recommend a disarm_range.")
        return 1
    print(f"  readings: {stats.n_valid}/{stats.n_total} valid")
    print(f"  ground floor (median): {stats.floor_m:.3f} m  "
          f"[min {stats.min_m:.3f}, max {stats.max_m:.3f}, "
          f"std {stats.std_m:.3f}]")
    print(f"  margin: {margin_m:.3f} m")
    print("\n  >>> recommended FC disarm_range = "
          f"{stats.disarm_range_m:.3f} m  "
          f"(ground floor {stats.floor_m:.3f} + margin {margin_m:.3f})")
    print("      set this on the FC so it treats <= this range as 'on the ground'.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="how long to stream ground readings (default: 5)")
    ap.add_argument("--rate", type=float, default=20.0,
                    help="read cadence in Hz (default: 20)")
    ap.add_argument("--margin", type=float, default=DEFAULT_MARGIN_M,
                    help=f"safety margin (m) added to the ground floor for the "
                         f"recommended disarm_range (default: {DEFAULT_MARGIN_M})")
    ap.add_argument("--mock", action="store_true",
                    help="use the hardware-free MOCK reader (no I2C) -- shows the "
                         "output format without a device")
    ap.add_argument("--i2c-bus", type=int, default=DEFAULT_I2C_BUS,
                    help=f"Linux I2C bus number (default: {DEFAULT_I2C_BUS})")
    ap.add_argument("--i2c-address", type=lambda s: int(s, 0),
                    default=DEFAULT_I2C_ADDRESS,
                    help=f"VL53L1X 7-bit I2C address (default: "
                         f"0x{DEFAULT_I2C_ADDRESS:02X})")
    ap.add_argument("--quiet", action="store_true",
                    help="don't stream per-reading lines, only print the summary")
    args = ap.parse_args()
    return run_characterize(
        seconds=args.seconds, rate_hz=args.rate, margin_m=args.margin,
        mock=args.mock, i2c_bus=args.i2c_bus, i2c_address=args.i2c_address,
        quiet=args.quiet,
    )


if __name__ == "__main__":
    sys.exit(main())
