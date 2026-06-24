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

``--calibrate`` runs the bench OFFSET-then-XTALK routine (operator prompts for a 17%
grey target @140 mm, then @~600 mm IN THE DARK) and persists the solve to
``.cache/lidar_calib.json`` keyed by ``--sensor-id``, so the live reader auto-applies
it. The register dance lives in the reader; this tool only prompts + orchestrates +
persists (it needs a REAL sensor -- ``--mock`` is not supported for ``--calibrate``).

Run (on the Pi, rig on the ground)::

    python -m lidar.tools.characterize --seconds 5
    python -m lidar.tools.characterize --mock          # no hardware (format demo)
    python -m lidar.tools.characterize --calibrate     # bench offset + xtalk solve
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lidar.io import lidar_calib_store                            # noqa: E402
from lidar.io.vl53l1x_reader import (                             # noqa: E402
    DEFAULT_I2C_ADDRESS, DEFAULT_I2C_BUS, DEFAULT_TIMING_BUDGET_US,
    DIST_MODE_SHORT, LIDAR_MIN_MM, MockRangeReader, RangeSample, VL53L1XReader,
)

#: Bench calibration target standoffs (mm). ST: offset against a 17% grey card at
#: ~140 mm; crosstalk against the same card at ~600 mm with NO ambient IR (dark).
OFFSET_TARGET_MM = 140
XTALK_TARGET_MM = 600
#: Frames averaged per bench step (matches the reader's default + the stored ``n``).
CALIB_FRAMES = 50

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


def run_calibrate(*, sensor_id: str, i2c_bus: int, i2c_address: int) -> int:
    """Bench OFFSET-then-XTALK calibration; persist to ``.cache/lidar_calib.json``.

    THIN orchestrator: prompts the operator, calls the reader's bench routines (which
    own the register dance), then persists via :mod:`lidar.io.lidar_calib_store`. ST
    order is offset first, then crosstalk. Needs a real sensor (no ``--mock``).

    The bench routines each END on ``_stop_ranging`` and we ``close()`` the reader in
    the ``finally`` BEFORE persisting -- so this tool deliberately does NOT ``read()``
    the (now-stopped) device afterward (a read on a stopped device would just time out
    -> a fail-closed invalid sample, but it is still a footgun). The newly-persisted
    cal is picked up by the LIVE ``lidar.main`` process, which constructs a FRESH
    :class:`VL53L1XReader` whose ``_init_sensor`` applies it + restarts continuous
    ranging. Do NOT add a post-calibrate ``read()`` here.
    """
    print(f"lidar CALIBRATE -- VL53L1X over I2C  bus={i2c_bus}  "
          f"addr=0x{i2c_address:02X}  sensor_id={sensor_id!r}")
    print("  (offset first, then crosstalk -- ST order)\n")
    try:
        reader = VL53L1XReader(i2c_bus=i2c_bus, i2c_address=i2c_address,
                               distance_mode=DIST_MODE_SHORT, sensor_id=sensor_id)
    except Exception as e:                                          # noqa: BLE001
        print(f"  ERROR: could not open the rangefinder ({e})")
        return 1
    try:
        input(f"  [1/2] OFFSET: place a 17% grey target flat at {OFFSET_TARGET_MM}mm, "
              "then press Enter...")
        offset_mm = reader.calibrate_offset(OFFSET_TARGET_MM, n=CALIB_FRAMES)
        print(f"        -> offset_mm = {offset_mm}")
        input(f"  [2/2] XTALK: place the 17% grey target at ~{XTALK_TARGET_MM}mm in "
              "the DARK (no ambient IR), then press Enter...")
        xtalk_raw = reader.calibrate_xtalk(XTALK_TARGET_MM, n=CALIB_FRAMES)
        print(f"        -> xtalk_raw = {xtalk_raw}")
    except Exception as e:                                          # noqa: BLE001
        print(f"  ERROR: calibration aborted ({e}) -- nothing saved")
        return 1
    finally:
        reader.close()

    path = lidar_calib_store.save(
        sensor_id, xtalk=xtalk_raw, offset_mm=offset_mm,
        distance_mode=DIST_MODE_SHORT, min_mm=LIDAR_MIN_MM,
        timing_budget_us=DEFAULT_TIMING_BUDGET_US, n=CALIB_FRAMES)
    print(f"\n  >>> saved calibration for sensor_id={sensor_id!r} -> {path}")
    print("      the live lidar reader auto-applies it on the next ranging start.")
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
    ap.add_argument("--calibrate", action="store_true",
                    help="run the bench OFFSET-then-XTALK calibration (prompts for a "
                         "17%% grey target @140mm then @~600mm DARK) and persist it")
    ap.add_argument("--sensor-id", default="default",
                    help="sensor id the calibration is keyed under in "
                         ".cache/lidar_calib.json (default: 'default')")
    args = ap.parse_args()
    if args.calibrate:
        if args.mock:
            print("  ERROR: --calibrate needs a real sensor (not --mock)")
            return 2
        return run_calibrate(sensor_id=args.sensor_id, i2c_bus=args.i2c_bus,
                             i2c_address=args.i2c_address)
    return run_characterize(
        seconds=args.seconds, rate_hz=args.rate, margin_m=args.margin,
        mock=args.mock, i2c_bus=args.i2c_bus, i2c_address=args.i2c_address,
        quiet=args.quiet,
    )


if __name__ == "__main__":
    sys.exit(main())
