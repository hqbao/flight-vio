"""lidar process: read a downward VL53L1X rangefinder over I2C, publish lidar.range.

The flight Pi carries a downward-facing VL53L1X time-of-flight rangefinder. This
STANDALONE process reads it over I2C and publishes each gated reading as a
:class:`~lidar.comms.wire.WireRange` POD on the ``lidar.range`` IPC topic, served
on its own ``oak.lidar`` endpoint. The ``fc`` UART sender opens a read-only client
on that endpoint, keeps the freshest range, and BUNDLES it into the dblink VIO-pose
frame (the range is NOT a separate dblink message; see :mod:`sky.fc.dblink`).

PURE PRODUCER -- unlike ``depth``/``vio``/``slam`` this process subscribes to
NOTHING: it owns no capture dependency, no calib barrier, no shared-memory rings.
It just opens the sensor and publishes. So a missing / ``--no-lidar`` lidar process
simply means the ``fc`` sender never sees a range (it sends ``range_valid=0``) --
the VIO send is completely unaffected.

Latest-wins, no back-pressure
-----------------------------
The reading loop publishes directly on a non-blocking ``IPCPubSub`` server
(``blocking=False`` -> drop-oldest on a slow consumer), so a slow / disconnected
``fc`` consumer can never stall the read loop. Each reading carries a host
``monotonic_ns`` capture instant the ``fc`` side uses as a freshness gate.

Data flow::

                                   lidar (oak.lidar)
                                   -----------------
    VL53L1X --I2C--> RangeReader.read() -> gate -> WireRange
                                          |  lidar.range
                                          v
                                   IPCPubSub server (oak.lidar)  --IPC--> fc client

Run::

    python -m lidar.main --endpoint oak.lidar --rate 50
    python -m lidar.main --mock          # no hardware (host dry-run / smoke)
"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lidar.comms import IPCPubSub, topics                          # noqa: E402
from lidar.comms.wire import WireRange                             # noqa: E402
from lidar.io.vl53l1x_reader import (                              # noqa: E402
    DEFAULT_I2C_ADDRESS, DEFAULT_I2C_BUS, DIST_MODE_SHORT,
    MockRangeReader, RangeReader, VL53L1XReader,
)

LOG = logging.getLogger("lidar.main")

#: Canonical endpoint the lidar process serves ``lidar.range`` on.
DEFAULT_LIDAR_ENDPOINT = "oak.lidar"
#: Default read cadence (Hz). The VL53L1X short-mode budget is ~20 ms, so ~50 Hz is
#: the practical ceiling; clamp so a runaway value can't busy-spin the bus.
DEFAULT_RATE_HZ = 50.0
_RATE_MIN_HZ, _RATE_MAX_HZ = 1.0, 100.0

_OUTPUT_TOPIC = topics.LIDAR_RANGE


def _clamp_rate(rate_hz: float) -> float:
    """Clamp the requested read cadence into the safe band."""
    return float(min(max(float(rate_hz), _RATE_MIN_HZ), _RATE_MAX_HZ))


def _build_reader(*, mock: bool, i2c_bus: int, i2c_address: int) -> RangeReader:
    """Construct the real I2C reader or the host mock.

    Kept tiny + separate so ``run_lidar`` is testable against the mock without an
    I2C bus. A real-reader open failure propagates (run_lidar decides it is fatal).
    """
    if mock:
        LOG.info("lidar: MOCK reader (no hardware) -- scripted readings")
        return MockRangeReader()
    return VL53L1XReader(i2c_bus=i2c_bus, i2c_address=i2c_address,
                         distance_mode=DIST_MODE_SHORT)


# --------------------------------------------------------------------------- #
def run_lidar(*,
              endpoint: str = DEFAULT_LIDAR_ENDPOINT,
              rate_hz: float = DEFAULT_RATE_HZ,
              mock: bool = False,
              i2c_bus: int = DEFAULT_I2C_BUS,
              i2c_address: int = DEFAULT_I2C_ADDRESS,
              max_reads: int = 0,
              reader: RangeReader | None = None) -> int:
    """Run the standalone lidar process until SIGTERM / Ctrl-C (or ``max_reads``).

    Opens the rangefinder (real I2C or the mock), serves ``lidar.range`` on a
    non-blocking ``IPCPubSub`` server, and loops at ``rate_hz`` publishing one
    :class:`WireRange` per reading. A real-reader open failure returns non-zero
    (the launcher treats lidar as NON-FATAL -- the rest of the stack keeps
    running, the fc sender just gets no range).
    """
    rate_hz = _clamp_rate(rate_hz)
    LOG.info("lidar: opening rangefinder (mock=%s) -> serving %s on %s @ %.1f Hz",
             mock, _OUTPUT_TOPIC, endpoint, rate_hz)
    try:
        # ``reader`` lets a test inject a pre-built reader (e.g. a scripted mock with
        # a known valid+invalid mix); production passes None -> build from mock/I2C.
        if reader is None:
            reader = _build_reader(mock=mock, i2c_bus=i2c_bus,
                                   i2c_address=i2c_address)
    except Exception as e:                                          # noqa: BLE001
        # NON-FATAL to the stack (mirror fc's failed-serial-open): log + exit
        # non-zero. The launcher does not take the pipeline down on a lidar fault.
        LOG.error("lidar: could NOT open the rangefinder (%s) -- lidar exiting; the "
                  "rest of the stack is unaffected (fc sends range_valid=0)", e)
        return 1

    # Output server: non-blocking (drop-oldest) so a slow / disconnected fc
    # consumer never stalls the read loop. No rings, no retained topics -- a pure
    # POD stream; the fc client just follows the latest.
    server = IPCPubSub(endpoint, role="server", blocking=False)
    server.start()

    stop = [False]

    def _on_sigterm(_signo, _frame):
        stop[0] = True
    # Ctrl-C (SIGINT) + launcher SIGTERM both request the same clean stop. Guarded:
    # signal handlers can only be installed on the MAIN thread, and the selftest
    # drives run_lidar from a worker thread -- there the stop is driven by max_reads
    # / the thread join instead, so a non-main-thread call must not crash.
    try:
        signal.signal(signal.SIGTERM, _on_sigterm)
        signal.signal(signal.SIGINT, _on_sigterm)
    except ValueError:
        LOG.debug("lidar: not on the main thread -> skipping signal handlers")

    period = 1.0 / rate_hz
    seq = 0
    n_valid = 0
    LOG.info("lidar[%s] streaming %s (gated range_m + validity)",
             endpoint, _OUTPUT_TOPIC)
    next_t = time.monotonic()
    try:
        while not stop[0]:
            sample = reader.read()                # NEVER raises (invalid on error)
            msg = WireRange(seq=seq, ts_ns=time.monotonic_ns(),
                            range_m=float(sample.range_m),
                            valid=1 if sample.valid else 0)
            server.publish(_OUTPUT_TOPIC, msg)
            seq += 1
            if sample.valid:
                n_valid += 1
            if max_reads > 0 and seq >= max_reads:
                LOG.info("lidar: reached max_reads=%d", max_reads)
                break
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()         # fell behind -> re-anchor
    except KeyboardInterrupt:
        LOG.info("lidar: SIGINT -> stopping")
    finally:
        try:
            reader.close()
        except Exception:                                          # noqa: BLE001
            pass
        try:
            server.close()
        except Exception:                                          # noqa: BLE001
            pass
        LOG.info("lidar: shutdown complete (published %d readings, %d valid)",
                 seq, n_valid)
    return 0


# --------------------------------------------------------------------------- #
def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")

    def _excepthook(args):
        LOG.error("THREAD CRASH in %s: %s: %s", args.thread.name,
                  args.exc_type.__name__, args.exc_value, exc_info=(
                      args.exc_type, args.exc_value, args.exc_traceback))
    threading.excepthook = _excepthook

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--endpoint", default=DEFAULT_LIDAR_ENDPOINT,
                    help=f"this process's IPC endpoint "
                         f"(default: {DEFAULT_LIDAR_ENDPOINT!r})")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                    help=f"I2C read + publish cadence in Hz, clamped "
                         f"[{int(_RATE_MIN_HZ)},{int(_RATE_MAX_HZ)}] "
                         f"(default: {DEFAULT_RATE_HZ:g})")
    ap.add_argument("--mock", action="store_true",
                    help="use the hardware-free MOCK reader (no I2C bus) -- host "
                         "dry-run / smoke, not flight")
    ap.add_argument("--i2c-bus", type=int, default=DEFAULT_I2C_BUS,
                    help=f"Linux I2C bus number (default: {DEFAULT_I2C_BUS} = "
                         f"/dev/i2c-{DEFAULT_I2C_BUS})")
    ap.add_argument("--i2c-address", type=lambda s: int(s, 0),
                    default=DEFAULT_I2C_ADDRESS,
                    help=f"VL53L1X 7-bit I2C address (default: "
                         f"0x{DEFAULT_I2C_ADDRESS:02X}, the bare breakout's factory "
                         f"address; override only if re-strapped)")
    ap.add_argument("--max-reads", type=int, default=0,
                    help="stop after publishing this many readings (0 = run forever)")
    args = ap.parse_args()

    return run_lidar(
        endpoint=args.endpoint,
        rate_hz=args.rate,
        mock=args.mock,
        i2c_bus=args.i2c_bus,
        i2c_address=args.i2c_address,
        max_reads=args.max_reads,
    )


if __name__ == "__main__":
    # Same os._exit pattern as the other split process mains -- prevent any
    # lingering non-daemon thread (IPCPubSub fan-out) from holding the process
    # past a shutdown deadline.
    import os as _os
    _rc = main()
    LOG.info("lidar: main returned, calling os._exit(%d)", int(_rc))
    logging.shutdown()
    _os.sys.stdout.flush()
    _os.sys.stderr.flush()
    _os._exit(int(_rc))
