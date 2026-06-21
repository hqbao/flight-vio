#!/usr/bin/env python3
"""Mock-sensor selftest for the ``lidar`` project (no I2C hardware).

Exercises the read -> gate -> publish path WITHOUT a device, three ways:

  (a) GATE -- :func:`lidar.io.vl53l1x_reader.gate_reading` is the pure validity
      rule: ``valid iff range_status == 0 AND LIDAR_MIN_MM <= dist_mm <=
      LIDAR_MAX_MM``. Both reject paths (a non-zero range_status; an out-of-band
      distance) MUST yield valid=0.
  (b) MOCK READER -- :class:`MockRangeReader` returns a scripted sequence and
      produces :class:`RangeSample`s with range_m in METRES, 0.0 on a reject, and
      the range_status carried through. A ``range_status != 0`` sample is invalid.
  (c) PUBLISH -- ``lidar.main.run_lidar(mock=True)`` reads + publishes WireRange on
      a real IPC server; a client on the endpoint receives them and the
      valid/invalid readings round-trip (range_m metres, valid 0/1) exactly as the
      ``fc`` sender will consume them.

  .venv/bin/python -m lidar.tests.lidar_mock_selftest
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from lidar.comms import IPCPubSub, topics                          # noqa: E402
from lidar.comms.wire import WireRange                             # noqa: E402
from lidar.io.vl53l1x_reader import (                              # noqa: E402
    LIDAR_MAX_MM, LIDAR_MIN_MM, MockRangeReader, RANGE_STATUS_OK,
    gate_reading,
)
from lidar.main import run_lidar                                   # noqa: E402


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


def test_gate() -> bool:
    print("[a] gate_reading: status==0 + in-band -> valid; else reject")
    _check(gate_reading(842, RANGE_STATUS_OK), "0.842 m, status 0 -> VALID")
    _check(gate_reading(LIDAR_MIN_MM, RANGE_STATUS_OK), "exactly the min -> VALID")
    _check(gate_reading(LIDAR_MAX_MM, RANGE_STATUS_OK), "exactly the max -> VALID")
    # range_status != 0 rejects even a sane distance.
    _check(not gate_reading(842, 4), "status 4 (signal fail) -> REJECT")
    _check(not gate_reading(842, 1), "status 1 (sigma fail) -> REJECT")
    # out-of-band distance rejects even with a valid status.
    _check(not gate_reading(LIDAR_MIN_MM - 1, RANGE_STATUS_OK),
           "below the min band -> REJECT")
    _check(not gate_reading(LIDAR_MAX_MM + 1, RANGE_STATUS_OK),
           "above the max band -> REJECT")
    _check(not gate_reading(0, RANGE_STATUS_OK), "0 mm (spurious zero) -> REJECT")
    return True


def test_mock_reader() -> bool:
    print("[b] MockRangeReader: scripted read -> gated RangeSample (mm -> m)")
    reader = MockRangeReader(script=[
        (842, RANGE_STATUS_OK),    # valid -> 0.842 m
        (840, 4),                  # status fail -> reject
        (5000, RANGE_STATUS_OK),   # out of band -> reject
    ])
    s0 = reader.read()
    _check(s0.valid and abs(s0.range_m - 0.842) < 1e-9,
           f"reading 0: VALID, range_m={s0.range_m:.3f} m (mm/1000)")
    _check(s0.range_status == RANGE_STATUS_OK, "reading 0: status carried (0)")
    s1 = reader.read()
    _check(not s1.valid and s1.range_m == 0.0,
           "reading 1: range_status != 0 -> INVALID, range_m forced 0.0")
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
    print("[c] run_lidar(mock) publishes WireRange; a client receives valid+invalid")
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
        "gate":                test_gate(),
        "mock reader":         test_mock_reader(),
        "publish round-trip":  test_publish_roundtrip(),
    }
    print("\n" + "=" * 64)
    all_ok = all(results.values())
    for name, ok in results.items():
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if all_ok:
        print("\nPASS -- lidar mock: gate (incl. range_status != 0 -> valid=0), "
              "mm->m conversion, and run_lidar publishes WireRange round-tripping "
              "the lidar.range contract.")
        return 0
    print("\nFAIL -- see the [FAIL] lines above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
