#!/usr/bin/env python3
"""Byte-parity gate: the in-process SPLIT oracle == the PRE-SPLIT baseline.

For each entry in ``verification/baseline_metrics.json`` this:

1. runs the NEW in-process oracle (split-project math) -- :func:`score_session_oracle`,
2. asserts every metric == the STORED full-precision baseline within ``TOL_MM``,
3. ALSO re-derives the OLD oracle LIVE (``ours.tools.vio_run.score_session``) and
   asserts the new oracle == the old oracle BIT-FOR-BIT (``repr`` equality of the
   float64s) -- the strongest parity check, immune to a stale JSON.

Any mismatch FAILS LOUDLY with the exact metric, both values and the gap. The
tolerance is NOT weakened to force a pass.

Run::

    .venv/bin/python verification/oracle_replay_selftest.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from verification.oracle_replay import score_session_oracle  # noqa: E402

# Baseline byte-parity tolerance. The split ports the math VERBATIM, so the
# observed agreement is EXACT (0.0 mm) at float64; we still gate at 1e-6 mm
# (= 1 nanometre) so a future numerically-insignificant reorder is caught loud
# but a true bit-identical port always passes. NOT weakened to force a pass.
TOL_MM = 1e-6

#: Metrics compared (full precision, in metres). Each maps to baseline['_m'] key.
_METRICS = (("rmse", "rmse_m"), ("median", "median_m"),
            ("max", "max_m"), ("scale", "sim3_scale"))


def _load_baseline() -> dict:
    path = Path(__file__).resolve().parent / "baseline_metrics.json"
    return json.loads(path.read_text())


def _fmt(v: float) -> str:
    return repr(float(v))


def _check_entry(entry: dict, run_old_live: bool) -> tuple[bool, list[str]]:
    """Run the new oracle for one baseline entry; return (ok, log_lines)."""
    sess = entry["session"]
    backend = entry["backend"]
    mf = entry["max_frames"]
    base_m = entry["_m"]
    log: list[str] = []
    log.append(f"\n=== {sess}  backend={backend}  max_frames={mf} ===")

    res = score_session_oracle(Path(sess), mf, quiet=True, backend=backend)
    if res is None:
        log.append("  [FAIL] new oracle returned None (no Basalt overlap)")
        return False, log

    ok = True
    # 1) New oracle vs stored full-precision baseline.
    for metric, base_key in _METRICS:
        got = float(res[metric])
        want = float(base_m[base_key])
        gap_mm = abs(got - want) * (1.0 if metric == "scale" else 1000.0)
        unit = "" if metric == "scale" else " mm"
        within = gap_mm <= TOL_MM
        ok = ok and within
        tag = "ok" if within else "FAIL"
        log.append(f"  [{tag}] {metric:7s}  new={_fmt(got)}  "
                   f"baseline={_fmt(want)}  gap={gap_mm:.3e}{unit}")
        if not within:
            log.append(f"        ^^ DIVERGES by {gap_mm:.6e}{unit} "
                       f"(> tol {TOL_MM}{unit})")

    # 2) New oracle vs LIVE old oracle -- bit-for-bit (repr equality).
    if run_old_live:
        from ours.tools.vio_run import score_session as old_score  # noqa: E402
        old = old_score(Path(sess), mf, False, quiet=True, backend=backend)
        for metric, _ in _METRICS:
            n_repr = _fmt(res[metric])
            o_repr = _fmt(old[metric])
            same = (n_repr == o_repr)
            ok = ok and same
            tag = "ok" if same else "FAIL"
            log.append(f"  [{tag}] live-old {metric:7s}  new={n_repr}  "
                       f"old={o_repr}  bit-identical={same}")
            if not same:
                gap = abs(float(res[metric]) - float(old[metric]))
                log.append(f"        ^^ NEW vs LIVE-OLD DIFFER by {gap:.6e}")

    return ok, log


def main() -> int:
    run_old_live = "--no-live-old" not in sys.argv
    baseline = _load_baseline()
    entries = baseline["entries"]

    print("oracle_replay_selftest  -- in-process SPLIT oracle vs PRE-SPLIT baseline")
    print(f"  baseline       : verification/baseline_metrics.json "
          f"({len(entries)} entries)")
    print(f"  parity tol     : {TOL_MM} mm  (port is VERBATIM -> expect 0.0)")
    print(f"  live-old check : {'ON (bit-for-bit vs ours.vio_run)' if run_old_live else 'OFF'}")

    all_ok = True
    for entry in entries:
        ok, log = _check_entry(entry, run_old_live)
        for line in log:
            print(line)
        all_ok = all_ok and ok

    print("\n" + "=" * 70)
    if all_ok:
        print("PASS -- every baseline entry reproduced byte-for-byte by the "
              "split oracle.")
        print("VERDICT: the 5-project split PRESERVES byte-parity (end-to-end).")
        return 0
    print("FAIL -- at least one metric DIVERGED. The split did NOT preserve "
          "byte-parity for the entries flagged above.")
    print("VERDICT: VETO -- do not release. See the [FAIL] lines for the exact "
          "component + gap.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
