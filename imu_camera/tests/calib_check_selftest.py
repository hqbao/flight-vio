#!/usr/bin/env python3
"""Self-test for the calibration check gate (:mod:`imu_camera.tools.calib_check`).

Two halves, both run offline against the gold sessions on disk (read-only) plus
synthetic faults built entirely IN MEMORY:

1. GOOD calib -- run the full check suite on a real gold session and assert it
   produces **no FAIL** and exit code 0. This is the false-positive guard: a
   known-good OAK-D W calib must sail through clean (the tool's thresholds are
   tuned so it does -- the only non-PASS rows are INFO for the absent imu_noise).

2. BROKEN calibs -- start from the real gold ``StereoCalib`` and inject ONE fault
   at a time (deep-copied, never touching disk), asserting the RIGHT check FAILs
   and the gate returns a nonzero exit:
     (a) non-orthonormal stereo rotation,
     (b) absurd baseline (7.5 m -- the cm-not-converted bug),
     (c) left/right size mismatch,
     (d) K inconsistent with fx/fy/cx/cy,
     (e) NaN in a distortion coefficient.

Run::

    python -m imu_camera.tests.calib_check_selftest
    python -m imu_camera.tests.calib_check_selftest --session sessions/gold/lab_loop_30s
"""
from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from imu_camera.io.reader import SessionReader  # noqa: E402
from imu_camera.tools.calib_check import (  # noqa: E402
    FAIL,
    PASS,
    exit_code,
    run_checks,
)


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def _fails(results: list, name_substr: str) -> bool:
    """True iff at least one result whose check name contains ``name_substr`` FAILs."""
    return any(r.status == FAIL and name_substr in r.name for r in results)


def _statuses(results: list) -> str:
    """Compact 'name=STATUS' dump of the FAIL rows, for diagnostics on assert miss."""
    return ", ".join(f"{r.name}={r.status}" for r in results if r.status == FAIL) or "<none>"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s",
                    help="gold session whose (valid) calib seeds the good + broken cases")
    args = ap.parse_args()

    print("calib_check_selftest")
    reader = SessionReader(Path(args.session))
    good = reader.calib

    # ----- 1. GOOD calib: no FAIL, exit 0 (with recorded-data checks). ----- #
    print("[good calib]")
    results = run_checks(good, reader)
    n_fail = sum(1 for r in results if r.status == FAIL)
    n_warn = sum(1 for r in results if r.status == "WARN")
    n_pass = sum(1 for r in results if r.status == PASS)
    _check(n_fail == 0, f"real gold calib has no FAIL (fails: {_statuses(results)})")
    _check(exit_code(results, strict=False) == 0, "exit code 0 (non-strict)")
    # A known-good calib should also be WARN-free so it passes a --strict gate; if
    # this ever trips, a threshold needs widening (reported, not silently tolerated).
    _check(n_warn == 0,
           f"real gold calib is WARN-free (strict-clean); {n_warn} warn / {n_pass} pass")
    _check(exit_code(results, strict=True) == 0, "exit code 0 even under --strict")

    # ----- 2. BROKEN calibs (in-memory faults; disk untouched). ----- #
    # (a) non-orthonormal stereo rotation.
    print("[broken: non-orthonormal stereo R]")
    bad = copy.deepcopy(good)
    bad.T_left_right[:3, :3] = np.array([[1.0, 0.5, 0.0],
                                         [0.0, 1.0, 0.0],
                                         [0.0, 0.0, 1.0]])  # shear, not a rotation
    res = run_checks(bad)
    _check(_fails(res, "stereo R orthonormal"), "non-orthonormal stereo R -> FAIL")
    _check(exit_code(res, strict=False) != 0, "nonzero exit")

    # (b) absurd baseline (7.5 m -- raw cm leaked through, never converted to m).
    print("[broken: absurd 7.5 m baseline]")
    bad = copy.deepcopy(good)
    bad.T_left_right[:3, 3] = np.array([7.5, 0.0, 0.0])
    res = run_checks(bad)
    _check(_fails(res, "stereo baseline"), "7.5 m baseline -> FAIL")
    _check(exit_code(res, strict=False) != 0, "nonzero exit")

    # (c) left/right image-size mismatch.
    print("[broken: L/R size mismatch]")
    bad = copy.deepcopy(good)
    bad.right.width = bad.left.width + 80  # right wider than left
    res = run_checks(bad)
    _check(_fails(res, "L/R size equal"), "L/R size mismatch -> FAIL")
    _check(exit_code(res, strict=False) != 0, "nonzero exit")

    # (d) K inconsistent with fx/fy/cx/cy. The reader builds K from the scalar
    #     fields, so to desync them we corrupt the scalar AFTER nothing rebuilds K --
    #     instead we mutate a scalar and compare against a stale captured K. The
    #     check compares cam.K (recomputed) against fx/fy/cx/cy, so to force a
    #     mismatch we monkeypatch the K property's backing via a subclass override.
    print("[broken: K inconsistent with fx/fy/cx/cy]")
    bad = copy.deepcopy(good)
    stale_K = bad.left.K.copy()
    # Shift cx so the scalars no longer match the (now-stale) K we pin onto the cam.

    class _PinnedK(type(bad.left)):  # dataclass subclass returning a fixed K
        @property
        def K(self):  # noqa: D401 - overrides CameraCalib.K with a frozen matrix
            return stale_K

    pinned = _PinnedK(**{f.name: getattr(bad.left, f.name)
                         for f in bad.left.__dataclass_fields__.values()})
    pinned.cx = bad.left.cx + 25.0  # scalars now disagree with the pinned stale K
    bad.left = pinned
    res = run_checks(bad)
    _check(_fails(res, "K matches"), "K inconsistent with fx/fy/cx/cy -> FAIL")
    _check(exit_code(res, strict=False) != 0, "nonzero exit")

    # (e) NaN in a distortion coefficient.
    print("[broken: NaN in dist]")
    bad = copy.deepcopy(good)
    bad.left.dist = bad.left.dist.copy()
    bad.left.dist[0] = np.nan
    res = run_checks(bad)
    _check(_fails(res, "dist coeffs"), "NaN dist coeff -> FAIL")
    _check(exit_code(res, strict=False) != 0, "nonzero exit")

    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
