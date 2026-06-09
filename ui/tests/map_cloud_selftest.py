#!/usr/bin/env python3
"""Unit tests for :func:`ui.viz.map_cloud.longest_consecutive_run`.

The SLAM 3D-map viewer keeps a LANDMARK (track id) only if it was a PnP inlier
across ``>= PERSIST_KF`` SUCCESSIVE keyframes. The single primitive that gate
needs is the longest run of consecutive keyframe indices a landmark appeared in.
These tests feed hand-checkable index sets and assert the run length matches the
spec (and that the >= PERSIST_KF keep/drop decision falls out correctly).

Run::

    python -m ui.tests.map_cloud_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ui.viz.map_cloud import longest_consecutive_run                  # noqa: E402


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def main() -> int:
    print("map_cloud_selftest: longest_consecutive_run")
    persist = 20                                  # the viewer's PERSIST_KF gate

    # --- Spec cases -------------------------------------------------------- #
    # {0..24} -> run 25 -> KEPT at 20.
    run_a = longest_consecutive_run(list(range(0, 25)))
    _check(run_a == 25, f"{{0..24}} -> 25 (got {run_a})")
    _check(run_a >= persist, "{0..24} run 25 KEPT at PERSIST_KF=20")

    # {0, 5, 40} -> run 1 (none adjacent) -> DROPPED at 20.
    run_b = longest_consecutive_run([0, 5, 40])
    _check(run_b == 1, f"{{0,5,40}} -> 1 (got {run_b})")
    _check(run_b < persist, "{0,5,40} run 1 DROPPED at PERSIST_KF=20")

    # {0..18} -> run 19 (just short) -> DROPPED at 20, but KEPT at 19.
    run_c = longest_consecutive_run(list(range(0, 19)))
    _check(run_c == 19, f"{{0..18}} -> 19 (got {run_c})")
    _check(run_c < persist, "{0..18} run 19 DROPPED at PERSIST_KF=20")
    _check(run_c >= 19, "{0..18} run 19 KEPT at PERSIST_KF=19 (off-by-one guard)")

    # --- A run BROKEN by a gap counts only the longest sub-run. ------------ #
    # {0..9, 20..29}: two runs of 10 -> longest 10 (dropped at 20, kept at 10).
    run_d = longest_consecutive_run(list(range(0, 10)) + list(range(20, 30)))
    _check(run_d == 10, f"{{0..9,20..29}} -> 10 longest sub-run (got {run_d})")
    _check(run_d < persist and run_d >= 10,
           "gap-broken two-runs-of-10 dropped at 20, kept at 10")

    # --- Run not anchored at 0 (offset window still counts). --------------- #
    run_e = longest_consecutive_run(list(range(12, 32)))   # 12..31 inclusive
    _check(run_e == 20, f"{{12..31}} -> 20 (offset window; got {run_e})")

    # --- A single index -> run 1; empty -> 0; trailing run wins. ----------- #
    _check(longest_consecutive_run([7]) == 1, "single index -> run 1")
    _check(longest_consecutive_run([]) == 0, "empty -> run 0")
    # Longest run at the END (not the start) must still be found.
    run_f = longest_consecutive_run([0, 2, 5, 6, 7, 8])    # tail run 5..8 = 4
    _check(run_f == 4, f"trailing run {{5..8}} -> 4 (got {run_f})")

    # --- Caller passes a SORTED UNIQUE seq (dedup is the caller's job): the
    #     source feeds ``sorted(set(...))`` so a keyframe seen twice counts once.
    run_g = longest_consecutive_run(sorted({5, 5, 5, 6, 7}))   # {5,6,7} -> 3
    _check(run_g == 3, f"deduped {{5,5,5,6,7}} -> {{5,6,7}} -> 3 (got {run_g})")

    print("\nALL MAP_CLOUD SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
