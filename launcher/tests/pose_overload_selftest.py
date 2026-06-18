#!/usr/bin/env python3
"""Self-test: the launcher's pose-rate OVERLOAD watchdog (`_pose_overload_rate`).

When the chosen resolution is too heavy for the box (e.g. 640x400 on a 4-core
Pi), nothing ERRORS -- the pipeline just can't keep real-time, the `pose.odom`
rate collapses, and the (remote) UI looks FROZEN with nothing in the log. The
launcher's `_start_pose_logger` watches the end-to-end pose rate vs `--fps` and
logs a LOUD "lower the resolution" warning when it stays low. The rate logic is
factored into the pure `_pose_overload_rate(st, now, n, target_fps)` so it is
testable WITHOUT a live device (the live path only runs on real hardware).

Drives the helper with synthetic (n, now) window sequences and asserts:
  (a) keeping up (rate >= 60% target) never warns,
  (b) a SUSTAINED slow rate warns after `_OVL_HOLD` windows, with the measured Hz,
  (c) a brief slow window that recovers does NOT warn (the run resets),
  (d) after a warning it is THROTTLED (no second warning within `_OVL_THROTTLE`),
  (e) `target_fps == 0` (unknown) never warns.

Run::

    .venv/bin/python -m launcher.tests.pose_overload_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import (                                       # noqa: E402
    _pose_overload_rate, _OVL_FRAC, _OVL_WIN, _OVL_HOLD, _OVL_THROTTLE,
)

T0 = 10_000.0          # large so `now - warn_t` clears the throttle on the 1st warn


def _run(target_fps: int, hz_per_window):
    """Feed one pose-rate per `_OVL_WIN` window; return [warn-rate or None, ...]."""
    st = {"rate_t": 0.0, "rate_n": 0, "warn_t": 0.0, "low": 0}
    _pose_overload_rate(st, T0, 1, target_fps)          # first pose anchors the clock
    out, t, n = [], T0, 1
    for hz in hz_per_window:
        t += _OVL_WIN + 0.01                            # just past the window
        n += int(round(hz * _OVL_WIN))                  # poses that arrived in it
        out.append(_pose_overload_rate(st, t, n, target_fps))
    return out


def main() -> int:
    fps = 20                                            # threshold = 0.6*20 = 12 Hz
    fails = []

    # (a) keeping up -> never warns.
    keep = _run(fps, [20, 20, 20, 20])
    if any(r is not None for r in keep):
        fails.append(f"keeping-up warned: {keep}")
    print(f"[a] 20 Hz @ 20 target -> no warn            ({keep})")

    # (b) sustained slow -> warns after _OVL_HOLD windows, reporting ~8 Hz.
    slow = _run(fps, [8] * (_OVL_HOLD + 1))
    warned = [r for r in slow if r is not None]
    if not warned:
        fails.append(f"sustained-slow did NOT warn: {slow}")
    elif not (7.0 <= warned[0] <= 9.0):
        fails.append(f"warned rate {warned[0]:.1f} not ~8 Hz")
    # The warn must fire AT the _OVL_HOLD-th slow window (0-indexed _OVL_HOLD-1),
    # and NOT in any earlier window.
    if any(r is not None for r in slow[:_OVL_HOLD - 1]):
        fails.append(f"warned BEFORE _OVL_HOLD={_OVL_HOLD} windows: {slow}")
    if slow[_OVL_HOLD - 1] is None:
        fails.append(f"did NOT warn at the _OVL_HOLD-th window: {slow}")
    print(f"[b] 8 Hz @ 20 target -> warns ~{warned[0]:.1f} Hz after "
          f"{_OVL_HOLD} windows   ({slow})")

    # (c) brief slow then recover -> no warn (the low-run resets).
    blip = _run(fps, [8, 20, 20])
    if any(r is not None for r in blip):
        fails.append(f"a recovered blip warned: {blip}")
    print(f"[c] 8,20,20 Hz -> no warn (run resets)      ({blip})")

    # (d) throttle: many slow windows -> at most one warn per _OVL_THROTTLE s. The
    # windows are _OVL_WIN(=3 s) apart, _OVL_THROTTLE=8 s, so of 6 slow windows
    # only ~the 1st-qualifying + every ~3rd thereafter fire (never two adjacent).
    longslow = _run(fps, [8] * 6)
    idx = [i for i, r in enumerate(longslow) if r is not None]
    adjacent = any(b - a == 1 for a, b in zip(idx, idx[1:]))
    if adjacent:
        fails.append(f"throttle broken (adjacent warns): {idx}")
    print(f"[d] 6 slow windows -> warns at {idx}, none adjacent (throttled)")

    # (e) unknown target (0) -> never warns.
    none_fps = _run(0, [1, 1, 1])
    if any(r is not None for r in none_fps):
        fails.append(f"target_fps=0 warned: {none_fps}")
    print(f"[e] target_fps=0 -> no warn                 ({none_fps})")

    if fails:
        print("\nFAIL:")
        for f in fails:
            print(f"  - {f}")
        return 1
    print(f"\npose_overload_selftest: ALL PASS  "
          f"(frac={_OVL_FRAC} win={_OVL_WIN}s hold={_OVL_HOLD} "
          f"throttle={_OVL_THROTTLE}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
