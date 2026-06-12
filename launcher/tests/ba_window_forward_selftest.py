#!/usr/bin/env python3
"""Self-test: the BA-Window capture defaults ON when the UI runs on the loose
path (it is a UI tool, so it "just works" without a flag), and OFF when headless
(lean flight path) or under ``--tight``. ``--ba-window`` forces it on (e.g.
headless for the PNG smoke); ``--no-ba-window`` forces it off.

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.resolve_ba_window` resolver with synthetic namespaces, and
confirms the launcher argparser registers both flags via ``--help``. Mirrors
``depth_icp_forward_selftest``.

The capture runs the SAME frozen ``run_ba``, so this default is oracle-safe (the
offline ``oracle_replay_selftest`` never goes through this launcher path).

Run::

    .venv/bin/python -m launcher.tests.ba_window_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import resolve_ba_window                      # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    base = dict(ba_window=False, no_ba_window=False, no_ui=False, tight=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_resolution() -> None:
    # (a) default with UI on the loose path -> ON (the "just works" case).
    assert resolve_ba_window(_ns()) is True
    print("[a] default (UI, loose)            -> ON  (no flag needed)            OK")

    # (b) headless (flight path) -> OFF (no UI to consume; keep it lean).
    assert resolve_ba_window(_ns(no_ui=True)) is False
    print("[b] --no-ui (headless / flight)    -> OFF (lean path)                 OK")

    # (c) tight path -> OFF (there is no loose BA window there).
    assert resolve_ba_window(_ns(tight=True)) is False
    print("[c] --tight (no loose BA window)   -> OFF                             OK")

    # (d) --ba-window forces it on even headless (e.g. the PNG smoke).
    assert resolve_ba_window(_ns(ba_window=True, no_ui=True)) is True
    print("[d] --ba-window --no-ui (force on) -> ON                              OK")

    # (e) --no-ba-window forces it off even with the UI shown.
    assert resolve_ba_window(_ns(no_ba_window=True)) is False
    print("[e] --no-ba-window (with UI)       -> OFF                             OK")

    # (f) --no-ba-window beats an explicit --ba-window.
    assert resolve_ba_window(_ns(ba_window=True, no_ba_window=True)) is False
    print("[f] --no-ba-window beats --ba-window                                  OK")


def test_help_registers_flags() -> None:
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--ba-window" in out.stdout, out.stdout
    assert "--no-ba-window" in out.stdout, out.stdout
    print("[g] launcher --help lists --ba-window + --no-ba-window                OK")


if __name__ == "__main__":
    test_resolution()
    test_help_registers_flags()
    print("\nba_window_forward_selftest: ALL PASS")
