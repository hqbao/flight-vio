#!/usr/bin/env python3
"""Self-test: the BA-Window capture defaults ON when the UI runs on the loose
path (it is a UI tool, so it "just works" without a flag), and OFF when headless
(lean flight path) or under ``--tight``. ``--ba-window`` forces it on (e.g.
headless for the PNG smoke); ``--no-ba-window`` forces it off.

The BA-window visualiser is PUBLISHED by the ``ba`` process now (the windowed-BA
backend moved there), so on top of the :func:`launcher.main.resolve_ba_window`
resolver this test also asserts the RESOLVED state forwards as ``--ba-window`` to
the BA subprocess via :func:`launcher.main.build_ba_args` -- and NOT to ``vio.main``
(VIO only bridges ``ba.window`` back via ``--ba-endpoint``). The loose windowed-BA
solve-size knobs ``--backend-window`` / ``--backend-iters`` likewise route to
``ba.main`` (forwarded only when non-default).

Fully OFFLINE (no spawning, no device): exercises the pure resolver + builder with
synthetic namespaces, and confirms the launcher argparser registers the flags via
``--help``.

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

from launcher.main import (                                         # noqa: E402
    build_ba_args, build_vio_args, resolve_ba_window,
)


def _ns(**over) -> types.SimpleNamespace:
    base = dict(ba_window=False, no_ba_window=False, no_ui=False, tight=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def _ba_ns(**over) -> types.SimpleNamespace:
    """A fuller namespace (the build_*_args builders read more fields)."""
    base = dict(kf_every=5, no_gyro=False, tight=False,
                stabilize_velocity=False, depth_icp=False, ba_window=False,
                backend_window=6, backend_iters=5, frontend_viz=False,
                direct=False)
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


def test_forwards_to_ba_not_vio() -> None:
    """The RESOLVED ba_window state forwards as --ba-window to ba.main, never vio."""
    cap, vio, slam, ba = "oak.capture", "oak.vio", "oak.slam", "oak.ba"

    # Resolved ON (loose + forced) -> build_ba_args carries --ba-window.
    argv = build_ba_args(_ba_ns(ba_window=True), vio, ba)
    assert "--ba-window" in argv, argv
    print("[g] resolved ON  -> '--ba-window' in ba argv                          OK")

    # Resolved OFF -> not forwarded.
    argv = build_ba_args(_ba_ns(ba_window=False), vio, ba)
    assert "--ba-window" not in argv, argv
    print("[h] resolved OFF -> '--ba-window' NOT in ba argv                      OK")

    # The backend left VIO -> --ba-window must NEVER reach the vio argv (VIO only
    # bridges ba.window back via --ba-endpoint, it does not produce it).
    argv = build_vio_args(_ba_ns(ba_window=True), cap, vio, slam)
    assert "--ba-window" not in argv, argv
    print("[i] resolved ON  -> '--ba-window' NOT in vio argv (left VIO)          OK")


def test_backend_size_forward() -> None:
    """--backend-window / --backend-iters route to ba.main only when non-default."""
    vio, ba = "oak.vio", "oak.ba"

    # Defaults (6 / 5) -> argv stays minimal (not forwarded).
    argv = build_ba_args(_ba_ns(), vio, ba)
    assert "--backend-window" not in argv and "--backend-iters" not in argv, argv
    print("[j] defaults (6/5)  -> backend-window/iters NOT in ba argv            OK")

    # Overridden -> both forwarded with their values.
    argv = build_ba_args(_ba_ns(backend_window=8, backend_iters=10), vio, ba)
    assert "--backend-window" in argv and "8" in argv, argv
    assert "--backend-iters" in argv and "10" in argv, argv
    print("[k] overridden (8/10) -> backend-window 8 / backend-iters 10 in ba    OK")


def test_help_registers_flags() -> None:
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--ba-window" in out.stdout, out.stdout
    assert "--no-ba-window" in out.stdout, out.stdout
    assert "--backend-window" in out.stdout, out.stdout
    assert "--backend-iters" in out.stdout, out.stdout
    print("[l] launcher --help lists --ba-window/--no-ba-window/--backend-*      OK")


if __name__ == "__main__":
    test_resolution()
    test_forwards_to_ba_not_vio()
    test_backend_size_forward()
    test_help_registers_flags()
    print("\nba_window_forward_selftest: ALL PASS")
