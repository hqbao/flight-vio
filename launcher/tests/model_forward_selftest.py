#!/usr/bin/env python3
"""Self-test: the launcher PARSES ``--model`` and FORWARDS it to the capture
subprocess argv only in the LIVE branch, only when set.

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.build_capture_args` builder with synthetic namespaces, and
confirms the real launcher argparser registers the flag via ``-m launcher.main
--help``. Mirrors ``use_camera_calib_forward_selftest`` -- ``--model`` is a
live-only device selector (replay has no device to choose), forwarded verbatim so
capture opens the operator-named OAK when several are connected.

Asserts:
  (a) live + value SET   -> ``--model VALUE`` IS in the capture argv (as a pair),
  (b) live + value UNSET -> ``--model`` is NOT in the capture argv,
  (c) REPLAY + value SET -> NOT forwarded (live-only),
  (d) the launcher CLI ``--help`` lists ``--model`` (parser registered).

Run::

    .venv/bin/python -m launcher.tests.model_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import build_capture_args                     # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    """A launcher-args namespace with sane defaults, overridable per test."""
    base = dict(width=640, height=400, fps=20, session=None, max_frames=0,
                no_gyro=False, recalibrate_bias=False, use_camera_calib=False,
                vl53l9cx=False, model=None)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_forwarding() -> None:
    cap = "oak.capture"

    # (a) live + value SET -> forwarded as a ["--model", VALUE] pair.
    argv = build_capture_args(_ns(model="lite"), cap)
    assert "--live" in argv, argv
    assert "--model" in argv and argv[argv.index("--model") + 1] == "lite", argv
    print("[a] live + --model lite SET   -> forwarded to capture argv           OK")

    # (b) live + value UNSET -> not forwarded.
    argv = build_capture_args(_ns(model=None), cap)
    assert "--live" in argv, argv
    assert "--model" not in argv, argv
    print("[b] live + --model UNSET      -> NOT in capture argv                 OK")

    # (c) replay + value SET -> NOT forwarded (live-only; replay has no device).
    argv = build_capture_args(
        _ns(session="sessions/gold/lab_loop_30s", model="lite"), cap)
    assert "--session" in argv and "--live" not in argv, argv
    assert "--model" not in argv, argv
    print("[c] replay + --model SET      -> NOT forwarded (live-only)           OK")


def test_parser_registers_flag() -> None:
    # (d) the real launcher CLI lists the flag in --help, so argparse registered it.
    repo = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=repo, capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert "--model" in out.stdout, out.stdout
    print("[d] launcher --help lists --model (parser registered)                OK")


def main() -> int:
    test_forwarding()
    test_parser_registers_flag()
    print("\nALL launcher --model FORWARDING CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
