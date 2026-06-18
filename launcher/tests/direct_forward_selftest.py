#!/usr/bin/env python3
"""Self-test: the launcher FORWARDS ``--direct`` to the VIO subprocess argv ONLY
when ``--direct`` is set, and NEVER on the default (loose/tight) path -- so the
default end-to-end run + the offline byte-parity oracle stay gap=0 byte-identical.

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.build_vio_args` builder with synthetic namespaces, and confirms
the real launcher argparser registers the flag via ``-m launcher.main --help`` (so a
typo'd action= / dest= is caught). Mirrors ``stabilize_velocity_forward_selftest``.

Unlike the tight-only flags, ``--direct`` is INDEPENDENT of ``--tight`` (the direct
front-end owns its own IMU seed), so it forwards on its own AND alongside --tight.

Asserts:
  (a) --direct SET                 -> ``--direct`` IS in vio argv,
  (b) --direct + --tight SET       -> BOTH ``--direct`` AND ``--tight`` in vio argv,
  (c) no flag (default)            -> ``--direct`` NOT in vio argv (default OFF),
  (d) --tight only (no --direct)   -> ``--direct`` NOT in vio argv,
  (e) the launcher CLI ``--help`` lists ``--direct`` (parser registered).

Run::

    .venv/bin/python -m launcher.tests.direct_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import build_vio_args                       # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    """A launcher-args namespace with sane defaults, overridable per test."""
    base = dict(kf_every=5, no_gyro=False,
                tight=False, stabilize_velocity=False, depth_icp=False,
                ba_window=False, frontend_viz=False, direct=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_forwarding() -> None:
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"

    # (a) --direct SET -> forwarded.
    argv = build_vio_args(_ns(direct=True), cap, vio, slam)
    assert "--direct" in argv, argv
    assert "--tight" not in argv, argv
    print("[a] --direct SET                 -> forwarded to vio argv               OK")

    # (b) --direct + --tight SET -> both forwarded (direct is tight-independent).
    argv = build_vio_args(_ns(direct=True, tight=True),
                          cap, vio, slam)
    assert "--direct" in argv, argv
    assert "--tight" in argv, argv
    print("[b] --direct + --tight SET       -> BOTH forwarded (independent)        OK")

    # (c) no flag -> default OFF end-to-end (the oracle path).
    argv = build_vio_args(_ns(), cap, vio, slam)
    assert "--direct" not in argv, argv
    print("[c] no flag (default)            -> NOT in vio argv (default OFF)       OK")

    # (d) --tight only -> --direct NOT forwarded (only --direct turns it on).
    argv = build_vio_args(_ns(tight=True), cap, vio, slam)
    assert "--direct" not in argv, argv
    assert "--tight" in argv, argv
    print("[d] --tight only (no --direct)   -> --direct NOT in vio argv            OK")


def test_help_registers_flag() -> None:
    """The real launcher parser must register the flag (catches action=/dest= typos)."""
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--direct" in out.stdout, out.stdout
    print("[e] launcher --help lists --direct                                      OK")


if __name__ == "__main__":
    test_forwarding()
    test_help_registers_flag()
    print("\ndirect_forward_selftest: ALL PASS")
