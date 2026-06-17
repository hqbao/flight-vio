#!/usr/bin/env python3
"""Self-test: the launcher FORWARDS ``--stabilize-velocity`` to the BA subprocess
argv ONLY when ``--tight`` AND ``--stabilize-velocity`` are both set, and NEVER on
the loose path (so the default end-to-end run -- and the offline oracle -- stay
byte-identical).

The windowed-BA backend (incl. the tight ``WindowedVIOMap``) lives in the ``ba``
process now, so the Phase-4 velocity-stabilize knob routes to ``ba.main`` via
:func:`launcher.main.build_ba_args` -- NOT to ``vio.main`` (it was inert on VIO once
the backend left). This test asserts the BA argv (and that VIO never carries the
flag any more).

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.build_ba_args` + :func:`launcher.main.build_vio_args` builders
with synthetic namespaces, and confirms the real launcher argparser registers the
flag via ``-m launcher.main --help`` (so a typo'd action= / dest= is caught).

Asserts:
  (a) --tight + --stabilize-velocity SET    -> ``--stabilize-velocity`` IS in BA argv,
  (b) --tight only (no stabilize)           -> ``--stabilize-velocity`` NOT in BA argv,
  (c) --stabilize-velocity WITHOUT --tight  -> NOT forwarded (loose has no vel state),
  (d) neither flag                          -> NOT in BA argv (default OFF),
  (e) the flag NEVER appears in the VIO argv (the backend left VIO),
  (f) the launcher CLI ``--help`` lists ``--stabilize-velocity`` (parser registered).

Run::

    .venv/bin/python -m launcher.tests.stabilize_velocity_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import build_ba_args, build_vio_args            # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    """A launcher-args namespace with sane defaults, overridable per test."""
    base = dict(kf_every=5, no_gyro=False, worker=False,
                tight=False, stabilize_velocity=False, depth_icp=False,
                ba_window=False, backend_window=6, backend_iters=5,
                frontend_viz=False, direct=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_forwarding() -> None:
    vio, ba = "oak.vio", "oak.ba"

    # (a) --tight + --stabilize-velocity SET -> forwarded to the BA argv.
    argv = build_ba_args(_ns(tight=True, stabilize_velocity=True), vio, ba)
    assert "--tight" in argv, argv
    assert "--stabilize-velocity" in argv, argv
    print("[a] --tight + --stabilize-velocity SET -> forwarded to ba argv         OK")

    # (b) --tight only -> stabilize NOT forwarded (tight default = oracle-tuned).
    argv = build_ba_args(_ns(tight=True, stabilize_velocity=False), vio, ba)
    assert "--tight" in argv, argv
    assert "--stabilize-velocity" not in argv, argv
    print("[b] --tight only (no stabilize)        -> NOT in ba argv                OK")

    # (c) --stabilize-velocity WITHOUT --tight -> dropped (loose has no vel state).
    argv = build_ba_args(_ns(tight=False, stabilize_velocity=True), vio, ba)
    assert "--tight" not in argv, argv
    assert "--stabilize-velocity" not in argv, argv
    print("[c] --stabilize-velocity WITHOUT --tight -> NOT forwarded (warned)      OK")

    # (d) neither flag -> default OFF end-to-end.
    argv = build_ba_args(_ns(), vio, ba)
    assert "--stabilize-velocity" not in argv, argv
    print("[d] neither flag                       -> NOT in ba argv (default OFF)  OK")


def test_not_in_vio_argv() -> None:
    """The backend left VIO -> --stabilize-velocity must NEVER reach the vio argv."""
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"
    argv = build_vio_args(_ns(tight=True, stabilize_velocity=True),
                          cap, vio, slam, use_worker=False)
    assert "--tight" in argv, argv
    assert "--stabilize-velocity" not in argv, argv
    print("[e] --tight + --stabilize-velocity     -> NOT in vio argv (left VIO)    OK")


def test_help_registers_flag() -> None:
    """The real launcher parser must register the flag (catches action=/dest= typos)."""
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--stabilize-velocity" in out.stdout, out.stdout
    print("[f] launcher --help lists --stabilize-velocity                          OK")


if __name__ == "__main__":
    test_forwarding()
    test_not_in_vio_argv()
    test_help_registers_flag()
    print("\nstabilize_velocity_forward_selftest: ALL PASS")
