#!/usr/bin/env python3
"""Self-test: the launcher FORWARDS ``--depth-icp`` to the BA subprocess argv ONLY
when ``--tight`` AND ``--depth-icp`` are both set, and NEVER on the loose path (so
the default end-to-end run -- and the offline oracle -- stay byte-identical).

The windowed-BA backend (incl. the tight ``WindowedVIOMap`` that owns the dense-ICP
relative-pose factor) lives in the ``ba`` process now, so this knob routes to
``ba.main`` via :func:`launcher.main.build_ba_args` -- NOT to ``vio.main`` (it was
inert on VIO once the backend left). This test asserts the BA argv (and that VIO
never carries the flag any more).

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.build_ba_args` + :func:`launcher.main.build_vio_args` builders
with synthetic namespaces, and confirms the launcher argparser registers the flag
via ``-m launcher.main --help``. Mirrors ``stabilize_velocity_forward_selftest``.

Asserts:
  (a) --tight + --depth-icp SET    -> ``--depth-icp`` IS in BA argv,
  (b) --tight only (no depth-icp)  -> ``--depth-icp`` NOT in BA argv,
  (c) --depth-icp WITHOUT --tight  -> NOT forwarded (loose has no factor graph),
  (d) neither flag                 -> NOT in BA argv (default OFF),
  (e) the flag NEVER appears in the VIO argv (the backend left VIO),
  (f) the launcher CLI ``--help`` lists ``--depth-icp`` (parser registered).

Run::

    .venv/bin/python -m launcher.tests.depth_icp_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import build_ba_args, build_vio_args            # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    base = dict(kf_every=5, no_gyro=False,
                tight=False, stabilize_velocity=False, depth_icp=False,
                ba_window=False, backend_window=6, backend_iters=5,
                frontend_viz=False, direct=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_forwarding() -> None:
    vio, ba = "oak.vio", "oak.ba"

    # (a) --tight + --depth-icp SET -> forwarded to the BA argv.
    argv = build_ba_args(_ns(tight=True, depth_icp=True), vio, ba)
    assert "--tight" in argv, argv
    assert "--depth-icp" in argv, argv
    print("[a] --tight + --depth-icp SET -> forwarded to ba argv                OK")

    # (b) --tight only -> depth-icp NOT forwarded (tight default = oracle-tuned).
    argv = build_ba_args(_ns(tight=True, depth_icp=False), vio, ba)
    assert "--depth-icp" not in argv, argv
    print("[b] --tight only (no depth-icp)        -> NOT in ba argv             OK")

    # (c) --depth-icp WITHOUT --tight -> dropped (loose has no factor graph).
    argv = build_ba_args(_ns(tight=False, depth_icp=True), vio, ba)
    assert "--tight" not in argv, argv
    assert "--depth-icp" not in argv, argv
    print("[c] --depth-icp WITHOUT --tight        -> NOT forwarded (warned)      OK")

    # (d) neither flag -> default OFF end-to-end.
    argv = build_ba_args(_ns(), vio, ba)
    assert "--depth-icp" not in argv, argv
    print("[d] neither flag                       -> NOT in ba argv (default)   OK")


def test_not_in_vio_argv() -> None:
    """The backend left VIO -> --depth-icp must NEVER reach the vio argv."""
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"
    argv = build_vio_args(_ns(tight=True, depth_icp=True),
                          cap, vio, slam)
    assert "--tight" in argv, argv
    assert "--depth-icp" not in argv, argv
    print("[e] --tight + --depth-icp              -> NOT in vio argv (left VIO)  OK")


def test_help_registers_flag() -> None:
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--depth-icp" in out.stdout, out.stdout
    print("[f] launcher --help lists --depth-icp                                 OK")


if __name__ == "__main__":
    test_forwarding()
    test_not_in_vio_argv()
    test_help_registers_flag()
    print("\ndepth_icp_forward_selftest: ALL PASS")
