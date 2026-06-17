#!/usr/bin/env python3
"""Self-test: the launcher wires the LEAN-flight SPAWN GATES correctly.

  --no-ba    -> the BA process is a launcher SPAWN gate now (the windowed-BA backend
                moved to ``ba``): when ba IS spawned VIO gets ``--ba-endpoint`` (the
                pass-through client); under --no-ba ba is NOT spawned so VIO gets NO
                ``--ba-endpoint`` (and the old ``--no-ba`` is no longer a vio flag),
  --no-slam  -> the ``--slam-endpoint`` is OMITTED on the --tight path (no SLAM process
                is spawned, so VIO never wires the loop.correction feedback),
  build_ba_args -> a pure consumer argv (--vio-endpoint / --endpoint [+ --tight]);
                ``--worker`` is NEVER forwarded (a no-op for ba).

Fully OFFLINE (no spawning, no device): exercises the pure
:func:`launcher.main.build_vio_args` + :func:`launcher.main.build_ba_args` builders
with synthetic namespaces, and confirms the real launcher argparser registers both
flags via ``-m launcher.main --help``. Mirrors ``stabilize_velocity_forward_selftest``.

Run::

    .venv/bin/python -m launcher.tests.no_ba_no_slam_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import build_ba_args, build_vio_args         # noqa: E402


def _ns(**over) -> types.SimpleNamespace:
    """A launcher-args namespace mirroring the real argparse defaults."""
    base = dict(kf_every=5, no_gyro=False, worker=False, tight=False,
                no_ba=False, no_slam=False, no_live_loop_correct=False,
                stabilize_velocity=False, depth_icp=False, ba_window=False,
                backend_window=6, backend_iters=5,
                frontend_viz=False, direct=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_no_ba() -> None:
    cap, vio, slam, ba = "oak.capture", "oak.vio", "oak.slam", "oak.ba"

    # (a) ba spawned (default, NOT --no-ba) -> VIO gets --ba-endpoint, NOT --no-ba.
    argv = build_vio_args(_ns(), cap, vio, slam, use_worker=False,
                          ba_ep=ba, ba_spawned=True)
    assert "--ba-endpoint" in argv and ba in argv, argv
    assert "--no-ba" not in argv, argv
    print("[a] ba spawned       -> '--ba-endpoint' in vio argv (no '--no-ba')    OK")

    # (b) --no-ba -> ba NOT spawned -> NO --ba-endpoint in the vio argv.
    argv = build_vio_args(_ns(no_ba=True), cap, vio, slam, use_worker=False,
                          ba_ep=ba, ba_spawned=False)
    assert "--ba-endpoint" not in argv, argv
    assert "--no-ba" not in argv, argv          # no longer a vio flag at all
    print("[b] --no-ba          -> NO '--ba-endpoint' in vio argv                OK")

    # (c) ba spawned on the TIGHT path -> --ba-endpoint + --tight both present.
    argv = build_vio_args(_ns(tight=True), cap, vio, slam, use_worker=False,
                          ba_ep=ba, ba_spawned=True)
    assert "--tight" in argv and "--ba-endpoint" in argv, argv
    print("[c] ba spawned+tight -> '--ba-endpoint' + '--tight' in vio argv       OK")


def test_build_ba_args() -> None:
    vio, ba = "oak.vio", "oak.ba"

    # (d) loose ba -> --vio-endpoint / --endpoint only, NO --tight, NO --worker.
    argv = build_ba_args(_ns(), vio, ba)
    assert argv == ["--vio-endpoint", vio, "--endpoint", ba], argv
    print("[d] build_ba_args (loose)  -> [--vio-endpoint, --endpoint] only        OK")

    # (e) tight ba -> --tight appended; still NO --worker (a no-op for ba).
    argv = build_ba_args(_ns(tight=True, worker=True), vio, ba)
    assert "--tight" in argv and "--worker" not in argv, argv
    print("[e] build_ba_args (tight)  -> '--tight' added, '--worker' never        OK")


def test_no_slam() -> None:
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"

    # (f) --tight WITHOUT --no-slam -> slam endpoint wired (loop.correction feedback).
    argv = build_vio_args(_ns(tight=True), cap, vio, slam, use_worker=False)
    assert "--slam-endpoint" in argv, argv
    print("[f] --tight (no --no-slam) -> '--slam-endpoint' wired                  OK")

    # (g) --tight + --no-slam -> slam endpoint OMITTED (no SLAM process to subscribe).
    argv = build_vio_args(_ns(tight=True, no_slam=True), cap, vio, slam, use_worker=False)
    assert "--tight" in argv and "--slam-endpoint" not in argv, argv
    print("[g] --tight + --no-slam    -> '--slam-endpoint' OMITTED                OK")


def test_help_registers_flags() -> None:
    """The real launcher parser must register both flags (catches action=/dest= typos)."""
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--no-ba" in out.stdout and "--no-slam" in out.stdout, out.stdout
    print("[h] launcher --help lists --no-ba + --no-slam                          OK")


if __name__ == "__main__":
    test_no_ba()
    test_build_ba_args()
    test_no_slam()
    test_help_registers_flags()
    print("\nno_ba_no_slam_forward_selftest: ALL PASS")
