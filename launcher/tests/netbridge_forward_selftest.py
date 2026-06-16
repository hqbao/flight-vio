#!/usr/bin/env python3
"""Self-test: the launcher's ``--forward HOST:PORT`` builds the netbridge.forward
argv correctly and is ADDITIVE -- it never perturbs the capture/vio argv, so the
default (local-UI / --no-ui) path + the offline byte-parity oracle stay gap=0.

Fully OFFLINE (no spawning, no device, no network): exercises the pure
:func:`launcher.main.build_forward_args` + :func:`launcher.main.parse_host_port`
builders with synthetic namespaces, and confirms the real launcher argparser
registers the flag via ``-m launcher.main --help``. Mirrors
``direct_forward_selftest`` / ``frontend_viz_forward_selftest``.

Asserts:
  (a) ``parse_host_port`` handles HOST:PORT, bare :PORT, and bare PORT,
  (b) ``build_forward_args`` carries the listen addr + the three endpoints + the
      capture resolution (so forward attaches to the SAME-sized rings),
  (c) the launcher CLI ``--help`` lists ``--forward`` (parser registered),
  (d) ``--forward`` does NOT appear in / alter the capture or vio argv (additive).

Run::

    .venv/bin/python -m launcher.tests.netbridge_forward_selftest
"""
from __future__ import annotations

import subprocess
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from launcher.main import (                                       # noqa: E402
    build_capture_args, build_forward_args, build_vio_args, parse_host_port,
)


def _ns(**over) -> types.SimpleNamespace:
    """A launcher-args namespace with sane defaults, overridable per test."""
    base = dict(width=54, height=42, fps=20, kf_every=5, session=None,
                max_frames=0, no_gyro=False, recalibrate_bias=False,
                use_camera_calib=False, vl53l9cx=True, worker=False,
                tight=False, stabilize_velocity=False, depth_icp=False,
                ba_window=False, frontend_viz=False, direct=False,
                forward=None, model=None, bridge_frames=False)
    base.update(over)
    return types.SimpleNamespace(**base)


def test_parse_host_port() -> None:
    assert parse_host_port("1.2.3.4:9000") == ("1.2.3.4", 9000)
    assert parse_host_port(":8787") == ("0.0.0.0", 8787)
    assert parse_host_port("8787") == ("0.0.0.0", 8787)
    assert parse_host_port(" 10.0.0.5:7000 ") == ("10.0.0.5", 7000)
    print("[a] parse_host_port: HOST:PORT / :PORT / PORT                            OK")


def test_build_forward_args() -> None:
    cap, vio, slam = "oak.cap.lc0ffe", "oak.vio.lc0ffe", "oak.slm.lc0ffe"
    host, port = parse_host_port(":8787")
    argv = build_forward_args(host, port, _ns(width=54, height=42),
                              cap, vio, slam)
    # Listen addr + the resolved (suffixed) endpoints + the capture resolution.
    assert "--listen" in argv and "0.0.0.0:8787" in argv, argv
    assert argv[argv.index("--capture-endpoint") + 1] == cap, argv
    assert argv[argv.index("--vio-endpoint") + 1] == vio, argv
    assert argv[argv.index("--slam-endpoint") + 1] == slam, argv
    assert argv[argv.index("--width") + 1] == "54", argv
    assert argv[argv.index("--height") + 1] == "42", argv
    print("[b] build_forward_args: listen + 3 endpoints + resolution               OK")


def test_pose_only_default() -> None:
    """DEFAULT (no --bridge-frames) is POSE-ONLY; --bridge-frames restores frames.

    The bridge defaults to low-bandwidth so a congested WiFi link doesn't back up:
    the heavy image topics are excluded unless the operator opts in.
    """
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"
    host, port = parse_host_port(":8787")
    default = build_forward_args(host, port, _ns(bridge_frames=False),
                                 cap, vio, slam)
    frames = build_forward_args(host, port, _ns(bridge_frames=True),
                                cap, vio, slam)
    assert "--pose-only" in default, default
    assert "--pose-only" not in frames, frames
    # --bridge-frames must reproduce the EXACT pre-change argv (backward compatible).
    assert frames == ["--listen", "0.0.0.0:8787",
                      "--capture-endpoint", cap, "--vio-endpoint", vio,
                      "--slam-endpoint", slam, "--width", "54",
                      "--height", "42"], frames
    print("[e] default => --pose-only; --bridge-frames => legacy argv (compat)      OK")


def test_additive() -> None:
    """--forward must not leak into the capture / vio argv (additive)."""
    cap, vio, slam = "oak.capture", "oak.vio", "oak.slam"
    # With --forward set, the capture + vio builders are UNCHANGED vs without it.
    base_cap = build_capture_args(_ns(forward=None), cap)
    fwd_cap = build_capture_args(_ns(forward=":8787"), cap)
    assert base_cap == fwd_cap, (base_cap, fwd_cap)
    base_vio = build_vio_args(_ns(forward=None), cap, vio, slam, use_worker=False)
    fwd_vio = build_vio_args(_ns(forward=":8787"), cap, vio, slam,
                             use_worker=False)
    assert base_vio == fwd_vio, (base_vio, fwd_vio)
    assert "--forward" not in base_cap and "--forward" not in base_vio
    print("[d] --forward is additive: capture/vio argv UNCHANGED                    OK")


def test_help_registers_flag() -> None:
    """The launcher parser must register --forward AND --bridge-frames."""
    root = Path(__file__).resolve().parents[2]
    out = subprocess.run(
        [sys.executable, "-m", "launcher.main", "--help"],
        cwd=str(root), capture_output=True, text=True, check=True)
    assert "--forward" in out.stdout, out.stdout
    assert "--bridge-frames" in out.stdout, out.stdout
    print("[c] launcher --help lists --forward + --bridge-frames                    OK")


if __name__ == "__main__":
    test_parse_host_port()
    test_build_forward_args()
    test_pose_only_default()
    test_additive()
    test_help_registers_flag()
    print("\nnetbridge_forward_selftest: ALL PASS")
