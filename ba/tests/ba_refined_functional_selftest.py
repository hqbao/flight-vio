#!/usr/bin/env python3
"""Functional probe: the ``ba`` process produces ``pose.refined`` over IPC.

Boots the SPLIT stack -- ``imu_camera.main`` (replay) + ``vio.main`` + ``ba.main``
-- on a gold session, subscribes to ``pose.refined`` over IPC on the BA endpoint,
and asserts that the BA process actually publishes refined poses:

* at least one :class:`~ba.comms.messages.PoseMsg` arrives on the BA endpoint's
  ``pose.refined`` topic,
* each carries a finite 4x4 ``T_world_cam`` and ``info['refined'] is True``.

This is the LIVE-path counterpart to the byte-parity oracle (which drives the
``sky.*`` solve directly): it proves the capture -> vio (keyframe) -> ba (windowed
BA) -> ``pose.refined`` IPC wiring works end-to-end, after the windowed-BA backend
was extracted into the ``ba`` process. NOT a determinism claim.

``--tight`` exercises the tight backend (and would ALSO publish ``ba.state``); the
default is the loose windowed BA, matching the in-VIO loose backend that was moved.

Run::

    .venv/bin/python -m ba.tests.ba_refined_functional_selftest
    .venv/bin/python -m ba.tests.ba_refined_functional_selftest --tight
    .venv/bin/python -m ba.tests.ba_refined_functional_selftest --session sessions/gold/lab_straight_20s
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ba.comms import IPCPubSub, RingRegistry, topics                # noqa: E402
from ba.comms.converters import to_local                           # noqa: E402
from ba.comms.messages import END                                  # noqa: E402
from ba.comms.wire import WireEnd                                  # noqa: E402


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def _await_calib_bundle(endpoint: str, timeout_s: float) -> None:
    got = threading.Event()
    client = IPCPubSub(endpoint, role="client", connect_timeout_s=timeout_s)
    client.subscribe("calib.bundle", lambda _wm: got.set())
    client.start()
    try:
        if not got.wait(timeout=timeout_s):
            raise TimeoutError(f"no calib.bundle from {endpoint!r} in {timeout_s}s")
    finally:
        client.stop()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default="sessions/gold/lab_straight_20s")
    ap.add_argument("--max-frames", type=int, default=200)
    ap.add_argument("--kf-every", type=int, default=5)
    ap.add_argument("--tight", action="store_true",
                    help="run the ba backend tight (also publishes ba.state)")
    args = ap.parse_args()

    pid = os.getpid()
    cap_ep = f"oak.cap.bf{pid & 0xFFF:x}"
    vio_ep = f"oak.vio.bf{pid & 0xFFF:x}"
    ba_ep = f"oak.ba.bf{pid & 0xFFF:x}"

    py = sys.executable
    env = dict(os.environ)
    lk = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE}

    print("ba_refined_functional")
    print(f"  session={args.session} max-frames={args.max_frames} "
          f"tight={args.tight}")

    vio_extra = ["--tight"] if args.tight else []
    vio_proc = subprocess.Popen(
        [py, "-m", "vio.main", "--capture-endpoint", cap_ep,
         "--endpoint", vio_ep, "--ba-endpoint", ba_ep,
         "--kf-every", str(args.kf_every), *vio_extra], env=env, **lk)
    ba_extra = ["--tight"] if args.tight else []
    ba_proc = subprocess.Popen(
        [py, "-m", "ba.main", "--vio-endpoint", vio_ep,
         "--endpoint", ba_ep, *ba_extra], env=env, **lk)
    time.sleep(0.3)
    cap_proc = subprocess.Popen(
        [py, "-m", "imu_camera.main", "--endpoint", cap_ep,
         "--session", args.session, "--max-frames", str(args.max_frames)],
        env=env, **lk)
    procs = (cap_proc, vio_proc, ba_proc)

    received: list = []
    lock = threading.Lock()
    rings = RingRegistry()                          # pose.refined is POD, no ring

    def on_refined(wm) -> None:
        if wm is END or isinstance(wm, WireEnd):
            return
        msg = to_local(topics.POSE_REFINED, wm, rings)
        if msg is END:
            return
        with lock:
            received.append(msg)

    client = None
    try:
        # The BA endpoint re-broadcasts the calib bundle once its server is up
        # (proving ba booted + attached to vio's kf rings).
        _await_calib_bundle(ba_ep, timeout_s=30.0)
        print("  ba: ready")
        client = IPCPubSub(ba_ep, role="client", connect_timeout_s=30.0)
        client.subscribe(topics.POSE_REFINED, on_refined)
        client.start()

        cap_proc.wait(timeout=180.0)
        vio_proc.wait(timeout=180.0)
        ba_proc.wait(timeout=180.0)
        time.sleep(1.0)                             # drain in-flight wire messages

        with lock:
            msgs = list(received)
        print(f"\n  received {len(msgs)} pose.refined message(s) on the BA endpoint")
        _check(len(msgs) >= 1,
               "at least one pose.refined arrived from the BA process")

        finite = [m for m in msgs
                  if np.all(np.isfinite(np.asarray(m.T_world_cam)))]
        _check(len(finite) == len(msgs),
               f"all {len(msgs)} refined poses are finite "
               f"({len(finite)}/{len(msgs)})")
        flagged = [m for m in msgs
                   if isinstance(m.info, dict) and m.info.get("refined") is True]
        _check(len(flagged) >= 1,
               f"a refined pose carried info['refined']=True (n={len(flagged)})")

        m0 = msgs[-1]
        p = np.asarray(m0.T_world_cam)[:3, 3]
        print(f"  last refined pose seq={m0.seq} pos=({p[0]:+.3f} {p[1]:+.3f} "
              f"{p[2]:+.3f}) info={ {k: m0.info[k] for k in list(m0.info)[:3]} }")

        print("\nBA-REFINED FUNCTIONAL PASS")
        return 0
    finally:
        if client is not None:
            try:
                client.stop()
            except Exception:                                      # noqa: BLE001
                pass
        for p in procs:
            if p.poll() is None:
                try:
                    p.terminate()
                except Exception:                                  # noqa: BLE001
                    pass
        for p in procs:
            try:
                p.wait(timeout=5.0)
            except Exception:                                      # noqa: BLE001
                try:
                    p.kill()
                except Exception:                                  # noqa: BLE001
                    pass
        for name, p in (("capture", cap_proc), ("vio", vio_proc),
                        ("ba", ba_proc)):
            try:
                _o, err = p.communicate(timeout=2.0)
            except Exception:                                      # noqa: BLE001
                err = b""
            if err and err.strip():
                tail = err.decode(errors="replace").splitlines()[-6:]
                print(f"\n  --- {name}.stderr (tail) ---\n  " + "\n  ".join(tail),
                      file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
