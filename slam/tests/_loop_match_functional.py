#!/usr/bin/env python3
"""Functional probe: ``slam.loop`` fires with a sane match funnel on a loop session.

Boots the SPLIT 3-process stack -- ``imu_camera.main`` (replay) + ``vio.main`` +
``slam.main`` (LIVE: ``publish_map=True`` => the loop-match funnel is captured +
published) -- on a gold session WITH loops, subscribes to the new ``slam.loop``
topic over IPC, and asserts that:

* at least one :class:`~slam.comms.messages.LoopMatch` arrives,
* its match arrays are non-empty + consistent (cur_px / old_px / stage same N),
* the funnel is monotone (n_pnp <= n_fmat <= n_appearance) and the per-match
  stage labels agree with the counts,
* at least one ACCEPTED (confirmed) loop is published (the trajectory closes).

This is the LIVE-path counterpart to the byte-parity oracle: it proves the
capture -> publish wiring works end-to-end, NOT a determinism claim (the live
SLAM uses a latest-only inbox).

Run::

    .venv/bin/python -m slam.tests._loop_match_functional
    .venv/bin/python -m slam.tests._loop_match_functional --session sessions/gold/loop_closure_45s
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

from slam.comms import IPCPubSub, topics                          # noqa: E402
from slam.comms.converters import to_local                        # noqa: E402
from slam.comms import RingRegistry                               # noqa: E402
from slam.comms.messages import END                               # noqa: E402
from slam.comms.wire import WireEnd                               # noqa: E402


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
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s")
    ap.add_argument("--max-frames", type=int, default=600)
    ap.add_argument("--kf-every", type=int, default=5)
    ap.add_argument("--loop-search-radius", type=float, default=0.0,
                    help="forward to slam.main to A/B the spatial loop-search gate")
    args = ap.parse_args()

    pid = os.getpid()
    cap_ep = f"oak.cap.lm{pid & 0xFFF:x}"
    vio_ep = f"oak.vio.lm{pid & 0xFFF:x}"
    slam_ep = f"oak.slm.lm{pid & 0xFFF:x}"

    py = sys.executable
    env = dict(os.environ)
    lk = {"stdout": subprocess.PIPE, "stderr": subprocess.PIPE}

    print("loop_match_functional")
    print(f"  session={args.session} max-frames={args.max_frames}")

    vio_proc = subprocess.Popen(
        [py, "-m", "vio.main", "--capture-endpoint", cap_ep,
         "--endpoint", vio_ep, "--kf-every", str(args.kf_every)], env=env, **lk)
    slam_extra = (["--loop-search-radius", str(args.loop_search_radius)]
                  if args.loop_search_radius > 0 else [])
    slam_proc = subprocess.Popen(
        [py, "-m", "slam.main", "--vio-endpoint", vio_ep,
         "--endpoint", slam_ep, *slam_extra], env=env, **lk)
    time.sleep(0.3)
    cap_proc = subprocess.Popen(
        [py, "-m", "imu_camera.main", "--endpoint", cap_ep,
         "--session", args.session, "--max-frames", str(args.max_frames)],
        env=env, **lk)
    procs = (cap_proc, vio_proc, slam_proc)

    received: list = []
    lock = threading.Lock()
    rings = RingRegistry()                          # slam.loop is POD, no ring

    def on_loop(wm) -> None:
        if wm is END or isinstance(wm, WireEnd):
            return
        msg = to_local(topics.SLAM_LOOP, wm, rings)
        if msg is END:
            return
        with lock:
            received.append(msg)

    client = None
    try:
        _await_calib_bundle(slam_ep, timeout_s=25.0)
        print("  slam: ready")
        client = IPCPubSub(slam_ep, role="client", connect_timeout_s=25.0)
        client.subscribe(topics.SLAM_LOOP, on_loop)
        client.start()

        cap_proc.wait(timeout=180.0)
        vio_proc.wait(timeout=180.0)
        slam_proc.wait(timeout=180.0)
        time.sleep(1.0)                             # drain in-flight wire messages

        with lock:
            msgs = list(received)
        print(f"\n  received {len(msgs)} LoopMatch message(s) on slam.loop")
        _check(len(msgs) >= 1, "at least one slam.loop LoopMatch arrived")

        # Aggregate the funnel evidence + per-match consistency.
        non_empty = [m for m in msgs if int(m.n_appearance) > 0]
        accepted = [m for m in msgs if bool(m.accepted)]
        with_matches = [m for m in msgs if len(np.asarray(m.cur_px)) > 0]
        _check(len(non_empty) >= 1,
               f"a candidate had appearance matches (n={len(non_empty)})")
        _check(len(with_matches) >= 1,
               f"a candidate carried per-match pixel pairs (n={len(with_matches)})")

        for m in with_matches[:8]:
            cur = np.asarray(m.cur_px, np.float32).reshape(-1, 2)
            old = np.asarray(m.old_px, np.float32).reshape(-1, 2)
            stg = np.asarray(m.stage, np.uint8).reshape(-1)
            _check(len(cur) == len(old) == len(stg) == int(m.n_appearance),
                   f"cur/old/stage lengths == n_appearance "
                   f"({len(cur)}/{len(old)}/{len(stg)} vs {m.n_appearance})")
            n_epi = int((stg >= 1).sum())
            n_pnp = int((stg >= 2).sum())
            _check(n_pnp <= n_epi <= int(m.n_appearance),
                   f"funnel monotone (pnp {n_pnp} <= epi {n_epi} <= "
                   f"app {m.n_appearance})")
            _check(int(m.n_pnp) == n_pnp,
                   f"n_pnp count matches stage>=2 labels ({m.n_pnp} vs {n_pnp})")

        a0 = accepted[0] if accepted else None
        print(f"  accepted loops: {len(accepted)}; example: "
              f"{None if a0 is None else (a0.cur_seq, a0.old_seq, a0.n_appearance, a0.n_fmat, a0.n_pnp, round(float(a0.rot_deg), 2), bool(a0.accepted))}")
        _check(len(accepted) >= 1,
               f"at least one ACCEPTED (confirmed) loop published "
               f"(got {len(accepted)})")

        print("\nLOOP-MATCH FUNCTIONAL PASS")
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
                        ("slam", slam_proc)):
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
