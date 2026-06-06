#!/usr/bin/env python3
"""Self-test: the out-of-process BA worker matches the in-thread reference.

Proves two things without a device:

1. **Correctness** -- feeding the same keyframe stream to the new
   :func:`ours.legacy.ba_worker_proc.start_ba_process` worker and to a direct,
   synchronous :class:`WindowedBAMap` produces the *same* world-frame correction
   ``C`` for each keyframe (the process is just a transport; the math is the same
   ``WindowedBAMap``). We compare the final correction after replaying a gold
   session, allowing for the async worker only ever returning the latest result.

2. **Liveness / shutdown** -- the worker starts, returns corrections, and stops
   cleanly within the teardown contract (``stop.set()`` + ``event.set()`` +
   ``thread.join(timeout)``).

Run::

    PYTHONPATH=$PWD python ours/tools/ba_worker_proc_selftest.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.lib import (  # noqa: E402
    RGBDVisualOdometry,
    OdometryConfig,
    SessionReader,
    WindowedBAMap,
    WindowedConfig,
)
from ours.lib.backend.bundle import BAConfig  # noqa: E402
from ours.legacy.ba_worker_proc import start_ba_process  # noqa: E402


def _cfg() -> WindowedConfig:
    return WindowedConfig(window=6, kf_every=5,
                          ba=BAConfig(max_iters=5, huber_px=2.0,
                                      use_gravity=True, use_vo_trans_prior=True))


def _keyframe_stream(session_dir: Path, kf_every: int, max_kf: int):
    """Replay the session's f2f VO and yield (T_cw, ids, pts, depth) per KF."""
    reader = SessionReader(session_dir)
    vo = RGBDVisualOdometry(reader.K, OdometryConfig(gyro_fuse=True))
    out = []
    kf = 0
    prev = None
    for i in range(len(reader)):
        fr = reader.load_frame(i)
        dt = 1.0 / 20.0 if prev is None else (fr.ts_ns - prev) * 1e-9
        prev = fr.ts_ns
        vo.process(fr.gray_left, fr.depth_m, R_prior=None, dt_s=dt)
        kf += 1
        if kf >= kf_every:
            kf = 0
            st = vo.frontend.tracks
            out.append((np.linalg.inv(vo.pose), st.ids.copy(),
                        st.points.copy(), fr.depth_m.copy()))
            if len(out) >= max_kf:
                break
    return reader.K, out


def main() -> int:
    session = Path("sessions/fast_push_15s")
    if not session.exists():
        session = Path("sessions/gold/lab_loop_30s")
    print(f"[ba_worker_proc] session: {session}")

    K, stream = _keyframe_stream(session, kf_every=5, max_kf=10)
    print(f"[ba_worker_proc] replayed {len(stream)} keyframes")

    # --- reference: synchronous WindowedBAMap (what the worker runs) ---------
    ref_map = WindowedBAMap(K, _cfg())
    ref_C = []
    for T_cw, ids, pts, depth in stream:
        ref_map.add_keyframe(T_cw, ids, pts, depth, accel_cam=None)
        post = ref_map.run_ba()
        ref_C.append(None if post is None else np.linalg.inv(post) @ T_cw)
    assert any(c is not None for c in ref_C), "reference produced no correction"

    # --- process worker: SYNCHRONOUS handshake so no keyframe is dropped -----
    # Submit one keyframe, then block until its correction arrives before
    # submitting the next. This guarantees a 1:1 correspondence with ref_C (the
    # async latest-wins path is exercised separately by the live read loop; here
    # we are proving the *math* is identical across the process boundary).
    st = start_ba_process(K, _cfg())
    assert st["thread"].is_alive(), "worker process did not start"
    got: list = []
    ok = True
    for i, (T_cw, ids, pts, depth) in enumerate(stream):
        st["submit"](T_cw, ids, pts, depth, None)
        c = None
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            c = st["poll"]()
            if c is not None:
                break
            time.sleep(0.01)
        got.append(c)
        if ref_C[i] is None:
            continue
        if c is None:
            print(f"  [FAIL] KF{i}: worker returned no correction "
                  f"(reference did)")
            ok = False
            continue
        err_t = float(np.linalg.norm(c[:3, 3] - ref_C[i][:3, 3]))
        dR = c[:3, :3] @ ref_C[i][:3, :3].T
        err_r = float(np.degrees(np.arccos(
            np.clip((np.trace(dR) - 1.0) * 0.5, -1.0, 1.0))))
        tag = "ok" if (err_t <= 1e-6 and err_r <= 1e-4) else "FAIL"
        if tag == "FAIL":
            ok = False
        print(f"  KF{i}: dt={err_t*1000:8.4f} mm  dR={err_r:8.5f} deg  [{tag}]")

    n_match = sum(1 for c in got if c is not None)
    print(f"[ba_worker_proc] worker corrections received: {n_match}/{len(stream)}")

    # --- clean shutdown (teardown contract) ----------------------------------
    st["stop"].set()
    st["event"].set()
    st["thread"].join(timeout=2.0)
    alive = st["thread"].is_alive()

    if alive:
        print("  [FAIL] worker process did not stop within join timeout")
        ok = False
    else:
        print("  [ok] worker process stopped cleanly")

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1



if __name__ == "__main__":
    raise SystemExit(main())
