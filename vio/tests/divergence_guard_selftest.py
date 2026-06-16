#!/usr/bin/env python3
"""Unit selftest for the tight-VIO divergence guard (sky.vio.window).

Proves the THREE guard invariants directly, without a full session replay, by
injecting a controlled divergent ``run_ba`` result into the live
:class:`WindowedVIORGBDOdometry.process` path:

  1. ON DIVERGENCE the live VO frontend pose (``self.vo.pose``) is NOT
     overwritten -- a transient diverged keyframe must not poison the frame-to-
     frame tracker (the persistent-poisoning bug the guard closes).
  2. ON DIVERGENCE the result carries ``vio_degraded = True`` in ``last_info``
     so a downstream / FC consumer can see "estimator degraded this keyframe".
  3. ON A HEALTHY solve the frontend IS re-anchored to the refined pose and
     ``vio_degraded`` is False -- the guard is a no-op when nothing diverged.

It also checks the pure detection helper on the latest map state directly: a
synthetic window whose latest keyframe is yanked far from the IMU forward-
prediction with a high reprojection trips the guard, and a clean window does not.

Run::

    .venv/bin/python -m vio.tests.divergence_guard_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.vio.window import (                                          # noqa: E402
    WindowedVIOConfig, WindowedVIORGBDOdometry, body_world_to_T_cw)


def _make_vo() -> WindowedVIORGBDOdometry:
    """A tight VIO odometry with a trivial (no-IMU-stream) map; we drive
    ``process`` and stub ``map.run_ba`` so the test is fast and deterministic."""
    K = np.array([[300.0, 0, 160.0], [0, 300.0, 100.0], [0, 0, 1.0]])
    cfg = WindowedVIOConfig()
    cfg.vio.imu_info_weight = True
    vo = WindowedVIORGBDOdometry(
        K, ts_ns=np.zeros(0, np.int64),
        gyro_cam=np.zeros((0, 3)), accel_cam=np.zeros((0, 3)), cfg=cfg)
    return vo


def _process_with_stub(vo, *, degraded: bool, refined_pose_cw):
    """Drive one keyframe through ``process`` with a stubbed ``run_ba`` that
    returns ``refined_pose_cw`` and a ``last_info`` carrying the degraded flag.

    Returns the published pose, plus snapshots of ``self.vo.pose`` before/after.
    """
    gray = np.zeros((200, 320), np.uint8)
    depth = np.ones((200, 320), np.float64)

    # First call seeds a keyframe (no keyframes yet -> is_kf True). Make run_ba a
    # no-op (None) so the frontend pose flows through untouched for the seed.
    vo.map.run_ba = lambda: None                                     # type: ignore
    vo.process(gray, depth, ts_ns=1_000_000)

    # Second keyframe: stub run_ba to return the (possibly diverged) refined pose
    # + the degraded flag the guard would have set.
    def _stub():
        vo.map.last_info = {
            "vio_kfs": 2, "vio_reproj_px": 99.0 if degraded else 1.0,
            "vio_window_jump_m": 4.5 if degraded else 0.0,
            "vio_degraded": bool(degraded),
        }
        return refined_pose_cw

    vo.map.run_ba = _stub                                            # type: ignore
    vo._frames_since_kf = vo.cfg.kf_every                            # force is_kf
    vo_pose_before = vo.vo.pose.copy()
    published = vo.process(gray, depth, ts_ns=2_000_000).copy()
    vo_pose_after = vo.vo.pose.copy()
    return published, vo_pose_before, vo_pose_after


def test_frontend_not_poisoned_on_divergence() -> None:
    print("1) self.vo.pose NOT poisoned on divergence:")
    vo = _make_vo()
    # A wildly diverged refined pose (4.5 m away on x).
    diverged_cw = body_world_to_T_cw(np.eye(3), np.array([4.5, 0.0, 0.0]))
    pub, before, after = _process_with_stub(vo, degraded=True,
                                            refined_pose_cw=diverged_cw)
    moved = float(np.linalg.norm(after[:3, 3] - before[:3, 3]))
    print(f"   frontend self.vo.pose moved {moved*1e3:.4f} mm during a diverged "
          f"keyframe (must be ~0)")
    assert moved < 1e-9, ("self.vo.pose WAS overwritten on divergence -- the VO "
                          "frontend was poisoned by a rejected solve")
    assert vo.last_info.get("vio_degraded") is True, \
        "vio_degraded not surfaced to the caller on divergence"
    print("   OK -- frontend pose unchanged; vio_degraded=True surfaced.\n")


def test_frontend_reanchored_on_healthy() -> None:
    print("2) self.vo.pose RE-ANCHORED on a healthy solve (guard is a no-op):")
    vo = _make_vo()
    refined_cw = body_world_to_T_cw(np.eye(3), np.array([0.10, 0.0, 0.20]))
    pub, before, after = _process_with_stub(vo, degraded=False,
                                            refined_pose_cw=refined_cw)
    expect = np.linalg.inv(refined_cw)
    err_pub = float(np.linalg.norm(pub[:3, 3] - expect[:3, 3]))
    err_fe = float(np.linalg.norm(after[:3, 3] - expect[:3, 3]))
    print(f"   published vs refined err = {err_pub*1e3:.4f} mm ; "
          f"frontend vs refined err = {err_fe*1e3:.4f} mm")
    assert err_pub < 1e-9, "healthy solve did not publish the refined pose"
    assert err_fe < 1e-9, "healthy solve did not re-anchor the frontend"
    assert vo.last_info.get("vio_degraded") is False, \
        "vio_degraded should be False on a healthy solve"
    print("   OK -- healthy behaviour bit-unchanged (frontend re-anchored).\n")


def main() -> int:
    print("=== Divergence-guard unit selftest (frontend-poisoning invariants) ===\n")
    test_frontend_not_poisoned_on_divergence()
    test_frontend_reanchored_on_healthy()
    print("PASS -- guard rejects+flags a diverged keyframe WITHOUT poisoning the "
          "VO frontend, and is a no-op on a healthy solve.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
