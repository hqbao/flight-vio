#!/usr/bin/env python3
"""Self-test for the SLAM 3D-map point cloud (``ours.lib.misc.geometry`` +
``ours.tools.slam_map3d``). Two parts:

1. SYNTHETIC unit check of :func:`keyframe_pointcloud` -- a known depth + pose
   must back-project to the exact world points (geometry is correct).
2. GOLD smoke -- build a map from a real session offline and assert it is sane
   (points exist, finite, room-scale, one camera per keyframe).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ours.lib.misc.geometry import keyframe_pointcloud                 # noqa: E402
from ours.tools.slam_map3d import build_map                            # noqa: E402

_FAILS = 0


def _check(cond: bool, msg: str) -> None:
    global _FAILS
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        _FAILS += 1


def test_synthetic() -> None:
    print("keyframe_pointcloud synthetic geometry")
    K = np.array([[100.0, 0, 32.0], [0, 100.0, 24.0], [0, 0, 1.0]])
    h, w = 48, 64
    depth = np.full((h, w), 2.0, np.float32)        # flat wall at z=2 m
    gray = np.full((h, w), 128, np.uint8)

    # Identity pose -> world == camera frame; stride 1, all pixels valid.
    pts, col = keyframe_pointcloud([np.eye(4)], [depth], [gray], K,
                                   stride=1, max_depth=8.0)
    _check(pts.shape[0] == h * w, f"all {h*w} pixels kept ({pts.shape[0]})")
    _check(np.allclose(pts[:, 2], 2.0), "every point sits on the z=2 m wall")
    _check(np.allclose(col, 128 / 255.0), "colour = grey intensity (128/255)")
    # Principal-ray pixel (cx,cy) back-projects to (0,0,2).
    centre = pts[(h // 2) * w + (w // 2)]
    _check(np.allclose(centre, [0, 0, 2], atol=0.02),
           f"centre pixel -> optical axis point {centre.round(3).tolist()}")

    # A pure +1 m world translation (T_world_cam) must shift the whole cloud +1 m.
    T = np.eye(4); T[0, 3] = 1.0
    pts2, _ = keyframe_pointcloud([T], [depth], [gray], K, stride=1)
    _check(np.allclose(pts2 - pts, [1, 0, 0]), "pose translation shifts the cloud")

    # Depth range gate drops far points.
    far = np.full((h, w), 50.0, np.float32)
    pts3, _ = keyframe_pointcloud([np.eye(4)], [far], [gray], K, max_depth=6.0)
    _check(pts3.shape[0] == 0, "points beyond max_depth are dropped")


def test_landmark_synthetic() -> None:
    print("keyframe_landmark_cloud draws only PnP inliers")
    from ours.lib.misc.geometry import keyframe_landmark_cloud
    K = np.array([[100.0, 0, 32.0], [0, 100.0, 24.0], [0, 0, 1.0]])
    depth = np.full((48, 64), 2.0, np.float32)
    ids = np.array([10, 11, 12, 13])
    px = np.array([[32, 24], [10, 10], [40, 30], [50, 20]], float)
    inl = np.array([10, 12])                          # only 2 of 4 are inliers
    p, _ = keyframe_landmark_cloud([np.eye(4)], [ids], [px], [depth], [inl], K)
    _check(len(p) == 2, f"only the 2 inlier pixels back-project ({len(p)})")
    _check(any(np.allclose(q, [0, 0, 2], atol=1e-3) for q in p),
           "inlier id 10 at the principal point -> (0,0,2)")
    # No inliers -> empty (not the dense cloud).
    p0, _ = keyframe_landmark_cloud([np.eye(4)], [ids], [px], [depth],
                                    [np.array([], int)], K)
    _check(len(p0) == 0, "no inliers -> no points")


def test_voxel_downsample() -> None:
    print("voxel_downsample fuses + drops low-count (noise) voxels")
    from ours.lib.misc.geometry import voxel_downsample
    # A dense 10x10x10 block at the origin (1000 pts in ~one 0.5 m voxel region)
    # + 5 isolated far-flung noise points (each alone in its voxel).
    g = np.linspace(0, 0.09, 10)
    block = np.array(np.meshgrid(g, g, g)).reshape(3, -1).T          # 1000 pts
    noise = np.array([[5, 0, 0], [0, 5, 0], [0, 0, 5], [9, 9, 9], [-9, -9, -9.]])
    pts = np.vstack([block, noise]).astype(np.float32)
    col = np.full((len(pts), 3), 0.5, np.float32)
    out, oc = voxel_downsample(pts, col, voxel=0.1, min_count=3)
    _check(len(out) >= 1, f"the dense block survives ({len(out)} voxel(s))")
    # Every isolated noise point (count 1 < 3) must be gone.
    for n in noise:
        _check(not np.any(np.all(np.abs(out - n) < 0.6, axis=1)),
               f"isolated noise {n.tolist()} dropped")
    _check(bool(np.all(np.isfinite(out))), "voxel centroids finite")


def test_gold_smoke() -> None:
    print("build_map on a gold session (offline): room/landmarks/dense")
    room = build_map("sessions/gold/lab_loop_30s", kf_every=5, max_frames=120,
                     use_slam=False, mode="room")                 # default
    lm = build_map("sessions/gold/lab_loop_30s", kf_every=5, max_frames=120,
                   use_slam=False, mode="landmarks")
    dense = build_map("sessions/gold/lab_loop_30s", kf_every=5, max_frames=120,
                      use_slam=False, mode="dense", stride=6)
    rp = room["points"]
    _check(room["n_kf"] > 0, f"keyframes collected ({room['n_kf']})")
    _check(len(rp) > 1000, f"room cloud has points ({len(rp)})")
    _check(len(rp) < len(dense["points"]),
           f"room (voxel-fused) sparser than raw dense "
           f"({len(rp)} < {len(dense['points'])})")
    _check(len(lm["points"]) < len(rp),
           f"landmarks sparser than room ({len(lm['points'])} < {len(rp)})")
    _check(bool(np.all(np.isfinite(rp))), "all room points finite")
    extent = float(np.linalg.norm(rp.max(0) - rp.min(0))) if len(rp) else 0.0
    _check(0.1 < extent < 100.0, f"room-scale extent ({extent:.1f} m)")
    _check(room["cams"].shape[0] == room["n_kf"], "one camera per keyframe")


def main() -> int:
    print("map3d_selftest")
    test_synthetic()
    test_landmark_synthetic()
    test_voxel_downsample()
    test_gold_smoke()
    print("ALL MAP3D SELFTESTS PASSED" if _FAILS == 0
          else f"MAP3D SELFTEST FAILED ({_FAILS})")
    return 0 if _FAILS == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
