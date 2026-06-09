#!/usr/bin/env python3
"""Unit tests for :mod:`ui.viz.surface_mesh` (depth-map surface meshing).

The Room Surface (3D mesh) viewer meshes a spatially-spread subset of VIO
keyframes' depth maps into ONE continuous shaded surface. These tests feed
hand-checkable inputs and assert the two pure builders:

* :func:`depth_surface_mesh` --
  - a FLAT depth patch (all valid, no discontinuity) -> the full grid is
    triangulated: ``r0*c0`` verts, ``2*(r0-1)*(c0-1)`` faces, all face indices in
    ``[0, r0*c0)``, one RGB colour per vertex; and the back-projected vertices
    land on the expected pinhole positions.
  - a depth map with a SHARP depth STEP -> the triangles spanning the step are
    REJECTED by the ``edge_max`` gate, while the flat regions on either side are
    kept (so the curtain across the discontinuity never meshes).
* :func:`select_spread_keyframes` -- greedy ``> spacing`` thinning keeps the
  spatially-spread subset (a cluster collapses to one keyframe; well-separated
  ones all survive), in input order.

Run::

    python -m ui.tests.surface_mesh_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ui.viz import surface_mesh                                      # noqa: E402
from ui.viz.surface_mesh import (                                    # noqa: E402
    depth_surface_mesh, select_spread_keyframes,
)


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


# A simple pinhole intrinsic (square pixels, centred principal point) for the
# back-projection arithmetic. Identity pose so world == camera frame.
_K = np.array([[100.0, 0.0, 5.5],
               [0.0, 100.0, 3.5],
               [0.0, 0.0, 1.0]], np.float64)
_R = np.eye(3, dtype=np.float64)
_T = np.zeros(3, np.float64)


def test_flat_patch() -> None:
    print("surface_mesh_selftest: depth_surface_mesh (flat patch)")
    # A flat 4x6 depth patch at 2.0 m, all valid, no discontinuity. stride=1, so
    # the vertex grid is the full 4x6 = 24 verts; cells = 3x5 = 15; every cell is
    # flat -> kept -> 2*15 = 30 faces.
    h, w = 4, 6
    depth = np.full((h, w), 2.0, np.float32)
    verts, faces, cols = depth_surface_mesh(
        depth, _R, _T, _K, stride=1, edge_max=0.1,
        min_depth=0.3, max_depth=6.0, gray=None)

    _check(verts.shape == (h * w, 3),
           f"flat grid -> {h * w} verts (got {verts.shape})")
    n_cells = (h - 1) * (w - 1)
    _check(faces.shape == (2 * n_cells, 3),
           f"flat grid -> {2 * n_cells} faces (got {faces.shape})")
    _check(int(faces.min()) >= 0 and int(faces.max()) < len(verts),
           f"face indices in [0,{len(verts)}) "
           f"(got [{faces.min()},{faces.max()}])")
    _check(cols.shape == (h * w, 3),
           f"one RGB colour per vertex (got {cols.shape})")

    # Back-projection check: vertex (row v, col u) = ((u-cx)/fx*z, (v-cy)/fy*z, z).
    # Row-major flatten -> index v*w + u. Check a couple of corners.
    fx, fy = _K[0, 0], _K[1, 1]
    cx, cy = _K[0, 2], _K[1, 2]
    z = 2.0
    want_00 = np.array([(0 - cx) / fx * z, (0 - cy) / fy * z, z])
    want_35 = np.array([(5 - cx) / fx * z, (3 - cy) / fy * z, z])  # row 3, col 5
    _check(np.allclose(verts[0], want_00, atol=1e-5),
           "vertex (0,0) back-projects to the expected pinhole point")
    _check(np.allclose(verts[3 * w + 5], want_35, atol=1e-5),
           "vertex (3,5) back-projects to the expected pinhole point")
    # All vertices share z (a flat patch) -> the surface is a planar sheet.
    _check(np.allclose(verts[:, 2], z),
           "every flat-patch vertex sits at the patch depth (planar surface)")


def test_depth_step_reject() -> None:
    print("surface_mesh_selftest: depth_surface_mesh (sharp-step reject)")
    # A 4x6 depth map: columns 0..2 at 1.0 m, columns 3..5 at 5.0 m -- a sharp
    # step between col 2 and col 3. stride=1 -> 4x6 vertex grid, 3x5 = 15 cells.
    # Cell columns span (c, c+1); the spread of a cell's 4 corners is 0 inside a
    # flat region but 4.0 across the step:
    #   cell-col 0 (cols 0-1): flat 1.0 -> kept
    #   cell-col 1 (cols 1-2): flat 1.0 -> kept
    #   cell-col 2 (cols 2-3): 1.0|5.0  -> spread 4.0 > 0.1 -> REJECTED
    #   cell-col 3 (cols 3-4): flat 5.0 -> kept
    #   cell-col 4 (cols 4-5): flat 5.0 -> kept
    # => 4 kept cell-columns x 3 rows = 12 cells -> 24 faces; 3 rejected.
    h, w = 4, 6
    depth = np.empty((h, w), np.float32)
    depth[:, :3] = 1.0
    depth[:, 3:] = 5.0
    verts, faces, cols = depth_surface_mesh(
        depth, _R, _T, _K, stride=1, edge_max=0.1,
        min_depth=0.3, max_depth=6.0, gray=None)

    kept_cells = 4 * (h - 1)                          # 4 cell-cols x 3 rows = 12
    _check(faces.shape == (2 * kept_cells, 3),
           f"step grid keeps {kept_cells} cells -> {2 * kept_cells} faces "
           f"(got {faces.shape[0]})")
    # The rejected (straddling) cells are 1 cell-col x 3 rows = 3 -> 6 dropped
    # faces; the full grid would have 2*15 = 30, so we must see exactly 30-6 = 24.
    _check(faces.shape[0] == 30 - 6,
           f"the 3 step-straddling cells (6 faces) are dropped "
           f"(got {faces.shape[0]} faces)")

    # CRUCIAL: NO kept triangle may span the step (have one corner at z=1 and
    # another at z=5). For every face, the corner-z spread must be <= edge_max.
    zc = verts[faces, 2]                              # (F, 3) corner depths
    spread = zc.max(axis=1) - zc.min(axis=1)
    _check(float(spread.max()) <= 0.1 + 1e-6,
           f"no kept triangle spans the step (max corner-z spread "
           f"{float(spread.max()):.3f} <= edge_max)")
    # Sanity: the kept triangles cover BOTH flat regions (some at z=1, some z=5).
    tri_z = zc.mean(axis=1)
    _check(np.any(np.isclose(tri_z, 1.0)) and np.any(np.isclose(tri_z, 5.0)),
           "both flat regions (near 1.0 m and far 5.0 m) are meshed")

    # edge_max=0 disables the reject ONLY where there is no spread; a huge
    # edge_max keeps everything (proves the gate is what drops the curtain).
    _, faces_all, _ = depth_surface_mesh(
        depth, _R, _T, _K, stride=1, edge_max=100.0,
        min_depth=0.3, max_depth=6.0, gray=None)
    _check(faces_all.shape[0] == 30,
           f"a permissive edge_max keeps the full grid (30 faces, got "
           f"{faces_all.shape[0]}) -> the reject is what drops the step cells")


def test_invalid_depth_skipped() -> None:
    print("surface_mesh_selftest: depth_surface_mesh (out-of-band depth skipped)")
    # A 3x3 patch all at 2.0 m EXCEPT one corner at 0.0 (below MIN_DEPTH) -> the
    # cells touching that corner are skipped (an invalid corner can't mesh).
    h, w = 3, 3
    depth = np.full((h, w), 2.0, np.float32)
    depth[0, 0] = 0.0                                 # invalid (below the band)
    _, faces, _ = depth_surface_mesh(
        depth, _R, _T, _K, stride=1, edge_max=0.1,
        min_depth=0.3, max_depth=6.0, gray=None)
    # 2x2 cells = 4; the one cell with the invalid corner (cell (0,0)) is dropped
    # -> 3 cells kept -> 6 faces.
    _check(faces.shape[0] == 6,
           f"the cell touching the invalid corner is dropped (3 cells -> 6 "
           f"faces, got {faces.shape[0]})")


def test_select_spread_keyframes() -> None:
    print("surface_mesh_selftest: select_spread_keyframes")
    # Three tight clusters (~0.05 m apart within, ~2 m apart between) -> greedy
    # > 0.3 m thinning keeps ONE per cluster (the first walked), so 3 survive.
    pos = np.array([
        [0.00, 0.0, 0.0], [0.05, 0.0, 0.0], [0.04, 0.0, 0.03],   # cluster A
        [2.00, 0.0, 0.0], [2.05, 0.0, 0.0],                       # cluster B
        [4.00, 0.0, 0.0],                                         # cluster C
    ], np.float64)
    sel = select_spread_keyframes(pos, spacing=0.3)
    _check(sel.dtype == np.int64 and sel.ndim == 1,
           f"selection is a 1-D int64 index array (got {sel.dtype} {sel.shape})")
    _check(sel.tolist() == [0, 3, 5],
           f"greedy spacing keeps one per cluster, first walked "
           f"(got {sel.tolist()}, want [0, 3, 5])")
    # The kept positions are pairwise > spacing apart.
    kept = pos[sel]
    d = np.linalg.norm(kept[:, None, :] - kept[None, :, :], axis=2)
    d[np.diag_indices(len(kept))] = np.inf
    _check(float(d.min()) > 0.3,
           f"all kept keyframes are > spacing apart (min gap "
           f"{float(d.min()):.2f} m)")

    # spacing <= 0 keeps everything; empty in -> empty out.
    _check(select_spread_keyframes(pos, spacing=0.0).tolist()
           == list(range(len(pos))),
           "spacing<=0 keeps every keyframe")
    _check(select_spread_keyframes(np.zeros((0, 3))).shape == (0,),
           "empty positions -> empty selection")


def test_module_constants() -> None:
    print("surface_mesh_selftest: tunable constants present")
    for name in ("KF_SPACING_M", "MESH_STRIDE", "EDGE_MAX_M",
                 "MIN_DEPTH_M", "MAX_DEPTH_M", "MAX_TRIANGLES"):
        _check(hasattr(surface_mesh, name), f"constant {name} defined")
    _check(surface_mesh.KF_SPACING_M > 0.0, "KF_SPACING_M > 0")
    _check(surface_mesh.MESH_STRIDE >= 1, "MESH_STRIDE >= 1")
    _check(surface_mesh.EDGE_MAX_M > 0.0, "EDGE_MAX_M > 0 (curtain reject on)")
    _check(surface_mesh.MIN_DEPTH_M < surface_mesh.MAX_DEPTH_M,
           "depth gate band is ordered")
    _check(surface_mesh.MAX_TRIANGLES > 0, "MAX_TRIANGLES > 0")


def main() -> int:
    test_flat_patch()
    test_depth_step_reject()
    test_invalid_depth_skipped()
    test_select_spread_keyframes()
    test_module_constants()
    print("\nALL SURFACE_MESH SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
