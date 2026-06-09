#!/usr/bin/env python3
"""Unit tests for :mod:`ui.viz.voxel_blocks` (occupancy grid + cube mesh).

The Room Blocks (3D voxel) viewer turns ALL keyframes' back-projected depth into
a 3D OCCUPANCY GRID (cells with ``>= MIN_HITS`` points survive) and merges the
occupied cells into ONE cube mesh (``M*8`` verts, ``M*12`` faces). These tests
feed hand-checkable inputs and assert:

* :func:`cube_mesh` -- 2 voxel centers -> 16 verts, 24 faces, all face indices in
  ``[0, 16)``; per-face colours are (24, 4) opaque RGBA; cubes are the right size.
* :func:`occupancy_voxels` -- only cells with ``>= MIN_HITS`` points survive, and
  the surviving centres land on the expected cell midpoints.

Run::

    python -m ui.tests.voxel_blocks_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ui.viz import voxel_blocks                                      # noqa: E402
from ui.viz.voxel_blocks import (                                    # noqa: E402
    CUBE_FACES, CUBE_VERTS, cube_mesh, occupancy_voxels,
)


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def test_cube_mesh() -> None:
    print("voxel_blocks_selftest: cube_mesh")
    # Two voxel centers, well separated, cube edge 1.0.
    centers = np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0]], np.float32)
    colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], np.float32)
    verts, faces, fcols = cube_mesh(centers, 1.0, colors=colors)

    _check(verts.shape == (2 * CUBE_VERTS, 3),
           f"2 cubes -> {2 * CUBE_VERTS} verts (got {verts.shape})")
    _check(verts.shape == (16, 3), "16 verts for 2 cubes")
    _check(faces.shape == (2 * CUBE_FACES, 3),
           f"2 cubes -> {2 * CUBE_FACES} faces (got {faces.shape})")
    _check(faces.shape == (24, 3), "24 faces for 2 cubes")

    # Every face index must reference a real vertex (no out-of-range index).
    _check(int(faces.min()) >= 0 and int(faces.max()) < len(verts),
           f"face indices in [0,{len(verts)}) (got [{faces.min()},{faces.max()}])")
    # The second cube's faces must offset into the second 8-vertex block.
    _check(int(faces[CUBE_FACES:].min()) >= CUBE_VERTS,
           "cube 1 faces offset by 8 into its own vertex block")

    # Per-face colours: (24, 4) opaque RGBA, each cube's colour on its 12 faces.
    _check(fcols.shape == (24, 4), f"face colours (24,4) (got {fcols.shape})")
    _check(np.allclose(fcols[:, 3], 1.0), "face colours opaque (alpha == 1)")
    _check(np.allclose(fcols[:CUBE_FACES, :3], [1.0, 0.0, 0.0]),
           "cube 0's 12 faces are red")
    _check(np.allclose(fcols[CUBE_FACES:, :3], [0.0, 1.0, 0.0]),
           "cube 1's 12 faces are green")

    # Cube 0 spans [-0.5, 0.5] in each axis (centre 0, edge 1).
    c0 = verts[:CUBE_VERTS]
    _check(np.allclose(c0.min(axis=0), -0.5) and np.allclose(c0.max(axis=0), 0.5),
           "cube 0 spans the unit cube about its centre")

    # Empty in -> empty out (and face_colors empty, not None, when colours given).
    ev, ef, ec = cube_mesh(np.zeros((0, 3), np.float32), 1.0,
                           colors=np.zeros((0, 3), np.float32))
    _check(ev.shape == (0, 3) and ef.shape == (0, 3) and ec.shape == (0, 4),
           "empty centers -> empty verts/faces/face_colors")
    # colors=None -> face_colors is None.
    _, _, none_c = cube_mesh(centers, 1.0, colors=None)
    _check(none_c is None, "colors=None -> face_colors None")


def test_occupancy_voxels() -> None:
    print("voxel_blocks_selftest: occupancy_voxels")
    vox = 0.10
    mh = 3
    # Cell A (key (0,0,0), centre 0.05^3): 4 points -> survives at MIN_HITS=3.
    a = np.full((4, 3), 0.02, np.float64)
    # Cell B (key (10,0,0)): only 2 points -> dropped at MIN_HITS=3.
    b = np.array([[1.01, 0.0, 0.0], [1.02, 0.0, 0.0]], np.float64)
    # Cell C (key (0,0,10), centre z=1.05): exactly 3 points -> survives.
    c = np.array([[0.01, 0.0, 1.00], [0.02, 0.0, 1.01], [0.03, 0.0, 1.02]],
                 np.float64)
    pts = np.concatenate([a, b, c], axis=0)
    centers, colors = occupancy_voxels(pts, voxel=vox, min_hits=mh)

    _check(centers.shape[0] == 2,
           f"only the 2 cells with >= {mh} hits survive (got {centers.shape[0]})")
    _check(colors.shape == (centers.shape[0], 3),
           f"one colour per surviving cell (got {colors.shape})")

    # The surviving centres are A's (0.05,0.05,0.05) and C's (0.05,0.05,1.05).
    got = {tuple(round(float(x), 3) for x in p) for p in centers}
    want = {(0.05, 0.05, 0.05), (0.05, 0.05, 1.05)}
    _check(got == want, f"survivor centres at cell midpoints (got {sorted(got)})")

    # The dropped cell B (centre x=1.05) must NOT appear among the survivors.
    _check(all(abs(x - 1.05) > 1e-6 for (x, _, _) in got),
           "cell B (2 hits < 3) dropped")

    # Empty in -> empty out.
    ec, ecol = occupancy_voxels(np.zeros((0, 3), np.float64), voxel=vox,
                                min_hits=mh)
    _check(ec.shape == (0, 3) and ecol.shape == (0, 3),
           "empty cloud -> empty centers/colors")


def test_module_constants() -> None:
    print("voxel_blocks_selftest: tunable constants present")
    for name in ("VOXEL_M", "MIN_HITS", "MIN_DEPTH_M", "MAX_DEPTH_M",
                 "EDGE_MAX_M", "STRIDE"):
        _check(hasattr(voxel_blocks, name), f"constant {name} defined")
    _check(0.0 < voxel_blocks.VOXEL_M < 1.0, "VOXEL_M ~ a sane voxel edge (m)")
    _check(voxel_blocks.MIN_HITS >= 1, "MIN_HITS >= 1")
    _check(voxel_blocks.MIN_DEPTH_M < voxel_blocks.MAX_DEPTH_M,
           "depth gate band is ordered")


def main() -> int:
    test_cube_mesh()
    test_occupancy_voxels()
    test_module_constants()
    print("\nALL VOXEL_BLOCKS SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
