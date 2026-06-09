"""Unit selftests for the pure-numpy floor-plan builder (ui.viz.floor_plan).

No Qt, no GL, no IPC -- just the projection (back-project keyframe depth by pose)
+ the ground-plane occupancy histogram on SYNTHETIC inputs with a known answer:

* ``test_backproject_single_pixel`` -- one depth pixel back-projects to the
  pinhole-predicted world point under identity AND a translated/rotated pose.
* ``test_ground_plane_projection`` -- the builder drops the optical-y (DOWN)
  axis: two points differing only in y land in the SAME ground cell.
* ``test_extent_and_cell_index`` -- a known point lands in the expected raster
  cell and the world<->pixel extent round-trips.
* ``test_wall_outscores_floor`` -- a tall column of points (a wall) scores higher
  (brighter) than a flat slab (floor) at the same point count, so walls read as
  the outline.
* ``test_min_cell_count_gate`` -- a cell under the count gate is dropped (the
  radial stereo-noise floor) while a dense cell survives.
* ``test_camera_path_projection`` -- the camera path projects onto the same grid
  pixels the raster uses.
* ``test_empty_inputs`` -- no points -> a harmless 1x1 raster + degenerate extent.

Run: ``.venv/bin/python -m ui.tests.floor_plan_selftest``
"""
from __future__ import annotations

import numpy as np

from ui.viz import floor_plan


def _check(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)
    print(f"  ok: {msg}")


# A simple pinhole intrinsic for the synthetic depth maps (fx=fy=100, principal
# point at the image centre of a 64x48 grid).
_K = np.array([[100.0, 0.0, 32.0],
               [0.0, 100.0, 24.0],
               [0.0, 0.0, 1.0]], dtype=np.float64)


def test_backproject_single_pixel() -> None:
    """One valid depth pixel -> the pinhole world point, under I and a pose."""
    h, w = 48, 64
    depth = np.zeros((h, w), np.float32)
    # Put a single valid pixel exactly at the principal point (cx,cy)=(32,24) at
    # z=2.0 m. Back-projection there is (0,0,z) in the camera frame.
    depth[24, 32] = 2.0
    # No edge reject (a lone pixel against zero-depth neighbours would be culled);
    # we want to verify the geometry, not the reject (covered elsewhere).
    pts = floor_plan.keyframes_to_ground_points(
        [depth], [np.eye(3)], [np.zeros(3)], _K, stride=1, edge_max=0.0)
    _check(pts.shape == (1, 3), f"identity pose -> exactly one point ({pts.shape})")
    _check(np.allclose(pts[0], [0.0, 0.0, 2.0], atol=1e-4),
           f"principal-point pixel back-projects to (0,0,z) ({pts[0]})")

    # A pure translation t=(1,-2,3) must shift the world point by t (Xw=R Xc+t).
    t = np.array([1.0, -2.0, 3.0])
    pts_t = floor_plan.keyframes_to_ground_points(
        [depth], [np.eye(3)], [t], _K, stride=1, edge_max=0.0)
    _check(np.allclose(pts_t[0], [1.0, -2.0, 5.0], atol=1e-4),
           f"translated pose shifts the world point by t ({pts_t[0]})")

    # A 90-deg rotation about optical-y maps camera +z -> world +x. The camera
    # point is (0,0,2); Xw = R Xc should be (2,0,0).
    Ry = np.array([[0.0, 0.0, 1.0],
                   [0.0, 1.0, 0.0],
                   [-1.0, 0.0, 0.0]])
    pts_r = floor_plan.keyframes_to_ground_points(
        [depth], [Ry], [np.zeros(3)], _K, stride=1, edge_max=0.0)
    _check(np.allclose(pts_r[0], [2.0, 0.0, 0.0], atol=1e-4),
           f"rotation maps camera +z to world +x ({pts_r[0]})")


def test_ground_plane_projection() -> None:
    """The builder drops optical-y (DOWN): same (x,z), different y -> same cell."""
    # Two points at the SAME (x,z)=(0.5, 0.5) but very different y (height): they
    # must accumulate in ONE ground cell (the vertical axis is dropped).
    # min_cell_count=1 so the 2-point cell isn't dropped by the noise gate (the
    # gate is tested separately) -- here we verify the PROJECTION only.
    pts = np.array([[0.5, -1.0, 0.5],
                    [0.5, 2.0, 0.5]], dtype=np.float64)
    rgb, extent = floor_plan.build_floor_plan(pts, cell_m=0.1, min_cell_count=1)
    # The two points span x in [0.5,0.5], z in [0.5,0.5] -> a 1x1 grid, both in it.
    _check(extent.width == 1 and extent.height == 1,
           f"co-located (x,z) points -> a 1x1 ground grid ({extent.width}x"
           f"{extent.height})")
    _check(rgb.shape == (1, 1, 3),
           f"raster is 1x1x3 for one occupied cell ({rgb.shape})")
    # That single cell saw 2 points spanning 3 m of height -> a strong score ->
    # a bright (non-background) colour.
    _check(int(rgb[0, 0].max()) > 60,
           f"the occupied cell is lit, not background ({rgb[0,0]})")


def test_extent_and_cell_index() -> None:
    """A known point lands in the expected cell; the extent round-trips."""
    # Points spanning x in [0,1], z in [0,2] at cell 0.5 -> width=3 (0,0.5,1),
    # height=5 (0,..,2). A point at (1.0, *, 2.0) is the top-right corner cell.
    pts = np.array([[0.0, 0.0, 0.0],
                    [1.0, 0.0, 2.0]], dtype=np.float64)
    rgb, extent = floor_plan.build_floor_plan(pts, cell_m=0.5, min_cell_count=1)
    _check(extent.width == 3 and extent.height == 5,
           f"grid is 3x5 cells for the (1m x 2m)/0.5 extent ({extent.width}x"
           f"{extent.height})")
    _check(abs(extent.x_min - 0.0) < 1e-9 and abs(extent.z_min - 0.0) < 1e-9,
           f"extent origin at the min corner ({extent.x_min},{extent.z_min})")
    # world_xz_to_px round-trip: (x=0.5, z=1.0) -> col 1, row 2.
    col, row = extent.world_xz_to_px(np.array([0.5]), np.array([1.0]))
    _check(abs(float(col[0]) - 1.0) < 1e-9 and abs(float(row[0]) - 2.0) < 1e-9,
           f"world->px maps (0.5,1.0) to (col 1, row 2) ({col[0]},{row[0]})")
    # The two corner points must light their corner cells (row 0 col 0 and the
    # last row/col), and the empty interior must stay background-dark.
    _check(int(rgb[0, 0].max()) > 60 and int(rgb[4, 2].max()) > 60,
           "both corner points light their cells")
    _check(int(rgb[2, 1].sum()) < int(rgb[0, 0].sum()),
           "an empty interior cell is darker than an occupied corner")


def test_wall_outscores_floor() -> None:
    """A tall column (wall) scores brighter than a flat slab (floor) at = count."""
    n = 200
    rng = np.random.default_rng(0)
    # FLOOR slab: many points spread in (x,z) over a 1x1 m patch at ~one height
    # (tiny y jitter). Placed around x~0 so it sits in its own grid region.
    floor_x = rng.uniform(-0.5, 0.5, n)
    floor_z = rng.uniform(-0.5, 0.5, n)
    floor_y = rng.normal(1.0, 0.01, n)                 # ~flat in height
    floor_pts = np.stack([floor_x, floor_z * 0 + 0.0, floor_z], axis=1)
    floor_pts[:, 1] = floor_y

    # WALL column: the SAME number of points but concentrated in ONE (x,z) cell,
    # spanning a 2 m vertical extent (floor->ceiling). Same count per cell as the
    # floor's busiest cell would be far lower, so to compare fairly we put BOTH in
    # a 1x1 cell each and check the wall cell is brighter.
    wall_x = rng.normal(5.0, 0.005, n)                 # all in one cell, far away
    wall_z = rng.normal(5.0, 0.005, n)
    wall_y = rng.uniform(0.0, 2.0, n)                  # tall column
    wall_pts = np.stack([wall_x, wall_y, wall_z], axis=1)

    pts = np.concatenate([floor_pts, wall_pts], axis=0)
    rgb, extent = floor_plan.build_floor_plan(
        pts, cell_m=0.1, height_weight=floor_plan.HEIGHT_WEIGHT)
    # Find the wall cell (near x=5,z=5) and the brightest floor cell.
    wcol, wrow = extent.world_xz_to_px(np.array([5.0]), np.array([5.0]))
    wcol = int(np.clip(round(float(wcol[0])), 0, extent.width - 1))
    wrow = int(np.clip(round(float(wrow[0])), 0, extent.height - 1))
    wall_bright = int(rgb[wrow, wcol].sum())
    # The floor occupies the region around x,z in [-0.5,0.5]; sample its cells.
    fcol0, frow0 = extent.world_xz_to_px(np.array([-0.5]), np.array([-0.5]))
    fcol1, frow1 = extent.world_xz_to_px(np.array([0.5]), np.array([0.5]))
    fr = slice(int(max(0, frow0[0])), int(min(extent.height, frow1[0] + 1)))
    fc = slice(int(max(0, fcol0[0])), int(min(extent.width, fcol1[0] + 1)))
    floor_region = rgb[fr, fc].reshape(-1, 3).sum(axis=1)
    floor_bright = int(floor_region.max()) if floor_region.size else 0
    _check(wall_bright > floor_bright,
           f"wall column ({wall_bright}) outscores the brightest floor cell "
           f"({floor_bright}) -> walls read as the outline")


def test_min_cell_count_gate() -> None:
    """Cells under ``min_cell_count`` points are dropped (the radial noise floor)."""
    # A dense cluster (5 pts in one cell) far from a sparse pair (2 pts in another):
    # at min_cell_count=3 the sparse cell must read as background, the dense as lit.
    dense = np.array([[5.0, 0.0, 5.0]] * 5, dtype=np.float64)
    dense[:, 1] = np.linspace(0.0, 1.0, 5)          # give it some vertical extent
    sparse = np.array([[0.0, 0.0, 0.0]] * 2, dtype=np.float64)
    pts = np.concatenate([dense, sparse], axis=0)
    rgb, extent = floor_plan.build_floor_plan(pts, cell_m=0.1, min_cell_count=3)
    dcol, drow = extent.world_xz_to_px(np.array([5.0]), np.array([5.0]))
    scol, srow = extent.world_xz_to_px(np.array([0.0]), np.array([0.0]))
    dcol = int(np.clip(round(float(dcol[0])), 0, extent.width - 1))
    drow = int(np.clip(round(float(drow[0])), 0, extent.height - 1))
    scol = int(np.clip(round(float(scol[0])), 0, extent.width - 1))
    srow = int(np.clip(round(float(srow[0])), 0, extent.height - 1))
    _check(int(rgb[drow, dcol].max()) > 60,
           f"the dense (5-pt) cell is lit ({rgb[drow,dcol]})")
    _check(int(rgb[srow, scol].max()) < 60,
           f"the sparse (2-pt) cell is dropped as noise ({rgb[srow,scol]})")


def test_camera_path_projection() -> None:
    """The camera path projects onto the same grid pixels the raster uses."""
    pts = np.array([[0.0, 0.0, 0.0],
                    [2.0, 0.0, 3.0]], dtype=np.float64)
    cams = np.array([[0.0, 0.5, 0.0],     # at the (x,z) origin corner
                     [2.0, 0.5, 3.0]],    # at the far corner
                    dtype=np.float64)
    rgb, path_px, extent = floor_plan.floor_plan_with_path(
        pts, cams, cell_m=0.5, min_cell_count=1)
    _check(path_px.shape == (2, 2), f"path has one (col,row) per cam ({path_px.shape})")
    # cam0 at world (0,0) -> pixel (0,0); cam1 at (2,3) -> (col 4, row 6).
    _check(np.allclose(path_px[0], [0.0, 0.0], atol=1e-6),
           f"first cam projects to the origin pixel ({path_px[0]})")
    _check(np.allclose(path_px[1], [4.0, 6.0], atol=1e-6),
           f"last cam projects to the far-corner pixel ({path_px[1]})")
    # The path pixels must lie within the raster (the window draws them on it).
    _check(0 <= path_px[:, 0].max() <= extent.width and
           0 <= path_px[:, 1].max() <= extent.height,
           "every path pixel is inside the raster extent")


def test_empty_inputs() -> None:
    """No points -> a harmless 1x1 raster + degenerate extent (no crash)."""
    rgb, extent = floor_plan.build_floor_plan(np.zeros((0, 3), np.float64))
    _check(rgb.shape == (1, 1, 3), f"empty -> 1x1x3 raster ({rgb.shape})")
    _check(extent.width == 1 and extent.height == 1, "empty -> 1x1 extent")
    # Empty keyframe list -> empty point cloud.
    pts = floor_plan.keyframes_to_ground_points([], [], [], _K)
    _check(pts.shape == (0, 3), f"no keyframes -> no points ({pts.shape})")


def main() -> int:
    print("test_backproject_single_pixel"); test_backproject_single_pixel()
    print("test_ground_plane_projection"); test_ground_plane_projection()
    print("test_extent_and_cell_index"); test_extent_and_cell_index()
    print("test_wall_outscores_floor"); test_wall_outscores_floor()
    print("test_min_cell_count_gate"); test_min_cell_count_gate()
    print("test_camera_path_projection"); test_camera_path_projection()
    print("test_empty_inputs"); test_empty_inputs()
    print("\nALL FLOOR_PLAN SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
