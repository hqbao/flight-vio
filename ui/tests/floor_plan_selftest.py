"""Unit selftests for the pure-numpy+cv2 floor-plan builder (ui.viz.floor_plan).

No Qt, no GL, no IPC -- just the projection (back-project keyframe depth by pose)
+ the ground-plane occupancy / wall-outline cleanup on SYNTHETIC inputs with a
known answer:

* ``test_backproject_single_pixel`` -- one depth pixel back-projects to the
  pinhole-predicted world point under identity AND a translated/rotated pose.
* ``test_ground_plane_projection`` -- the builder drops the optical-y (DOWN)
  axis: two points differing only in y land in the SAME ground cell.
* ``test_extent_and_cell_index`` -- a known point lands in the expected raster
  cell and the world<->pixel extent round-trips.
* ``test_floor_extent_gate`` -- a FLAT slab (small vertical extent = floor) is
  dropped from the occupied region while a TALL column (a wall) survives, so the
  outline tracks vertical structure, not the swept floor.
* ``test_noise_island_dropped`` -- a small isolated noise island is removed by the
  morphology + connected-component filter while a large solid region survives.
* ``test_outline_is_boundary`` -- the rendered wall is the OUTLINE (boundary) of a
  solid region: its border cells are lit, its interior stays dark.
* ``test_clean_wall_mask_helper`` -- the cv2 cleanup helper directly: open drops a
  thin streak, the component filter drops a small blob, the gradient is a boundary.
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
    # must accumulate in ONE ground cell (the vertical axis is dropped). Relax the
    # cleanup knobs (count=1, no component-area floor, FILLED not outline) so the
    # single occupied cell isn't scrubbed -- here we verify the PROJECTION only.
    pts = np.array([[0.5, -1.0, 0.5],
                    [0.5, 2.0, 0.5]], dtype=np.float64)
    rgb, extent = floor_plan.build_floor_plan(
        pts, cell_m=0.1, min_cell_count=1, min_component_cells=1, outline=False)
    # The two points span x in [0.5,0.5], z in [0.5,0.5] -> a 1x1 grid, both in it.
    _check(extent.width == 1 and extent.height == 1,
           f"co-located (x,z) points -> a 1x1 ground grid ({extent.width}x"
           f"{extent.height})")
    _check(rgb.shape == (1, 1, 3),
           f"raster is 1x1x3 for one occupied cell ({rgb.shape})")
    # That single cell saw 2 points spanning 3 m of height (a tall column, well
    # over the floor-extent gate) -> it survives as occupied -> a bright colour.
    _check(int(rgb[0, 0].max()) > 60,
           f"the occupied (tall) cell is lit, not background ({rgb[0,0]})")


def test_extent_and_cell_index() -> None:
    """A known point lands in the expected cell; the extent round-trips."""
    # Points spanning x in [0,1], z in [0,2] at cell 0.5 -> width=3 (0,0.5,1),
    # height=5 (0,..,2). A point at (1.0, *, 2.0) is the top-right corner cell.
    pts = np.array([[0.0, 0.0, 0.0],
                    [1.0, 0.0, 2.0]], dtype=np.float64)
    # Relax the cleanup (count=1, no floor-extent gate, component floor=1, FILLED)
    # so the two lone single-height corner cells survive -- here we verify the
    # EXTENT / cell indexing only (the gate + cleanup are tested separately).
    rgb, extent = floor_plan.build_floor_plan(
        pts, cell_m=0.5, min_cell_count=1, floor_extent_m=0.0,
        min_component_cells=1, outline=False)
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


def test_floor_extent_gate() -> None:
    """A flat slab (floor) is gated out by vertical extent; a tall block survives.

    The explicit "wall = vertical extent" gate: a cell whose points span less than
    ``floor_extent_m`` in height is flat floor and dropped from the occupied region,
    so the rendered outline tracks vertical structure (walls), not the swept floor.
    """
    rng = np.random.default_rng(0)
    # A solid FLOOR slab: a 1x1 m patch of many points at ~one height (extent ~0).
    n_floor = 4000
    fx = rng.uniform(0.0, 1.0, n_floor)
    fz = rng.uniform(0.0, 1.0, n_floor)
    fy = rng.normal(1.0, 0.01, n_floor)                # ~flat -> extent << gate
    floor = np.stack([fx, fy, fz], axis=1)
    # A solid TALL block at a far (x,z) region: a 1x1 m footprint of points each
    # spanning a 2 m vertical column (a wall/structure) -> extent >> gate.
    n_wall = 4000
    wx = rng.uniform(5.0, 6.0, n_wall)
    wz = rng.uniform(5.0, 6.0, n_wall)
    wy = rng.uniform(0.0, 2.0, n_wall)                 # tall column -> big extent
    wall = np.stack([wx, wy, wz], axis=1)
    pts = np.concatenate([floor, wall], axis=0)
    # FILLED region (not the outline) so we can probe interior cells; default gate.
    rgb, extent = floor_plan.build_floor_plan(
        pts, cell_m=0.1, floor_extent_m=floor_plan.FLOOR_EXTENT_M, outline=False)

    def region_lit(x0, z0, x1, z1):
        c0, r0 = extent.world_xz_to_px(np.array([x0]), np.array([z0]))
        c1, r1 = extent.world_xz_to_px(np.array([x1]), np.array([z1]))
        rs = slice(int(max(0, r0[0])), int(min(extent.height, r1[0] + 1)))
        cs = slice(int(max(0, c0[0])), int(min(extent.width, c1[0] + 1)))
        sub = rgb[rs, cs].reshape(-1, 3)
        return int(sub.max()) if sub.size else 0

    floor_lit = region_lit(0.0, 0.0, 1.0, 1.0)
    wall_lit = region_lit(5.0, 5.0, 6.0, 6.0)
    # The wall block reads as a bright (near-white) outline; the gated-out floor is
    # at most the faint raw-occupancy context wash (well below the bright outline).
    _check(floor_lit < 120,
           f"the flat floor slab is gated out (only faint context, {floor_lit})")
    _check(wall_lit > 200,
           f"the tall block survives the extent gate (bright outline, {wall_lit})")


def test_noise_island_dropped() -> None:
    """A small isolated noise island is dropped; a large solid region survives.

    The morphology OPEN + connected-component area filter remove the small isolated
    star-burst blobs while keeping the large connected room region.
    """
    rng = np.random.default_rng(1)
    # A LARGE solid tall region (a wall sheet) -> a big connected component.
    n_big = 8000
    bx = rng.uniform(0.0, 2.0, n_big)
    bz = rng.uniform(0.0, 0.4, n_big)                  # a 2.0 x 0.4 m wall footprint
    by = rng.uniform(0.0, 2.0, n_big)                  # tall
    big = np.stack([bx, by, bz], axis=1)
    # A TINY isolated tall island far away (a single ~1-cell speck of noise).
    n_isle = 8
    isx = rng.normal(8.0, 0.01, n_isle)
    isz = rng.normal(8.0, 0.01, n_isle)
    isy = rng.uniform(0.0, 2.0, n_isle)                # tall, so the gate keeps it
    isle = np.stack([isx, isy, isz], axis=1)
    pts = np.concatenate([big, isle], axis=0)
    rgb, extent = floor_plan.build_floor_plan(
        pts, cell_m=0.1, min_component_cells=floor_plan.MIN_COMPONENT_CELLS,
        outline=False)
    # The big region's centre must be lit; the isolated island must be dropped.
    bc, br = extent.world_xz_to_px(np.array([1.0]), np.array([0.2]))
    ic, ir = extent.world_xz_to_px(np.array([8.0]), np.array([8.0]))
    bc = int(np.clip(round(float(bc[0])), 0, extent.width - 1))
    br = int(np.clip(round(float(br[0])), 0, extent.height - 1))
    ic = int(np.clip(round(float(ic[0])), 0, extent.width - 1))
    ir = int(np.clip(round(float(ir[0])), 0, extent.height - 1))
    # The big region reads as a bright (near-white) filled mask; the dropped island
    # is at most the faint raw-occupancy context wash (no bright mask cell).
    _check(int(rgb[br, bc].max()) > 200,
           f"the large solid region survives (bright, {rgb[br,bc]})")
    _check(int(rgb[ir, ic].max()) < 120,
           f"the small isolated noise island is dropped (faint, {rgb[ir,ic]})")


def test_outline_is_boundary() -> None:
    """The wall mask is the OUTLINE of a region: border lit, interior dark.

    With ``outline=True`` (the default) the wall is the morphological-gradient
    boundary of the cleaned occupied region, so an interior cell deep inside a
    solid block stays dark while the block's border is lit -- a thin wall LINE, not
    a filled footprint.
    """
    rng = np.random.default_rng(2)
    # A big solid tall block (3 x 3 m footprint), all points spanning a 2 m column.
    n = 30000
    bx = rng.uniform(0.0, 3.0, n)
    bz = rng.uniform(0.0, 3.0, n)
    by = rng.uniform(0.0, 2.0, n)
    block = np.stack([bx, by, bz], axis=1)
    # A few FLAT corner points well outside the block: they are gated out as floor
    # (so they don't occupy), but they enlarge the raster so the block has FREE
    # SPACE around it to form a boundary against (a block filling the whole raster
    # has no boundary -- a degenerate synthetic case, never a real room).
    corners = np.array([[-1.0, 0.0, -1.0], [4.0, 0.0, -1.0],
                        [-1.0, 0.0, 4.0], [4.0, 0.0, 4.0]], dtype=np.float64)
    pts = np.concatenate([block, corners], axis=0)
    rgb, extent = floor_plan.build_floor_plan(pts, cell_m=0.1, outline=True)
    # The block's centre (1.5, 1.5) is deep interior -> on the outline it is dark.
    cc, cr = extent.world_xz_to_px(np.array([1.5]), np.array([1.5]))
    cc = int(np.clip(round(float(cc[0])), 0, extent.width - 1))
    cr = int(np.clip(round(float(cr[0])), 0, extent.height - 1))
    # The interior is NOT on the outline -> at most the faint context wash, never
    # the bright (near-white) wall line.
    _check(int(rgb[cr, cc].max()) < 120,
           f"the block's deep interior is NOT lit on the outline ({rgb[cr,cc]})")
    # The outline DOES exist (some bright wall-line cells) and is THIN: far fewer
    # bright cells than the ~30x30-cell filled footprint would have (a boundary
    # ring is O(perimeter), not O(area)).
    bright = (rgb.max(axis=2) > 200)
    n_bright = int(bright.sum())
    _check(n_bright > 0, f"the region boundary is lit as the wall outline "
           f"({n_bright} bright cells)")
    _check(n_bright < 0.5 * (extent.width * extent.height),
           f"the outline is THIN (a boundary, not a filled block): {n_bright} of "
           f"{extent.width * extent.height} cells")


def test_clean_wall_mask_helper() -> None:
    """The cv2 cleanup helper directly: open drops streaks, components drop blobs."""
    h, w = 40, 40
    occ = np.zeros((h, w), np.uint8)
    occ[10:30, 10:30] = 1                  # a large solid 20x20 block (a real region)
    occ[5, 0:25] = 1                       # a 1-cell-thick horizontal streak (noise)
    occ[35:38, 36:39] = 1                  # a tiny isolated 3x3 blob (noise island)
    # FILLED first: the open must erase the thin streak, the component filter the
    # tiny blob, leaving only the big block.
    filled = floor_plan._clean_wall_mask(
        occ, open_px=3, close_px=3, min_component_cells=40, outline=False)
    _check(not filled[5, 0:25].any(),
           "MORPH_OPEN erased the 1-cell-thick streak")
    _check(not filled[35:38, 36:39].any(),
           "the connected-component filter dropped the tiny isolated blob")
    _check(filled[20, 20],
           "the large solid block survived the cleanup")
    # OUTLINE: the block's interior is now dark, its border lit (a boundary line).
    line = floor_plan._clean_wall_mask(
        occ, open_px=3, close_px=3, min_component_cells=40, outline=True)
    _check(not line[20, 20],
           "the block's deep interior is dark on the outline")
    _check(line[10, 10] or line[10, 20] or line[20, 10],
           "the block's border is lit as the outline")


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
    print("test_floor_extent_gate"); test_floor_extent_gate()
    print("test_noise_island_dropped"); test_noise_island_dropped()
    print("test_outline_is_boundary"); test_outline_is_boundary()
    print("test_clean_wall_mask_helper"); test_clean_wall_mask_helper()
    print("test_camera_path_projection"); test_camera_path_projection()
    print("test_empty_inputs"); test_empty_inputs()
    print("\nALL FLOOR_PLAN SELFTESTS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
