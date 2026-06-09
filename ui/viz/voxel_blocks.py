"""UI-only occupancy-grid + cube-mesh helpers for the "Room Blocks" viewer.

The vendored :mod:`ui.comms.lib.misc.geometry` is byte-identical across all five
projects and must NOT be edited, so the voxel/occupancy post-processing the Room
Blocks window needs lives here (UI-only, a pure consumer of the same keyframe
depth + pose data the SLAM-map viewer already accumulates).

Two pure functions, both heavily vectorised (numpy only, no KD-tree / no Qt):

* :func:`occupancy_voxels` -- bin the back-projected world points of ALL VIO
  keyframes into a 3D voxel grid and keep only cells that received
  ``>= min_hits`` points (multi-keyframe agreement rejects thin stereo noise).
  Returns the occupied voxel CENTERS + a per-voxel HEIGHT colour so floor /
  walls / ceiling read distinctly.
* :func:`cube_mesh` -- turn ``(M, 3)`` voxel centers + a cube size into ONE
  merged triangle mesh (``M*8`` verts, ``M*12`` faces) so the whole occupancy
  grid renders as a single shaded ``GLMeshItem`` -- vastly cheaper than ``M``
  individual ``GLBoxItem``s.

Camera convention: the world points are in the SAME camera-optical world frame
the VIO keyframe poses live in (``Xw = R Xc + t``); the viewer applies its own
ENU display rotation, exactly like the point-cloud map. "World-down" for the
height colour is therefore the optical-world ``+y`` axis (the OpenCV optical
frame is +x right, +y DOWN, +z forward), so larger ``y`` == lower in the room.
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- #
# Tunable build constants (commented; the source binds these into its build).
# --------------------------------------------------------------------------- #
#: Default voxel edge length (m). ~0.12 m blocks are coarse enough to merge
#: many keyframe hits per cell (so a real surface lights up a solid shell) yet
#: fine enough that a room's walls / door gaps stay recognisable. LOWER for a
#: finer, more detailed (but noisier + heavier) grid; RAISE for chunkier blocks.
VOXEL_M = 0.12
#: A voxel is OCCUPIED only once it has received ``>= MIN_HITS`` back-projected
#: points across the keyframes. Real surfaces are hit by many rays from many
#: viewpoints, so their cells fill up; thin/transient stereo noise lands a stray
#: point or two and is dropped. RAISE for a cleaner (sparser) shell, LOWER to
#: fill the room faster (noisier). 3 mirrors the dense voxel-fuse min_count.
MIN_HITS = 3
#: Valid-depth band (m) for back-projecting a keyframe depth pixel: outside this
#: range stereo depth is too noisy/unreliable, so the pixel is skipped (same band
#: the dense geometry helper + the landmark map use).
MIN_DEPTH_M = 0.3
MAX_DEPTH_M = 6.0
#: Depth-discontinuity reject (m): a pixel whose depth differs from a 4-connected
#: neighbour by more than this is a "flying pixel" on a foreground/background edge
#: (it interpolates to a point floating between two surfaces) and is dropped --
#: the same edge-reject ``keyframe_pointcloud`` applies with its ``edge_max``.
#: 0.0 disables the reject.
EDGE_MAX_M = 0.1
#: Sub-sample stride when back-projecting each keyframe depth map (take every
#: ``STRIDE``-th pixel in u and v). The occupancy grid only needs enough hits per
#: cell to clear ``MIN_HITS``; striding keeps the per-keyframe point budget (and
#: the rebuild cost) bounded without changing the room shape. 3 keeps the live
#: rebuild within the source's ~2.5 Hz budget (~250 ms on the gold sessions ->
#: ~20 k voxels at VOXEL_M=0.12) while the walls/surfaces still fill solidly.
#: LOWER (toward 1 = full grid) for a denser grid at a higher rebuild cost.
STRIDE = 3

# The 8 corner offsets of a unit cube centred at the origin, scaled by the half
# edge at build time. Order is fixed so :data:`_CUBE_FACES` indexes them
# consistently for every cube. (x, y, z) each in {-1, +1}.
_CUBE_CORNERS = np.array([
    [-1.0, -1.0, -1.0],   # 0
    [+1.0, -1.0, -1.0],   # 1
    [+1.0, +1.0, -1.0],   # 2
    [-1.0, +1.0, -1.0],   # 3
    [-1.0, -1.0, +1.0],   # 4
    [+1.0, -1.0, +1.0],   # 5
    [+1.0, +1.0, +1.0],   # 6
    [-1.0, +1.0, +1.0],   # 7
], dtype=np.float64)

# The 12 triangles (2 per face, 6 faces) of the cube, as indices into the 8
# corners above. Wound consistently (CCW seen from outside) so the 'shaded'
# renderer lights every outward face. This is the per-cube face template; the
# merged mesh adds ``cube_index * 8`` to every index so each cube indexes its
# own 8-vertex block.
_CUBE_FACES = np.array([
    [0, 1, 2], [0, 2, 3],   # -z (bottom)
    [4, 6, 5], [4, 7, 6],   # +z (top)
    [0, 4, 5], [0, 5, 1],   # -y
    [3, 2, 6], [3, 6, 7],   # +y
    [0, 3, 7], [0, 7, 4],   # -x
    [1, 5, 6], [1, 6, 2],   # +x
], dtype=np.int64)

#: Vertices / faces a single cube contributes -- the merge offsets by these.
CUBE_VERTS = _CUBE_CORNERS.shape[0]    # 8
CUBE_FACES = _CUBE_FACES.shape[0]      # 12


# --------------------------------------------------------------------------- #
def _height_colors(centers: np.ndarray) -> np.ndarray:
    """Per-voxel RGB ``(M,3)`` float32 keyed by height along world-DOWN.

    Colours by the optical-world ``+y`` (DOWN) coordinate so the floor, walls and
    ceiling read as distinct bands. The value is min-max normalised across the
    occupied cells and mapped through a simple blue(low)->green->red(high) ramp:
    because ``+y`` is DOWN, the FLOOR (max y) comes out warm/red and the CEILING
    (min y) cool/blue -- a stable, recognisable height cue. A degenerate (flat)
    set falls back to a uniform mid colour.
    """
    if centers.shape[0] == 0:
        return np.zeros((0, 3), np.float32)
    y = centers[:, 1].astype(np.float64)         # optical-world DOWN
    lo, hi = float(y.min()), float(y.max())
    span = hi - lo
    if span < 1e-6:                               # flat slab -> uniform colour
        return np.full((centers.shape[0], 3), 0.6, np.float32)
    t = (y - lo) / span                           # 0 (ceiling) .. 1 (floor)
    # Piecewise blue->green->red ramp (a cheap, monotonic "jet"-like map).
    r = np.clip(1.5 * t - 0.25, 0.0, 1.0)
    g = np.clip(1.0 - np.abs(2.0 * t - 1.0), 0.15, 1.0)
    b = np.clip(1.25 - 1.5 * t, 0.0, 1.0)
    return np.stack([r, g, b], axis=1).astype(np.float32)


def occupancy_voxels(points: np.ndarray, *, voxel: float = VOXEL_M,
                     min_hits: int = MIN_HITS):
    """Bin world points into a voxel grid; keep cells with ``>= min_hits`` hits.

    ``points`` is an ``(N,3)`` cloud in the camera-optical world frame (the union
    of ALL keyframes' back-projected depth points). Each point is floored to a
    ``voxel``-metre integer cell key; a cell is OCCUPIED only when it received
    ``>= min_hits`` points (multi-keyframe / multi-ray agreement -- thin stereo
    noise lands too few hits and is dropped). Returns
    ``(centers (M,3) float32, colors (M,3) float32)``: the geometric CENTRE of
    each occupied cell and its height colour (see :func:`_height_colors`).
    Empty in -> empty out.

    Pure numpy: a single ``np.unique`` over the integer keys gives the per-cell
    hit counts in one pass (no KD-tree, no Python loop over points).
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        empty = np.zeros((0, 3), np.float32)
        return empty, empty
    vox = float(voxel)
    # Integer cell key per point (floor divide by the voxel edge).
    keys = np.floor(pts / vox).astype(np.int64)
    uniq, counts = np.unique(keys, axis=0, return_counts=True)
    keep = counts >= int(min_hits)
    if not np.any(keep):
        empty = np.zeros((0, 3), np.float32)
        return empty, empty
    # Cell centre = (key + 0.5) * voxel (the midpoint of the cell the key spans).
    centers = ((uniq[keep].astype(np.float64) + 0.5) * vox).astype(np.float32)
    return centers, _height_colors(centers)


def cube_mesh(centers: np.ndarray, size: float, *,
              colors: np.ndarray | None = None):
    """Merge ``(M,3)`` voxel centers into ONE cube mesh for a single GLMeshItem.

    Builds the vertices + faces of ``M`` axis-aligned cubes of edge ``size`` in a
    fully vectorised way (no per-cube Python loop), so the whole occupancy grid is
    one static mesh the GL renderer draws in a single call:

    * verts ``(M*8, 3)`` -- each cube's centre broadcast over the 8 corner
      offsets (``_CUBE_CORNERS`` scaled by the half edge).
    * faces ``(M*12, 3)`` -- the 12-triangle cube template tiled ``M`` times with
      a per-cube ``+ 8*cube_index`` offset so each cube indexes its own vertices.
    * face_colors ``(M*12, 4)`` -- the per-voxel RGB (if ``colors`` given) lifted
      to opaque RGBA and repeated across that cube's 12 faces, so each block is
      flat-shaded in its height colour.

    Returns ``(verts, faces, face_colors)``. With ``colors=None`` the returned
    ``face_colors`` is ``None`` (the caller may apply a uniform colour). Empty in
    -> empty arrays out (and ``face_colors`` empty/None to match).
    """
    cen = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
    m = cen.shape[0]
    half = float(size) * 0.5
    if m == 0:
        verts = np.zeros((0, 3), np.float32)
        faces = np.zeros((0, 3), np.int64)
        fcols = None if colors is None else np.zeros((0, 4), np.float32)
        return verts, faces, fcols

    # (M,8,3): every cube centre + the 8 half-edge-scaled corner offsets, then
    # flatten to (M*8, 3) so vertex (cube*8 + corner) is contiguous per cube.
    verts = (cen[:, None, :] + _CUBE_CORNERS[None, :, :] * half)
    verts = verts.reshape(m * CUBE_VERTS, 3).astype(np.float32)

    # (M,12,3): the face template + the per-cube vertex-block offset (8*cube),
    # flattened to (M*12, 3). int64 indices into ``verts``.
    offsets = (np.arange(m, dtype=np.int64) * CUBE_VERTS)[:, None, None]
    faces = (_CUBE_FACES[None, :, :] + offsets).reshape(m * CUBE_FACES, 3)

    if colors is None:
        return verts, faces, None
    # Per-voxel RGB -> opaque RGBA, then repeat each cube's colour across its 12
    # faces (face index cube*12 + f) so the merged mesh is flat-shaded per block.
    rgb = np.clip(np.asarray(colors, np.float32).reshape(-1, 3), 0.0, 1.0)
    if rgb.shape[0] != m:                          # mismatch -> uniform grey
        rgb = np.full((m, 3), 0.7, np.float32)
    rgba = np.concatenate([rgb, np.ones((m, 1), np.float32)], axis=1)
    face_colors = np.repeat(rgba, CUBE_FACES, axis=0).astype(np.float32)
    return verts, faces, face_colors
