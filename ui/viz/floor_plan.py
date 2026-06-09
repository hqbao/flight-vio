"""UI-only top-down FLOOR-PLAN builder (pure numpy, no GL / no Qt / no display).

The 3D map viewers (point cloud, voxel, surface mesh) are heavy GL on this Mac
AND hard to read (noisy marginal depth seen in perspective). This module builds a
LIGHT, READABLE alternative: a 2D top-down OCCUPANCY raster of the room -- the
walls/vertical structure read as a top-down OUTLINE, with the camera path drawn
over it -- so the room LAYOUT is obvious at a glance. Because the result is a 2D
raster it renders as a cheap pyqtgraph ``ImageItem`` (no ``GLViewWidget``), and --
crucially -- it can be written to a PNG with pure numpy/cv2 with NO GL/display, so
the build is VISUALLY VERIFIABLE offscreen (the GL viewers were not).

It is a pure CONSUMER of the SAME VIO keyframe feed the 3D maps use (denoised
``depth_m`` + each keyframe's own VIO pose ``[R | t]``): no new topic, no
data-path change, no new dependency.

Frame / projection plane (which axis is the ground, which is height)
--------------------------------------------------------------------
The keyframe poses + the back-projected world points live in the CAMERA-OPTICAL
world frame (OpenCV optical: ``+x`` right, ``+y`` DOWN, ``+z`` forward), the same
frame the SLAM-map / surface-mesh builders use. The viewer's optical->NED map is
``_M_OPT_TO_NED = [[0,0,1],[1,0,0],[0,1,0]]``, whose Down row picks the optical
``+y`` axis -- so optical ``+y`` is world-DOWN (the VERTICAL axis) and the GROUND
plane is the optical ``(x, z)`` plane. The floor plan therefore:

* PROJECTS each world point onto ``(x, z)`` (drops the vertical ``y``), and
* uses ``y`` (vertical extent within a cell) to tell a WALL (a tall column of
  points spanning floor->ceiling) from the FLOOR (points at ~one height).

Occupancy weighting (why walls read as an outline)
--------------------------------------------------
A vertical wall is hit by depth pixels across its whole height, so its ground
cell accumulates MANY points spanning a LARGE vertical extent; a patch of floor
accumulates points at ~one height (small extent). We therefore score each cell by
its point COUNT modulated by its vertical EXTENT (see :data:`HEIGHT_WEIGHT`), so
tall structure (walls) outscores flat floor -- the room reads as an outline with
the floor kept faint. The score is then mapped through a dark->bright colormap.

Everything here is pure numpy + a single ``np.add.at`` histogram, so a rebuild is
far cheaper than a 3D mesh build (it can run at a higher rate than the surface
map). No Qt, no GL, no device, no comms.
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- #
# Tunable build constants (the source binds these into its build; each is
# commented with which way to turn it).
# --------------------------------------------------------------------------- #
#: Ground-plane grid cell size (m). Each occupancy cell is CELL_M x CELL_M on the
#: optical ``(x, z)`` floor. ~8 cm is fine enough to resolve a wall as a thin
#: outline yet coarse enough that a handful of stray depth points don't speckle
#: the raster. LOWER for a finer (sharper but noisier / larger) plan; RAISE for a
#: coarser, smoother, lighter plan.
CELL_M = 0.08
#: Depth-map subsample stride: bin every ``STRIDE``-th pixel in u and v. The plan
#: only needs the room SHAPE, not every pixel, so a stride of 4 (1/16 the points)
#: keeps the build cheap while the walls/outline survive. LOWER (toward 1) for a
#: denser, heavier plan; RAISE for a lighter, sparser one.
STRIDE = 4
#: Valid-depth band (m) for the floor plan. ``MIN`` matches the other builders
#: (below it stereo is unreliable). ``MAX`` is deliberately TIGHTER than the
#: SLAM-map / surface builders' 6.0 m: a dense per-pixel occupancy plan seen
#: top-down is dominated by the FAR depth, where stereo range error grows ~with
#: range^2 and SPRAYS points radially along each viewing ray -- a "starburst" fan
#: from every camera that smears the walls. Those fans are the single biggest
#: readability killer top-down, so the plan uses only the RELIABLE near band
#: (~2.5 m), which both gold sessions read far more clearly at (verified by the
#: saved PNGs). The sparse SLAM landmark map can afford 6 m because it keeps only
#: PnP-inlier landmarks; the dense plan cannot. RAISE for more reach in a big room
#: (at the cost of more radial fan); LOWER for an even crisper near outline.
MIN_DEPTH_M = 0.3
MAX_DEPTH_M = 2.5
#: Edge-reject threshold (m): drop "flying pixels" on a depth discontinuity (a
#: foreground/background edge back-projects to points floating BETWEEN the two
#: surfaces, which would smear the plan). A pixel is kept only if BOTH its
#: vertical and horizontal depth gradient are <= this. 0 disables the reject.
#: SAME idea as the surface mesh's ``EDGE_MAX_M``.
EDGE_MAX_M = 0.1
#: How strongly a cell's VERTICAL EXTENT boosts its occupancy score, so a wall
#: (tall column of points) outscores the floor (points at ~one height). The cell
#: score is ``count * (1 + HEIGHT_WEIGHT * extent_m)``: 0 -> pure point count
#: (walls still read because they collect more points, but the floor competes);
#: HIGHER -> walls dominate more strongly (floor fades further). ~2.5/m keeps a
#: clear wall outline with a faint floor.
HEIGHT_WEIGHT = 2.5
#: Cap on the grid's larger side (cells). A runaway extent (a diverged pose
#: shooting a point far away) must not allocate a giant raster; clamp the longer
#: axis to this many cells (the build then drops out-of-grid points). At
#: CELL_M=0.08 this is ~80 m across -- far beyond any indoor room.
MAX_GRID_CELLS = 1024
#: Percentile the score raster is normalised against (instead of the raw max) so a
#: single very dense cell can't wash the whole plan out to one faint tone. The
#: top few percent of cells saturate to full brightness; the rest spread across
#: the ramp. RAISE toward 100 to use the true max (more contrast lost to outliers).
SCORE_CLIP_PCT = 99.0
#: Minimum points a ground cell must collect to be drawn (else it is treated as
#: empty). This is the floor-plan analogue of ``voxel_downsample``'s ``min_count``:
#: raw stereo depth at far/grazing range SPRAYS thin radial noise outward from each
#: camera (few points per cell, spread across many cells), whereas a real surface
#: is hit by MANY rays across keyframes so its cells are dense. Dropping cells
#: under this count removes the radial "starburst" noise and leaves the walls as a
#: crisp outline. RAISE for a cleaner (sparser, more holes) plan; LOWER to keep
#: fainter structure (noisier). 3 mirrors the voxel fuse default.
MIN_CELL_COUNT = 3


class FloorPlanExtent:
    """World<->pixel mapping for a built floor-plan raster (optical (x, z) plane).

    The raster rows index the optical ``z`` (forward) axis and columns index the
    optical ``x`` (right) axis; ``(x_min, z_min)`` is the world coordinate of the
    raster's ``(col=0, row=0)`` corner and ``cell_m`` the metres-per-cell. This is
    everything a window needs to place the ``ImageItem`` in world metres (so pan/
    zoom read in metres) and to map the camera path onto the SAME pixels.

    Stored as a tiny POD (no numpy state) so it is trivially picklable / loggable
    and the window can position the image with ``setRect``.
    """

    __slots__ = ("x_min", "z_min", "cell_m", "width", "height")

    def __init__(self, x_min: float, z_min: float, cell_m: float,
                 width: int, height: int) -> None:
        self.x_min = float(x_min)
        self.z_min = float(z_min)
        self.cell_m = float(cell_m)
        self.width = int(width)        # raster columns (along optical x)
        self.height = int(height)      # raster rows    (along optical z)

    # ------------------------------------------------------------------ #
    def world_xz_to_px(self, x: np.ndarray, z: np.ndarray) -> tuple[np.ndarray,
                                                                    np.ndarray]:
        """Optical ``(x, z)`` world metres -> fractional raster ``(col, row)``.

        Vectorised; the caller rounds/clips as needed. Used by the window to draw
        the camera path on the SAME pixel grid as the occupancy raster.
        """
        col = (np.asarray(x, np.float64) - self.x_min) / self.cell_m
        row = (np.asarray(z, np.float64) - self.z_min) / self.cell_m
        return col, row

    def world_extent(self) -> tuple[float, float, float, float]:
        """``(x_min, x_max, z_min, z_max)`` world bounds of the raster (metres)."""
        return (self.x_min, self.x_min + self.width * self.cell_m,
                self.z_min, self.z_min + self.height * self.cell_m)


# --------------------------------------------------------------------------- #
def keyframes_to_ground_points(depths, Rs, ts, K, *,
                               stride: int = STRIDE,
                               min_depth: float = MIN_DEPTH_M,
                               max_depth: float = MAX_DEPTH_M,
                               edge_max: float = EDGE_MAX_M):
    """Back-project keyframe depth maps to world points, gated + strided.

    Mirrors the SLAM-map / surface builders' geometry: each keyframe's depth is
    back-projected with the pinhole to its camera frame and transformed to the
    camera-optical WORLD frame by the keyframe's OWN VIO pose ``Xw = R Xc + t``
    (one pose source per keyframe -> seam-free). The depth grid is subsampled by
    ``stride`` and gated to ``[min_depth, max_depth]``; when ``edge_max > 0`` a
    pixel on a depth discontinuity (a "flying pixel") is also dropped.

    * ``depths`` -- list of ``(H,W)`` metric depth maps.
    * ``Rs`` / ``ts`` -- per-keyframe ``(3,3)`` rotation / ``(3,)`` translation.
    * ``K`` -- ``(3,3)`` intrinsic for the full-res depth grid.

    Returns ``(N,3)`` float32 world points in the optical frame (empty when none
    valid). Pure numpy; each keyframe is back-projected VECTORISED (no per-pixel
    loop), then all keyframes are stacked.
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    s = max(1, int(stride))
    parts: list[np.ndarray] = []
    for i in range(len(depths)):
        d = np.asarray(depths[i], dtype=np.float32)
        if d.ndim != 2:
            continue
        h, w = d.shape
        # Per-pixel validity over the FULL grid first, so the edge gradient is
        # computed on the native resolution (a discontinuity between adjacent
        # full-res pixels), THEN subsampled by stride -- matching the dense
        # geometry helper's edge reject.
        m = np.isfinite(d) & (d >= float(min_depth)) & (d <= float(max_depth))
        if edge_max > 0.0:
            # Drop flying pixels: a foreground/background edge interpolates to
            # points floating between the two surfaces. ``append`` keeps the diff
            # the same shape as the grid (the last row/col compares to itself).
            dv = np.abs(np.diff(d, axis=0, append=d[-1:]))
            dh = np.abs(np.diff(d, axis=1, append=d[:, -1:]))
            m &= (dv <= float(edge_max)) & (dh <= float(edge_max))
        keep = m[::s, ::s]
        if not np.any(keep):
            continue
        # Pixel coordinates of the kept, subsampled grid.
        us = np.arange(0, w, s, dtype=np.float64)
        vs = np.arange(0, h, s, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)                      # (r0, c0)
        z = d[::s, ::s].astype(np.float64)
        uu, vv, z = uu[keep], vv[keep], z[keep]           # flat (M,)
        # Pinhole back-projection to the camera frame, then to the world by the
        # keyframe's OWN pose.
        cam = np.stack([(uu - cx) * z / fx, (vv - cy) * z / fy, z], axis=1)
        R = np.asarray(Rs[i], dtype=np.float64).reshape(3, 3)
        t = np.asarray(ts[i], dtype=np.float64).reshape(3)
        parts.append((cam @ R.T + t).astype(np.float32))
    if not parts:
        return np.zeros((0, 3), np.float32)
    return np.concatenate(parts, axis=0)


def _colormap(t: np.ndarray) -> np.ndarray:
    """Normalised score ``(H,W)`` in [0,1] -> RGB ``(H,W,3)`` uint8 floor-plan.

    A clean dark->bright single-hue ramp (deep navy background -> cyan-white
    structure): the FLOOR (low score) stays a faint dark blue while WALLS (high
    score) read as a bright cyan-white outline. Monotonic in ``t`` so brighter ==
    more occupied, which is the reading the eye expects. Pure numpy (no matplotlib).
    """
    t = np.clip(np.asarray(t, np.float64), 0.0, 1.0)
    # Background floor of the ramp so an empty cell is a dark (not pure black)
    # navy -- the plan reads as a "lit room" rather than holes in black.
    r = np.clip(0.05 + 0.95 * t ** 1.3, 0.0, 1.0)         # red rises last
    g = np.clip(0.08 + 0.92 * t ** 0.9, 0.0, 1.0)         # green mid
    b = np.clip(0.20 + 0.80 * t ** 0.6, 0.0, 1.0)         # blue lifts first
    rgb = np.stack([r, g, b], axis=-1)
    return (rgb * 255.0 + 0.5).astype(np.uint8)


def build_floor_plan(points: np.ndarray, *,
                     cell_m: float = CELL_M,
                     height_weight: float = HEIGHT_WEIGHT,
                     score_clip_pct: float = SCORE_CLIP_PCT,
                     min_cell_count: int = MIN_CELL_COUNT,
                     max_grid_cells: int = MAX_GRID_CELLS):
    """Bin world points onto the ground plane -> an occupancy raster + extent.

    ``points`` are ``(N,3)`` world points in the camera-optical frame (from
    :func:`keyframes_to_ground_points`). They are projected onto the GROUND plane
    by DROPPING the vertical optical ``+y`` (down) axis -- so the plan uses optical
    ``x`` (right, raster columns) and ``z`` (forward, raster rows). Each cell is
    scored by its point COUNT modulated by the VERTICAL EXTENT of the points in it
    (``count * (1 + height_weight * extent_m)``) so a wall (a tall column) outscores
    the floor; cells under ``min_cell_count`` points are dropped (the radial stereo
    noise floor); the score is normalised to the ``score_clip_pct`` percentile (so
    one dense cell can't wash the plan out) and mapped through :func:`_colormap`.

    Returns ``(rgb (H,W,3) uint8, extent FloorPlanExtent)``. With no points a 1x1
    black raster + a degenerate extent is returned (the window shows an empty plan).

    Implementation: a single ``np.add.at`` scatter accumulates per-cell count, sum
    of height and sum of height^2 -- so the per-cell vertical extent is computed
    without any Python loop over cells (``extent ~= sqrt(var) * 2``, a robust
    spread proxy). Pure numpy, O(N).
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if pts.shape[0] == 0:
        return (np.zeros((1, 1, 3), np.uint8),
                FloorPlanExtent(0.0, 0.0, float(cell_m), 1, 1))

    x = pts[:, 0]        # optical right  -> raster columns
    y = pts[:, 1]        # optical DOWN   -> vertical (height) axis, NOT binned
    z = pts[:, 2]        # optical forward-> raster rows
    cell = float(cell_m)

    # Grid bounds from the point cloud's (x, z) extent, padded one cell so the
    # extreme points fall strictly inside the raster.
    x_min, x_max = float(x.min()), float(x.max())
    z_min, z_max = float(z.min()), float(z.max())
    width = int(np.floor((x_max - x_min) / cell)) + 1
    height = int(np.floor((z_max - z_min) / cell)) + 1
    width = max(1, min(width, int(max_grid_cells)))
    height = max(1, min(height, int(max_grid_cells)))

    # Cell index per point; clip so any point beyond the (clamped) grid lands on
    # the border cell rather than indexing out of bounds.
    col = np.clip(((x - x_min) / cell).astype(np.int64), 0, width - 1)
    row = np.clip(((z - z_min) / cell).astype(np.int64), 0, height - 1)
    flat = row * width + col                                  # row-major cell id
    ncells = width * height

    # Scatter-accumulate per cell: count, sum(y), sum(y^2). One pass, no per-cell
    # loop. The vertical EXTENT proxy is 2*std(y) = 2*sqrt(E[y^2]-E[y]^2).
    count = np.zeros(ncells, np.float64)
    sum_y = np.zeros(ncells, np.float64)
    sum_y2 = np.zeros(ncells, np.float64)
    np.add.at(count, flat, 1.0)
    np.add.at(sum_y, flat, y)
    np.add.at(sum_y2, flat, y * y)
    nz = count > 0
    mean_y = np.zeros(ncells, np.float64)
    var_y = np.zeros(ncells, np.float64)
    mean_y[nz] = sum_y[nz] / count[nz]
    var_y[nz] = np.maximum(sum_y2[nz] / count[nz] - mean_y[nz] ** 2, 0.0)
    extent_m = 2.0 * np.sqrt(var_y)                          # ~full vertical span

    # Cell score: point count BOOSTED by vertical extent so a wall (tall column)
    # outscores a flat floor patch (extent ~0). Floor cells keep a small score so
    # the floor reads faint rather than vanishing.
    score = count * (1.0 + float(height_weight) * extent_m)
    # Drop the thin radial stereo-noise floor: a cell hit by fewer than
    # ``min_cell_count`` points is treated as empty (real surfaces are hit by many
    # rays across keyframes; noise sprays thin). This is what turns the radial
    # "starburst" into a crisp wall outline.
    score[count < float(min_cell_count)] = 0.0

    # Normalise to a high percentile of the NON-EMPTY scores (not the raw max) so a
    # single very dense cell doesn't compress everything else to near-black.
    pos = score[score > 0]
    if pos.size:
        hi = float(np.percentile(pos, float(score_clip_pct)))
    else:
        hi = 1.0
    hi = hi if hi > 1e-9 else 1.0
    norm = np.clip(score / hi, 0.0, 1.0).reshape(height, width)

    rgb = _colormap(norm)
    extent = FloorPlanExtent(x_min, z_min, cell, width, height)
    return rgb, extent


def floor_plan_with_path(points: np.ndarray, cams: np.ndarray, *,
                         cell_m: float = CELL_M,
                         height_weight: float = HEIGHT_WEIGHT,
                         min_cell_count: int = MIN_CELL_COUNT):
    """Convenience: build the raster AND project the camera path onto its pixels.

    Returns ``(rgb (H,W,3) uint8, path_px (M,2) float32, extent)`` where
    ``path_px`` is the keyframe camera positions ``cams`` (``(M,3)`` optical-world)
    projected to fractional raster ``(col, row)`` on the SAME grid as the raster --
    so a caller (the offscreen PNG verifier, a test) can overlay the path without
    re-deriving the extent. The window draws the path itself in world metres via
    the returned ``extent`` (see :class:`FloorPlanExtent`); this helper is mainly
    for the headless PNG check + the unit tests.
    """
    rgb, extent = build_floor_plan(points, cell_m=cell_m,
                                   height_weight=height_weight,
                                   min_cell_count=min_cell_count)
    cams = np.asarray(cams, dtype=np.float64).reshape(-1, 3)
    if cams.shape[0] == 0:
        return rgb, np.zeros((0, 2), np.float32), extent
    col, row = extent.world_xz_to_px(cams[:, 0], cams[:, 2])
    path_px = np.stack([col, row], axis=1).astype(np.float32)
    return rgb, path_px, extent
