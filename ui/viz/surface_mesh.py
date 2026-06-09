"""UI-only depth-map surface meshing for the "Room Surface" viewer.

The vendored :mod:`ui.comms.lib.misc.geometry` is byte-identical across all five
projects and must NOT be edited, so the depth->surface meshing the Room Surface
window needs lives here (UI-only, a pure consumer of the same keyframe depth +
pose data the SLAM-map viewer already accumulates).

Why per-keyframe depth meshing (and not marching cubes / Poisson)
-----------------------------------------------------------------
The project deliberately keeps its deps to numpy + cv2 + pyqtgraph (for the
eventual C port), so the heavy iso-surface libraries (skimage / scipy / open3d /
trimesh) are unavailable. The dependency-free way to turn metric depth into a
CONTINUOUS shaded surface is to mesh EACH keyframe's depth map directly: every
2x2 block of valid depth pixels becomes two triangles, so the depth grid renders
as a connected surface (walls / floor as smooth sheets) instead of disconnected
points or blocky cubes. The only cleanup needed is dropping triangles that span a
depth discontinuity (the "curtain"/"flying-pixel" sheets stretched between a
foreground and a background surface) -- without that reject the room fills with
sheets and nothing reads.

Two pure functions, both heavily vectorised (numpy only, no KD-tree / no Qt):

* :func:`select_spread_keyframes` -- greedy spatial thinning of the VIO keyframes
  by camera POSITION: keep a keyframe only if it is > ``KF_SPACING_M`` from every
  already-kept one. This covers the room from a few well-separated viewpoints and
  bounds overlap/redundancy + total triangle count.
* :func:`depth_surface_mesh` -- back-project ONE keyframe's (subsampled) depth map
  to a world-frame vertex grid, triangulate adjacent grid cells, and keep a
  triangle only when all four corners are valid AND their depth spread is small
  (the edge reject). Returns ``(verts, faces, vertex_colors)`` for that keyframe.

The caller stacks each selected keyframe's mesh into ONE merged ``GLMeshItem``.

Camera convention: the world points are in the SAME camera-optical world frame the
VIO keyframe poses live in (``Xw = R Xc + t``); the viewer applies its own ENU
display rotation, exactly like the point-cloud map. "World-down" for the height
colour is therefore the optical-world ``+y`` axis (the OpenCV optical frame is +x
right, +y DOWN, +z forward), so larger ``y`` == lower in the room.
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------- #
# Tunable build constants (commented; the source binds these into its build).
# --------------------------------------------------------------------------- #
#: Greedy keyframe-spacing threshold (m). A VIO keyframe is KEPT for meshing only
#: if its camera position is > this distance from every already-kept keyframe, so
#: the room is meshed from a few well-separated viewpoints instead of dozens of
#: near-duplicate ones. RAISE for fewer viewpoints (lighter, coarser coverage);
#: LOWER toward 0 to mesh more keyframes (denser, heavier, more overlap). ~0.3 m
#: keeps a handful of viewpoints per room while still covering it.
KF_SPACING_M = 0.3
#: Depth-map subsample stride: mesh every ``MESH_STRIDE``-th pixel in u and v, so
#: a (H,W) depth map yields an ``(H/stride) x (W/stride)`` vertex grid. This caps
#: the triangle budget per keyframe (a full-res grid would be enormous) while the
#: room's surfaces stay smooth. LOWER (toward 1) for a finer surface at a higher
#: triangle cost; RAISE for a coarser, lighter mesh. 3 is a good balance.
MESH_STRIDE = 3
#: Edge-reject threshold (m): a quad (2x2 vertex cell) is meshed only if the
#: spread (max - min) of its four corner depths is <= this. This DROPS the
#: "curtain" triangles stretched across a foreground/background depth
#: discontinuity (e.g. a near object's edge against a far wall) -- essential or
#: the room fills with spurious sheets. RAISE to tolerate more slope (fewer holes,
#: more curtains); LOWER for a cleaner surface (more holes at real edges).
EDGE_MAX_M = 0.1
#: Valid-depth band (m) for meshing a keyframe depth pixel: outside this range
#: stereo depth is too noisy/unreliable, so the corner is invalid and any quad
#: touching it is skipped (same band the dense geometry helper + the landmark map
#: use).
MIN_DEPTH_M = 0.3
MAX_DEPTH_M = 6.0
#: Hard cap on the merged mesh's triangle count. A continuous-surface mesh can
#: explode (many keyframes x a fine grid), so the source coarsens MESH_STRIDE /
#: KF_SPACING_M and logs it before exceeding this. ~1.5 M triangles renders
#: smoothly in pyqtgraph while keeping the room recognisable.
MAX_TRIANGLES = 1_500_000


# --------------------------------------------------------------------------- #
def select_spread_keyframes(positions: np.ndarray, *,
                            spacing: float = KF_SPACING_M) -> np.ndarray:
    """Greedy spatial thinning of keyframes by camera POSITION.

    ``positions`` is an ``(N,3)`` array of keyframe camera positions (in the
    camera-optical world frame). Walking them in the given order, KEEP a keyframe
    only if its position is > ``spacing`` from EVERY already-kept keyframe; this
    yields a spatially-spread subset that covers the room from a few viewpoints
    while bounding overlap/redundancy (and so the merged triangle count).

    Returns the ``(K,)`` int64 indices (into ``positions``) of the kept
    keyframes, in input order. ``spacing <= 0`` keeps every keyframe. Empty in ->
    empty out.

    Pure numpy: each candidate is tested against the kept set with one vectorised
    distance compare (no KD-tree); K is small (a handful of viewpoints), so the
    O(N*K) walk is cheap.
    """
    pos = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    n = pos.shape[0]
    if n == 0:
        return np.zeros((0,), np.int64)
    if spacing <= 0.0:
        return np.arange(n, dtype=np.int64)

    sp2 = float(spacing) * float(spacing)            # compare squared distances
    kept_idx: list[int] = []
    kept_pos: list[np.ndarray] = []
    for i in range(n):
        p = pos[i]
        if kept_pos:
            # Squared distance to every already-kept position in one shot; keep
            # only if the NEAREST kept keyframe is farther than ``spacing``.
            d2 = np.sum((np.asarray(kept_pos) - p) ** 2, axis=1)
            if float(d2.min()) <= sp2:
                continue
        kept_idx.append(i)
        kept_pos.append(p)
    return np.asarray(kept_idx, dtype=np.int64)


def _height_colors(verts: np.ndarray, *,
                   gray: np.ndarray | None = None,
                   y_lo: float | None = None,
                   y_hi: float | None = None) -> np.ndarray:
    """Per-vertex RGB ``(V,3)`` float32 keyed by height along world-DOWN.

    Colours each vertex by its optical-world ``+y`` (DOWN) coordinate so the
    floor, walls and ceiling read as distinct bands. The value is normalised over
    the [``y_lo``, ``y_hi``] range (the WHOLE room's height span, passed in so
    every keyframe's mesh shares one consistent gradient) and mapped through a
    blue(high/ceiling)->green->red(low/floor) ramp: because ``+y`` is DOWN, a
    larger ``y`` is LOWER in the room, so the FLOOR comes out warm/red and the
    CEILING cool/blue -- a stable, recognisable height cue.

    When ``gray`` (per-vertex intensity in [0,1]) is given the height colour is
    modulated by it (a darker pixel dims the band) so surface texture reads on top
    of the height gradient. A degenerate (flat) height span falls back to a
    uniform mid colour.
    """
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    if v.shape[0] == 0:
        return np.zeros((0, 3), np.float32)
    y = v[:, 1]                                       # optical-world DOWN
    lo = float(y.min()) if y_lo is None else float(y_lo)
    hi = float(y.max()) if y_hi is None else float(y_hi)
    span = hi - lo
    if span < 1e-6:                                   # flat slab -> uniform colour
        rgb = np.full((v.shape[0], 3), 0.6, np.float64)
    else:
        t = np.clip((y - lo) / span, 0.0, 1.0)        # 0 (ceiling) .. 1 (floor)
        # Piecewise blue->green->red ramp (a cheap, monotonic "jet"-like map).
        r = np.clip(1.5 * t - 0.25, 0.0, 1.0)
        g = np.clip(1.0 - np.abs(2.0 * t - 1.0), 0.15, 1.0)
        b = np.clip(1.25 - 1.5 * t, 0.0, 1.0)
        rgb = np.stack([r, g, b], axis=1)
    if gray is not None:
        # Modulate the height band by surface intensity (keep a floor so a dark
        # pixel still shows its band rather than going black).
        g_mod = np.clip(np.asarray(gray, np.float64).reshape(-1), 0.0, 1.0)
        rgb = rgb * (0.4 + 0.6 * g_mod)[:, None]
    return rgb.astype(np.float32)


def depth_surface_mesh(depth: np.ndarray, R: np.ndarray, t: np.ndarray,
                       K: np.ndarray, *,
                       stride: int = MESH_STRIDE,
                       edge_max: float = EDGE_MAX_M,
                       min_depth: float = MIN_DEPTH_M,
                       max_depth: float = MAX_DEPTH_M,
                       gray: np.ndarray | None = None,
                       y_lo: float | None = None,
                       y_hi: float | None = None):
    """Mesh ONE keyframe's depth map into a connected world-frame surface.

    Builds a triangle mesh from the keyframe's (subsampled) depth grid:

    1. Subsample the depth map by ``stride`` to an ``(R0,C0)`` vertex grid; mark a
       grid vertex VALID when its depth is finite and within
       ``[min_depth, max_depth]``.
    2. Back-project every grid vertex with the pinhole
       (``X=(u-cx)/fx*z``, ``Y=(v-cy)/fy*z``, ``Z=z``) and transform to the world
       by the keyframe's own pose ``Xw = R Xc + t`` (one pose source per keyframe
       -> seam-free odom-frame surfaces, the same consistent-odom approach the
       landmark map uses). Invalid vertices are still emitted (so the flat
       row-major index ``row*C0 + col`` stays valid) but no triangle references
       them.
    3. Triangulate adjacent grid cells: each 2x2 cell of corners
       ``(r,c),(r,c+1),(r+1,c),(r+1,c+1)`` -> two triangles
       (top-left/​bottom-right split). KEEP a cell's triangles only when all four
       corners are VALID **and** the corner-depth spread (max-min) <= ``edge_max``
       -- this drops the "curtain" triangles stretched across a depth
       discontinuity (without it the room fills with sheets).
    4. Colour each vertex by HEIGHT along world-down (see :func:`_height_colors`),
       optionally modulated by ``gray`` intensity. ``y_lo`` / ``y_hi`` carry the
       WHOLE room's height span so every keyframe shares one gradient.

    ``depth`` is ``(H,W)`` metric; ``R`` ``(3,3)`` / ``t`` ``(3,)`` the keyframe's
    VIO pose; ``K`` ``(3,3)`` the rectified-left intrinsic. ``gray`` (optional) is
    the ``(H,W)`` keyframe image in [0,255]. Returns
    ``(verts (V,3) float32, faces (F,3) int64, vertex_colors (V,3) float32)``;
    no valid cell -> all-empty arrays.

    Fully vectorised: the four corner validity/spread tests and the two-triangle
    emission run over the whole grid at once (no per-cell Python loop).
    """
    d = np.asarray(depth, dtype=np.float32)
    if d.ndim != 2:
        return (np.zeros((0, 3), np.float32),
                np.zeros((0, 3), np.int64),
                np.zeros((0, 3), np.float32))
    h, w = d.shape
    s = max(1, int(stride))

    # ---- (1) Subsample to the vertex grid + per-vertex validity ----
    vs = np.arange(0, h, s, dtype=np.int64)          # sampled rows
    us = np.arange(0, w, s, dtype=np.int64)          # sampled cols
    r0, c0 = vs.shape[0], us.shape[0]
    if r0 < 2 or c0 < 2:                             # need a 2x2 cell to mesh
        return (np.zeros((0, 3), np.float32),
                np.zeros((0, 3), np.int64),
                np.zeros((0, 3), np.float32))
    z = d[np.ix_(vs, us)]                             # (r0, c0) sampled depth
    valid = np.isfinite(z) & (z >= float(min_depth)) & (z <= float(max_depth))

    # ---- (2) Back-project EVERY grid vertex to the world (row-major flatten) ----
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    uu, vv = np.meshgrid(us.astype(np.float64), vs.astype(np.float64))  # (r0,c0)
    zf = z.astype(np.float64)
    # Invalid depths are clamped to a finite placeholder so the back-projection
    # never produces NaN/inf vertices (no triangle references them anyway).
    zsafe = np.where(valid, zf, 0.0)
    cam = np.stack([(uu - cx) * zsafe / fx,
                    (vv - cy) * zsafe / fy,
                    zsafe], axis=2).reshape(-1, 3)    # (r0*c0, 3) camera frame
    R = np.asarray(R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(t, dtype=np.float64).reshape(3)
    verts = (cam @ R.T + t)                           # (r0*c0, 3) world frame

    # ---- (3) Triangulate the 2x2 cells, keep valid + non-discontinuous ones ----
    # Corner index grids (row-major: idx = row*c0 + col). Each (r0-1, c0-1) cell
    # has corners TL/TR/BL/BR; two triangles split along the TL->BR diagonal.
    rows = np.arange(r0 - 1, dtype=np.int64)[:, None]
    cols = np.arange(c0 - 1, dtype=np.int64)[None, :]
    tl = (rows * c0 + cols)                            # (r0-1, c0-1)
    tr = tl + 1
    bl = tl + c0
    br = bl + 1

    # A cell meshes only when ALL four corners are valid AND their depth spread is
    # within edge_max (drop the fg/bg "curtain" triangles).
    cell_valid = (valid[:-1, :-1] & valid[:-1, 1:]
                  & valid[1:, :-1] & valid[1:, 1:])
    if float(edge_max) > 0.0:
        zc = np.stack([z[:-1, :-1], z[:-1, 1:], z[1:, :-1], z[1:, 1:]], axis=0)
        spread = zc.max(axis=0) - zc.min(axis=0)      # (r0-1, c0-1)
        cell_valid &= (spread <= float(edge_max))
    if not np.any(cell_valid):
        return (np.zeros((0, 3), np.float32),
                np.zeros((0, 3), np.int64),
                np.zeros((0, 3), np.float32))

    cv = cell_valid.reshape(-1)
    tl, tr, bl, br = tl.reshape(-1), tr.reshape(-1), bl.reshape(-1), br.reshape(-1)
    tl, tr, bl, br = tl[cv], tr[cv], bl[cv], br[cv]
    # Two triangles per kept cell, wound CCW (consistent so 'shaded' lights them):
    #   (TL, BL, BR) and (TL, BR, TR).
    tri1 = np.stack([tl, bl, br], axis=1)
    tri2 = np.stack([tl, br, tr], axis=1)
    faces = np.concatenate([tri1, tri2], axis=0).astype(np.int64)

    # ---- (4) Per-vertex height colour (shared room gradient via y_lo/y_hi) ----
    gray_v = None
    if gray is not None:
        g = np.asarray(gray, dtype=np.float32)
        if g.shape == (h, w):
            gray_v = (g[np.ix_(vs, us)].reshape(-1) / 255.0)
    colors = _height_colors(verts, gray=gray_v, y_lo=y_lo, y_hi=y_hi)
    return verts.astype(np.float32), faces, colors
