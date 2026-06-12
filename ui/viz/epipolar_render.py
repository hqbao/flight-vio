"""Shared renderer for the stereo "Epipolar / Rectification" view (DRY core).

ONE renderer, two callers:

* the OFFLINE learning tool :mod:`depth.tools.epipolar_explorer` (a recorded gold
  session -> a before/after PNG), and
* the LIVE in-app window :mod:`ui.qt.epipolar_window` (a live raw left+right pair
  off ``imucam.sample`` + the retained ``calib.stereo`` -> the SAME figure, every
  frame).

The canonical "is my rectification correct?" view: the same evenly-spaced
horizontal scanlines drawn across a left|right pair, BEFORE rectification (top row
-- corresponding corners drift off the lines, non-zero vertical row-mismatch) vs
AFTER rectification (bottom row -- they snap onto the same scanline, mismatch
-> ~0). A handful of strong Shi-Tomasi corners on the left are located in the
right by a same-row-band block search; their vertical disparity (row mismatch) is
annotated and the median reported per row, so the before -> after collapse is
quantitative, not just visual.

WHY a shared module
-------------------
Both callers need the IDENTICAL compute (block-match the same way) + the IDENTICAL
figure (scanlines, corner markers, captions, median readout) -- only the DATA
source differs (a recorded right vs a live rectified right). Keeping the compute +
draw here means the offline tool and the live window can never drift in how they
measure or draw "row mismatch".

Output convention
-----------------
:func:`render_epipolar` returns an **RGB** ``uint8`` ndarray (the convention every
other ``ui.viz`` renderer follows, blitted by the Qt windows as
``QImage.Format_RGB888``). The offline tool, which writes a PNG via ``cv2.imwrite``
(BGR), converts with a single ``[..., ::-1]`` channel swap.

cv2 is only a drawing backend here (text + lines + circles); importing this module
is what pulls it, never the base UI -- mirrors :mod:`ui.viz.ba_render` /
:mod:`ui.viz.loop_render`. ``sky`` is imported only for the corner detector.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from sky.front.corners import good_features_to_track


# --------------------------------------------------------------------------- #
# Compute core -- block-match a left corner into the right image (both ways).
# --------------------------------------------------------------------------- #
@dataclass
class CornerMatch:
    """One left corner located in the right image (before OR after rectification).

    ``row_mismatch`` = ``yr - yl`` (signed vertical disparity in pixels): the whole
    point of rectification is to drive |row_mismatch| -> 0. ``score`` is the
    normalized block-match cost at the chosen right pixel (lower = better) and
    ``valid`` says whether the match cleared the acceptance gates.
    """

    xl: float            # left corner x (column)
    yl: float            # left corner y (row)
    xr: float            # matched right x (column)
    yr: float            # matched right y (row)
    row_mismatch: float  # yr - yl (signed, pixels)
    disparity: float     # xl - xr (horizontal disparity, for sanity only)
    score: float         # block-match cost (lower better)
    valid: bool          # cleared the score / disparity gates


def block_match_same_band(left: np.ndarray, right: np.ndarray,
                          xl: int, yl: int, *, half: int = 5,
                          row_band: int = 12, max_disp: int = 96,
                          min_disp: int = 0) -> CornerMatch:
    """Locate the left patch at ``(xl, yl)`` in ``right`` over a small (row, disp) box.

    A brute-force normalized-SAD search: for every candidate right row in
    ``yl +/- row_band`` and every horizontal disparity ``d`` in
    ``[min_disp, max_disp]`` (the right match lies at ``xl - d``, left-ward, since
    the right camera sees the scene shifted left), score the
    ``(2*half+1)x(2*half+1)`` patch and keep the cheapest. The row search band is
    what lets us *see* a vertical mis-rectification: on a raw right image the best
    match is found several rows off; on the rectified right it collapses onto the
    same row. Returns a :class:`CornerMatch` with the signed row mismatch.
    """
    H, W = right.shape
    # Patch must be fully inside the left image, else the corner is unusable.
    if not (half <= xl < W - half and half <= yl < H - half):
        return CornerMatch(xl, yl, xl, yl, 0.0, 0.0, np.inf, False)

    lpatch = left[yl - half:yl + half + 1, xl - half:xl + half + 1].astype(np.float32)
    # Zero-mean the patch so a global brightness offset between the two cameras
    # does not bias the SAD (a poor man's normalization -- enough for a viz).
    lpatch = lpatch - lpatch.mean()

    best_cost = np.inf
    best_xr = xl
    best_yr = yl
    y_lo = max(half, yl - row_band)
    y_hi = min(H - half - 1, yl + row_band)
    for yr in range(y_lo, y_hi + 1):
        for d in range(min_disp, max_disp + 1):
            xr = xl - d
            if xr < half or xr >= W - half:
                continue
            rpatch = right[yr - half:yr + half + 1,
                           xr - half:xr + half + 1].astype(np.float32)
            rpatch = rpatch - rpatch.mean()
            cost = float(np.abs(lpatch - rpatch).mean())
            if cost < best_cost:
                best_cost = cost
                best_xr = xr
                best_yr = yr

    npix = lpatch.size
    valid = np.isfinite(best_cost) and (best_cost / 255.0) < 0.18 and npix > 0
    return CornerMatch(
        xl=float(xl), yl=float(yl), xr=float(best_xr), yr=float(best_yr),
        row_mismatch=float(best_yr - yl), disparity=float(xl - best_xr),
        score=float(best_cost), valid=bool(valid))


def detect_left_corners(left: np.ndarray, *, max_corners: int = 14) -> np.ndarray:
    """Strong Shi-Tomasi corners on the left image (the rows we track across)."""
    return good_features_to_track(
        np.clip(left, 0, 255).astype(np.uint8),
        max_corners=int(max_corners), quality_level=0.02, min_distance=24.0)


def match_corners(left: np.ndarray, right: np.ndarray,
                  corners: np.ndarray) -> list[CornerMatch]:
    """Locate each left corner in ``right`` (one :class:`CornerMatch` per corner)."""
    out: list[CornerMatch] = []
    H, W = left.shape
    for (cx, cy) in np.asarray(corners, dtype=np.float64).reshape(-1, 2):
        xl, yl = int(round(cx)), int(round(cy))
        out.append(block_match_same_band(left, right, xl, yl))
    return out


def median_abs_mismatch(matches: list[CornerMatch]) -> tuple[float, int]:
    """Median |row mismatch| over the valid matches, and how many were valid."""
    vals = [abs(m.row_mismatch) for m in matches if m.valid]
    if not vals:
        return float("nan"), 0
    return float(np.median(vals)), len(vals)


@dataclass
class EpipolarRender:
    """One frame's before/after panels + corner matches, ready to render.

    The LIVE record :class:`ui.modules.ipc_sources.IpcEpipolarSource` produces and
    the window blits via :func:`render_epipolar_record`. ``*_before`` are the RAW
    left+right pair; ``*_after`` are the SAME pair after the Left/Right rectifier
    warps. ``matches_before`` locate the left corners in the RAW right (rows drift),
    ``matches_after`` in the RECTIFIED right (rows align). All panels are ``(H, W)``
    uint8 grayscale on the same grid.
    """

    seq: int
    ts_ns: int
    left_before: np.ndarray
    right_before: np.ndarray
    left_after: np.ndarray
    right_after: np.ndarray
    matches_before: list[CornerMatch]
    matches_after: list[CornerMatch]


# --------------------------------------------------------------------------- #
# Renderer -- numpy -> RGB image of the before/after scanline pair.
# Theme mirrors ui/qt/theme.py (RGB; the Qt window blits Format_RGB888). The
# offline PNG tool swaps to BGR with a single [..., ::-1].
# --------------------------------------------------------------------------- #
_BG = (13, 27, 23)         # #0d1117 dark page (RGB)
_TEXT = (230, 237, 243)    # #e6edf3 light text
_SCAN = (90, 150, 180)     # muted steel-blue scanline (subtle)
_GOOD = (124, 255, 92)     # NVG green: a corner on/near its scanline (good)
_BAD = (255, 59, 48)       # warning red: a corner off its scanline (bad)

_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _to_rgb(gray: np.ndarray) -> np.ndarray:
    """Grayscale uint8 -> 3-channel RGB canvas we can draw coloured overlays on."""
    g = np.clip(gray, 0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2RGB)


def _draw_scanlines(img: np.ndarray, n: int = 13) -> None:
    """Draw ``n`` evenly-spaced horizontal scanlines across ``img`` (in place).

    These are the SAME rows on the left and right panels of a stereo row: after a
    correct rectification a left feature and its right match straddle the same one.
    """
    h = img.shape[0]
    for y in np.linspace(0.08 * h, 0.92 * h, n):
        cv2.line(img, (0, int(y)), (img.shape[1] - 1, int(y)), _SCAN, 1, cv2.LINE_AA)


def _annotate_matches(left_rgb: np.ndarray, right_rgb: np.ndarray,
                      matches: list[CornerMatch], tol_rows: float = 1.5) -> None:
    """Mark each corner on the left, its match on the right, and the row mismatch.

    Green = the match sits within ``tol_rows`` of the left corner's row (correctly
    row-aligned); red = it is off by more (a vertical mis-rectification). A short
    horizontal tick on the right marks the LEFT corner's row, so the gap to the
    actual match (the dot) is the visible row mismatch.
    """
    for m in matches:
        if not m.valid:
            continue
        ok = abs(m.row_mismatch) <= tol_rows
        col = _GOOD if ok else _BAD
        xl, yl = int(round(m.xl)), int(round(m.yl))
        xr, yr = int(round(m.xr)), int(round(m.yr))
        cv2.circle(left_rgb, (xl, yl), 4, col, 1, cv2.LINE_AA)
        cv2.circle(right_rgb, (xr, yr), 4, col, -1, cv2.LINE_AA)
        # The left corner's row, drawn on the right as a reference tick: the
        # vertical gap from this tick to the filled dot IS the row mismatch.
        cv2.line(right_rgb, (xr - 9, yl), (xr + 9, yl), col, 1, cv2.LINE_AA)
        if not ok:
            cv2.putText(right_rgb, f"{m.row_mismatch:+.0f}px", (xr + 8, yr - 4),
                        _FONT, 0.4, col, 1, cv2.LINE_AA)


def _stereo_row(left_rgb: np.ndarray, right_rgb: np.ndarray,
                gap: int = 8) -> np.ndarray:
    """Stack a left|right pair side by side with a thin separator gutter."""
    h = left_rgb.shape[0]
    sep = np.full((h, gap, 3), _BG, dtype=np.uint8)
    return np.hstack([left_rgb, sep, right_rgb])


def render_epipolar(left_before: np.ndarray, right_before: np.ndarray,
                    left_after: np.ndarray, right_after: np.ndarray,
                    matches_before: list[CornerMatch],
                    matches_after: list[CornerMatch], *,
                    before_label: str, after_label: str,
                    n_scanlines: int = 13) -> np.ndarray:
    """Render the 2-row before/after epipolar figure as an RGB ``uint8`` ndarray.

    Top row  = ``left_before`` | ``right_before`` (rows do NOT align before rect).
    Bottom row = ``left_after`` | ``right_after`` (rows snap onto the same
    scanline). Each row carries the shared scanlines, the corner markers, and a
    per-row median |row mismatch| readout so the before -> after collapse is
    quantitative. ``before_label`` / ``after_label`` are the caption titles each
    caller supplies (the offline tool says "chip-rectified LEFT | RAW right"; the
    live window says "RAW left | RAW right" / "rectified left | rectified right").
    """
    # --- Build the four panels (RGB) with scanlines + corner annotations. ---- #
    lb, rb = _to_rgb(left_before), _to_rgb(right_before)
    la, ra = _to_rgb(left_after), _to_rgb(right_after)
    for panel in (lb, rb, la, ra):
        _draw_scanlines(panel, n_scanlines)
    _annotate_matches(lb, rb, matches_before)
    _annotate_matches(la, ra, matches_after)

    med_before, n_before = median_abs_mismatch(matches_before)
    med_after, n_after = median_abs_mismatch(matches_after)

    top = _stereo_row(lb, rb)
    bot = _stereo_row(la, ra)
    width = max(top.shape[1], bot.shape[1])

    # --- Per-row caption strips (panel labels + median row mismatch). -------- #
    def _caption(title: str, sub: str) -> np.ndarray:
        strip = np.full((42, width, 3), _BG, dtype=np.uint8)
        cv2.putText(strip, title, (8, 18), _FONT, 0.5, _TEXT, 1, cv2.LINE_AA)
        cv2.putText(strip, sub, (8, 36), _FONT, 0.42, _TEXT, 1, cv2.LINE_AA)
        return strip

    before_med = ("median |row mismatch| = n/a (no valid corner matches)"
                  if n_before == 0
                  else f"median |row mismatch| = {med_before:.1f}px "
                       f"over {n_before} corners  (off-row = RED)")
    after_med = ("median |row mismatch| = n/a (no valid corner matches)"
                 if n_after == 0
                 else f"median |row mismatch| = {med_after:.1f}px "
                      f"over {n_after} corners  (on-row = GREEN)")
    cap_top = _caption(before_label, before_med)
    cap_bot = _caption(after_label, after_med)

    # --- Header banner explaining the read. ---------------------------------- #
    banner = np.full((56, width, 3), _BG, dtype=np.uint8)
    cv2.putText(banner, "Stereo rectification epipolar view: do matches land on the "
                        "SAME scanline after rectify?",
                (10, 22), _FONT, 0.52, _TEXT, 1, cv2.LINE_AA)
    cv2.putText(banner, "scanlines drawn across BOTH images; a corner + its right "
                        "match should straddle one line (GREEN) once rectified",
                (10, 44), _FONT, 0.42, _TEXT, 1, cv2.LINE_AA)

    return np.ascontiguousarray(np.vstack([banner, cap_top, top, cap_bot, bot]))


def render_epipolar_record(rec: "EpipolarRender | None",
                           n_scanlines: int = 13) -> np.ndarray:
    """Render a LIVE :class:`EpipolarRender` (or a placeholder when ``None``).

    The live caller's convenience wrapper around :func:`render_epipolar`: it uses
    the LIVE captions (the live path delivers a genuinely RAW left+right pair, so
    the BEFORE row is "RAW left | RAW right" and the AFTER row "rectified left |
    rectified right" -- both columns are warped, unlike the offline tool whose left
    is already chip-rectified). ``None`` -> a small "waiting for calib / stereo…"
    placeholder so the window has something to show before data arrives.
    """
    if rec is None:
        ph = np.full((220, 560, 3), _BG, dtype=np.uint8)
        cv2.putText(ph, "waiting for calib / stereo...", (20, 110),
                    _FONT, 0.7, _TEXT, 1, cv2.LINE_AA)
        cv2.putText(ph, "(needs capture's raw left+right + calib.stereo)",
                    (20, 145), _FONT, 0.45, _TEXT, 1, cv2.LINE_AA)
        return ph
    return render_epipolar(
        rec.left_before, rec.right_before, rec.left_after, rec.right_after,
        rec.matches_before, rec.matches_after,
        before_label="BEFORE  -  RAW left  |  RAW right (unrectified)",
        after_label="AFTER   -  rectified left  |  rectified right",
        n_scanlines=n_scanlines)


def status_line(matches_before: list[CornerMatch],
                matches_after: list[CornerMatch]) -> str:
    """One-line median row-mismatch before/after summary (for the window status)."""
    mb, nb = median_abs_mismatch(matches_before)
    ma, na = median_abs_mismatch(matches_after)
    b = "n/a" if nb == 0 else f"{mb:.1f}px ({nb})"
    a = "n/a" if na == 0 else f"{ma:.1f}px ({na})"
    return f"median |row mismatch|  before {b}  ->  after {a}"
