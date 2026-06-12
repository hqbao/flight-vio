#!/usr/bin/env python3
"""Stereo rectification epipolar explorer -- "is my rectification correct?".

WHAT THIS TEACHES
-----------------
SGM block matching only works if a 3D point lands on the **same row** in the left
and right image: then the disparity search collapses to a 1-D scan along the row
(``sky.depth.stereo``). Raw cameras do NOT satisfy this -- lens distortion plus
the inter-camera rotation push corresponding points onto *different* rows
(``docs/ALGORITHMS.md`` §1.5 measures ~48% of corners off by >=2 rows). The
``Left/RightRectifier`` warps re-project both views onto a common, distortion-free,
row-aligned virtual stereo pair so that, after rectification, a feature in the left
image sits on the **same scanline** as its match in the right.

This tool draws that, the canonical "is my rectification correct" view: the same
~13 horizontal scanlines across a left|right pair, BEFORE rectification (top row --
matches drift off the lines, with non-zero vertical row-mismatch) vs AFTER
rectification (bottom row -- matches snap onto the same scanline, vertical mismatch
-> ~0). A handful of strong Shi-Tomasi corners are detected in the left image and
located in the right by a same-row-band block search; their vertical disparity
(row mismatch) is annotated and the median is reported, shrinking from raw -> rect.

WHAT THE SESSION ACTUALLY PROVIDES (be honest about "before")
-------------------------------------------------------------
A recorded gold session stores, per frame, the chip's **already-rectified LEFT**
(``rectifiedLeft``) plus a **raw, unrectified RIGHT** (``syncedRight``) -- see
``depth/io/reader.py`` and ``ALGORITHMS.md`` §1.5. So the only genuinely *raw*
image available is the right one. This tool is honest about that:

* TOP ("before") row  = chip-rectified LEFT  |  RAW right (the right is truly
  unrectified -- this is where the row-mismatch is visible). The left panel is
  labelled "chip-rectified" so no one reads it as raw.
* BOTTOM ("after") row = the SAME chip-rectified LEFT (already on the rectified
  grid the matcher uses) | ``RightRectifier.rectify(raw_right)`` (our warp). The
  left is NOT re-warped through ``LeftRectifier`` -- that expects a *raw* left,
  and feeding it the chip-rectified left would double-rectify. We keep the chip
  left as the common rectified grid (exactly what the replay depth path does:
  ``from_calib(..., rectify_left=False)``).

The teaching point lives entirely in the RIGHT column: raw-right rows do not line
up with the left; rectified-right rows do.

WHY OFFLINE / STANDALONE
------------------------
The live UI publishes only left+depth, never the right frame, so the before/after
right pair only exists from a recorded session. This tool touches NOTHING in the
live path / comms / oracle: it only IMPORTS ``sky.depth.stereo`` rectifiers and
reads a gold session. gap=0 is trivially unaffected.

USAGE
-----
Headless render (no display needed -- the verifiable evidence)::

    .venv/bin/python -m depth.tools.epipolar_explorer \\
        --session sessions/gold/lab_loop_30s --frame 40 --render /tmp/epipolar.png

Writes a 2-row figure (top = before, bottom = after) with scanlines + corner
row-mismatch annotations. Dependency-free plot (numpy -> cv2 image), no matplotlib.

Dependencies: numpy + cv2 only (cv2 lazy-imported, the offline-tool convention).
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Run as a module (-m) or as a script: make the repo root importable either way.
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2  # noqa: E402  (approved dep; only used as an array/PNG backend here)

from depth.io.reader import SessionReader                            # noqa: E402
from sky.depth.stereo import RightRectifier                # noqa: E402
from sky.front.corners import good_features_to_track       # noqa: E402


# --------------------------------------------------------------------------- #
# Compute core -- shared by the headless render (and any future interactive UI).
# --------------------------------------------------------------------------- #
@dataclass
class CornerMatch:
    """One left corner located in the right image (before OR after rectification).

    ``row_mismatch`` = ``yr - yl`` (signed vertical disparity in pixels): the
    whole point of rectification is to drive |row_mismatch| -> 0. ``score`` is the
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


@dataclass
class EpipolarFrame:
    """Everything the figure needs for one gold frame, before and after rectify.

    All panels are uint8 grayscale on the left intrinsic grid. ``left_before`` and
    ``left_after`` are the SAME chip-rectified left (see module docstring) -- both
    are kept so the figure layout is symmetric and the labels can be explicit.
    """

    left_before: np.ndarray   # (H, W) uint8 -- chip-rectified left (session)
    right_before: np.ndarray  # (H, W) uint8 -- RAW right (session, unrectified)
    left_after: np.ndarray    # (H, W) uint8 -- chip-rectified left (rectified grid)
    right_after: np.ndarray   # (H, W) uint8 -- RightRectifier.rectify(raw right)
    corners: np.ndarray       # (N, 2) float32 left corner (x, y) used for matching
    matches_before: list[CornerMatch]  # corners located in the RAW right
    matches_after: list[CornerMatch]   # corners located in the RECTIFIED right


def _block_match_same_band(left: np.ndarray, right: np.ndarray,
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


def compute_epipolar_frame(session: str | Path, index: int,
                           max_corners: int = 14) -> EpipolarFrame:
    """Load one gold frame, rectify the right, and match a few corners both ways.

    Returns the four panels plus, for a handful of strong left corners, where each
    is found in the RAW vs the RECTIFIED right (so the figure can annotate the
    vertical row-mismatch collapsing from before -> after).
    """
    sr = SessionReader(session)
    frame = sr.load_frame(index, load_right=True)
    if frame.gray_right is None:
        raise RuntimeError(
            f"frame {index} of {session} has no recorded right image "
            "(the epipolar view needs left+right to show row alignment)")

    left = frame.gray_left                       # chip-rectified left (session)
    right_raw = frame.gray_right                 # RAW right (session, unrectified)
    right_rect = RightRectifier.from_calib(sr.calib).rectify(right_raw)
    right_rect_u8 = np.clip(right_rect, 0, 255).astype(np.uint8)

    # Strong corners on the (rectified) left -- the features whose rows we track.
    # Detected on the left we already have; matched into both right images below.
    corners = good_features_to_track(
        left, max_corners=max_corners, quality_level=0.02, min_distance=24.0)

    matches_before: list[CornerMatch] = []
    matches_after: list[CornerMatch] = []
    for (cx, cy) in corners:
        xl, yl = int(round(cx)), int(round(cy))
        matches_before.append(_block_match_same_band(left, right_raw, xl, yl))
        matches_after.append(_block_match_same_band(left, right_rect_u8, xl, yl))

    return EpipolarFrame(
        left_before=left, right_before=right_raw,
        left_after=left, right_after=right_rect_u8,
        corners=corners,
        matches_before=matches_before, matches_after=matches_after)


# --------------------------------------------------------------------------- #
# Headless render -- numpy -> cv2 PNG of the before/after scanline pair.
# Theme mirrors ui/qt/theme.py / sgm_cost_explorer so the tool looks part of the
# suite. Colours are BGR for cv2.
# --------------------------------------------------------------------------- #
_BG = (23, 27, 13)         # #0d1117 dark page
_TEXT = (243, 237, 230)    # #e6edf3 light text
_SCAN = (180, 150, 90)     # muted steel-blue scanline (subtle, BGR)
_GOOD = (92, 255, 124)     # NVG green: a corner on/near its scanline (good)
_BAD = (48, 59, 255)       # warning red: a corner off its scanline (bad)
_LINK = (0, 176, 255)      # amber: the left->right correspondence link


def _to_bgr(gray: np.ndarray) -> np.ndarray:
    """Grayscale uint8 -> 3-channel BGR canvas we can draw coloured overlays on."""
    g = np.clip(gray, 0, 255).astype(np.uint8)
    return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)


def _draw_scanlines(img: np.ndarray, n: int = 13) -> None:
    """Draw ``n`` evenly-spaced horizontal scanlines across ``img`` (in place).

    These are the SAME rows on the left and right panels of a stereo row: after a
    correct rectification a left feature and its right match straddle the same one.
    """
    h = img.shape[0]
    for y in np.linspace(0.08 * h, 0.92 * h, n):
        cv2.line(img, (0, int(y)), (img.shape[1] - 1, int(y)), _SCAN, 1, cv2.LINE_AA)


def _annotate_matches(left_bgr: np.ndarray, right_bgr: np.ndarray,
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
        cv2.circle(left_bgr, (xl, yl), 4, col, 1, cv2.LINE_AA)
        cv2.circle(right_bgr, (xr, yr), 4, col, -1, cv2.LINE_AA)
        # The left corner's row, drawn on the right as a reference tick: the
        # vertical gap from this tick to the filled dot IS the row mismatch.
        cv2.line(right_bgr, (xr - 9, yl), (xr + 9, yl), col, 1, cv2.LINE_AA)
        if not ok:
            cv2.putText(right_bgr, f"{m.row_mismatch:+.0f}px", (xr + 8, yr - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)


def _median_abs_mismatch(matches: list[CornerMatch]) -> tuple[float, int]:
    """Median |row mismatch| over the valid matches, and how many were valid."""
    vals = [abs(m.row_mismatch) for m in matches if m.valid]
    if not vals:
        return float("nan"), 0
    return float(np.median(vals)), len(vals)


def _stereo_row(left_bgr: np.ndarray, right_bgr: np.ndarray, gap: int = 8,
                ) -> np.ndarray:
    """Stack a left|right pair side by side with a thin separator gutter."""
    h = left_bgr.shape[0]
    sep = np.full((h, gap, 3), _BG, dtype=np.uint8)
    return np.hstack([left_bgr, sep, right_bgr])


def render_epipolar_png(ef: EpipolarFrame, out_path: str | Path,
                        n_scanlines: int = 13) -> str:
    """Write the 2-row before/after epipolar figure to ``out_path`` (abs path back).

    Top row  = chip-rectified LEFT | RAW right (raw rows do not align).
    Bottom row = chip-rectified LEFT | RECTIFIED right (rows snap onto the same
    scanline). Each row carries the shared scanlines, the corner markers, and a
    per-row median |row mismatch| readout so the before -> after collapse is
    quantitative, not just visual.
    """
    # --- Build the four panels (BGR) with scanlines + corner annotations. ---- #
    lb, rb = _to_bgr(ef.left_before), _to_bgr(ef.right_before)
    la, ra = _to_bgr(ef.left_after), _to_bgr(ef.right_after)
    for panel in (lb, rb, la, ra):
        _draw_scanlines(panel, n_scanlines)
    _annotate_matches(lb, rb, ef.matches_before)
    _annotate_matches(la, ra, ef.matches_after)

    med_before, n_before = _median_abs_mismatch(ef.matches_before)
    med_after, n_after = _median_abs_mismatch(ef.matches_after)

    top = _stereo_row(lb, rb)
    bot = _stereo_row(la, ra)
    width = max(top.shape[1], bot.shape[1])

    # --- Per-row caption strips (panel labels + median row mismatch). -------- #
    def _caption(text: str, sub: str) -> np.ndarray:
        strip = np.full((42, width, 3), _BG, dtype=np.uint8)
        cv2.putText(strip, text, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    _TEXT, 1, cv2.LINE_AA)
        cv2.putText(strip, sub, (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    _TEXT, 1, cv2.LINE_AA)
        return strip

    before_med = ("median |row mismatch| = n/a (no valid corner matches)"
                  if n_before == 0
                  else f"median |row mismatch| = {med_before:.1f}px "
                       f"over {n_before} corners  (off-row = RED)")
    after_med = ("median |row mismatch| = n/a (no valid corner matches)"
                 if n_after == 0
                 else f"median |row mismatch| = {med_after:.1f}px "
                      f"over {n_after} corners  (on-row = GREEN)")
    cap_top = _caption(
        "BEFORE  -  chip-rectified LEFT  |  RAW right (unrectified)", before_med)
    cap_bot = _caption(
        "AFTER   -  chip-rectified LEFT  |  RightRectifier(raw right)", after_med)

    # --- Header banner explaining the read. ---------------------------------- #
    banner = np.full((56, width, 3), _BG, dtype=np.uint8)
    cv2.putText(banner, "Stereo rectification epipolar view: do matches land on the "
                        "SAME scanline after rectify?",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, _TEXT, 1, cv2.LINE_AA)
    cv2.putText(banner, "scanlines drawn across BOTH images; a corner + its right "
                        "match should straddle one line (GREEN) once rectified",
                (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.42, _TEXT, 1, cv2.LINE_AA)

    out = np.vstack([banner, cap_top, top, cap_bot, bot])
    out_path = str(Path(out_path).resolve())
    cv2.imwrite(out_path, out)
    return out_path


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Stereo rectification epipolar explorer (learning tool): draw "
                    "scanlines over a left|right pair, before vs after rectify, to "
                    "show corresponding features landing on the same row.")
    ap.add_argument("--session", default="sessions/gold/lab_loop_30s",
                    help="recorded gold session directory")
    ap.add_argument("--frame", type=int, default=40,
                    help="frame index within the session")
    ap.add_argument("--max-corners", type=int, default=14,
                    help="how many strong left corners to track across the pair")
    ap.add_argument("--render", metavar="PNG", default=None,
                    help="headless: write the before/after epipolar PNG to this "
                         "path and exit (no display / Qt needed)")
    args = ap.parse_args(argv)

    ef = compute_epipolar_frame(args.session, args.frame,
                                max_corners=args.max_corners)
    med_before, n_before = _median_abs_mismatch(ef.matches_before)
    med_after, n_after = _median_abs_mismatch(ef.matches_after)
    H, W = ef.left_before.shape
    print(f"loaded {args.session} frame {args.frame}: {W}x{H}, "
          f"{len(ef.corners)} corners")
    print(f"raw  right: median |row mismatch| = {med_before:.2f}px "
          f"({n_before} valid)")
    print(f"rect right: median |row mismatch| = {med_after:.2f}px "
          f"({n_after} valid)")

    if args.render:
        out = render_epipolar_png(ef, args.render)
        print(f"wrote {out}")
        return 0

    # No interactive window for this tool: the PNG is the artefact. Without
    # --render, point the user at it (mirrors how the offline tools are run in CI).
    print("note: this tool is headless-only; pass --render <PNG> to write the figure")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
