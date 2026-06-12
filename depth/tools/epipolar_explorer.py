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
This tool reads a recorded session; the LIVE equivalent is the in-app
``ui.qt.epipolar_window`` (raw left+right off ``imucam.sample`` + the retained
``calib.stereo``). Both share ONE renderer + compute core
(:mod:`ui.viz.epipolar_render`) so they can never drift -- this tool is the
session caller, the window the live caller. This tool touches NOTHING in the
live path / comms / oracle: it only reads a gold session and rectifies the right.
gap=0 is trivially unaffected.

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

from depth.io.reader import SessionReader                            # noqa: E402
from sky.depth.stereo import RightRectifier                         # noqa: E402
from ui.viz.epipolar_render import (                                # noqa: E402
    CornerMatch, detect_left_corners, match_corners, median_abs_mismatch,
)


# --------------------------------------------------------------------------- #
# Session compute -- load one gold frame, rectify the right, match corners both
# ways. The block-match + corner detection live in ``ui.viz.epipolar_render`` so
# this tool and the live window measure "row mismatch" IDENTICALLY (DRY).
# --------------------------------------------------------------------------- #
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

    # Strong corners on the (rectified) left -- the features whose rows we track;
    # then locate each in BOTH right images (shared compute, same as the window).
    corners = detect_left_corners(left, max_corners=max_corners)
    matches_before = match_corners(left, right_raw, corners)
    matches_after = match_corners(left, right_rect_u8, corners)

    return EpipolarFrame(
        left_before=left, right_before=right_raw,
        left_after=left, right_after=right_rect_u8,
        corners=corners,
        matches_before=matches_before, matches_after=matches_after)


def render_epipolar_png(ef: EpipolarFrame, out_path: str | Path,
                        n_scanlines: int = 13) -> str:
    """Write the 2-row before/after epipolar figure to ``out_path`` (abs path back).

    Delegates the figure to the SHARED :func:`ui.viz.epipolar_render.render_epipolar`
    (one renderer for this tool + the live window), then converts its RGB canvas to
    BGR for ``cv2.imwrite``.
    """
    import cv2  # noqa: PLC0415 (approved dep; array/PNG backend, lazy like sgm_cost)
    from ui.viz.epipolar_render import render_epipolar  # noqa: PLC0415

    rgb = render_epipolar(
        ef.left_before, ef.right_before, ef.left_after, ef.right_after,
        ef.matches_before, ef.matches_after,
        before_label="BEFORE  -  chip-rectified LEFT  |  RAW right (unrectified)",
        after_label="AFTER   -  chip-rectified LEFT  |  RightRectifier(raw right)",
        n_scanlines=n_scanlines)
    out_path = str(Path(out_path).resolve())
    cv2.imwrite(out_path, rgb[..., ::-1])        # RGB -> BGR for cv2
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
    med_before, n_before = median_abs_mismatch(ef.matches_before)
    med_after, n_after = median_abs_mismatch(ef.matches_after)
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
