#!/usr/bin/env python3
"""Selftest for the stereo rectification epipolar explorer (offline learning tool).

Mirrors ``depth/tests/stereo_sgm_selftest.py``: runs the headless render on a gold
frame and asserts the artefact is real and the teaching claim holds. Specifically:

  1. ``compute_epipolar_frame`` loads a gold frame, rectifies the right, and matches
     a handful of left corners into BOTH the raw and the rectified right;
  2. ``render_epipolar_png`` writes a non-blank, correctly-shaped 2-row figure;
  3. rectification REDUCES (never worsens) the median |row mismatch| of the matched
     corners -- the whole point of the view (raw rows drift, rectified rows align).

This is an OFFLINE tool: it imports only ``sky.depth.stereo`` rectifiers + the
session reader, never the live path / comms / oracle, so it cannot affect gap=0.

Usage::

    python -m depth.tests.epipolar_explorer_selftest
    python -m depth.tests.epipolar_explorer_selftest --session sessions/gold/lab_loop_30s
"""
from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from depth.tools.epipolar_explorer import (                          # noqa: E402
    compute_epipolar_frame, render_epipolar_png,
)
# The block-match / median helpers now live in the SHARED renderer that the live
# window and this offline tool both call (DRY); import the median from there.
from ui.viz.epipolar_render import median_abs_mismatch as _median_abs_mismatch  # noqa: E402


def _check(session: str, frame: int) -> bool:
    """Run the tool headless on one gold frame and assert it is non-trivial.

    Returns True on PASS. Prints a one-line PASS/FAIL with the measured numbers so
    the CI log alone tells you whether rectification is doing its job.
    """
    import cv2  # noqa: PLC0415  (lazy, offline-tool convention -- flight stays cv2-free)

    ef = compute_epipolar_frame(session, frame, max_corners=14)

    # --- The compute core actually found corners and matched them both ways. -- #
    n_corners = len(ef.corners)
    assert n_corners > 0, f"{session}#{frame}: no corners detected"
    assert len(ef.matches_before) == n_corners
    assert len(ef.matches_after) == n_corners

    med_before, n_before = _median_abs_mismatch(ef.matches_before)
    med_after, n_after = _median_abs_mismatch(ef.matches_after)
    assert n_before > 0 and n_after > 0, (
        f"{session}#{frame}: no valid corner matches "
        f"(before={n_before}, after={n_after})")

    # --- The teaching claim: rectification must not WORSEN the row alignment. - #
    # (We assert <= rather than strict < so a frame that is already near-perfect
    # raw -- a few exist -- does not flap the test; the point is rectify never hurts
    # and across sessions it strictly helps, which the printed numbers show.)
    assert med_after <= med_before + 1e-6, (
        f"{session}#{frame}: rectification WORSENED row mismatch "
        f"({med_before:.2f}px -> {med_after:.2f}px)")

    # --- The render writes a real, non-blank, correctly-stacked figure. ------- #
    with tempfile.TemporaryDirectory() as td:
        out = render_epipolar_png(ef, Path(td) / "epipolar.png")
        img = cv2.imread(out, cv2.IMREAD_COLOR)
        assert img is not None, f"{session}#{frame}: render produced no readable PNG"
        H, W = ef.left_before.shape
        # Two stereo rows (each = left|right + gutter) + banner + 2 captions, so the
        # figure is taller than two image heights and wider than two image widths.
        assert img.shape[0] > 2 * H, f"figure too short: {img.shape}"
        assert img.shape[1] >= 2 * W, f"figure too narrow: {img.shape}"
        # Non-blank: it must contain real image content (variance) AND the coloured
        # overlays (scanlines / markers are not pure gray, so B != G somewhere).
        assert float(img.std()) > 5.0, f"figure looks blank (std={img.std():.2f})"
        b, g = img[..., 0].astype(np.int16), img[..., 1].astype(np.int16)
        assert int(np.abs(b - g).max()) > 0, "figure has no coloured overlay drawn"

    print(f"PASS {session}#{frame}: {n_corners} corners, "
          f"median |row mismatch| raw={med_before:.2f}px -> "
          f"rect={med_after:.2f}px, figure {img.shape[1]}x{img.shape[0]}")
    return True


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default=None,
                    help="single gold session to test (default: a representative set)")
    ap.add_argument("--frame", type=int, default=40)
    args = ap.parse_args(argv)

    if args.session:
        cases = [(args.session, args.frame)]
    else:
        # A representative spread; each must individually pass.
        cases = [
            ("sessions/gold/lab_loop_30s", 40),
            ("sessions/gold/lab_straight_20s", 60),
            ("sessions/gold/corridor_60s", 80),
        ]

    ok = True
    for session, frame in cases:
        if not Path(session).exists():
            print(f"SKIP {session}#{frame}: session not present")
            continue
        ok = _check(session, frame) and ok

    print("ALL PASS" if ok else "FAILURES")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
