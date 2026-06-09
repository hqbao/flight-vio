"""2D renderer for the "Loop Closure" window (ALGORITHMS.md viz #1).

Draws the loop-closure verification funnel as ONE flat RGB image (cv2 backend,
numpy out -- no OpenGL, so it is light + PNG-verifiable):

* two keyframe GRAY images side-by-side -- CURRENT (left) | MATCHED-OLD (right),
  each labelled with its source frame seq. When a keyframe gray was evicted from
  the UI's buffer a dark placeholder pane is drawn in its place (the counts +
  verdict still render -- the loop is shown even without the image);
* one LINE per matched ORB keypoint, from its pixel in the current pane to its
  pixel in the old pane, COLOUR-CODED by the furthest verification stage it
  survived: GREY = appearance-only / dropped, YELLOW = epipolar (fundamental)
  inlier but NOT a PnP inlier, GREEN = PnP inlier (a confirmed-loop
  correspondence). This is the funnel made visible -- you SEE the grey appearance
  haze thin to the yellow epipolar set and finally to the green metric inliers;
* an overlay funnel readout ("appearance 98 -> epipolar 93 -> PnP 60"), the
  rotation-gate verdict ("rot 12.98 deg <= gate 30 deg"), and a big
  ACCEPTED / REJECTED banner.

The data is REAL: the matched pixels + per-match stage + funnel counts are the
SLAM engine's own loop-verification output, published on ``slam.loop`` (see
``slam.modules.publish_loops``); the grays are the keyframe images the VIO
published on ``keyframe`` and the UI buffered by seq. Nothing is invented.

cv2 is only a drawing backend here -- importing this module is what pulls it, not
the base UI (mirrors :mod:`ui.viz.gyrofuse_render`).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

# Per-match verification-stage labels (must match
# ``slam.mathlib.loop.loopclosure``: 0 = appearance, 1 = epipolar, 2 = PnP).
STAGE_APPEARANCE = 0
STAGE_EPIPOLAR = 1
STAGE_PNP = 2

# Palette (RGB -- the canvas is blitted as Format_RGB888 by the window).
_BG = (13, 17, 23)             # theme.BG  #0d1117
_PANE_BG = (22, 27, 34)        # slightly lighter pane fill
_GRID = (42, 50, 61)           # theme.GRID #2a323d
_TEXT = (230, 237, 243)        # theme.TEXT
_TEXT_DIM = (139, 148, 158)    # theme.TEXT_DIM
_GOOD = (124, 255, 92)         # theme.GOOD -- ACCEPTED / PnP inlier (green)
_WARN = (255, 176, 0)          # theme.WARN
_BAD = (255, 59, 48)           # theme.BAD -- REJECTED banner (red)

# The three stage colours (the funnel legend).
_C_DROPPED = (120, 130, 140)   # grey  -- appearance-only / dropped (stage 0)
_C_EPIPOLAR = (255, 196, 0)    # yellow -- epipolar inlier, not PnP (stage 1)
_C_PNP = (90, 230, 120)        # green  -- PnP inlier (stage 2)
_STAGE_COLOR = {STAGE_APPEARANCE: _C_DROPPED,
                STAGE_EPIPOLAR: _C_EPIPOLAR,
                STAGE_PNP: _C_PNP}

_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class LoopEvent:
    """One loop-closure event to render (the last one shown; loops are sporadic).

    All fields come straight off a ``slam.loop`` ``LoopMatch`` plus the two GRAY
    keyframe images the UI looked up by seq (``None`` when evicted).

    * ``cur_seq`` / ``old_seq`` -- the two keyframes' source frame seqs.
    * ``cur_gray`` / ``old_gray`` -- ``(H, W)`` uint8/float gray, or ``None`` if
      that keyframe's gray was evicted from the UI buffer (-> placeholder pane).
    * ``cur_px`` / ``old_px`` -- ``(N, 2)`` matched pixels in EACH keyframe's own
      pixel coords (same order; row i is one match).
    * ``stage`` -- ``(N,)`` uint8 per-match stage (0/1/2).
    * ``n_appearance`` / ``n_fmat`` / ``n_pnp`` -- the funnel counts.
    * ``rot_deg`` -- the loop's relative rotation vs odometry (NaN if no pair).
    * ``rot_gate_deg`` -- the rotation gate threshold (0 = gate disabled).
    * ``accepted`` -- True iff the candidate became a confirmed loop edge.
    """

    cur_seq: int
    old_seq: int
    cur_gray: np.ndarray | None
    old_gray: np.ndarray | None
    cur_px: np.ndarray
    old_px: np.ndarray
    stage: np.ndarray
    n_appearance: int
    n_fmat: int
    n_pnp: int
    rot_deg: float
    rot_gate_deg: float
    accepted: bool


def _to_u8_rgb(gray: np.ndarray | None, h: int, w: int,
               placeholder: str) -> np.ndarray:
    """A (h, w, 3) uint8 RGB pane: the gray scaled to fit, or a placeholder."""
    pane = np.full((h, w, 3), _PANE_BG, np.uint8)
    if gray is None:
        # Evicted keyframe: a dark pane with a centred note (the counts + verdict
        # still render around it, so the loop is shown without the image).
        msg = placeholder
        tw = cv2.getTextSize(msg, _FONT, 0.5, 1)[0][0]
        cv2.putText(pane, msg, ((w - tw) // 2, h // 2), _FONT, 0.5,
                    _TEXT_DIM, 1, cv2.LINE_AA)
        return pane
    g = np.asarray(gray)
    if g.dtype != np.uint8:
        g = np.clip(g, 0, 255).astype(np.uint8)
    gh, gw = g.shape[:2]
    # Letterbox-fit the gray into the pane (keep aspect; the pixel mapping uses
    # the SAME scale + offset so the match lines land on the right pixels).
    scale = min(w / gw, h / gh)
    nw, nh = max(1, int(round(gw * scale))), max(1, int(round(gh * scale)))
    resized = cv2.resize(g, (nw, nh), interpolation=cv2.INTER_AREA)
    rgb = np.stack([resized] * 3, axis=2)
    ox, oy = (w - nw) // 2, (h - nh) // 2
    pane[oy:oy + nh, ox:ox + nw] = rgb
    return pane


def _pane_xform(gray: np.ndarray | None, h: int, w: int):
    """Return ``(scale, ox, oy)`` mapping a keyframe pixel -> this pane's pixel.

    Mirrors the letterbox-fit in :func:`_to_u8_rgb` so a match drawn at keyframe
    pixel ``(u, v)`` lands at ``(ox + u*scale, oy + v*scale)`` inside the pane.
    Returns ``None`` when the gray is missing (no lines drawn for that side).
    """
    if gray is None:
        return None
    g = np.asarray(gray)
    gh, gw = g.shape[:2]
    scale = min(w / gw, h / gh)
    nw, nh = max(1, int(round(gw * scale))), max(1, int(round(gh * scale)))
    return scale, (w - nw) // 2, (h - nh) // 2


def render_loop(ev: LoopEvent | None, width: int = 1100,
                height: int = 560) -> np.ndarray:
    """Render one :class:`LoopEvent` (or a waiting screen) to an (H, W, 3) uint8.

    Layout: a header band, two side-by-side keyframe panes with the colour-coded
    match lines spanning them, a legend, and a footer with the funnel + gate
    verdict. ``ev=None`` draws a "waiting for a loop closure" screen.
    """
    canvas = np.full((height, width, 3), _BG, np.uint8)

    # --- header band ----------------------------------------------------- #
    cv2.rectangle(canvas, (0, 0), (width - 1, 34), _PANE_BG, cv2.FILLED)
    cv2.putText(canvas, "LOOP CLOSURE", (12, 23), _FONT, 0.62, _TEXT, 1,
                cv2.LINE_AA)
    cv2.putText(canvas, "current  |  matched-old  -  ORB match funnel",
                (185, 23), _FONT, 0.44, _TEXT_DIM, 1, cv2.LINE_AA)

    if ev is None:
        msg = "waiting for a loop closure ..."
        tw = cv2.getTextSize(msg, _FONT, 0.7, 1)[0][0]
        cv2.putText(canvas, msg, ((width - tw) // 2, height // 2), _FONT, 0.7,
                    _TEXT_DIM, 1, cv2.LINE_AA)
        return canvas

    # --- two keyframe panes ---------------------------------------------- #
    pane_top, pane_bot = 44, height - 84
    pane_h = pane_bot - pane_top
    gap = 8
    pane_w = (width - 3 * gap) // 2
    lx, rx = gap, gap + pane_w + gap                      # left / right pane x0

    left = _to_u8_rgb(ev.cur_gray, pane_h, pane_w,
                      f"keyframe {ev.cur_seq} not buffered")
    right = _to_u8_rgb(ev.old_gray, pane_h, pane_w,
                       f"keyframe {ev.old_seq} not buffered")
    canvas[pane_top:pane_top + pane_h, lx:lx + pane_w] = left
    canvas[pane_top:pane_top + pane_h, rx:rx + pane_w] = right
    for x0 in (lx, rx):
        cv2.rectangle(canvas, (x0, pane_top), (x0 + pane_w - 1, pane_bot - 1),
                      _GRID, 1)
    cv2.putText(canvas, f"CURRENT  kf {ev.cur_seq}", (lx + 6, pane_top + 18),
                _FONT, 0.46, _TEXT, 1, cv2.LINE_AA)
    cv2.putText(canvas, f"MATCHED-OLD  kf {ev.old_seq}", (rx + 6, pane_top + 18),
                _FONT, 0.46, _TEXT, 1, cv2.LINE_AA)

    # --- match lines (only when BOTH grays are present -> real pixel maps) - #
    xf_l = _pane_xform(ev.cur_gray, pane_h, pane_w)
    xf_r = _pane_xform(ev.old_gray, pane_h, pane_w)
    if xf_l is not None and xf_r is not None:
        sl, oxl, oyl = xf_l
        sr, oxr, oyr = xf_r
        cur = np.asarray(ev.cur_px, np.float32).reshape(-1, 2)
        old = np.asarray(ev.old_px, np.float32).reshape(-1, 2)
        stg = np.asarray(ev.stage, np.uint8).reshape(-1)
        n = min(len(cur), len(old), len(stg))
        # Draw in stage order (dropped first, PnP last) so the GREEN confirmed
        # correspondences sit ON TOP of the grey haze -- the inliers stay legible.
        for want in (STAGE_APPEARANCE, STAGE_EPIPOLAR, STAGE_PNP):
            color = _STAGE_COLOR[want]
            for i in range(n):
                if int(stg[i]) != want:
                    continue
                p0 = (lx + int(round(oxl + cur[i, 0] * sl)),
                      pane_top + int(round(oyl + cur[i, 1] * sl)))
                p1 = (rx + int(round(oxr + old[i, 0] * sr)),
                      pane_top + int(round(oyr + old[i, 1] * sr)))
                cv2.line(canvas, p0, p1, color, 1, cv2.LINE_AA)
                cv2.circle(canvas, p0, 2, color, -1, cv2.LINE_AA)
                cv2.circle(canvas, p1, 2, color, -1, cv2.LINE_AA)

    # --- legend (the three stage colours) -------------------------------- #
    ly = pane_bot + 16
    legend = [(_C_DROPPED, "dropped"), (_C_EPIPOLAR, "epipolar"),
              (_C_PNP, "PnP inlier")]
    x = 12
    for color, label in legend:
        cv2.line(canvas, (x, ly - 4), (x + 22, ly - 4), color, 2, cv2.LINE_AA)
        cv2.putText(canvas, label, (x + 28, ly), _FONT, 0.42, _TEXT_DIM, 1,
                    cv2.LINE_AA)
        x += 40 + cv2.getTextSize(label, _FONT, 0.42, 1)[0][0]

    # --- funnel readout + rotation-gate verdict -------------------------- #
    fy = pane_bot + 40
    funnel = (f"appearance {ev.n_appearance}  ->  epipolar {ev.n_fmat}  ->  "
              f"PnP {ev.n_pnp}")
    cv2.putText(canvas, funnel, (12, fy), _FONT, 0.52, _TEXT, 1, cv2.LINE_AA)

    if ev.rot_gate_deg and ev.rot_gate_deg > 0.0 and np.isfinite(ev.rot_deg):
        ok = ev.rot_deg <= ev.rot_gate_deg
        rcol = _GOOD if ok else _BAD
        rtxt = (f"rot {ev.rot_deg:.2f} deg "
                f"{'<=' if ok else '>'} gate {ev.rot_gate_deg:.0f} deg")
    elif np.isfinite(ev.rot_deg):
        rcol, rtxt = _TEXT_DIM, f"rot {ev.rot_deg:.2f} deg  (gate off)"
    else:
        rcol, rtxt = _TEXT_DIM, "rot n/a  (no odometry pair)"
    fw = cv2.getTextSize(funnel, _FONT, 0.52, 1)[0][0]
    cv2.putText(canvas, rtxt, (12 + fw + 30, fy), _FONT, 0.48, rcol, 1,
                cv2.LINE_AA)

    # --- ACCEPTED / REJECTED banner (bottom-right) ----------------------- #
    verdict = "ACCEPTED" if ev.accepted else "REJECTED"
    vcol = _GOOD if ev.accepted else _BAD
    vw = cv2.getTextSize(verdict, _FONT, 0.8, 2)[0][0]
    cv2.putText(canvas, verdict, (width - vw - 16, fy + 2), _FONT, 0.8, vcol, 2,
                cv2.LINE_AA)
    return canvas
