"""Resolution-aware vision tuning for the from-scratch VIO/SLAM pipeline.

The whole pipeline was tuned at the **640x400** baseline. Almost every threshold
in it is implicitly tied to that resolution because it is expressed in *pixels*:
corner spacing, the KLT window, the PnP reprojection gate, the stereo disparity
search range, the ORB feature budget, the loop-closure epipolar threshold, ...

Running at a *lower* resolution is the cheapest way to save CPU (cost scales with
the pixel count), but it shrinks every one of those pixel distances. A 12 px
corner spacing on a 640-wide image is 6 px on a 320-wide one; a 96 px disparity
search covers twice the metric depth range at half the width; the same KLT
window now spans twice the field of view. Left unchanged, the baseline numbers
become too coarse at low resolution and feature tracking / depth / pose quality
degrade — the symptom the user hits when they drop the frame size to run lighter.

This module is the single place that scales those pixel-unit parameters from the
640x400 baseline to the live ``(width, height)``. The scale factor is
``s = width / 640`` (proportional downscale keeps the aspect ratio, so width
alone is enough). **Metric** parameters (depths in metres, speeds in m/s, the
gyro-fusion gates in degrees) are resolution-independent and are deliberately
left untouched.

Each scaled parameter can also be **overridden at runtime** (``None`` = keep the
auto-scaled value). That is the knob set we co-tune per resolution; the verified
values live in ``docs/RESOLUTION_TUNING.md``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# DATA-ONLY by design: this module is pure resolution arithmetic + a frozen
# dataclass of scaled parameters. It imports NO project math, so it stays
# headless-safe and bit-identical across every project's vendored ``comms/``.
# The per-subsystem CONFIG BUILDERS that turn a ``ResolutionProfile`` into a
# concrete cfg live in the OWNING project's math library (each builder imports
# only the math it owns): e.g. ``imu_camera.mathlib.resolution_build.sgm_config``
# builds the SGMConfig; vio builds frontend/odometry/BA cfgs; slam builds the
# loop cfg. Keeping the builders out of here is what lets ``comms`` stay generic.

# The resolution every baseline threshold in the pipeline was tuned at.
BASELINE_W = 640
BASELINE_H = 400


def _round_odd(x: float, lo: int = 3) -> int:
    """Nearest odd integer >= ``lo`` (KLT / corner windows must be odd)."""
    n = int(round(x))
    if n % 2 == 0:
        n += 1
    return max(lo, n)


@dataclass(frozen=True)
class ResolutionProfile:
    """Resolution-scaled vision parameters + per-parameter runtime overrides.

    Build with :meth:`for_resolution`; each project's math library then turns this
    profile into a concrete cfg via its own builder (e.g.
    ``imu_camera.mathlib.resolution_build.sgm_config`` -> SGMConfig). The seven
    scalable fields below are the runtime knobs (each exposed as a CLI flag);
    everything else is derived from them + the stored resolution.
    """

    width: int = BASELINE_W
    height: int = BASELINE_H
    # --- the seven runtime-tunable, resolution-scaled parameters ------------
    max_corners: int = 400        # frontend: Shi-Tomasi corner budget
    min_distance: float = 12.0    # frontend: min px between corners
    klt_win: int = 21             # frontend: KLT window (odd px)
    klt_levels: int = 3           # frontend: KLT pyramid levels
    reproj_px: float = 2.0        # odometry: PnP RANSAC reprojection gate (px)
    num_disparities: int = 96     # stereo: SGM disparity search range (px)
    orb_features: int = 800       # loop closure: ORB feature budget

    # ------------------------------------------------------------------ #
    @property
    def scale(self) -> float:
        """Linear resolution scale vs the 640x400 baseline (``width / 640``)."""
        return self.width / BASELINE_W

    @classmethod
    def for_resolution(cls, width: int, height: int,
                       **overrides) -> "ResolutionProfile":
        """Scale every pixel-unit parameter from the baseline to ``width``.

        ``overrides`` may carry any of the seven tunable fields; a value of
        ``None`` (the argparse default) means "keep the auto-scaled value", so
        CLI flags can be forwarded straight through without branching.
        """
        s = width / BASELINE_W
        # Pyramid levels: drop one level per halving of resolution (a smaller
        # image needs a shallower pyramid; an extra level just blurs a few-pixel
        # image to mush). 640->3, 320->2, 160->1.
        auto_levels = 3 if s >= 1.0 else max(1, int(round(3 + math.log2(s))))
        auto = {
            # Linear in the side length, floored so we never starve the solver.
            "max_corners": max(80, int(round(400 * s))),
            "min_distance": max(4.0, 12.0 * s),
            "klt_win": _round_odd(21 * s, lo=7),
            "klt_levels": auto_levels,
            "reproj_px": max(1.0, 2.0 * s),
            # Disparity range is a pixel distance -> scales with width; keep it
            # even and not too small so the near-depth range survives.
            "num_disparities": max(32, int(round(96 * s / 2)) * 2),
            "orb_features": max(200, int(round(800 * s))),
        }
        # Apply only the overrides the caller actually set (non-None).
        auto.update({k: v for k, v in overrides.items() if v is not None})
        return cls(width=int(width), height=int(height), **auto)

    # ------------------------------------------------------------------ #
    def describe(self) -> str:
        """One-line summary for the startup log."""
        return (f"{self.width}x{self.height} (s={self.scale:.2f}): "
                f"corners={self.max_corners} min_dist={self.min_distance:.1f}px "
                f"klt={self.klt_win}px/{self.klt_levels}lvl "
                f"reproj={self.reproj_px:.1f}px ndisp={self.num_disparities} "
                f"orb={self.orb_features}")
