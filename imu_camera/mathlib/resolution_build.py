"""Per-project config builders that turn a generic ``ResolutionProfile`` into the
concrete cfg objects this project's math needs.

The profile itself (``imu_camera.comms.lib.config.resolution``) is data-only and
headless — it imports no math. The builders live HERE so the math import (SGM)
is owned by the project that uses it (imu_camera owns the stereo/SGM library);
vio/slam carry their own builders for frontend/odometry/loop. This is what keeps
the vendored ``comms`` package generic + bit-identical across projects.
"""
from __future__ import annotations

from dataclasses import replace

from imu_camera.comms.lib.config.resolution import ResolutionProfile
from imu_camera.mathlib.stereo.stereo import SGMConfig


def sgm_config(res: ResolutionProfile, *, fast: bool) -> SGMConfig:
    """Dense-SGM depth config at this resolution.

    ``fast`` selects the half-res live preset (cheaper census + 4 paths +
    internal downscale); either way the disparity search range is set from the
    resolution so the metric near-depth bound (``fx*B/num_disparities``) stays
    roughly constant across resolutions (``fx`` scales with width too). Logic is
    verbatim from the pre-split ``ResolutionProfile.sgm`` so depth stays
    byte-identical.
    """
    base = SGMConfig.live() if fast else SGMConfig()
    return replace(base, num_disparities=int(res.num_disparities))
