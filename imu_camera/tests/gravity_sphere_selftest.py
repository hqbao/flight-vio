#!/usr/bin/env python3
"""Self-test for the Gravity Sphere tool (:mod:`imu_camera.tools.gravity_sphere`).

Runs fully offline (read-only) and proves the two things that make the figure
trustworthy:

1. THE SNAP IS REAL. For BOTH data sources (the synthetic demo AND the real
   stored calib if one is on disk) the calibrated faces ``T(a-b)`` sit closer to
   the gravity magnitude ``g`` than the raw faces do -- i.e. the green dots really
   are nearer the g-sphere than the red ones. This is the whole teaching claim.

2. THE RENDER IS NON-BLANK. The headless ``--render`` writes a real PNG with
   actual drawn content (not an all-background image), checked by both a minimum
   byte size and pixel-variance over the decoded image.

Plus solver / reconstruction sanity:

* the synthetic demo recovers the injected model to a tight residual, and
* the stored-calib reconstruction round-trips (``cal.apply(reconstructed)`` lands
  on ``g * dir`` to numerical precision) so the "before" dots shown are faithful
  to the persisted (T, b), not invented.

Run::

    python -m imu_camera.tests.gravity_sphere_selftest
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.sensors.accel_calib import SIX_FACES                     # noqa: E402
from imu_camera.tools.gravity_sphere import (                     # noqa: E402
    SphereData,
    from_demo,
    from_stored_calib,
    render_sphere_png,
)


def _check(cond: bool, msg: str) -> None:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        raise SystemExit(1)


def _snap_is_real(data: SphereData) -> tuple[float, float]:
    """Return (raw RMS |a|-g, cal RMS |a|-g); the cal RMS must be smaller."""
    g = data.calib.g
    raw_rms = float(np.sqrt(np.mean(
        (np.linalg.norm(data.raw_faces, axis=1) - g) ** 2)))
    cal_rms = float(np.sqrt(np.mean(
        (np.linalg.norm(data.cal_faces, axis=1) - g) ** 2)))
    return raw_rms, cal_rms


def _png_is_nontrivial(path: Path) -> bool:
    """A PNG with real drawn content: non-trivial size AND pixel variance.

    Size guards against a header-only stub; pixel variance guards against an
    all-one-colour (blank) canvas. Uses an image backend if available, else
    falls back to the byte-size check alone (still catches an empty render).
    """
    if not path.exists() or path.stat().st_size < 5000:
        return False
    try:
        import matplotlib.image as mpimg
        img = mpimg.imread(str(path))
        return float(np.var(img)) > 1e-4
    except Exception:                                # pragma: no cover - backend-dep
        return path.stat().st_size > 5000


def main() -> int:
    print("gravity_sphere_selftest")

    # ----- 1. Synthetic demo: snap is real + solver recovers the model. ----- #
    print("[demo source]")
    demo = from_demo()
    _check(demo.synthetic, "demo is flagged synthetic (honest labelling)")
    _check(demo.raw_faces.shape == (6, 3) and demo.cal_faces.shape == (6, 3),
           "six raw + six calibrated 3-vectors")
    raw_rms, cal_rms = _snap_is_real(demo)
    _check(cal_rms < raw_rms,
           f"calibrated nearer |g| than raw (snap): raw={raw_rms:.3f} -> "
           f"cal={cal_rms:.4f} m/s^2")
    # The raw ellipsoid must be visibly distorted (else there's nothing to show),
    # and the solver must drive the residual right down.
    _check(raw_rms > 0.1, f"raw faces are genuinely off the sphere (raw_rms={raw_rms:.3f})")
    _check(demo.calib.residual_g < 0.1,
           f"solver recovered the model (residual_g={demo.calib.residual_g:.4f})")

    # ----- 2. Real stored calib if present: snap + faithful reconstruction. - #
    print("[stored-calib source]")
    try:
        stored = from_stored_calib(None)
        have_stored = True
    except LookupError as exc:
        print(f"  [info] no stored accel calib on disk ({exc}); skipping stored "
              "checks (demo path is the verified one here)")
        have_stored = False

    if have_stored:
        _check(not stored.synthetic, "stored source is flagged NOT synthetic")
        s_raw, s_cal = _snap_is_real(stored)
        _check(s_cal <= s_raw,
               f"stored: calibrated nearer |g| than raw: raw={s_raw:.3f} -> "
               f"cal={s_cal:.4f} m/s^2")
        # Reconstruction faithfulness: applying the stored model to the
        # reconstructed raw faces must land EXACTLY on g*dir (numerical).
        g = stored.calib.g
        landed = stored.calib.apply(stored.raw_faces)
        target = g * SIX_FACES
        max_err = float(np.max(np.abs(landed - target)))
        _check(max_err < 1e-6,
               f"reconstructed raw faces round-trip onto g*dir (max err {max_err:.2e})")

    # ----- 3. Headless render writes a non-blank PNG. ----- #
    print("[render]")
    src = stored if have_stored else demo
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "gravity_sphere.png"
        written = render_sphere_png(src, str(out))
        _check(Path(written) == out.resolve(), "render returns the written path")
        _check(_png_is_nontrivial(out),
               f"PNG is non-trivial (size {out.stat().st_size} bytes, has content)")

    print("ALL PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
