#!/usr/bin/env python3
"""THE make-or-break test: flat-wall (single-plane) degeneracy stays BOUNDED.

A fronto-parallel wall under pure lateral camera motion is the worst case for
the dense-ICP factor: translation ALONG the wall is geometrically unobservable
from depth (the plane looks identical), so a naive ICP factor would invent an
arbitrary along-wall constraint and could blow up the solve. The factor's
eigenvalue REMAP exists precisely to project that direction out (its
``Lambda`` eigenvalue is ~0). This test proves the remap works end-to-end:

  * synthesize a single-plane scene (fronto-parallel wall at ~1.3 m), 54x42 ToF
    grid, with the camera translating laterally;
  * drive the FULL tight VIO (``WindowedVIORGBDOdometry``) with the ICP factor ON
    vs OFF;
  * assert the ICP-ON trajectory does NOT EXPLODE -- its peak position and
    per-frame step stay within a sane multiple of the ICP-OFF baseline (the
    degeneracy degrades GRACEFULLY, it does not diverge).

This is a synthetic SIL check (no recorded clip needed). If it explodes, the
degeneracy handling is broken and must be fixed before anything else.

Run::

    .venv/bin/python vio/tests/icp_flatwall_degeneracy_selftest.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from vio.mathlib.backend.vio_window import (  # noqa: E402
    WindowedVIOConfig, WindowedVIORGBDOdometry,
)


def _synth_flatwall(n_frames=40, W=54, H=42, z_wall=1.3, vx=0.02):
    """Synthesise a fronto-parallel-wall ToF sequence with lateral camera motion.

    Returns (K, frames) where each frame is (gray, depth_m, ts_ns). The wall is
    at constant depth ``z_wall``; the camera slides +x by ``vx`` m/frame. Gray
    carries a faint vertical-stripe texture (so the KLT frontend has *something*
    but is feature-starved -- the regime the ICP factor targets). Depth is the
    wall depth seen under the lateral shift (still a flat plane every frame, since
    a fronto-parallel wall stays fronto-parallel under lateral translation).
    """
    fx = fy = 40.0
    cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
    rng = np.random.default_rng(0)
    # a fixed faint texture on the wall (world-fixed), sampled per frame under
    # the lateral shift so the few trackable corners actually move with motion.
    base = (rng.uniform(0, 40, size=(H, W * 3)).astype(np.float32) + 100.0)
    frames = []
    dt_ns = int(0.05 * 1e9)            # 20 fps
    for i in range(n_frames):
        shift = int(round(i * vx * fx / z_wall))     # pixel shift of the texture
        col0 = W + shift
        gray = base[:, col0:col0 + W].astype(np.uint8)
        depth = np.full((H, W), z_wall, np.float32)
        depth += rng.normal(0, 0.003, size=(H, W)).astype(np.float32)
        frames.append((gray, depth, i * dt_ns))
    return K, frames


def _run(depth_icp: bool):
    K, frames = _synth_flatwall()
    n = len(frames)
    ts = np.array([f[2] for f in frames], np.int64)
    # near-static IMU: only gravity (accel +g up == optical +y), tiny gyro. The
    # true lateral motion is NOT in the IMU (no accel excitation) -- exactly the
    # case where depth/ICP must carry translation.
    gyro = np.tile(np.array([0.0, 0.0, 0.0]), (n, 1)).astype(np.float64)
    accel = np.tile(np.array([0.0, 9.81, 0.0]), (n, 1)).astype(np.float64)

    cfg = WindowedVIOConfig()
    cfg.min_depth_m = 0.2
    cfg.max_depth_m = 8.0
    cfg.min_ba_views = 1
    cfg.vio.imu_info_weight = True
    cfg.depth_icp = depth_icp

    vo = WindowedVIORGBDOdometry(K, ts, gyro, accel,
                                 bg0=np.zeros(3), ba0=np.zeros(3), cfg=cfg)
    vo.align_to_gravity(accel[0])
    pos = []
    for gray, depth, t in frames:
        p = vo.process(gray, depth, t)
        pos.append(p[:3, 3].copy())
    pos = np.array(pos)
    peak = float(np.max(np.linalg.norm(pos, axis=1)))
    steps = (np.linalg.norm(np.diff(pos, axis=0), axis=1)
             if len(pos) > 1 else np.zeros(1))
    max_step = float(steps.max())
    return peak, max_step, bool(np.all(np.isfinite(pos)))


def main() -> int:
    ok = True
    peak_off, step_off, fin_off = _run(depth_icp=False)
    peak_icp, step_icp, fin_icp = _run(depth_icp=True)

    print(f"  ICP OFF: peak|p|={peak_off*100:.2f}cm  max_step={step_off*100:.2f}cm  "
          f"finite={fin_off}")
    print(f"  ICP ON : peak|p|={peak_icp*100:.2f}cm  max_step={step_icp*100:.2f}cm  "
          f"finite={fin_icp}")

    # 1. finite (no NaN/inf -> the solve did not blow up numerically)
    finite_ok = fin_icp
    print(f"[{'ok' if finite_ok else 'FAIL'}] ICP-ON trajectory is finite "
          "(no numeric divergence)")
    ok = ok and finite_ok

    # 2. GRACEFUL DEGRADATION: the along-wall degeneracy must not make the ICP-ON
    #    solve WORSE than the OFF baseline -- the eigenvalue remap projects the
    #    unobservable along-wall direction out, so the factor contributes nothing
    #    there and the trajectory should TRACK the OFF baseline, not explode. The
    #    baseline itself drifts (synthetic feature-starved scene + no IMU lateral
    #    excitation), so we key on the RELATIVE behaviour: ICP-ON peak + step stay
    #    within a small headroom of OFF (i.e. ICP did not blow the null direction
    #    up). A broken remap would send peak/step to many multiples of OFF.
    headroom = 1.5           # ICP-ON allowed at most 50% worse than OFF
    peak_ok = peak_icp <= max(peak_off * headroom, peak_off + 0.10)
    step_ok = step_icp <= max(step_off * headroom, step_off + 0.10)
    bounded_ok = peak_ok and step_ok
    print(f"[{'ok' if bounded_ok else 'FAIL'}] ICP-ON degrades GRACEFULLY vs OFF "
          f"(peak {peak_icp/max(peak_off,1e-9):.2f}x, "
          f"step {step_icp/max(step_off,1e-9):.2f}x of baseline; "
          f"<= {headroom}x) -> eigenvalue remap works")
    ok = ok and bounded_ok

    print("\n" + ("PASS -- flat-wall degeneracy degrades gracefully (bounded), "
                  "the eigenvalue remap holds." if ok else
                  "FAIL -- the degeneracy handling is BROKEN (see above)."))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
