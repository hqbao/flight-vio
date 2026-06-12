"""Monte-Carlo validation of the IMU preintegration covariance (Phase 0).

This is THE correctness gate for the 9x9 preintegration covariance ``Sigma`` added
to :func:`sky.vio.imu.preintegrate_imu`. Without a correct ``Sigma`` the
tight IMU factor would be silently mis-weighted (``Omega_I = Sigma^-1``), with no
crash -- so the analytic propagation must be checked against ground truth.

How the gate works
------------------
1. Build a realistic IMU segment (a short fast-motion arc, ~0.25 s @ 200 Hz) and
   preintegrate it ONCE to get the analytic covariance ``Sigma`` (residual order
   ``eta = [dphi(3); dvel(3); dpos(3)]``).
2. Draw N noisy re-integrations. Each segment's midpoint gyro/accel gets additive
   white noise matching the SAME discrete density the propagation assumes:
   ``Cov(n) = diag(sigma_g^2/dt, sigma_a^2/dt)`` (continuous density / dt). This
   is exactly the ``Q/dt`` term in ``Sigma <- A Sigma A^T + B (Q/dt) B^T``.
3. Compute the EMPIRICAL covariance of the resulting deltas:
   ``dphi = Log(dR_nom^T dR_noisy)``, ``dvel = dv - dv_nom``, ``dpos = dp - dp_nom``.
4. Assert empirical ~= analytic within Monte-Carlo error (relative Frobenius +
   a per-block check), plus the two "sanity" gates from the plan: ``Sigma`` grows
   monotonically over the segment, and its position 1-sigma is cm-scale over the
   ~0.25 s interval.

The MC noise is injected at the segment midpoint (the same quantity the
linearisation differentiates), so the empirical covariance is the ground truth
for the analytic recursion -- a mismatch means a wrong A/B block or ordering.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.vio.imu import (  # noqa: E402
    ImuNoise,
    preintegrate_imu,
    so3_log,
)


def make_segment(rate_hz: float = 200.0, dur_s: float = 0.25, seed: int = 3):
    """A fast-motion IMU arc: time-varying gyro + accel, ~0.25 s @ 200 Hz.

    Returns ``(ts_ns, gyro, accel)`` -- a contiguous body-frame block exactly like
    one inter-keyframe interval. Not at-rest: the motion makes the covariance
    cross-couple all 9 states so the test exercises every A/B block.
    """
    n = int(round(rate_hz * dur_s)) + 1
    t = np.linspace(0.0, dur_s, n)
    ts_ns = np.round(t * 1e9).astype(np.int64)
    # Smoothly varying angular velocity (rad/s) and specific force (m/s^2),
    # including a gravity-like DC term so accel magnitude is realistic (~9.8).
    gyro = np.column_stack([
        0.4 * np.sin(2.0 * t + 0.3),
        0.7 * np.cos(1.5 * t),
        -0.5 * np.sin(3.0 * t + 1.0),
    ])
    accel = np.column_stack([
        1.5 * np.sin(2.5 * t),
        -9.81 + 0.8 * np.cos(2.0 * t + 0.5),
        2.0 * np.sin(1.2 * t + 0.2),
    ])
    rng = np.random.default_rng(seed)
    gyro = gyro + rng.normal(0.0, 1e-4, gyro.shape)
    accel = accel + rng.normal(0.0, 1e-3, accel.shape)
    return ts_ns, gyro, accel


def empirical_covariance(ts_ns, gyro, accel, bg, ba, noise: ImuNoise,
                         n_draws: int, seed: int):
    """Monte-Carlo covariance of the 9-state delta under measurement noise.

    Each segment ``k->k+1`` of length ``dt`` gets independent additive white noise
    on its midpoint gyro/accel with std ``sqrt(sigma^2/dt)`` -- the discrete
    realisation of the continuous density the propagation assumes. The deltas are
    expressed relative to the noise-free nominal preintegration, in residual order
    ``[dphi; dvel; dpos]``.
    """
    nom = preintegrate_imu(ts_ns, gyro, accel, bg, ba, noise=noise)
    dt_seg = (ts_ns[1:].astype(np.float64) - ts_ns[:-1].astype(np.float64)) * 1e-9

    # Per-segment midpoint of the clean signal (what preintegrate_imu uses).
    g_mid = 0.5 * (gyro[1:] + gyro[:-1])
    a_mid = 0.5 * (accel[1:] + accel[:-1])
    # Std of the additive midpoint noise per segment (sqrt(density^2 / dt)).
    sg = noise.sigma_g / np.sqrt(dt_seg)         # (S,)
    sa = noise.sigma_a / np.sqrt(dt_seg)
    nseg = dt_seg.shape[0]

    rng = np.random.default_rng(seed)
    etas = np.empty((n_draws, 9))
    # Reconstruct a 2-sample block per segment whose midpoint equals the noisy
    # midpoint: set both endpoints to the noisy midpoint value so preintegrate's
    # own 0.5*(s[k]+s[k+1]) recovers exactly the perturbed midpoint, segment by
    # segment. (We integrate each segment independently and chain the deltas.)
    for d in range(n_draws):
        ng = g_mid + rng.normal(0.0, 1.0, (nseg, 3)) * sg[:, None]
        na = a_mid + rng.normal(0.0, 1.0, (nseg, 3)) * sa[:, None]
        # Chain the per-segment preintegrations (each a 2-sample block whose
        # midpoint is the perturbed value) into one accumulated delta.
        dR = np.eye(3)
        dv = np.zeros(3)
        dp = np.zeros(3)
        for k in range(nseg):
            dt = dt_seg[k]
            ts2 = np.array([0, int(round(dt * 1e9))], np.int64)
            g2 = np.array([ng[k], ng[k]])
            a2 = np.array([na[k], na[k]])
            seg = preintegrate_imu(ts2, g2, a2, bg, ba, noise=noise)
            # accumulate: dp += dv*dt_local handled inside; chain in body frame
            # using the same composition the loop uses (dR @ dR_inc, etc.).
            dp = dp + dv * dt + dR @ seg.dp
            dv = dv + dR @ seg.dv
            dR = dR @ seg.dR
        dphi = so3_log(nom.dR.T @ dR)
        etas[d] = np.concatenate([dphi, dv - nom.dv, dp - nom.dp])

    emp = np.cov(etas, rowvar=False)
    return nom, emp


def block_report(name, analytic, empirical):
    fro_a = np.linalg.norm(analytic)
    fro_d = np.linalg.norm(empirical - analytic)
    rel = fro_d / max(fro_a, 1e-18)
    print(f"  {name:10s}  ||A||_F={fro_a:.4e}  rel_err={rel:.3f}")
    return rel


def main() -> int:
    ts, gyro, accel = make_segment()
    bg = np.zeros(3)
    ba = np.zeros(3)
    noise = ImuNoise(sigma_g=1.5e-3, sigma_a=2.0e-2)

    n_draws = 4000
    nom, emp = empirical_covariance(ts, gyro, accel, bg, ba, noise,
                                    n_draws=n_draws, seed=20260610)
    ana = nom.cov
    assert ana is not None, "covariance not propagated"

    print(f"=== IMU preintegration covariance Monte-Carlo (N={n_draws}) ===")
    print(f"  segment dt = {nom.dt*1e3:.1f} ms, {len(ts)} samples")

    # Whole-matrix relative Frobenius error.
    rel_full = np.linalg.norm(emp - ana) / max(np.linalg.norm(ana), 1e-18)
    print(f"  full 9x9 relative Frobenius error = {rel_full:.4f}")

    # Per-block (rot / vel / pos diagonal blocks -- the dominant magnitudes).
    rel_rot = block_report("rot(dphi)", ana[0:3, 0:3], emp[0:3, 0:3])
    rel_vel = block_report("vel(dvel)", ana[3:6, 3:6], emp[3:6, 3:6])
    rel_pos = block_report("pos(dpos)", ana[6:9, 6:9], emp[6:9, 6:9])

    # Position 1-sigma magnitude (sqrt of trace of pos block) -- must be cm-scale
    # over ~0.25 s, NOT metres or microns (sanity on the noise densities / B_k).
    pos_1sigma_m = float(np.sqrt(np.trace(ana[6:9, 6:9])))
    print(f"  position 1-sigma over segment = {pos_1sigma_m*100:.3f} cm")

    # Monotonic growth: covariance trace must increase as we integrate more of
    # the segment (information is only added, never removed).
    traces = []
    for cut in (3, len(ts) // 2, len(ts)):
        sub = preintegrate_imu(ts[:cut], gyro[:cut], accel[:cut], bg, ba,
                               noise=noise)
        traces.append(float(np.trace(sub.cov)))
    monotonic = all(traces[i] < traces[i + 1] for i in range(len(traces) - 1))
    print(f"  cov trace growth = {['%.3e' % v for v in traces]}  "
          f"(monotonic={monotonic})")

    # sqrt_info contract: sqrt_info^T sqrt_info == inv(cov).
    info = nom.sqrt_info.T @ nom.sqrt_info
    info_err = np.linalg.norm(info @ ana - np.eye(9))
    print(f"  ||sqrt_info^T sqrt_info @ cov - I|| = {info_err:.2e}")

    # --- gates -------------------------------------------------------------
    # MC error on a covariance from N draws scales ~ sqrt(2/N) ~ 2.2% here; allow
    # generous margins (the blocks span orders of magnitude, so the full-matrix
    # Frobenius is dominated by the largest block).
    ok = (
        rel_full < 0.08
        and rel_rot < 0.10
        and rel_vel < 0.10
        and rel_pos < 0.12
        # mm..cm scale over a ~0.25 s interval: NOT metres (bad B_k) and NOT
        # microns (noise densities zeroed). Consumer MEMS gives ~mm here.
        and 5e-4 < pos_1sigma_m < 0.20
        and monotonic
        and info_err < 1e-8
    )
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
