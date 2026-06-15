#!/usr/bin/env python3
"""Self-test for the IMU->camera rotation solver (Wahba/Kabsch).

Fully OFFLINE, pure math (no depthai, no hardware): proves the solver in
:mod:`sky.sensors.imu_cam_extrinsic` recovers a KNOWN rotation from synthetic
at-rest accel measurements, that the result is a proper rotation, that it is
robust to small noise, and that it rejects degenerate (parallel) pose sets.

Why this matters: the recovered rotation feeds the gravity-aligned startup
attitude AND the gyro rotation prior, so a wrong solve is a flight-safety bug
(bad attitude to the FC). This is the guard that the maths is correct before any
device-side capture trusts it.

Run::

    .venv/bin/python -m imu_camera.tests.imu_cam_extrinsic_selftest
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sky.sensors.imu_cam_extrinsic import (                       # noqa: E402
    POSES, residual_deg, solve_imu_cam_rotation)
from sky.sensors.calib_store import (                             # noqa: E402
    load_imu_cam_rotation, save_imu_cam_rotation)


def _rot(axis: np.ndarray, deg: float) -> np.ndarray:
    """A proper rotation about ``axis`` by ``deg`` (Rodrigues)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    th = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def _synth_measurements(R_true: np.ndarray, ups: np.ndarray,
                        *, g: float = 9.81,
                        noise: float = 0.0, seed: int = 0) -> np.ndarray:
    """IMU-frame accel for each optical up-direction under R_true (IMU->optical).

    ``R_true @ m = e`` => ``m = R_true^T @ e`` (the accel measured in the IMU
    frame), scaled to g, optionally perturbed by Gaussian noise (m/s^2).
    """
    rng = np.random.default_rng(seed)
    m = (R_true.T @ ups.T).T * g
    if noise > 0:
        m = m + rng.normal(0.0, noise, size=m.shape)
    return m


def test_recovers_known_rotation() -> None:
    ups = np.array([p.up_cam for p in POSES], dtype=float)
    for name, R_true in [
        ("diag(1,-1,-1) [the OAK-D W / corrected-Lite value]",
         np.diag([1.0, -1.0, -1.0])),
        ("Rx(90) [the wrong nominal Lite EEPROM value]", _rot([1, 0, 0], 90)),
        ("random-ish 37deg about (1,2,3)", _rot([1, 2, 3], 37)),
    ]:
        m = _synth_measurements(R_true, ups)
        R = solve_imu_cam_rotation(m, ups)
        assert np.allclose(R, R_true, atol=1e-9), (name, R, R_true)
        assert np.isclose(np.linalg.det(R), 1.0), name
        assert np.max(residual_deg(R, m, ups)) < 1e-6, name
    print("[a] recovers known R exactly (clean data), det=+1, residual~0       OK")


def test_noise_robust() -> None:
    ups = np.array([p.up_cam for p in POSES], dtype=float)
    R_true = np.diag([1.0, -1.0, -1.0])
    # ~0.15 m/s^2 noise (typical BMI270 at-rest jitter): recovered within ~1.5deg.
    m = _synth_measurements(R_true, ups, noise=0.15, seed=7)
    R = solve_imu_cam_rotation(m, ups)
    # angular distance between R and R_true
    ang = np.degrees(np.arccos(np.clip((np.trace(R.T @ R_true) - 1) / 2, -1, 1)))
    assert ang < 2.0, ang
    assert np.max(residual_deg(R, m, ups)) < 3.0, residual_deg(R, m, ups)
    print(f"[b] noise 0.15 m/s^2 -> recovered within {ang:.2f} deg              OK")


def test_two_poses_suffice() -> None:
    # The two orthogonal poses (lens-up +z, upright-forward -y) alone determine R.
    ups = np.array([POSES[0].up_cam, POSES[1].up_cam], dtype=float)
    R_true = _rot([0.3, 1, -0.2], 65)
    m = _synth_measurements(R_true, ups)
    R = solve_imu_cam_rotation(m, ups)
    assert np.allclose(R, R_true, atol=1e-9), (R, R_true)
    print("[c] two orthogonal poses determine R uniquely                       OK")


def test_rejects_degenerate() -> None:
    # Two parallel (anti-parallel) directions: no unique rotation -> must raise.
    ups = np.array([[0, 0, 1.0], [0, 0, -1.0]])
    m = np.array([[0, 0, 9.81], [0, 0, -9.81]])
    try:
        solve_imu_cam_rotation(m, ups)
        raise AssertionError("expected ValueError on parallel-only directions")
    except ValueError:
        pass
    # And <2 pairs.
    try:
        solve_imu_cam_rotation(np.array([[0, 0, 9.81]]), np.array([[0, 0, 1.0]]))
        raise AssertionError("expected ValueError on a single pair")
    except ValueError:
        pass
    print("[d] degenerate (parallel / single) pose sets raise                  OK")


def test_store_roundtrip() -> None:
    import tempfile

    R = np.diag([1.0, -1.0, -1.0])
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "imu_calib.json"
        save_imu_cam_rotation("DEV123", R, n_poses=3, residual_deg=0.4, path=path)
        got = load_imu_cam_rotation("DEV123", path=path)
        assert got is not None and np.allclose(got, R), got
        # Unknown device -> None.
        assert load_imu_cam_rotation("NOPE", path=path) is None
        # A non-rotation entry (det != 1) must be rejected, not returned.
        save_imu_cam_rotation("BADD", np.diag([1.0, 1.0, 2.0]), n_poses=2,
                              path=path)
        assert load_imu_cam_rotation("BADD", path=path) is None
    print("[e] calib_store roundtrip + rejects non-rotation                    OK")


def main() -> int:
    test_recovers_known_rotation()
    test_noise_robust()
    test_two_poses_suffice()
    test_rejects_degenerate()
    test_store_roundtrip()
    print("\nALL imu_cam_extrinsic solver + store CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
