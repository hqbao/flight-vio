"""Solve the IMU->camera rotation from at-rest accelerometer in known poses.

The OAK EEPROM ships an IMU->camera extrinsic, but on some devices it is a wrong
NOMINAL value rather than a measured calibration. The OAK-D Lite (BMI270) is the
case that motivated this module: its EEPROM ``getImuToCameraExtrinsics`` returns a
clean ``Rx(90deg)`` that, applied to the live accel, points startup gravity at the
optical +y axis instead of the correct axis -- so the gravity-aligned initial
attitude is flipped ~180deg in roll. (The OAK-D W's BNO086 EEPROM value is a real
measured ``~Rx(180deg)`` and is correct.)

This recovers the rotation EMPIRICALLY, with no dependence on the EEPROM:

* At rest the accelerometer measures **specific force = -gravity**, i.e. it points
  *up*, expressed in the IMU frame.
* In a KNOWN camera pose the *up* direction in the camera OPTICAL frame
  (x-right, y-down, z-forward) is known a-priori (e.g. lens pointed at the ceiling
  => up is optical +z).
* Each pose therefore gives one ``(measured_imu_dir, known_optical_dir)`` pair, and
  >=2 non-parallel pairs determine the rotation uniquely via Wahba/Kabsch
  (orthogonal Procrustes with a determinant fix so the result is a proper
  rotation, det = +1).

The math here is pure and offline-testable (see
``sky/tests/imu_cam_extrinsic_selftest.py``); the live capture + persistence live
in :mod:`imu_camera.tools.imu_cam_calib` and :mod:`sky.sensors.calib_store`.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CalibPose:
    """One calibration pose: a human instruction + the up-direction it implies.

    ``up_cam`` is the unit specific-force (accelerometer) vector the camera should
    read in this pose, in the OPTICAL frame (x-right, y-down, z-forward). At rest
    the accel points UP (opposite gravity).
    """

    key: str
    instruction: str
    up_cam: np.ndarray


#: The canonical pose set the wizard walks through. Poses 1+2 are orthogonal and
#: already determine the rotation uniquely; pose 3 (anti-parallel to 1) adds a
#: redundant z-axis sample so noise on the vertical axis is averaged. Every
#: direction is easy to instruct unambiguously, which is what makes the capture
#: repeatable in the field.
POSES: tuple[CalibPose, ...] = (
    CalibPose("lens-up", "Lay the camera flat, LENS POINTING UP AT THE CEILING",
              np.array([0.0, 0.0, 1.0])),
    CalibPose("upright-forward",
              "Stand the camera UPRIGHT, lens LOOKING LEVEL FORWARD, TOP POINTING UP",
              np.array([0.0, -1.0, 0.0])),
    CalibPose("lens-down", "Lay the camera flat, LENS FACING DOWN onto the table",
              np.array([0.0, 0.0, -1.0])),
)


def _unit_rows(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    n = np.linalg.norm(a, axis=1, keepdims=True)
    if np.any(n < 1e-9):
        raise ValueError("a calibration direction has near-zero magnitude")
    return a / n


def solve_imu_cam_rotation(meas_imu: np.ndarray,
                           up_cam: np.ndarray) -> np.ndarray:
    """Rotation ``R`` (IMU->optical) best mapping each ``meas_imu_i`` to ``up_cam_i``.

    ``meas_imu`` (N,3) are the mean at-rest accel vectors in the IMU frame, one per
    pose; ``up_cam`` (N,3) are the corresponding known specific-force directions in
    the camera optical frame. Solves the Wahba problem
    ``argmin_R sum_i || R m_hat_i - e_hat_i ||^2`` over proper rotations via SVD
    (Kabsch) with the determinant correction that forbids a reflection.

    Requires >=2 non-parallel directions; raises ``ValueError`` otherwise. The
    returned ``R`` is the IMU->camera rotation to USE in place of the EEPROM's.
    """
    M = _unit_rows(np.atleast_2d(meas_imu))
    E = _unit_rows(np.atleast_2d(up_cam))
    if M.shape != E.shape or M.shape[0] < 2:
        raise ValueError("need >=2 matched (measured, expected) direction pairs")
    # Observability: the directions must span >=2 dimensions, else the rotation
    # about the common axis is unconstrained (e.g. two anti-parallel poses both on
    # the z axis leave a free DOF). The 2nd singular value of the stacked unit
    # directions is the span measure; a det(R)~1 check does NOT catch this because
    # the SVD still returns *a* proper rotation, just an arbitrary one.
    if float(np.linalg.svd(E, compute_uv=False)[1]) < 0.1:
        raise ValueError("degenerate poses (directions parallel/collinear) -- "
                         "use >=2 non-parallel orientations")
    # H = sum e_i m_i^T ; R = U diag(1,1,det(U V^T)) V^T maps m -> e.
    H = E.T @ M
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1.0, 1.0, d]) @ Vt
    return R


def residual_deg(R: np.ndarray, meas_imu: np.ndarray,
                 up_cam: np.ndarray) -> np.ndarray:
    """Per-pose angular error (degrees) between ``R @ measured`` and ``expected``."""
    M = _unit_rows(np.atleast_2d(meas_imu))
    E = _unit_rows(np.atleast_2d(up_cam))
    out = []
    for m, e in zip(M, E):
        c = float(np.clip((R @ m) @ e, -1.0, 1.0))
        out.append(np.degrees(np.arccos(c)))
    return np.asarray(out, dtype=np.float64)
