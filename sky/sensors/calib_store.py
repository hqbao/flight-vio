"""Persisted IMU calibration, keyed by device id (gyro bias + accel calibration).

A physical OAK-D's IMU has two kinds of correction that we want to keep across
runs so the operator does not recalibrate every flight:

* **gyro bias** -- the per-axis zero-rate offset (rad/s). A near-constant sensor
  property (it does drift slowly with temperature; see the ``temp_c`` field kept
  alongside each entry for a future temperature-aware model).
* **accel calibration** -- the full affine correction ``a_cal = T (a_raw - b)``
  from the six-position routine (see :mod:`sky.sensors.accel_calib`).

Both live in one tiny JSON file under the (gitignored) repo ``.cache`` dir, keyed
by device id so several cameras never clobber each other::

    {"<device_id>": {
        "gyro":  {"bias": [bx,by,bz], "n": 137, "ts": ..., "temp_c": null},
        "accel": {"T": [[...]], "bias": [...], "residual_g": ..., "g": ...,
                  "n_poses": 6, "ts": ...}
    }}

This module supersedes the old gyro-only bias store. It transparently MIGRATES
the two legacy on-disk shapes on read:

* the old gyro-only file ``.cache/imu_bias.json`` (auto-loaded if the new file is
  absent), and
* the old per-device shape ``{"bias": [...], "n":..., "ts":...}`` (gyro at the
  entry top level instead of under a ``"gyro"`` key).
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np

from .accel_calib import AccelCalibration

# Repo-root/.cache/imu_calib.json (.cache is gitignored). This file is
# sky/sensors/calib_store.py, so parents[2] is the repo root.
_CACHE_DIR = Path(__file__).resolve().parents[2] / ".cache"
_DEFAULT_PATH = _CACHE_DIR / "imu_calib.json"
_LEGACY_PATH = _CACHE_DIR / "imu_bias.json"


def default_path() -> Path:
    """Where the IMU calibration cache lives (repo ``.cache/imu_calib.json``)."""
    return _DEFAULT_PATH


def _load_all(path: Path) -> dict:
    """Load the whole cache dict, migrating from the legacy file if needed."""
    for p in (path, _LEGACY_PATH if path == _DEFAULT_PATH else None):
        if p is None:
            continue
        try:
            with open(p, "r") as fh:
                data = json.load(fh)
            if isinstance(data, dict):
                return data
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            continue
    return {}


def _save_all(path: Path, data: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w") as fh:
        json.dump(data, fh, indent=2)
    tmp.replace(path)        # atomic on POSIX -> never a half-written cache
    return path


def _entry(data: dict, device_id: str) -> dict:
    e = data.get(str(device_id))
    return e if isinstance(e, dict) else {}


# -- gyro bias ------------------------------------------------------------- #
def load_gyro_bias(device_id: str,
                   path: Path | None = None) -> np.ndarray | None:
    """Return the cached gyro bias (rad/s) or ``None`` if absent/invalid."""
    e = _entry(_load_all(path or _DEFAULT_PATH), device_id)
    gyro = e.get("gyro") if isinstance(e.get("gyro"), dict) else e  # legacy
    if not isinstance(gyro, dict) or "bias" not in gyro:
        return None
    b = np.asarray(gyro["bias"], dtype=np.float64)
    if b.shape != (3,) or not np.all(np.isfinite(b)):
        return None
    return b


def save_gyro_bias(device_id: str, bias: np.ndarray, n_samples: int,
                   path: Path | None = None,
                   temp_c: float | None = None) -> Path:
    """Persist the gyro bias for ``device_id`` (merges into the existing file)."""
    p = path or _DEFAULT_PATH
    data = _load_all(p)
    e = _entry(data, device_id)
    # Drop a legacy top-level "bias" so the entry is clean going forward.
    e.pop("bias", None)
    e.pop("n", None)
    e["gyro"] = {
        "bias": [float(x) for x in np.asarray(bias, dtype=np.float64)],
        "n": int(n_samples),
        "ts": time.time(),
        "temp_c": (None if temp_c is None else float(temp_c)),
    }
    data[str(device_id)] = e
    return _save_all(p, data)


# -- accel calibration ----------------------------------------------------- #
def load_accel_calib(device_id: str,
                     path: Path | None = None) -> AccelCalibration | None:
    """Return the cached :class:`AccelCalibration` or ``None`` if absent."""
    e = _entry(_load_all(path or _DEFAULT_PATH), device_id)
    acc = e.get("accel")
    if not isinstance(acc, dict) or "T" not in acc or "bias" not in acc:
        return None
    try:
        cal = AccelCalibration.from_dict(acc)
    except (KeyError, ValueError, TypeError):
        return None
    if not (np.all(np.isfinite(cal.T)) and np.all(np.isfinite(cal.bias))):
        return None
    return cal


def save_accel_calib(device_id: str, cal: AccelCalibration, n_poses: int,
                     path: Path | None = None) -> Path:
    """Persist the accel calibration for ``device_id`` (merges into the file)."""
    p = path or _DEFAULT_PATH
    data = _load_all(p)
    e = _entry(data, device_id)
    acc = cal.to_dict()
    acc["n_poses"] = int(n_poses)
    acc["ts"] = time.time()
    e["accel"] = acc
    data[str(device_id)] = e
    return _save_all(p, data)


# -- IMU->camera rotation (extrinsic) -------------------------------------- #
def load_imu_cam_rotation(device_id: str,
                          path: Path | None = None) -> np.ndarray | None:
    """Return the cached IMU->camera rotation ``R`` (3x3) or ``None`` if absent.

    This overrides the device EEPROM's ``getImuToCameraExtrinsics`` rotation when
    present -- the operator runs the pose wizard
    (:mod:`imu_camera.tools.imu_cam_calib`) when the factory extrinsic is wrong
    (e.g. the OAK-D Lite's BMI270 nominal value, which flips the startup attitude
    ~180deg). Validated as a finite proper rotation before use; anything else
    yields ``None`` so a corrupt entry falls back to the EEPROM value.
    """
    e = _entry(_load_all(path or _DEFAULT_PATH), device_id)
    ext = e.get("imu_cam")
    if not isinstance(ext, dict) or "R" not in ext:
        return None
    R = np.asarray(ext["R"], dtype=np.float64)
    if R.shape != (3, 3) or not np.all(np.isfinite(R)):
        return None
    # Must be a proper rotation (orthonormal, det +1) -- reject a garbage solve.
    if not (np.allclose(R @ R.T, np.eye(3), atol=1e-4)
            and np.isclose(np.linalg.det(R), 1.0, atol=1e-4)):
        return None
    return R


def save_imu_cam_rotation(device_id: str, R: np.ndarray, n_poses: int,
                          residual_deg: float | None = None,
                          path: Path | None = None) -> Path:
    """Persist the IMU->camera rotation for ``device_id`` (merges into the file)."""
    p = path or _DEFAULT_PATH
    data = _load_all(p)
    e = _entry(data, device_id)
    e["imu_cam"] = {
        "R": [[float(x) for x in row]
              for row in np.asarray(R, dtype=np.float64).reshape(3, 3)],
        "n_poses": int(n_poses),
        "residual_deg": (None if residual_deg is None else float(residual_deg)),
        "ts": time.time(),
    }
    data[str(device_id)] = e
    return _save_all(p, data)
