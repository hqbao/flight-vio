"""Persisted per-sensor VL53L1X calibration (crosstalk + range offset).

The bench routine (:mod:`lidar.tools.characterize` ``--calibrate``) solves THIS
rangefinder's own crosstalk + part-to-part range offset against a known target. This
module persists that solve so the live ``lidar`` process can auto-apply it on the
next ranging start instead of running uncalibrated -- exactly mirroring how
:mod:`imu_camera.device.camera_calib_store` persists the per-device stereo calib and
:mod:`sky.sensors.calib_store` persists the per-device IMU calib.

Hardware-free by construction (so a host / CI can import it)
------------------------------------------------------------
This module pulls in ONLY ``json`` + ``time`` + ``pathlib`` -- no ``smbus2``, no
device. So the cv2-free, smbus2-free dev host can SAVE a bench solve (keyed by the
abstract ``sensor_id``) and the live reader can LOAD it, with no I2C dependency
reaching this file.

On-disk shape -- one tiny JSON under the (gitignored) repo ``.cache`` dir, keyed by
sensor id so several rangefinders never clobber each other::

    {"<sensor_id>": {
        "xtalk": <int raw uint16>,      # the ULD CalibrateXtalk raw value (NOT scaled)
        "offset_mm": <int>,             # round(target_mm - mean_measured)
        "distance_mode": <int>,         # the mode the cal was taken in
        "min_mm": <int>,                # the gate floor in effect at cal time
        "timing_budget_us": <int>,      # the timing budget the cal was taken in
        "n": <int>,                     # frames averaged
        "ts": <float>                   # epoch seconds
    }}

``xtalk`` is the RAW uint16 :meth:`lidar.io.vl53l1x_reader.VL53L1XReader.calibrate_xtalk`
returns -- NOT pre-scaled. The reader scales it to the plane-offset register on apply
(``(xtalk_raw << 9) // 1000``), so the stored value stays device-formula-agnostic.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

# Repo-root/.cache/lidar_calib.json (.cache is gitignored). This file is
# lidar/io/lidar_calib_store.py, so parents[2] is the repo root (matching the camera
# store's parents[2] from imu_camera/device/camera_calib_store.py).
_CACHE_DIR = Path(__file__).resolve().parents[2] / ".cache"
_DEFAULT_PATH = _CACHE_DIR / "lidar_calib.json"

#: The integer fields every valid stored entry must carry (finite-check before use).
_INT_FIELDS = ("xtalk", "offset_mm", "distance_mode", "min_mm", "timing_budget_us", "n")

#: Magnitude bound on the persisted part-to-part range offset (mm). A real 4cm-4m
#: sensor's offset is at most a few hundred mm; a value past this is corrupt and would
#: (a) overflow the signed int16 ``offset_mm * 4`` pack done at apply time and (b) inject
#: a huge constant height bias. Reject the whole entry -> run uncalibrated (honest valid
#: readings are safe; the FC gates on ``valid``), never refuse to range.
_OFFSET_MM_ABS_MAX = 2000
#: Inclusive range on the persisted RAW xtalk uint16 (the ULD CalibrateXtalk value,
#: stored unscaled). Anything outside [0, 0xFFFF] could not have come from the bench
#: solve (which clamps to uint16) -> corrupt -> reject the entry.
_XTALK_MIN, _XTALK_MAX = 0, 0xFFFF


def default_path() -> Path:
    """Where the lidar calibration cache lives (repo ``.cache/lidar_calib.json``)."""
    return _DEFAULT_PATH


def _load_all(path: Path) -> dict:
    """Load the whole cache dict, or ``{}`` if absent/corrupt (never raises)."""
    try:
        with open(path, "r") as fh:
            data = json.load(fh)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return {}
    return data if isinstance(data, dict) else {}


def _save_all(path: Path, data: dict) -> Path:
    """Atomically write the whole cache dict (mirrors the camera store's ``_save_all``)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with open(tmp, "w") as fh:
        json.dump(data, fh, indent=2)
    tmp.replace(path)        # atomic on POSIX -> never a half-written cache
    return path


def _entry(data: dict, sensor_id: str) -> dict:
    e = data.get(str(sensor_id))
    return e if isinstance(e, dict) else {}


def save(sensor_id: str, *, xtalk: int, offset_mm: int, distance_mode: int,
         min_mm: int, timing_budget_us: int, n: int,
         path: Path | None = None) -> Path:
    """Persist this sensor's calibration (merges into the existing file).

    ``xtalk`` is the RAW uint16 from
    :meth:`lidar.io.vl53l1x_reader.VL53L1XReader.calibrate_xtalk` (stored unscaled);
    ``offset_mm`` is the signed offset from :meth:`...calibrate_offset`.
    """
    p = path or _DEFAULT_PATH
    data = _load_all(p)
    data[str(sensor_id)] = {
        "xtalk": int(xtalk),
        "offset_mm": int(offset_mm),
        "distance_mode": int(distance_mode),
        "min_mm": int(min_mm),
        "timing_budget_us": int(timing_budget_us),
        "n": int(n),
        "ts": time.time(),
    }
    return _save_all(p, data)


def load(sensor_id: str, path: Path | None = None) -> dict | None:
    """Return the saved calibration dict for ``sensor_id`` or ``None``.

    Returns ``None`` when there is no entry for this sensor, or the file is
    absent/corrupt, or the stored entry is missing/non-integer/non-finite on any
    required field, OR a magnitude-checked field is out of range -- NEVER raises, so a
    missing or damaged cache is a clean "run uncalibrated", not a crash on the live
    ranging path. The returned dict carries the integer fields (``xtalk``,
    ``offset_mm``, ...) ready for the reader to apply.

    MAGNITUDE validation (not just type): ``offset_mm`` must be within
    +/-``_OFFSET_MM_ABS_MAX`` (a corrupt large offset would overflow the int16 mm x 4
    pack and inject a huge constant height bias) and ``xtalk`` must be a uint16
    (``[0, 0xFFFF]``). Any failure rejects the WHOLE entry -> the caller runs
    uncalibrated + warns, honoring the never-refuse-to-range contract.
    """
    e = _entry(_load_all(path or _DEFAULT_PATH), sensor_id)
    if not e:
        return None
    out: dict = {}
    for k in _INT_FIELDS:
        v = e.get(k)
        # bool is an int subclass but never a valid calibration value -> reject it.
        if not isinstance(v, int) or isinstance(v, bool):
            return None
        out[k] = v
    # Magnitude / range guards: reject corrupt values that pass the type check but
    # would inject a huge height bias (offset) or wrap the plane-offset reg (xtalk).
    if abs(out["offset_mm"]) > _OFFSET_MM_ABS_MAX:
        return None
    if not (_XTALK_MIN <= out["xtalk"] <= _XTALK_MAX):
        return None
    return out
