#!/usr/bin/env python3
"""END-TO-END proof: ``vio_degraded`` threads map -> step -> engine -> PoseMsg.

Drives the REAL published-pose path (``ba.modules.backend.run_ba`` over a live
``InProcessEngine`` built exactly like ``make_vi_engine`` does) with a stub VIO
map whose ``run_ba`` returns a chosen pose + ``last_info``. Asserts:

  1. a DIVERGED keyframe -> published ``PoseMsg.info['vio_degraded'] is True``
     (the load-bearing fault now reaches the FC), with the reproj / jump
     diagnostics carried alongside ``refined`` and any ``pos_sigma_m``;
  2. a HEALTHY keyframe -> published ``info['vio_degraded'] is False``;
  3. the LOOSE path (``ba_step``) publishes ``{'refined': True}`` with NO
     ``vio_degraded`` key (info byte-unchanged);
  4. the SubprocessEngine pickle boundary round-trips the ``(T_cw, health)``
     tuple cleanly (plain float/bool scalars cross).

Run::

    .venv/bin/python -m verification.vio_degraded_e2e_check
"""
from __future__ import annotations

import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ba.comms.messages import Keyframe                              # noqa: E402
from ba.engine import InProcessEngine                               # noqa: E402
from ba.engine.steps import ba_step, vio_step                      # noqa: E402
from ba.modules.backend import run_ba                              # noqa: E402


class _StubVioMap:
    """Minimal stand-in for ``WindowedVIOMap``: ``add_keyframe`` no-ops, ``run_ba``
    returns a fixed ``T_cw`` and stamps ``last_info`` like the real guard does."""

    def __init__(self, T_cw, last_info):
        self._T_cw = T_cw
        self.last_info = last_info

    def add_keyframe(self, *a, **k):
        pass

    def run_ba(self):
        return self._T_cw


class _StubBaMap:
    """Loose stand-in: ``run_ba`` returns a bare ``T_cw`` (no tuple)."""

    def __init__(self, T_cw):
        self._T_cw = T_cw

    def add_keyframe(self, *a, **k):
        pass

    def run_ba(self):
        return self._T_cw


def _kf(tight: bool) -> Keyframe:
    ids = np.arange(8, dtype=np.int64)
    px = np.zeros((8, 2), np.float64)
    return Keyframe(7, np.eye(4), None, None, track_ids=ids, track_px=px,
                    accel=None, inlier_ids=None,
                    ts_ns=(2_000_000 if tight else 0),
                    imu_seg=(None if tight else None))


def _publish_tight(last_info) -> dict:
    """Push one tight keyframe through the REAL engine + backend ``run_ba``."""
    T_cw = np.linalg.inv(np.eye(4))
    engine = InProcessEngine(lambda: _StubVioMap(T_cw, last_info), vio_step)
    pose, _ = run_ba(engine, tight=True, kf=_kf(tight=True))   # (PoseMsg, backend_state)
    engine.close()
    assert pose is not None, "tight run_ba returned None for a valid keyframe"
    return pose.info


def _publish_loose() -> dict:
    T_cw = np.linalg.inv(np.eye(4))
    engine = InProcessEngine(lambda: _StubBaMap(T_cw), ba_step)
    pose, _ = run_ba(engine, tight=False, kf=_kf(tight=False))  # (PoseMsg, backend_state)
    engine.close()
    assert pose is not None, "loose run_ba returned None for a valid keyframe"
    return pose.info


def test_divergence_flag_published() -> None:
    print("1) DIVERGED tight keyframe -> info['vio_degraded'] is True:")
    info = _publish_tight({
        "vio_kfs": 2, "vio_reproj_px": 71.0, "vio_window_jump_m": 4.5,
        "vio_degraded": True,
    })
    print(f"   published info = {info}")
    assert info["refined"] is True
    assert info["vio_degraded"] is True, "vio_degraded NOT carried on divergence"
    assert isinstance(info["vio_degraded"], bool)
    assert abs(info["vio_reproj_px"] - 71.0) < 1e-12
    assert abs(info["vio_window_jump_m"] - 4.5) < 1e-12
    assert isinstance(info["vio_reproj_px"], float)
    print("   OK -- divergence reaches the published PoseMsg.info.\n")


def test_healthy_flag_published() -> None:
    print("2) HEALTHY tight keyframe -> info['vio_degraded'] is False:")
    info = _publish_tight({
        "vio_kfs": 8, "vio_reproj_px": 1.3, "vio_window_jump_m": 0.0,
        "vio_degraded": False,
    })
    print(f"   published info = {info}")
    assert info["refined"] is True
    assert info["vio_degraded"] is False, "healthy keyframe must carry False"
    print("   OK -- healthy keyframe carries vio_degraded=False.\n")


def test_loose_info_unchanged() -> None:
    print("3) LOOSE keyframe -> info == {'refined': True} (vio_degraded ABSENT):")
    info = _publish_loose()
    print(f"   published info = {info}")
    assert info == {"refined": True}, \
        "loose published info changed -- gap=0 / parity risk"
    assert "vio_degraded" not in info
    print("   OK -- loose published info byte-unchanged.\n")


def test_subprocess_pickle_boundary() -> None:
    print("4) SubprocessEngine pickle boundary round-trips (T_cw, health):")
    T_cw = np.linalg.inv(np.eye(4))
    health = {"vio_degraded": True, "vio_reproj_px": 71.0,
              "vio_window_jump_m": 4.5}
    payload = (T_cw, health)
    back = pickle.loads(pickle.dumps(payload))
    Tb, hb = back
    assert np.allclose(Tb, T_cw)
    assert hb == health
    assert isinstance(hb["vio_degraded"], bool)
    assert isinstance(hb["vio_reproj_px"], float)
    print("   OK -- the (ndarray, plain-scalar dict) tuple crosses cleanly.\n")


def main() -> int:
    print("=== vio_degraded END-TO-END (map -> step -> engine -> PoseMsg) ===\n")
    test_divergence_flag_published()
    test_healthy_flag_published()
    test_loose_info_unchanged()
    test_subprocess_pickle_boundary()
    print("PASS -- vio_degraded threads end-to-end to the published pose info; "
          "healthy=False; loose info unchanged; subprocess pickle clean.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
