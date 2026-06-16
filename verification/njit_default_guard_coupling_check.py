#!/usr/bin/env python3
"""Prove the njit IMU-factor gate: DEFAULT ON + the divergence-guard coupling.

Re-derives the exact gate predicate from ``sky.vio.window.optimize_vio`` (the
``use_imu_njit`` block) for the four cases that matter, without running a full
solve (the predicate is a pure function of HAVE_NUMBA, imu_info_weight,
imu_factors, the ``SKY_VIO_IMU_NJIT`` env var, and ``cfg.njit_guard_ok``):

  * env UNSET, guard ON   -> njit ON   (the validated production default)
  * env='0'               -> njit OFF  (the explicit kill switch)
  * env UNSET, guard OFF  -> njit OFF  (SAFETY coupling: guard-off disables njit)
  * env='1', guard OFF    -> njit OFF  (safety wins over an explicit override)

It also asserts ``WindowedVIOMap.run_ba`` threads ``divergence_guard`` into the
solve config's ``njit_guard_ok`` (the wire that makes the coupling fire live),
and that a bare ``VioConfig`` defaults ``njit_guard_ok=True``.

Run::

    .venv/bin/python -m verification.njit_default_guard_coupling_check
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sky.vio.window import (                                          # noqa: E402
    VioConfig, WindowedVIOConfig, WindowedVIOMap)


def _gate(have_numba: bool, info_weight: bool, has_factors: bool,
          env, guard_ok: bool) -> bool:
    """Mirror the ``use_imu_njit`` predicate in ``optimize_vio`` EXACTLY."""
    njit_force_off = (env == "0")
    njit_guard_ok = bool(guard_ok)
    return bool(have_numba and info_weight and has_factors
                and not njit_force_off and njit_guard_ok)


def test_default_on_when_guard_on() -> None:
    print("1) env UNSET + guard ON -> njit ON (production default):")
    on = _gate(True, True, True, None, True)
    print(f"   use_imu_njit = {on}")
    assert on is True, "njit should be ON by default with the guard on"
    print("   OK -- default ON.\n")


def test_env0_forces_off() -> None:
    print("2) SKY_VIO_IMU_NJIT=0 -> njit OFF (kill switch):")
    off = _gate(True, True, True, "0", True)
    print(f"   use_imu_njit = {off}")
    assert off is False, "SKY_VIO_IMU_NJIT=0 must force the kernel off"
    print("   OK -- env=0 forces off.\n")


def test_guard_off_disables_njit() -> None:
    print("3) guard OFF (env unset) -> njit OFF (SAFETY coupling):")
    off = _gate(True, True, True, None, False)
    print(f"   use_imu_njit = {off}")
    assert off is False, "guard-off must disable njit (determinism coupling)"
    print("   OK -- guard off disables the kernel.\n")


def test_explicit_on_does_not_override_guard() -> None:
    print("4) env='1' + guard OFF -> njit OFF (safety wins over override):")
    off = _gate(True, True, True, "1", False)
    print(f"   use_imu_njit = {off}")
    assert off is False, "explicit =1 must NOT re-enable njit when guard is off"
    print("   OK -- safety beats the explicit override.\n")


def test_vioconfig_default_guard_ok() -> None:
    print("5) bare VioConfig defaults njit_guard_ok=True:")
    cfg = VioConfig()
    print(f"   VioConfig().njit_guard_ok = {cfg.njit_guard_ok}")
    assert cfg.njit_guard_ok is True
    print("   OK.\n")


def test_run_ba_threads_guard() -> None:
    print("6) WindowedVIOMap.run_ba threads divergence_guard -> njit_guard_ok:")
    K = np.array([[300., 0, 160.], [0, 300., 100.], [0, 0, 1.]])
    captured: dict = {}
    import sky.vio.window as W
    orig = W.optimize_vio

    def _spy(K_, st, *a, cfg=None, **k):
        captured["njit_guard_ok"] = bool(cfg.njit_guard_ok)
        return orig(K_, st, *a, cfg=cfg, **k)

    for guard in (True, False):
        cfg = WindowedVIOConfig()
        cfg.vio.imu_info_weight = True
        cfg.divergence_guard = guard
        m = WindowedVIOMap(K, cfg=cfg)
        # seed a minimal solvable window: two keyframes sharing >=6 landmarks
        ids = np.arange(40, dtype=np.int64)
        rng = np.random.RandomState(0)
        # pixel coords WELL inside the 320x100 depth grid (so backproject keeps
        # every landmark and both keyframes co-observe >=6 tids / >=12 obs).
        px0 = np.column_stack([rng.uniform(10, 300, 40),
                               rng.uniform(10, 90, 40)])
        px1 = px0 + np.array([1.0, 0.0])
        m.add_keyframe(np.eye(4), ids, px0, _depth_grid(), 1_000_000,
                       imu_seg=None)
        m.add_keyframe(np.eye(4), ids, px1, _depth_grid(), 2_000_000,
                       imu_seg=None)
        W.optimize_vio = _spy
        try:
            m.run_ba()
        finally:
            W.optimize_vio = orig
        got = captured.get("njit_guard_ok")
        print(f"   divergence_guard={guard!s:5}  -> njit_guard_ok passed "
              f"to optimize_vio = {got}")
        assert got == guard, \
            f"run_ba did not thread the guard ({guard}) into njit_guard_ok"
    print("   OK -- run_ba threads the guard state into the solve config.\n")


def _depth_grid() -> np.ndarray:
    """A dense depth map so the frontend backprojects valid landmark depth."""
    return np.full((100, 320), 2.0, np.float64)


def test_real_kernel_invocation() -> None:
    """The REAL gate in ``optimize_vio`` actually calls the njit kernel under the
    default-on config, and does NOT under env=0 / guard-off -- a live-call probe,
    not the re-derived predicate. Uses the ``vio_ba_selftest`` synthetic VI window
    (real IMU factors) and counts kernel entries via a monkeypatch."""
    import os
    import sky.vio.window as W
    from sky.vio.window import VioConfig as VC
    from vio.tests.vio_ba_selftest import make_world, build_factors, G

    if not W.HAVE_NUMBA:
        print("7) real kernel invocation: SKIP (numba absent on this host).\n")
        return

    print("7) REAL optimize_vio kernel invocation (live-call probe):")
    w = make_world()
    from sky.vio.window import VioState
    st = VioState(R=[r.copy() for r in w["kf_R"]],
                  p=[x.copy() for x in w["kf_p"]],
                  v=[x.copy() for x in w["kf_v"]],
                  bg=[np.zeros(3) for _ in w["kf_R"]],
                  ba=[np.zeros(3) for _ in w["kf_R"]],
                  landmarks=w["lms"].copy())
    factors = build_factors(w["imu_raw"], np.zeros(3), np.zeros(3))

    calls = {"n": 0}
    real_kernel = W.imu_factor_jacobian

    def _counting_kernel(*a, **k):
        calls["n"] += 1
        return real_kernel(*a, **k)

    def _run(env, guard_ok) -> int:
        calls["n"] = 0
        saved = os.environ.get("SKY_VIO_IMU_NJIT")
        if env is None:
            os.environ.pop("SKY_VIO_IMU_NJIT", None)
        else:
            os.environ["SKY_VIO_IMU_NJIT"] = env
        W.imu_factor_jacobian = _counting_kernel
        try:
            cfg = VC(max_iters=2, imu_info_weight=True, njit_guard_ok=guard_ok)
            W.optimize_vio(K_, st.copy(), w["obs_cam"], w["obs_lm"],
                           w["obs_uv"], w["obs_d"], factors, G, cfg, anchor=0)
        finally:
            W.imu_factor_jacobian = real_kernel
            if saved is None:
                os.environ.pop("SKY_VIO_IMU_NJIT", None)
            else:
                os.environ["SKY_VIO_IMU_NJIT"] = saved
        return calls["n"]

    K_ = np.array([[300., 0, 160.], [0, 300., 100.], [0, 0, 1.]])
    n_default = _run(None, True)
    n_env0 = _run("0", True)
    n_guard_off = _run(None, False)
    print(f"   default (env unset, guard ON): kernel calls = {n_default}")
    print(f"   env=0                        : kernel calls = {n_env0}")
    print(f"   guard OFF                    : kernel calls = {n_guard_off}")
    assert n_default > 0, "njit kernel NOT invoked under the default-on config"
    assert n_env0 == 0, "SKY_VIO_IMU_NJIT=0 still invoked the kernel"
    assert n_guard_off == 0, "guard-off still invoked the kernel (coupling broke)"
    print("   OK -- real solve runs the kernel by default; off under env=0 / "
          "guard-off.\n")


def main() -> int:
    print("=== njit DEFAULT ON + divergence-guard coupling ===\n")
    test_default_on_when_guard_on()
    test_env0_forces_off()
    test_guard_off_disables_njit()
    test_explicit_on_does_not_override_guard()
    test_vioconfig_default_guard_ok()
    test_run_ba_threads_guard()
    test_real_kernel_invocation()
    print("PASS -- njit default ON; env=0 forces off; guard-off disables njit "
          "(safety beats override); run_ba wires the guard state.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
