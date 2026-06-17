#!/usr/bin/env python3
"""KEY self-test for the TIGHT live-pose IMU forward-propagation + ZUPT.

This pins the exact behaviour the user asked for: when vision is ABSENT (covered
camera) or too weak to solve WHILE MOVING, the live ``pose.odom`` on the
``--tight`` path must KEEP MOVING via the IMU (dead-reckon) instead of freezing;
and when the device is STATIONARY a Zero-Velocity Update (ZUPT) must hold the pose
still (no drift) -- preserving the static-drift win.

It drives :class:`vio.modules.propagate_imu.PropagateImu` exactly as the live
:class:`~vio.modules.pipeline.OdometryModule` frame-chain drives it (a ctx with
``retain_imu`` on, per-frame retained IMU segments keyed by seq, a frozen vision
pose to simulate vision dropout), and asserts the published live pose with hard
numeric thresholds. It also unit-checks the underlying primitives
:func:`sky.vio.imu.predict_state` (against a hand-integrated trajectory)
and :func:`sky.vio.imu.imu_at_rest`.

Run::

    .venv/bin/python -m vio.tests.imu_propagate_selftest
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from vio.comms.module import ModuleContext                       # noqa: E402
from vio.comms import LocalPubSub                                # noqa: E402
from vio.modules.propagate_imu import propagate_imu             # noqa: E402
from sky.vio.imu import imu_at_rest, predict_state      # noqa: E402

G = 9.81
G_WORLD = np.array([0.0, G, 0.0])     # optical-world "down" = +y
DT_NS = 5_000_000                     # 5 ms IMU step (200 Hz)


# --------------------------------------------------------------------------- #
# Minimal Step carrier stand-in (matches vio.modules.carriers.Step's fields used).
# --------------------------------------------------------------------------- #
@dataclass
class _Frame:
    seq: int
    ts_ns: int
    gray_left: object = None
    depth_m: object = None


@dataclass
class _Step:
    frame: _Frame
    pose: np.ndarray
    info: dict
    accel_cam: object = None
    at_rest: bool = False


def _make_ctx(kf_every: int = 10_000) -> ModuleContext:
    """A tight-path ctx: retain_imu on, large kf_every so no re-anchor fires."""
    ctx = ModuleContext(LocalPubSub(), "odometry")
    ctx.state["retain_imu"] = True
    ctx.state["kf_every"] = int(kf_every)
    ctx.state["g_world"] = (0.0, G, 0.0)
    ctx.state["imu_segs"] = {}
    return ctx


def _accel_seg(seq: int, accel_body: np.ndarray, gyro_body: np.ndarray,
               n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """A constant-(accel,gyro) IMU block of ``n`` samples for frame ``seq``.

    ``accel_body`` is the SPECIFIC FORCE the sensor reports (so an at-rest sensor
    reads +g upward = optical -y). ``gyro_body`` is the angular rate. The block's
    timestamps are contiguous with the previous frame's via ``seq``.
    """
    t0 = seq * (n - 1) * DT_NS
    ts = np.array([t0 + k * DT_NS for k in range(n)], np.int64)
    gyro = np.tile(np.asarray(gyro_body, float), (n, 1))
    accel = np.tile(np.asarray(accel_body, float), (n, 1))
    return ts, gyro, accel


# --------------------------------------------------------------------------- #
def test_predict_state_dead_reckons_translation() -> None:
    """Primitive check: a trapezoidal accel pulse integrates to a known forward
    displacement with the velocity returning to ~0 (classic dead-reckoning)."""
    # Forward = optical +z. At rest the sensor reads +g up (-y). A forward push
    # of +A m/s^2 along +z adds +A to the z specific force.
    A = 4.0          # m/s^2 push (clearly above any ZUPT band)
    rest = np.array([0.0, -G, 0.0])     # specific force at rest (up = -y)
    R = np.eye(3); p = np.zeros(3); v = np.zeros(3)
    bg = np.zeros(3); ba = np.zeros(3)

    # phase 1: accelerate +z for 0.5 s, phase 2: decelerate -z for 0.5 s.
    half = int(0.5 / (DT_NS * 1e-9))
    for phase, sgn in ((0, +1.0), (1, -1.0)):
        accel = rest + np.array([0.0, 0.0, sgn * A])
        ts = np.array([(phase * half + k) * DT_NS for k in range(half + 1)],
                      np.int64)
        gyro = np.zeros((half + 1, 3))
        accel_arr = np.tile(accel, (half + 1, 1))
        R, p, v = predict_state(R, p, v, ts, gyro, accel_arr, bg, ba, G_WORLD)

    # Analytic forward kinematics for a +A pulse 0..0.5 s then -A 0.5..1 s:
    #   x(0.5) = 0.5*A*0.25 = 0.5 m,  v(0.5) = A*0.5 = 2 m/s
    #   x(1.0) = 0.5 + 2*0.5 - 0.5*A*0.25 = 0.5 + 1.0 - 0.5 = 1.0 m,  v(1.0) = 0
    z = float(p[2]); vz = float(v[2])
    print(f"predict_state trapezoid: z={z:+.3f} m (expect ~1.00)  "
          f"vz={vz:+.3f} m/s (expect ~0.0)")
    assert abs(z - 1.0) < 0.05, f"forward displacement off: {z}"
    assert abs(vz) < 0.02, f"velocity did not return to rest: {vz}"
    # No spurious drift on the gravity axes (push was purely along +z).
    assert abs(p[0]) < 1e-6 and abs(p[1]) < 1e-6, f"lateral/vert drift {p}"
    print("  OK -- predict_state integrates a known pulse to the right place")


def test_imu_at_rest_gate() -> None:
    rest = np.tile(np.array([0.0, -G, 0.0]), (5, 1))
    # Forward push of 4 m/s^2 along +z: |accel| = sqrt(g^2+16) -> dev 0.78 > 0.3.
    moving = np.tile(np.array([0.0, -G, 4.0]), (5, 1))
    spin = np.tile(np.array([0.0, -G, 0.0]), (5, 1))
    # A true-rest accel bias must STILL read at rest (it is inside the band).
    rest_biased = np.tile(np.array([0.08, -G, 0.08]), (5, 1))
    gyro0 = np.zeros((5, 3))
    gyro_fast = np.tile(np.array([0.0, 1.0, 0.0]), (5, 1))  # 1 rad/s yaw
    assert imu_at_rest(gyro0, rest), "at-rest IMU not detected as rest"
    assert imu_at_rest(gyro0, rest_biased), "at-rest+bias not detected as rest"
    assert not imu_at_rest(gyro0, moving), "linear push falsely called rest"
    assert not imu_at_rest(gyro_fast, spin), "fast yaw falsely called rest"
    print("imu_at_rest: rest=True, rest+bias=True, push=False, spin=False  OK")


# --------------------------------------------------------------------------- #
def test_covered_camera_dead_reckons() -> None:
    """THE KEY TEST: vision ABSENT (frozen pose) + real IMU translation ->
    the live published pose.odom MUST advance via the IMU (not freeze)."""
    ctx = _make_ctx()
    step_obj = propagate_imu
    # Frozen vision pose: PnP failed every frame, so step.pose never changes (the
    # loose-path freeze symptom). The IMU, however, carries a real forward push
    # of 4 m/s^2 -- clearly above the ZUPT band so the live pose dead-reckons.
    # The failed solve is signalled via step.info (ok=False) exactly as
    # EstimateMotion does on a covered camera, so PropagateImu skips the vision
    # correction and pure-dead-reckons (does not pull back toward the stale pose).
    frozen = np.eye(4)
    failed_vis = {"ok": False, "n_inliers": 0}

    A = 4.0
    rest = np.array([0.0, -G, 0.0])
    # 20 frames, 5 IMU samples each at 200 Hz -> 0.02 s per frame, 0.4 s total.
    # First 10 frames accelerate +z, next 10 decelerate (net forward motion).
    n_per = 5
    poses = []
    dr_flags = []
    seq = 0
    # frame 0 establishes the anchor (first frame just seeds the nav-state).
    for fi in range(22):
        sgn = +1.0 if fi < 11 else -1.0
        accel = rest + np.array([0.0, 0.0, sgn * A])
        seg = _accel_seg(seq, accel, np.zeros(3), n_per)
        ctx.state["imu_segs"][seq] = seg
        st = _Step(_Frame(seq, int(seg[0][-1])), frozen.copy(), dict(failed_vis))
        out = step_obj(ctx, st)
        poses.append(out.pose[:3, 3].copy())
        # The TIGHT-only DR flag the UI reads: vision was lost every frame here,
        # so the live pose is inertial dead-reckoning -> inertial_dr must be True.
        dr_flags.append(bool(out.info.get("inertial_dr")))
        seq += 1

    # inertial_dr stamped True on every vision-lost frame (the amber-badge
    # condition). Frame 0 only seeds the anchor; assert from frame 1 on.
    assert all(dr_flags[1:]), \
        f"inertial_dr not set on covered/vision-lost frames: {dr_flags}"

    pos = np.array(poses)
    # The published camera->world position is the nav-state position (body==cam).
    fwd = float(pos[-1, 2] - pos[0, 2])
    total_path = float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)))
    print(f"covered-camera dead-reckon: net forward z = {fwd:+.3f} m, "
          f"path = {total_path:.3f} m  (vision was FROZEN at 0)")
    # Hard gate: the live pose ADVANCED forward by a sane dead-reckoned amount,
    # i.e. it did NOT freeze. Net displacement is positive and non-trivial.
    assert fwd > 0.1, f"live pose did NOT advance via IMU (froze): fwd={fwd}"
    assert total_path > 0.1, f"live pose path is ~0 (froze): {total_path}"
    assert np.all(np.isfinite(pos)), "non-finite dead-reckoned pose"
    print("  OK -- covered camera + motion: live pose dead-reckons via IMU")


def test_inertial_dr_flag_vision_ok() -> None:
    """The TIGHT-only ``inertial_dr`` flag is FALSE when this frame's vision
    solve is trusted (ok + enough inliers) -> the UI shows OK, not the amber
    inertial-DR badge. Mirrors the covered-camera test but with a GOOD solve."""
    ctx = _make_ctx()
    step_obj = propagate_imu
    rest = np.array([0.0, -G, 0.0])
    n_per = 5
    # A trusted vision solve every frame: ok=True with inliers above the
    # _MIN_VIS_INLIERS gate -> vis_ok True -> inertial_dr False.
    good_vis = {"ok": True, "n_inliers": 50}
    seq = 0
    flags = []
    for _ in range(6):
        seg = _accel_seg(seq, rest, np.zeros(3), n_per)
        ctx.state["imu_segs"][seq] = seg
        st = _Step(_Frame(seq, int(seg[0][-1])), np.eye(4), dict(good_vis))
        out = step_obj(ctx, st)
        flags.append(bool(out.info.get("inertial_dr")))
        seq += 1
    # Frame 0 only seeds the anchor; the flag is still written there too. With a
    # trusted solve every frame, NO frame should report inertial DR.
    assert not any(flags), f"inertial_dr falsely set on vision-OK frames: {flags}"
    print("inertial_dr flag: False on trusted-vision frames  OK")


def test_stationary_zupt_no_drift() -> None:
    """ZUPT: vision frozen + IMU at rest (accel~g, gyro~0) -> live pose holds
    still (no drift). With a small accel BIAS present, ZUPT must still hold it."""
    ctx = _make_ctx()
    step_obj = propagate_imu
    frozen = np.eye(4)
    rest = np.array([0.0, -G, 0.0])
    # Add a realistic accel bias so a pure forward-integrator would WALK off;
    # the ZUPT must reject it. (Bias is within the at-rest gravity band.)
    bias = np.array([0.05, 0.0, 0.05])     # ~0.05 m/s^2 on x,z (under the band)
    n_per = 5
    seq = 0
    poses = []
    for fi in range(60):                   # 60 frames at rest (~1.2 s)
        seg = _accel_seg(seq, rest + bias, np.zeros(3), n_per)
        ctx.state["imu_segs"][seq] = seg
        st = _Step(_Frame(seq, int(seg[0][-1])), frozen.copy(), {})
        out = step_obj(ctx, st)
        poses.append(out.pose[:3, 3].copy())
        seq += 1
    pos = np.array(poses)
    drift = float(np.linalg.norm(pos[-1] - pos[0]))
    max_excursion = float(np.max(np.linalg.norm(pos - pos[0], axis=1)))
    print(f"stationary ZUPT: net drift = {drift*1000:.3f} mm, "
          f"max excursion = {max_excursion*1000:.3f} mm over {len(pos)} frames")
    # Hard gate: at rest the pose does not walk off. Without ZUPT a 0.07 m/s^2
    # bias over 1.2 s would double-integrate to ~5 cm; ZUPT must keep it << 1 mm.
    assert drift < 1e-3, f"ZUPT failed: pose drifted {drift*1000:.2f} mm at rest"
    assert max_excursion < 1e-3, f"ZUPT excursion {max_excursion*1000:.2f} mm"
    print("  OK -- stationary: ZUPT holds the pose still (static-drift win kept)")


def test_static_camera_dynamic_object_no_drift() -> None:
    """A hand waved in front of a STATIC camera: IMU at rest (ZUPT) but vision
    reports a spurious MOVING pose (a dynamic object dominates PnP). With the ZUPT
    translation gate ON the live pose must NOT follow it; with the gate OFF (the
    old behaviour) it drifts after the object -- the contrast proves the fix."""
    import vio.modules.propagate_imu as _pim

    def _run(freeze: bool) -> float:
        _pim._ZUPT_FREEZE_TRANS = freeze
        ctx = _make_ctx()
        rest = np.array([0.0, -G, 0.0])
        good = {"ok": True, "n_inliers": 50, "track_overlap_ratio": 1.0}
        seq = 0
        # Phase 1: settle ZUPT with a STATIC vision pose (> _ZUPT_HOLD frames).
        for _ in range(8):
            seg = _accel_seg(seq, rest, np.zeros(3), 5)
            ctx.state["imu_segs"][seq] = seg
            _pim.propagate_imu(ctx, _Step(_Frame(seq, int(seg[0][-1])),
                                          np.eye(4), dict(good)))
            seq += 1
        # Phase 2: the object waves +/-30 cm in x (a spurious vision translation)
        # while the IMU stays at rest. Capture the published position excursion.
        poses = []
        for k in range(24):
            seg = _accel_seg(seq, rest, np.zeros(3), 5)
            ctx.state["imu_segs"][seq] = seg
            vis = np.eye(4)
            vis[0, 3] = 0.30 * np.sin(k * 0.5)
            out = _pim.propagate_imu(ctx, _Step(_Frame(seq, int(seg[0][-1])),
                                                vis, dict(good)))
            poses.append(out.pose[:3, 3].copy())
            seq += 1
        pos = np.array(poses)
        return float(np.max(np.linalg.norm(pos - pos[0], axis=1)))

    try:
        exc_on = _run(True)
        exc_off = _run(False)
    finally:
        _pim._ZUPT_FREEZE_TRANS = True       # restore default for the other tests
    print(f"static cam + dynamic object: max pose excursion  "
          f"gate ON = {exc_on*100:.2f} cm   gate OFF = {exc_off*100:.2f} cm")
    assert exc_on < 0.02, \
        f"ZUPT gate FAILED: static-cam pose followed the object {exc_on*100:.1f} cm"
    assert exc_off > 0.05, \
        f"contrast weak: gate-OFF should follow the object (got {exc_off*100:.1f} cm)"
    assert np.isfinite(exc_off), "gate-OFF excursion non-finite"
    print("  OK -- static camera + moving object: ZUPT holds position "
          "(vision-translation pull suppressed); gate-OFF drifts (contrast)")


def test_backend_bias_feedforward() -> None:
    """PLAN P2: the tight backend's optimised bias, fed on ``backend.state``, is
    adopted into the live dead-reckon bias via a HEALTH-GATED per-keyframe low-pass.
    Verifies: (1) a healthy, fresh, hysteresis-satisfied stream converges the live
    bias toward the backend's; (2) a DEGRADED stream is NOT adopted; (3) fewer than
    the hysteresis count is NOT adopted; (4) a STALE (older-seq) state is dropped."""
    import vio.modules.propagate_imu as _pim
    from vio.modules.loop_inbox import BackendStateInbox

    BA_BE = np.array([0.0, 0.0, 0.30])     # the backend's optimised accel bias
    rest = np.array([0.0, -G, 0.0])

    def _drive(states) -> np.ndarray:
        """Run one frame per entry in ``states`` (a backend.state dict or None),
        pushing it to the inbox; return the final live nav accel-bias."""
        ctx = _make_ctx()
        ctx.state["backend_inbox"] = binbox = BackendStateInbox()
        seq = 0
        for st in states:
            if st is not None:
                binbox.push(st)
            seg = _accel_seg(seq, rest, np.zeros(3), 5)
            ctx.state["imu_segs"][seq] = seg
            _pim.propagate_imu(ctx, _Step(_Frame(seq, int(seg[0][-1])),
                                          np.eye(4), {"ok": True, "n_inliers": 50}))
            seq += 1
        return ctx.state["live_nav"]["ba"].copy()

    def _msgs(n, ba=BA_BE, degraded=False, seq0=0):
        return [{"seq": seq0 + i, "bg": np.zeros(3), "ba": ba,
                 "degraded": degraded} for i in range(n)]

    # (1) healthy + fresh + hysteresis met -> bias converges toward the backend's.
    ba_fed = _drive(_msgs(12))
    assert ba_fed[2] > 0.20, f"healthy backend bias not adopted: ba={ba_fed}"
    assert np.linalg.norm(ba_fed - BA_BE) < 0.1, f"did not converge: {ba_fed}"

    # (2) DEGRADED -> never adopted (the safety gate): bias stays ~0.
    ba_deg = _drive(_msgs(12, degraded=True))
    assert np.linalg.norm(ba_deg) < 1e-9, f"degraded bias WAS adopted: {ba_deg}"

    # (3) hysteresis: fewer than _BACKEND_HEALTHY_HOLD healthy -> not yet adopted.
    ba_few = _drive(_msgs(_pim._BACKEND_HEALTHY_HOLD - 1))
    assert np.linalg.norm(ba_few) < 1e-9, f"adopted before hysteresis: {ba_few}"

    # (4) STALE: after adopting up to seq=20, an older seq=10 (5x bias) is dropped.
    ba_stale = _drive(_msgs(21) + _msgs(5, ba=BA_BE * 5.0, seq0=10))
    assert ba_stale[2] < 0.45, f"stale older-seq bias was adopted (should drop): {ba_stale}"

    # (5) sustained DEGRADE after adopting -> the held bias DECAYS toward the zero
    # seed (safety-reviewer FMEA: a frozen stale bias must not contaminate forever).
    ba_decay = _drive(_msgs(21) + _msgs(20, degraded=True, seq0=21))
    assert np.linalg.norm(ba_decay) < 0.5 * np.linalg.norm(BA_BE), \
        f"adopted bias did NOT decay on sustained degrade: {ba_decay}"

    print(f"backend bias feed-forward: healthy adopted (ba_z={ba_fed[2]:.3f}->{BA_BE[2]}), "
          "degraded gated, hysteresis honored, stale dropped, sustained-degrade decays  OK")


def test_empty_imu_segment_held() -> None:
    """Regression (caught live): a frame whose retained IMU segment is EMPTY
    (size-0 arrays, as PreintegratePrior stores for a no-sample packet) must NOT
    crash -- the nav pose is held and published, not indexed out of bounds."""
    ctx = _make_ctx()
    step_obj = propagate_imu
    frozen = np.eye(4)
    # frame 0 seeds the anchor with a normal segment.
    seg0 = _accel_seg(0, np.array([0.0, -G, 0.0]), np.zeros(3), 5)
    ctx.state["imu_segs"][0] = seg0
    step_obj(ctx, _Step(_Frame(0, int(seg0[0][-1])), frozen.copy(), {}))
    # frame 1 has an EMPTY segment (the live no-IMU packet shape).
    empty = (np.zeros(0, np.int64), np.zeros((0, 3)), np.zeros((0, 3)))
    ctx.state["imu_segs"][1] = empty
    out = step_obj(ctx, _Step(_Frame(1, 0), frozen.copy(), {}))
    assert np.all(np.isfinite(out.pose)), "empty-segment frame produced NaN/Inf"
    print("empty IMU segment: held without crashing  OK")


def test_loose_path_passthrough() -> None:
    """LOOSE path (retain_imu off): PropagateImu is a no-op -- step.pose is the
    untouched vision pose, so pose.odom byte-parity is unaffected."""
    ctx = ModuleContext(LocalPubSub(), "odometry")
    ctx.state["retain_imu"] = False
    ctx.state["kf_every"] = 5
    step_obj = propagate_imu
    vis = np.eye(4); vis[0, 3] = 1.234      # arbitrary vision pose
    st = _Step(_Frame(7, 999), vis.copy(), {"x": 1})
    out = step_obj(ctx, st)
    assert out is st, "loose path must pass the same Step object through"
    assert np.array_equal(out.pose, vis), "loose path must NOT touch step.pose"
    assert "live_nav" not in ctx.state, "loose path must not allocate nav-state"
    assert "is_kf_frame" not in ctx.state, "loose path must not stamp cadence"
    print("loose path: PropagateImu is a pure pass-through  OK")


def main() -> int:
    print("=== TIGHT live-pose IMU propagation + ZUPT self-test ===\n")
    test_predict_state_dead_reckons_translation()
    test_imu_at_rest_gate()
    print()
    test_covered_camera_dead_reckons()
    test_inertial_dr_flag_vision_ok()
    test_stationary_zupt_no_drift()
    test_static_camera_dynamic_object_no_drift()
    test_backend_bias_feedforward()
    print()
    test_empty_imu_segment_held()
    test_loose_path_passthrough()
    print("\nPASS -- live pose dead-reckons via IMU under vision dropout, "
          "ZUPT holds it still at rest, loose path untouched.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
