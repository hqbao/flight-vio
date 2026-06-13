"""``DirectOdometryEngine`` -- LIVE glue for dense direct RGB-D VO (``--direct``).

WHAT THIS IS
------------
The opt-in ``--direct`` odometry mode wires the offline-proven dense direct
RGB-D visual odometry (:func:`sky.front.direct.estimate_pose_direct`) into the
live VIO worker as a THIRD front-end (alongside the loose default and ``--tight``).
At the 54x42 VL53-class ToF target the sparse corner/KLT frontend suffers scale
collapse + feature starvation; the offline harness ``verification/direct_vo_bench``
proved that dense direct + the accurate per-pixel ToF depth + a full-6-DoF IMU
dead-reckon seed + a divergence guard beats the sparse VIO on ALL four gold
sessions (scale 0.41 -> 0.93, divergence killed). This class ports that EXACT
working loop to the live streaming setting.

It owns ALL per-keyframe state the bench's ``run_session_direct`` held in locals
(the current keyframe gray/depth + its world pose, the previous-frame relative
pose, the IMU dead-reckon nav-state) and exposes ONE method,
:meth:`process_frame`, called once per live depth frame. The heavy math
(``estimate_pose_direct``) lives in the LEAF ``sky.front.direct``; this class is
pure process glue (keyframe management + the IMU seed + the divergence guard), so
``sky`` stays a leaf and the live-only logic stays in ``vio/``.

WHY IT IS A SEPARATE FRONT-END (NOT a tweak of the sparse path)
--------------------------------------------------------------
Direct VO does not track corners or run PnP: it aligns EVERY gradient pixel by
photometric Gauss-Newton. So it replaces the whole sparse chain
(``track_features -> ... -> estimate_motion``) rather than slotting into it. It
still produces a per-frame world pose + a keyframe stream on the SAME topics
(``pose.odom`` / ``pose.vo`` / ``keyframe`` / ``frame.tracks`` / ``frame.inliers``),
so the UI + SLAM downstream are byte-for-byte unchanged at the IPC contract level
(comms untouched -- no new topic).

THE LIVE IMU SEED (reusing the live dead-reckon, NOT rebuilt)
------------------------------------------------------------
The bench's ``_ImuDeadReckoner`` reads the whole offline IMU stream up front and
slices it per frame. LIVE, the IMU arrives as the SAME per-frame ``imu_segs`` the
``--tight`` path already retains (``preintegrate_prior`` rotates each packet's raw
IMU into the camera optical frame and stores ``(ts, gyro_cam, accel_cam)`` keyed
by frame seq; ``--direct`` forces that retention on). So this engine consumes the
EXACT same live per-frame IMU block the tight nav-state consumes -- it does not
re-open the session or rebuild a parallel IMU pipeline. It maintains a nav-state
``(R, p, v)`` propagated with the SAME :func:`sky.vio.imu.predict_state` the live
tight path runs, gravity-levelled once with :func:`sky.vio.imu.gravity_aligned_R0`,
and pulled toward each accepted VO fix with :func:`sky.vio.imu.complementary_correct`
(the SAME complementary filter, same gains as the bench / the live tight path).

THE DIVERGENCE GUARD (ported verbatim from the bench)
-----------------------------------------------------
A frame's VO pose is REJECTED -- replaced by the IMU dead-reckon prediction, which
is ALSO what the dead-reckoner is then corrected toward (protecting the seed
velocity from a diverged fix) -- when EITHER the estimator flags ``diverged`` OR
the VO keyframe-relative translation exceeds ``vo_imu_ratio`` x the IMU-dead-
reckoned keyframe-relative translation (with an absolute floor so a near-static
frame cannot inflate the ratio). This is the bench's Stage-2b guard, the lever
that killed the quick_motion divergence.

DEFAULTS (per the brief): guard ON; point-to-plane geometric term OFF (the
ablation showed it is redundant at 54x42), available behind ``geo_weight`` if the
caller wants it.
"""
from __future__ import annotations

import logging

import numpy as np

from sky.front.direct import DirectConfig, estimate_pose_direct
from sky.math import se3_inv, se3_log
from sky.vio.imu import (
    complementary_correct, gravity_aligned_R0, predict_state)

LOG = logging.getLogger("vio.direct")

# Complementary-correction gains (bounded in [0, 1]) -- the EXACT values the live
# tight path (propagate_imu._K_POS/_K_VEL/_K_ROT) and the offline bench use:
# firmly vision-anchored (pos/att error half-life ~2.4 frames) with a deliberately
# small k_vel that only bleeds the phantom-drift velocity down. Reused verbatim.
_K_POS = 0.25
_K_VEL = 0.05
_K_ROT = 0.25


def _g_world_default() -> np.ndarray:
    """World gravity ACCELERATION vector (optical-world "down" = +y).

    The exact convention :func:`sky.vio.imu.predict_state` /
    :class:`sky.vio.window.WindowedVIOConfig` use, shared with the live tight path.
    """
    return np.array([0.0, 9.81, 0.0])


class DirectOdometryEngine:
    """Frame-to-keyframe dense direct RGB-D VO with an IMU seed + divergence guard.

    Live port of the bench's ``run_session_direct`` loop. Construct once (with the
    ToF-grid ``K`` from the calib bundle), then call :meth:`process_frame` per
    depth frame; it returns the world pose ``T_world_cur`` (camera->world 4x4) and
    a flag for whether the frame opened a new keyframe (so the worker can emit a
    keyframe on exactly that cadence -- a NATURAL keyframe scheme, not a fixed
    kf_every count).

    Keyframing matches the bench: a new keyframe is promoted when the keyframe-
    relative translation/rotation exceeds the thresholds, the alignment failed to
    converge, the overlap dropped, ``kf_max_gap`` frames elapsed, or the guard
    rejected the frame (a rejected frame breaks the keyframe -> re-anchor).
    """

    def __init__(self, K: np.ndarray, *,
                 cfg: DirectConfig | None = None,
                 kf_trans_m: float = 0.10,
                 kf_rot_deg: float = 6.0,
                 kf_max_gap: int = 12,
                 converged_overlap_min: float = 0.30,
                 vo_imu_ratio: float = 4.0,
                 vo_imu_floor_m: float = 0.03) -> None:
        self.K = np.asarray(K, dtype=np.float64)
        # geo OFF by default at 54x42 (the ablation showed it redundant); the
        # caller can pass a cfg with geo_weight>0 to turn the point-to-plane term
        # back on behind the same flag.
        self.cfg = cfg if cfg is not None else DirectConfig(geo_weight=0.0)
        self.kf_trans_m = float(kf_trans_m)
        self.kf_rot_deg = float(kf_rot_deg)
        self.kf_max_gap = int(kf_max_gap)
        self.converged_overlap_min = float(converged_overlap_min)
        self.vo_imu_ratio = float(vo_imu_ratio)
        self.vo_imu_floor_m = float(vo_imu_floor_m)

        # --- keyframe state (was run_session_direct's locals) ----------------
        self._gray_kf: np.ndarray | None = None      # current keyframe gray
        self._depth_kf: np.ndarray | None = None     # current keyframe depth (m)
        self._T_world_kf = np.eye(4)                 # keyframe camera->world pose
        self._T_prev_kf = np.eye(4)                  # previous frame -> keyframe (seed)
        self._frames_since_kf = 0                    # gap counter for kf_max_gap
        self._prev_ts: int | None = None

        # --- IMU dead-reckon nav-state (body == camera optical frame) --------
        # Anchored to the first frame in :meth:`process_frame`; from there it is
        # propagated with predict_state and corrected toward each accepted fix.
        self._R = np.eye(3)
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._bg = np.zeros(3)                        # gyro bias (seeded at frame 0)
        self._ba = np.zeros(3)                        # accel bias (left 0, as tight)
        self._g_world = _g_world_default()
        self._T_world_kf_dr = np.eye(4)              # IMU keyframe pose (for the seed)
        self._dr_last_ts: int | None = None          # last propagated ts
        self._dr_anchor_ts: int | None = None        # last VO-correction ts (k_vel dt)
        self._dr_ready = False                       # nav-state anchored yet?

        # --- diagnostics (read by the live pose-sanity smoke / logging) ------
        self.n_frames = 0
        self.n_converged = 0
        self.n_rejected = 0

    # ------------------------------------------------------------------ #
    # IMU dead-reckon helpers (live streaming; reuse sky.vio.imu verbatim)
    # ------------------------------------------------------------------ #
    def _anchor_imu(self, ts_ns: int, accel_cam0: np.ndarray | None,
                    bg0: np.ndarray | None) -> None:
        """Gravity-level the IMU nav-state at the first frame (bench ``init_at``).

        Pins the dead-reckoned origin to the camera-trajectory origin (identity)
        and levels roll/pitch from the startup accelerometer via
        :func:`gravity_aligned_R0`, so the dead-reckoned + photometric world frames
        share an origin + gravity. Velocity starts at zero (the sessions begin
        near-static). The gyro bias is seeded from the same near-static window the
        preintegrator measured (``bg0``); accel bias left at zero, as the live
        tight path also seeds ba0 = 0.
        """
        a0 = (np.asarray(accel_cam0, np.float64) if accel_cam0 is not None
              else np.array([0.0, -9.81, 0.0]))
        self._R = gravity_aligned_R0(a0)             # T_world_cam0 = identity here
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._bg = (np.asarray(bg0, np.float64) if bg0 is not None
                    else np.zeros(3))
        self._ba = np.zeros(3)
        self._dr_last_ts = int(ts_ns)
        self._dr_anchor_ts = int(ts_ns)
        # The keyframe IMU anchor is the PHOTOMETRIC world origin (identity), NOT
        # the gravity-aligned nav-pose. The dead-reckoner's gravity rotation lives
        # in ``self._R`` and is carried into the seed THROUGH the relative pose
        # ``init_T = inv(T_world_cur_dr) @ T_world_kf_dr`` -- baking the gravity
        # rotation into the anchor too would cancel it out of the seed (the seed's
        # rotation would collapse to identity), which silently rotates the chained
        # dead-reckon translations into the wrong frame and the world pose explodes.
        # This matches the offline bench (``run_session_direct`` seeds
        # ``T_world_kf_dr = np.eye(4)`` while ``dr.init_at`` levels only ``dr._R``).
        self._T_world_kf_dr = np.eye(4)
        self._dr_ready = True

    def _propagate(self, ts_ns: int, seg) -> None:
        """Forward-integrate the nav-state to ``ts_ns`` over this frame's IMU block.

        ``seg`` is the live ``(ts, gyro_cam, accel_cam)`` block this frame's packet
        carried (the SAME per-frame retention the tight path uses); we integrate it
        with the gravity-aware :func:`predict_state` -- the SAME predictState the
        live tight nav-state runs. Advances ``(R, p, v)`` and ``last_ts``.
        """
        if seg is None:
            self._dr_last_ts = int(ts_ns)
            return
        ts = np.asarray(seg[0], np.int64)
        gyro = np.asarray(seg[1], np.float64)
        accel = np.asarray(seg[2], np.float64)
        if ts.size >= 2:
            self._R, self._p, self._v = predict_state(
                self._R, self._p, self._v, ts, gyro, accel,
                self._bg, self._ba, self._g_world)
        self._dr_last_ts = int(ts_ns)

    def _correct_toward(self, T_world_cam_vis: np.ndarray, ts_ns: int) -> None:
        """Soft-pull the nav-state toward an accepted metric VO fix (bench rule).

        Closes a bounded fraction of the position/attitude error and bleeds the
        position error into velocity over the inter-correction interval, so the
        velocity that drives the NEXT seed stays scaled by depth-true vision. On a
        guard-rejected frame the caller passes the IMU-only pose here, so this is a
        self-correction that leaves the velocity IMU-driven (un-poisoned).
        """
        T = np.asarray(T_world_cam_vis, np.float64)
        dt_anchor = max((int(ts_ns) - int(self._dr_anchor_ts or ts_ns)) * 1e-9, 0.0)
        self._R, self._p, self._v = complementary_correct(
            self._R, self._p, self._v, T[:3, :3], T[:3, 3],
            dt_anchor, _K_POS, _K_VEL, _K_ROT)
        self._dr_anchor_ts = int(ts_ns)

    def _pose(self) -> np.ndarray:
        """Current 4x4 ``T_world_cam`` nav-pose."""
        out = np.eye(4)
        out[:3, :3] = self._R
        out[:3, 3] = self._p
        return out

    # ------------------------------------------------------------------ #
    # The per-frame entry point
    # ------------------------------------------------------------------ #
    def process_frame(self, gray: np.ndarray, depth: np.ndarray, ts_ns: int,
                      seg, accel_cam0: np.ndarray | None = None,
                      bg0: np.ndarray | None = None
                      ) -> tuple[np.ndarray, bool, dict]:
        """Run one live direct-VO frame; return ``(T_world_cur, is_kf, info)``.

        Parameters
        ----------
        gray, depth : the (already ToF-reduced, live) intensity + metric-depth
            grids for THIS frame, at the calib-bundle resolution (``K``).
        ts_ns : this frame's device timestamp (ns).
        seg : the live per-frame IMU block ``(ts, gyro_cam, accel_cam)`` (camera
            optical frame) retained by ``preintegrate_prior``, or None when the
            packet carried no IMU. Drives the dead-reckon seed.
        accel_cam0, bg0 : startup gravity-level accel (camera frame) + gyro bias
            for the FIRST frame's IMU anchor; ignored after the anchor is set.

        Returns the world pose ``T_world_cur`` (camera->world 4x4), ``is_kf`` (this
        frame opened a new keyframe -> the worker emits a keyframe), and an ``info``
        dict (``converged`` / ``diverged`` / ``rejected`` / ``valid_frac`` /
        ``n_pixels`` / ``inertial_dr`` + a sparse-compatible ``ok`` / ``n_inliers``
        so the downstream publishers + the UI badge behave as on the other modes).
        """
        gray = np.ascontiguousarray(gray)
        depth = np.ascontiguousarray(depth, dtype=np.float32)
        ts_ns = int(ts_ns)
        self.n_frames += 1

        # --- FIRST frame: this IS the first keyframe; anchor everything. -----
        if self._gray_kf is None:
            self._gray_kf = gray
            self._depth_kf = depth
            self._T_world_kf = np.eye(4)
            self._T_prev_kf = np.eye(4)
            self._frames_since_kf = 0
            self._prev_ts = ts_ns
            self._anchor_imu(ts_ns, accel_cam0, bg0)
            self.n_converged += 1
            info = {"ok": True, "n_inliers": self.cfg.max_pixels,
                    "converged": True, "diverged": False, "rejected": False,
                    "valid_frac": 1.0, "n_pixels": 0, "inertial_dr": False,
                    "inlier_ids": np.empty((0,), dtype=np.int64)}
            return self._T_world_kf.copy(), True, info

        # --- (1) IMU 6-DoF seed: dead-reckon to THIS frame ------------------ #
        # The seed is the relative pose keyframe->cur expressed as the direct
        # estimator's T_cur_kf (point ref-cam -> cur-cam):
        #   T_cur_kf = inv(T_world_cur_dr) @ T_world_kf_dr
        #
        # When the dead-reckoner is anchored (the session has IMU) we ALWAYS use
        # this IMU-relative seed -- even on a frame whose own IMU block is empty.
        # ``_propagate`` is a no-op with < 2 samples (predict_state returns the
        # nav-pose unchanged), so ``T_world_cur_dr`` is still the gravity-aligned
        # nav-pose and ``init_T`` still carries the GRAVITY rotation. This is what
        # the offline bench does (``dr.propagate_to`` is called every frame and is
        # a no-op with no samples). The earlier fall-through to an IDENTITY
        # ``_T_prev_kf`` seed on a no-IMU frame was the divergence bug: it dropped
        # the gravity rotation from the seed, so the resulting ``T_world_cur``
        # rotation was identity, and the every-frame complementary correction then
        # slerped the dead-reckoner's gravity attitude toward identity -- destroying
        # the gravity alignment and exploding the dead-reckon over the no-depth
        # startup frames. We only fall back to the previous-frame relative pose when
        # the dead-reckoner was NEVER anchored (a pure-vision session, no IMU at
        # all); then the guard has no IMU reference and is disabled, as before.
        T_world_cur_dr = None
        if self._dr_ready:
            self._propagate(ts_ns, seg)            # no-op when seg has < 2 samples
            T_world_cur_dr = self._pose()
            init_T = se3_inv(T_world_cur_dr) @ self._T_world_kf_dr
        else:
            init_T = self._T_prev_kf.copy()

        # --- (2) dense direct frame-to-KEYFRAME alignment ------------------- #
        T_cur_kf, est_info = estimate_pose_direct(
            self._gray_kf, self._depth_kf, gray, self.K,
            depth_cur=depth, init_T=init_T, cfg=self.cfg)
        converged = bool(est_info["converged"])
        if converged:
            self.n_converged += 1

        # --- (3) divergence GUARD (ported verbatim from the bench) ---------- #
        # Reject the VO pose + accept the IMU dead-reckon when the solve looks
        # divergent (the estimator's own ``diverged`` OR a VO step >> the IMU-
        # predicted step). The dead-reckoner is then corrected toward its OWN
        # prediction (no-op pull), NOT the diverged VO fix -- protecting the seed
        # velocity. Only possible when an IMU reference exists this frame.
        vo_imu_diverged = False
        if T_world_cur_dr is not None:
            vo_trans = float(np.linalg.norm(se3_inv(T_cur_kf)[:3, 3]))
            imu_trans = float(np.linalg.norm(se3_inv(init_T)[:3, 3]))
            imu_ref = max(imu_trans, self.vo_imu_floor_m)
            vo_imu_diverged = (vo_trans / imu_ref) > self.vo_imu_ratio
        guard_reject = (bool(est_info.get("diverged")) or vo_imu_diverged) \
            and (T_world_cur_dr is not None)

        if guard_reject:
            self.n_rejected += 1
            T_world_cur = T_world_cur_dr.copy()
            # Re-express the accepted IMU pose as T_cur_kf so the keyframe-
            # promotion test + next-frame seeding stay consistent.
            T_cur_kf = se3_inv(T_world_cur_dr) @ self._T_world_kf_dr
            vo_fix_for_dr = T_world_cur_dr        # protect velocity: feed IMU-only
        else:
            T_world_cur = self._T_world_kf @ se3_inv(T_cur_kf)
            vo_fix_for_dr = T_world_cur           # accept the metric VO fix

        # Pull the IMU nav-state toward the ACCEPTED fix (bench complementary
        # correction) so the velocity feeding the NEXT seed stays vision-scaled.
        if self._dr_ready:
            self._correct_toward(vo_fix_for_dr, ts_ns)

        # --- keyframe promotion test (bench rule) --------------------------- #
        self._frames_since_kf += 1
        xi = se3_log(T_cur_kf)
        trans = float(np.linalg.norm(xi[:3]))
        rot_deg = float(np.degrees(np.linalg.norm(xi[3:])))
        overlap = float(est_info["valid_frac"])
        promote = (
            trans >= self.kf_trans_m
            or rot_deg >= self.kf_rot_deg
            or self._frames_since_kf >= self.kf_max_gap
            or (not converged)
            or overlap < self.converged_overlap_min
            or guard_reject
        )
        if promote:
            # The new keyframe is THIS frame; anchor its world pose (photometric
            # AND the IMU nav-state's world pose, so the next seed measures
            # keyframe->cur from a fresh anchor).
            self._T_world_kf = T_world_cur.copy()
            if self._dr_ready:
                self._T_world_kf_dr = self._pose()
            self._gray_kf = gray
            self._depth_kf = depth
            self._frames_since_kf = 0
            self._T_prev_kf = np.eye(4)           # we are AT the keyframe now
        else:
            self._T_prev_kf = T_cur_kf            # seed next frame from this rel-pose
        self._prev_ts = ts_ns

        info = {
            "ok": converged and not guard_reject,
            # Map the dense-pixel health onto the sparse ``n_inliers`` field so the
            # downstream inlier publisher / UI tracking badge behave sensibly. A
            # rejected frame reports 0 (vision lost -> dead-reckoning).
            "n_inliers": (0 if guard_reject
                          else int(round(overlap * est_info["n_pixels"]))),
            "converged": converged,
            "diverged": bool(est_info.get("diverged")),
            "rejected": guard_reject,
            "valid_frac": overlap,
            "n_pixels": int(est_info["n_pixels"]),
            # AMBER "inertial DR" badge when the live pose is carried by the IMU
            # (the guard rejected this VO frame) -- same semantics as the tight
            # path's inertial_dr flag.
            "inertial_dr": guard_reject,
            "inlier_ids": np.empty((0,), dtype=np.int64),
        }
        return T_world_cur, promote, info
