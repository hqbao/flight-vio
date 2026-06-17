"""``propagate_imu`` step (TIGHT path only): IMU forward-propagate the live pose.

Driven directly by three selftests (``imu_propagate``, ``imu_push_response``,
``tight_live_regression``) as ``PropagateImu().run(ctx, step)``, so the public
surface keeps a single ``ctx`` arg (any object exposing ``.state``); the helper
functions below take the ``state`` dict explicitly. ``state`` IS ``ctx.state`` --
the worker's shared :class:`~vio.comms.module.ModuleContext` state dict (or the
selftest's), so all the per-frame nav-state lives there as before.

The gap this closes
-------------------
On the loose path the live displayed position is ``pose.odom``, published EVERY
frame by :func:`~vio.modules.publishers.publish_pose` from the per-frame
VISION-ONLY odometry (PnP). When vision is absent (covered camera) or too weak to
solve (white wall) the PnP either fails or freezes translation, so the live pose
FREEZES even while the device is physically moving -- the "covered camera + move =
stays still" symptom. Basalt does not freeze: it propagates the IMU every frame
(predictState) so the live pose keeps reacting to motion (and drifts) until vision
re-locks and pulls it back.

This step adds exactly that, but ONLY on the ``--tight`` path (gated on
``retain_imu`` -- the same flag that turns on per-frame IMU retention). It owns a
live body->world nav-state ``(R, p, v, bg, ba)`` plus the fixed world gravity, and
on EVERY frame the live pose DEAD-RECKONS CONTINUOUSLY from the IMU; vision is
applied only as a SMOOTH partial correction. The three pieces:

1. **Gap-free forward-propagation (instant, full-magnitude response).** The
   retained raw IMU block for this frame is integrated forward under gravity
   (:func:`sky.vio.imu.predict_state`). To avoid dropping the segment
   BETWEEN this block's first sample and the previous block's last sample (the
   per-frame packet cut ``(prev_ts, ts]`` shares no boundary sample, so a naive
   per-block integration silently loses ~1-of-N inter-sample segments -> the live
   pose only captures a FRACTION of the true displacement), the previous block's
   final sample is prepended to this block. The interval integrated is therefore
   exactly ``(prev_block_last_ts, this_block_last_ts]`` with no gap -- the full
   accel double-integral, so a fast push shows up at 100 %, not ~50 %.

2. **Velocity-gated ZUPT (no mid-motion pause).** A Zero-Velocity Update freezes
   translation ONLY when the IMU is GENUINELY at rest: accel ~ g AND gyro ~ 0
   (:func:`sky.vio.imu.imu_at_rest`) AND the live velocity estimate is
   small (``|v| < _ZUPT_VEL``), sustained for a few frames (hysteresis via
   ``zupt_run``). Accel+gyro ALONE cannot tell "at rest" from "cruising at
   constant velocity" -- both read ``|accel| ~ g``, ``|gyro| ~ 0`` -- so the old
   accel/gyro-only gate froze the pose mid-push during the constant-velocity
   cruise (the PAUSE). Adding the velocity gate keeps the IMU integrating through
   the cruise (``|v|`` is large -> no ZUPT), so the pose tracks the full motion;
   after the push the decel drives ``|v| -> 0`` and ZUPT re-engages -> no rest
   drift (the static-drift win is preserved).

3. **Smooth complementary vision correction (no snap / overshoot).** When a fresh
   vision fix is available (every keyframe), the live nav-state is nudged a
   BOUNDED FRACTION of the way toward it
   (:func:`sky.vio.imu.complementary_correct`) -- position, velocity, and
   attitude each get a small error-state feedback term -- instead of the old hard
   ``p = p_vis`` jump + ``v = displacement/dt`` velocity injection. The live pose
   dead-reckons continuously; vision pulls the accumulated drift back gradually
   over a few keyframes. No visible snap, no overshoot from a bad injected
   velocity, drift still fully corrected.

The propagated pose REPLACES ``step.pose`` so the downstream
:class:`PublishPose` emits the IMU-propagated pose on ``pose.odom`` -- the live
marker dead-reckons through any blind interval instead of freezing.

Closed-loop SLAM correction (LIVE + ``--tight`` only)
----------------------------------------------------
Basalt's realtime VIO has NO loop closure, so its live pose drifts unboundedly. We
do: the SLAM process runs a pose-graph that, on a revisit, rewrites the keyframe
poses (``loop.correction``). This step feeds that correction back into the LIVE
nav-state so the accumulated drift is BOUNDED on revisits ("closed loop"):

4. **Smooth loop-correction blend (no hard snap, no oscillation).** PropagateImu
   remembers, per keyframe seq, the PRE-correction body->world pose it published
   there (``kf_pose_pre`` ring). When a ``LoopCorrection`` arrives (handed in over
   a thread-safe inbox by ``vio.main``, LIVE-only), it picks a CONVERGED corrected
   keyframe it still has a pre-correction pose for, computes the world-frame SE(3)
   delta ``T_delta = T_corrected @ inv(T_pre)``
   (:func:`sky.vio.imu.loop_correction_delta`), and queues it as a PENDING
   correction. On each subsequent frame a BOUNDED FRACTION of the REMAINING delta
   is applied to the live pose (geodesic SE(3) interpolation,
   :func:`sky.vio.imu.scale_se3_delta` + :func:`apply_se3_left`), so the
   live trajectory is pulled smoothly back onto the loop-corrected one over a few
   frames -- NOT a one-shot teleport (we just removed a hard jump; do not
   reintroduce one). Between corrections the live pose dead-reckons as before.

   The keyframe targeted is NOT the most-recent one: SLAM re-confirms a loop EVERY
   keyframe while revisiting a seen area, emitting a STREAM of corrections, and the
   just-inserted keyframe's pose-graph pose is still settling (it jumps cm-scale
   solve-to-solve). Targeting the newest keyframe therefore chases a MOVING target
   whose remainder flips sign at the keyframe cadence -- a sawtooth the bounded
   gain cannot damp (the "loop-correction teleport/oscillation" symptom). The blend
   instead (a) LOCKS onto a CONVERGED older keyframe (at least ``_LOOP_SETTLE_KF``
   keyframes back from the freshest corrected one) and re-measures the SAME seq
   across re-confirmations -- a STATIONARY target the remainder shrinks monotonically
   onto -- and (b) FREEZES once converged: a re-confirmation whose NET remaining
   delta (target minus what is already blended in, ``loop_applied``) is below
   ``_LOOP_MIN_RECORRECT_M`` / ``_LOOP_MIN_RECORRECT_DEG`` is DROPPED, so the live
   pose holds steady after the loop closes instead of dithering on solve noise. A
   genuinely NEW loop (large net delta) still clears the floor and corrects.

This path is GATED on ``loop_correct`` (set only by the live ``--tight`` builder):
when off, no inbox is allocated, no pre-correction ring is kept, and the loop
blend never runs -- so the feedback is purely additive over the existing tight
behaviour.

LOOSE path: ``retain_imu`` is False, so this step is a pass-through no-op (it never
allocates a nav-state and never touches ``step.pose``). The byte-parity oracle is
therefore untouched -- ``pose.odom`` stays the vision-only odometry pose. The loop
correction is ``--tight``-only and LIVE-only, so the offline / oracle path is
byte-identical with or without it.

Placement: this step runs AFTER ``CorrectTilt`` (so ``step.pose`` is the final
vision pose used for the correction) and BEFORE ``PublishPose`` (so the published
``pose.odom`` is the IMU-propagated pose). It also OWNS the keyframe-cadence
counter and stamps ``ctx.state["is_kf_frame"]`` so the later ``EmitKeyframe`` does
not duplicate the cadence (single source of truth).
"""
from __future__ import annotations

import logging
import os
from typing import Any

import numpy as np

from sky.math import se3_from_Rp as _se3
from sky.math import se3_inv as _se3_inv
from sky.math import so3_log

from sky.vio.window import T_cw_to_body_world, body_world_to_T_cw
from sky.vio.imu import (
    apply_se3_left, complementary_correct, imu_at_rest, loop_correction_delta,
    predict_state, scale_se3_delta)
from .carriers import Step

LOG = logging.getLogger("vio.propagate_imu")

# --- velocity-gated ZUPT tuning -------------------------------------------- #
# At-rest velocity gate: ZUPT only fires when the live speed estimate is below
# this (m/s). During a push |v| is well above it, so the IMU keeps integrating
# through the constant-velocity cruise (no mid-motion freeze); a true rest sits
# at ~0 m/s, comfortably under the gate. 0.05 m/s = 5 cm/s -- below any real hand
# push, above the residual velocity noise a damped at-rest state carries.
_ZUPT_VEL = 0.05
# Hysteresis: require the accel/gyro at-rest gate to hold for this many
# CONSECUTIVE frames before ZUPT engages, so a single quiet frame in the middle
# of a motion (e.g. the instant accel crosses zero between accel and decel)
# cannot flicker the pose to a frozen state.
_ZUPT_HOLD = 3

# --- complementary vision-correction gains (all in [0, 1]; bounded => stable) #
# The correction runs EVERY frame whose vision solve is valid (a fresh per-frame
# PnP fix is available from EstimateMotion -- not only at keyframes), so each gain
# is the fraction of the error closed PER FRAME (~40 Hz). Small per-frame gains
# give a smooth, continuous pull that bleeds off the (bias-free) dead-reckoning
# drift without any visible snap, while the gap-free IMU integration carries the
# instant high-frequency response between (and through) corrections.
#
# Fraction of the POSITION error closed toward the vision pose per VALID-vision
# frame. 0.25/frame => the error half-life is ~2.4 frames (~60 ms): firmly
# vision-anchored (drift cannot run away) yet smooth (no snap). On a covered /
# failed-vision frame NO correction is applied -- the pose pure-dead-reckons.
_K_POS = 0.25
# Position error bled into VELOCITY as a damped rate (1/s after the /dt_anchor in
# the helper). Small: just enough to pull the phantom drift VELOCITY down (the
# bias double-integral) without the destabilising full ``v = displacement/dt``
# injection of the old hard re-anchor.
_K_VEL = 0.05
# Fraction of the ATTITUDE (geodesic) error slerped toward the vision attitude
# per valid-vision frame. Vision rotation is already excellent (gyro-fused PnP),
# so anchor it firmly.
_K_ROT = 0.25
# Minimum PnP inliers for the vision fix to be trusted for the correction. Below
# this (covered camera / textureless wall) vision is treated as ABSENT and the
# pose pure-dead-reckons from the IMU (the covered-camera-keeps-moving win).
_MIN_VIS_INLIERS = 8

# --- track-continuity gate + re-anchor (fast-motion "giật về" fix) ----------
# The per-frame track-overlap RATIO (survivors/prev, from odometry.estimate)
# scales the TRANSLATION correction gain: full when tracks persist, -> 0 when a
# fast frame loses most tracks (survivors are KLT-slip-suspect). Rotation
# correction is untouched (vision yaw is reliable). MORE IMPORTANT: a re-anchor
# OFFSET forgives the IMU-vs-vision position gap that accumulates while vision is
# unreliable -- so when vision re-engages it does NOT yank the (correctly
# IMU-tracked) arc back to the under-estimated ("ghì lại") vision absolute (THE
# snap). The offset tracks (nav - p_vis) when unreliable and FREEZES when
# reliable, so the live pose keeps the IMU's real translation and vision only
# corrects NEW drift relative to the re-anchor. LIVE + --tight only -> gap=0
# untouched. Env knobs for live tuning; inf/0 disable.
_OVERLAP_FULL = float(os.environ.get("OAKD_OVERLAP_FULL", "0.6"))  # >= -> full
_OVERLAP_MIN = float(os.environ.get("OAKD_OVERLAP_MIN", "0.3"))    # <= -> none
_REANCHOR = os.environ.get("OAKD_REANCHOR", "1") != "0"            # re-anchor on
# At genuine rest (ZUPT) the IMU OWNS translation: a moving object in view (a hand
# waved past a static camera) makes PnP report a spurious translation that the
# vision pull would otherwise drag the position along with. Default ON; 0 disables.
_ZUPT_FREEZE_TRANS = os.environ.get("OAKD_ZUPT_FREEZE_TRANS", "1") != "0"
# --- sensor-dropout (frame-gap) safety guard -------------------------------
# The IMU blocks are contiguous (prev_tail prepended so the integrated interval
# is gap-free), which is correct WHEN frames are consecutive. But on a SENSOR
# DROPOUT -- the OAK USB-crashes and re-enumerates (seconds of no camera AND no
# IMU), a frame is starved, a worker stalls -- the next block's first sample is
# SECONDS after prev_tail. Prepending across that boundary makes predict_state
# dead-reckon ``v*dt + 0.5*a*dt^2`` over the WHOLE blackout -> a metres-large
# pose JUMP the instant the stream returns (the "lúc đứng lúc chạy / nhảy quá
# xa" the user saw; dangerous for the FC). When the boundary gap exceeds this
# many seconds we treat it as a dropout: integrate only the FRESH block (drop
# the stale tail), zero the now-meaningless velocity (vision re-anchors position
# over the next frames -- a smooth recovery, NOT a snap), and flag the frame.
# 0.25s = 5 frames @20fps -- far above normal frame jitter, far below any real
# blackout. LIVE + --tight only -> oracle/loose byte-identical (never reached,
# and the recorded replay sessions have no dropout so it never fires anyway).
_SENSOR_GAP_S = float(os.environ.get("OAKD_SENSOR_GAP_S", "0.25"))
# Backend->live BIAS feed-forward (PLAN P2, LIVE + --tight only). The tight BA's
# optimised (bg, ba) arrive on backend.state; the dead-reckon adopts them via a
# per-keyframe LOW-PASS (_K_BIAS ~0.4 -> tau ~0.5s at ~4Hz keyframes), HEALTH-
# GATED: only when the solve is NOT vio_degraded AND has held healthy for
# _BACKEND_HEALTHY_HOLD keyframes (hysteresis: fast to distrust, slow to trust)
# AND the seq is fresh. So a diverging BA is NEVER fed (decoupled fallback). The
# pose/velocity re-base is deliberately NOT done here (math/arch review: redundant
# with the vision pull + staleness lag); bias is the unambiguous win. 0 disables.
_BACKEND_FEEDBACK = os.environ.get("OAKD_BACKEND_FEEDBACK", "1") != "0"
# CLAMP so a mis-set env can't defeat the two safety properties (safety-reviewer):
# _K_BIAS in [0, 0.6] (never a hard set -> always a low-pass), HOLD >= 2 (never
# no hysteresis).
_K_BIAS = float(np.clip(float(os.environ.get("OAKD_K_BIAS", "0.4")), 0.0, 0.6))
_BACKEND_HEALTHY_HOLD = max(2, int(os.environ.get("OAKD_BACKEND_HEALTHY_HOLD", "3")))
# Sustained-divergence DECAY (FMEA: a held last-good bias is a stale contaminant if
# the BA stays degraded while the true bias drifts). After this many CONSECUTIVE
# degraded keyframes, leak the adopted bias back toward the zero seed (the decoupled
# fallback's assumption) by _BACKEND_DECAY per keyframe.
_BACKEND_DECAY_HOLD = max(1, int(os.environ.get("OAKD_BACKEND_DECAY_HOLD", "3")))
_BACKEND_DECAY = float(np.clip(float(os.environ.get("OAKD_BACKEND_DECAY", "0.1")), 0.0, 1.0))

# --- closed-loop SLAM correction blend (LIVE + --tight only) --------------- #
# A loop closure rewrites the keyframe poses; the world-frame SE(3) delta between
# the revisited keyframe's pre-correction live pose and its corrected pose is the
# accumulated drift to remove from the LIVE pose. It is applied SMOOTHLY -- a
# bounded FRACTION of the REMAINING delta per frame -- so a revisit pulls the live
# trajectory back onto the loop-corrected one over a few frames, never a one-shot
# teleport (the hard jump we deliberately avoid). 0.20/frame => the delta decays
# with a ~3-frame half-life: visibly smooth at ~40 Hz (~75 ms), yet the drift is
# essentially fully removed within ~0.4 s of the revisit.
_LOOP_BLEND_GAIN = 0.20
# Stop blending once the REMAINING correction is below this (m for translation,
# rad for rotation) -- the geometric decay never reaches exactly zero, so a small
# floor retires the pending correction cleanly instead of applying ever-tinier
# deltas forever. 1 mm / ~0.06 deg is well below any visible / meaningful drift.
_LOOP_DONE_TRANS_M = 1e-3
_LOOP_DONE_ROT_RAD = 1e-3
# Keep at most this many recent keyframe pre-correction poses (seq -> (R, p)) so
# the SE(3) delta can be computed when a (possibly delayed) loop correction
# arrives. Bounds memory on a long live session; comfortably covers the SLAM
# solve + IPC latency between a keyframe's emission and its loop correction.
_LOOP_KF_POSE_KEEP = 256

# --- loop re-firing stability (the TELEPORT/OSCILLATION fix) ---------------- #
# SLAM re-confirms a loop EVERY keyframe while revisiting a seen area, so it emits
# a STREAM of loop.corrections (not one). Two effects make the naive blend
# oscillate, and these two knobs kill each:
#
# (a) TARGET A CONVERGED KEYFRAME, not the newest. The pose-graph pose of the
# just-inserted keyframe is still settling (it jumps cm-scale solve-to-solve),
# so targeting ``max(seq)`` chases a MOVING target -> the remainder flips sign at
# the keyframe cadence -> a sawtooth the bounded gain can't damp. An OLDER
# keyframe has converged (sub-mm solve-to-solve), so we target the newest keyframe
# that is at least this many keyframes BACK from the freshest corrected one -- a
# stationary target the blend actually converges onto.
_LOOP_SETTLE_KF = 4
# (b) FREEZE ONCE CONVERGED. Each re-confirmation re-derives the target; once the
# correction has been blended in, successive targets differ only by solve noise.
# If the NET remaining delta (freshest target minus what is already in the live
# pose, ``loop_applied``) is below these floors, DROP the correction -- do not
# re-queue it. This freezes the live pose after the loop has converged (kills the
# residual sawtooth) while a genuinely NEW loop (large net delta) still fires.
# 1.5 cm / 1.0 deg: below any meaningful drift, above the cm-scale solve jitter the
# diagnosis showed (target jitter ~0.2-0.5 cm once the older keyframe settled).
_LOOP_MIN_RECORRECT_M = 0.015
_LOOP_MIN_RECORRECT_DEG = 1.0


def propagate_imu(ctx: Any, step: Step) -> Step:
    """TIGHT-path IMU forward-propagation of the live pose (LOOSE = no-op).

    Was ``PropagateImu(StepBase)``. ``ctx`` is any object exposing ``.state`` (the
    odometry worker's :class:`~vio.comms.module.ModuleContext`, or a selftest's
    own ctx); kept as the single arg so the three tight selftests' direct
    ``run(ctx, step)`` calls are byte-compatible. All nav-state lives in
    ``ctx.state``; the helpers below take that dict explicitly.
    """
    state = ctx.state
    # LOOSE / oracle path: retain_imu is False -> pure pass-through. Never
    # allocate state, never touch step.pose (byte-identical pose.odom).
    if not state.get("retain_imu"):
        return step

    # --- keyframe-cadence (single source of truth, shared with emit_keyframe)
    # propagate_imu runs FIRST in the tail of the chain, so it owns the kf
    # counter and stamps the boolean emit_keyframe consumes. This avoids two
    # steps independently tracking kf_every (which would desync the vision
    # correction from the actual keyframe emission).
    n = state.get("kf_count", 0) + 1
    is_kf = n >= state["kf_every"]
    state["kf_count"] = 0 if is_kf else n
    state["is_kf_frame"] = bool(is_kf)

    g_world = np.asarray(
        state.get("g_world", (0.0, 9.81, 0.0)), np.float64)

    # Live nav-state: body->world (R, p), world velocity v, biases bg/ba.
    nav = state.get("live_nav")
    # Vision pose for this frame (camera->world == body->world here, body ==
    # camera optical frame) -> body->world (R, p) for the nav-state.
    R_vis, p_vis = T_cw_to_body_world(np.linalg.inv(step.pose))

    # Is THIS frame's vision solve a trustworthy absolute fix? estimate_motion
    # stamps step.info with the per-frame PnP result; a covered camera /
    # textureless wall fails the solve (ok == False) or returns too few
    # inliers, in which case the live pose must PURE-DEAD-RECKON (no pull
    # toward a stale / garbage vision pose) -- the covered-camera win.
    info = step.info or {}
    vis_ok = bool(info.get("ok", True)) and \
        int(info.get("n_inliers", _MIN_VIS_INLIERS)) >= _MIN_VIS_INLIERS
    # TIGHT-only DR indicator for the UI: True when this live pose is being
    # carried by the IMU dead-reckoning (vision lost) rather than a trusted
    # vision fix -- the viewer shows an AMBER "inertial DR" badge for it vs
    # the RED "tracking lost" badge on the loose (no-IMU-fallback) path. Set
    # ONCE here (after the retain_imu gate, so loose/oracle never reaches it)
    # so every downstream return path carries it; step.info is a COPY of
    # vo.last_info (see estimate_motion), so this never mutates the oracle key.
    if isinstance(step.info, dict):
        step.info["inertial_dr"] = not vis_ok

    if nav is not None and state.get("loop_correct") \
            and nav.get("loop_applied") is not None:
            # Closed-loop frame consistency: the per-frame vision pose lives in the
            # ORIGINAL (pre-loop, drifted) world frame, but the live nav-state has
            # been shifted by the accumulated loop correction (``loop_applied``).
            # If we corrected the nav toward the RAW vision pose, the every-frame
        # complementary pull would drag the loop correction straight back out
        # (vision fires every frame; the loop closes rarely). So transform the
        # vision fix by the SAME loop correction before using it: vision then
        # anchors the live pose to the LOOP-CORRECTED trajectory, not the
        # drifted one. This is the standard "apply the loop transform to BOTH
        # the pose and the incoming measurements" re-framing.
        R_vis, p_vis = apply_se3_left(
            nav["loop_applied"][:3, :3], nav["loop_applied"][:3, 3],
            R_vis, p_vis)

    if nav is None:
        # First frame on the tight path: anchor the live state to the vision
        # pose with zero velocity and zero bias. From here on it dead-reckons
        # continuously and is pulled toward vision by a smooth correction.
        nav = {
            "R": R_vis, "p": p_vis, "v": np.zeros(3),
            "bg": np.zeros(3), "ba": np.zeros(3),
            # backend-bias feed-forward tracking (PLAN P2): last adopted keyframe
            # seq (staleness gate, -1 = none yet) + consecutive-healthy keyframe
            # count (hysteresis before trusting the BA's bias).
            "backend_bias_seq": -1,
            "backend_healthy_run": 0,
            "backend_degraded_run": 0,
            # anchor_dt accumulates wall time since the last vision
            # correction (used to scale the velocity-feedback term).
            "anchor_dt": 0.0,
            # zupt_run counts consecutive accel/gyro-at-rest frames for the
            # ZUPT hysteresis.
            "zupt_run": 0,
            # prev_tail holds the LAST raw IMU sample (ts, gyro_cam,
            # accel_cam) of the previously integrated block, prepended to the
            # next block so the inter-block segment is never dropped.
            "prev_tail": None,
            # --- closed-loop SLAM correction (LIVE + --tight only) ---------
            # kf_pose_pre: recent keyframe seq -> PRE-correction (R, p) live
            # pose (the STABLE anchor the loop SE(3) delta is measured against;
            # never re-anchored, so it always reflects the true drift).
            # loop_delta: the REMAINING world-frame correction (R_d, p_d) still
            # to bleed into the live pose, None when none is pending.
            # loop_applied: the 4x4 world-frame correction ALREADY blended into
            # the live pose so far (None = identity); a newer full-graph
            # correction subtracts this to get its remainder.
            "kf_pose_pre": {},
            "loop_delta": None,
            "loop_applied": None,
            # loop_target_seq: the keyframe seq the blend is currently locked
            # onto. SLAM re-confirms the loop every keyframe, so to keep the
            # blend target STATIONARY (not chasing the advancing newest
            # keyframe) we lock onto one converged keyframe and re-measure the
            # SAME seq across re-confirmations -- only re-locking onto a newer
            # keyframe when a genuinely NEW loop (large net delta) appears.
            "loop_target_seq": None,
        }
        state["live_nav"] = nav
        # Seed the pre-correction pose for THIS keyframe too (so an early loop
        # closure that revisits frame 0 still has an anchor).
        _record_kf_pose(state, nav, step.frame.seq)
        return step

    # --- pull this frame's retained raw IMU block (camera optical frame) ----
    # preintegrate_prior stores an EMPTY segment (size-0 arrays) for a frame
    # whose packet carried no IMU samples, so guard on the sample count.
    seg = state["imu_segs"].get(step.frame.seq)
    has_imu = seg is not None and np.asarray(seg[0]).size >= 1

    if has_imu:
        ts_raw, gyro_raw, accel_raw = (
            np.asarray(seg[0], np.int64), np.asarray(seg[1], np.float64),
            np.asarray(seg[2], np.float64))
        # --- (1) gap-free interval: prepend the previous block's tail -------
        # The per-frame packet cut is (prev_ts, ts], so consecutive blocks
        # share NO boundary sample; prepending the previous block's last
        # sample makes the integrated interval exactly
        # (prev_block_last_ts, this_block_last_ts] with no dropped segment.
        # EXCEPT across a SENSOR DROPOUT (boundary gap > _SENSOR_GAP_S): the
        # prev_tail is seconds stale, so prepending would integrate the entire
        # blackout into one giant step -> a dangerous pose jump on reconnect.
        # Refuse it -- integrate only the fresh block (small internal dts) and
        # zero the stale velocity so position holds and re-anchors smoothly
        # instead of snapping; flag the frame as a gap-recovery for the UI/FC.
        tail = nav.get("prev_tail")
        boundary_gap = (
            (int(ts_raw[0]) - int(tail[0])) * 1e-9 if tail is not None else 0.0)
        if boundary_gap > _SENSOR_GAP_S:
            ts, gyro, accel = ts_raw, gyro_raw, accel_raw
            nav["v"] = np.zeros(3)            # blind interval -> velocity unknown
            nav["vis_offset"] = None          # re-seed the re-anchor on re-lock
            nav["anchor_dt"] = 0.0            # stale dt the vel-feedback divides by
            if isinstance(step.info, dict):
                step.info["sensor_gap_s"] = float(boundary_gap)
                step.info["inertial_dr"] = True
        elif tail is not None and int(tail[0]) < int(ts_raw[0]):
            ts = np.concatenate(([np.int64(tail[0])], ts_raw))
            gyro = np.vstack((tail[1][None, :], gyro_raw))
            accel = np.vstack((tail[2][None, :], accel_raw))
        else:
            ts, gyro, accel = ts_raw, gyro_raw, accel_raw
        # Remember this block's last sample for the next frame's boundary.
        nav["prev_tail"] = (int(ts_raw[-1]), gyro_raw[-1].copy(),
                            accel_raw[-1].copy())
    else:
        ts = gyro = accel = None

    if ts is None or ts.size < 2:
        # No usable IMU for this frame: dead-reckoning cannot advance without
        # samples. Still apply the smooth vision correction when vision is
        # valid (so the drift is pulled back), then hold/publish the nav pose.
        if vis_ok:
            _vision_correct(nav, R_vis, p_vis)
        step.pose = _finalize(state, nav, step.frame.seq, is_kf)
        return step

    # Tight backend -> live BIAS feed-forward (PLAN P2): fold the backend's latest
    # optimised (bg, ba) into the dead-reckon bias BEFORE this frame's predict,
    # health-gated (a diverging BA is never fed). No-op until a fresh, trusted
    # backend.state arrives; LIVE + --tight only.
    _adopt_backend_bias(state, nav)

    # --- (2) velocity-gated ZUPT: only freeze when GENUINELY at rest --------
    # imu_at_rest uses raw |gyro|/|accel| magnitudes (frame-invariant), so the
    # camera-frame samples give the same verdict as the IMU-frame ones. But
    # accel+gyro alone cannot tell rest from constant-velocity cruise (both
    # read |accel|~g, |gyro|~0), so we ALSO require the live speed to be small
    # and the at-rest gate to have held for a few frames (hysteresis).
    accel_rest = imu_at_rest(
        gyro, accel, gravity=float(np.linalg.norm(g_world)))
    nav["zupt_run"] = nav["zupt_run"] + 1 if accel_rest else 0
    speed = float(np.linalg.norm(nav["v"]))
    zupt = (accel_rest and speed < _ZUPT_VEL
            and nav["zupt_run"] >= _ZUPT_HOLD)

    if zupt:
        # Genuinely at rest: hold velocity at zero, freeze translation, but
        # still integrate rotation so a slow at-rest yaw is tracked without
        # the position walking off (the static-drift win).
        nav["v"] = np.zeros(3)
        R_new, _, _ = predict_state(
            nav["R"], nav["p"], np.zeros(3), ts, gyro, accel,
            nav["bg"], nav["ba"], np.zeros(3))
        nav["R"] = R_new
    else:
        # --- (3) forward-propagate the IMU (real motion or cruise) ----------
        R_new, p_new, v_new = predict_state(
            nav["R"], nav["p"], nav["v"], ts, gyro, accel,
            nav["bg"], nav["ba"], g_world)
        nav["R"], nav["p"], nav["v"] = R_new, p_new, v_new

    # accumulate the interval for the velocity-feedback scaling.
    nav["anchor_dt"] += (int(ts[-1]) - int(ts[0])) * 1e-9

    # --- (4) smooth vision correction EVERY valid-vision frame --------------
    # Replaces the old hard keyframe re-anchor: a small per-frame
    # complementary pull toward the fresh PnP fix bleeds off the dead-reckoning
    # drift continuously (no snap, no overshoot). On a covered / failed-vision
    # frame this is skipped, so the pose pure-dead-reckons through the blind
    # interval (keeps moving) until vision re-locks and pulls it back.
    # Track-continuity reliability scale (0..1) from the overlap ratio: full pull
    # when most tracks survived, fading to 0 as a fast frame loses them.
    ratio = float(info.get("track_overlap_ratio", 1.0))
    overlap_scale = (1.0 if _OVERLAP_FULL <= _OVERLAP_MIN else float(
        np.clip((ratio - _OVERLAP_MIN) / (_OVERLAP_FULL - _OVERLAP_MIN),
                0.0, 1.0)))
    # Gate the POSITION pull on info["ok"] ONLY -- a frame where PnP actually
    # produced a vision translation fix. The inlier-COUNT gate (n_inliers >= 8) is
    # dropped on purpose: it is redundant with (a) the overlap gate above (lost
    # tracks -> low overlap -> the pull fades) and (b) the frontend's own
    # low_inliers_frozen freeze, which returns ok == False with translation held,
    # so this gate already excludes it. All the count threshold added was blocking
    # good few-inlier frames the overlap gate already trusts. On the degenerate
    # vision paths (too_few_points / pnp_failed / low_inliers_frozen) p_vis is the
    # gyro-propagated / frozen pose, NOT a position fix, so pulling DR toward it
    # would yank the IMU-tracked arc back -- exactly the "giật về" snap we removed.
    vis_scale = overlap_scale if bool(info.get("ok", True)) else 0.0
    # ZUPT owns translation at genuine rest: zero the vision TRANSLATION pull (and
    # freeze the re-anchor offset) so a moving object in view -- e.g. a hand waved
    # in front of a STATIC camera -- cannot drag the position. The predict-side
    # ZUPT already froze the IMU translation but did NOT gate this vision pull
    # (that was the hand-wave drift). ROTATION is still corrected below (the
    # gyro-anchored vision yaw stays good, robust to the dynamic object).
    freeze_trans = bool(zupt and _ZUPT_FREEZE_TRANS)
    trans_scale = 0.0 if freeze_trans else vis_scale
    # Re-anchor offset, updated EVERY frame EXCEPT while ZUPT freezes translation:
    # tracks (nav - p_vis) when vision is unreliable (low scale -> the accumulated
    # IMU-vs-vision gap is forgiven), FREEZES while reliable (or at rest). So when
    # vision re-engages after a fast sweep the target is p_vis + frozen_offset ~ nav
    # -- NO yank back to the under-estimated vision absolute (the "giật về" snap).
    p_target = p_vis
    if _REANCHOR:
        off = nav.get("vis_offset")
        if off is None:
            off = (nav["p"] - p_vis).copy()
        elif not freeze_trans:
            gap = nav["p"] - p_vis
            off = off * vis_scale + gap * (1.0 - vis_scale)
        nav["vis_offset"] = off
        p_target = p_vis + off
    if vis_scale > 0.0:
        _vision_correct(nav, R_vis, p_target,
                        k_pos=_K_POS * trans_scale, k_vel=_K_VEL * trans_scale)

    # Replace the published live pose with the IMU-propagated one (camera->world),
    # AFTER the smooth closed-loop SLAM correction is bled in (no-op when no
    # loop correction is pending or the feedback is disabled).
    step.pose = _finalize(state, nav, step.frame.seq, is_kf)
    return step


def _vision_correct(nav: dict, R_vis: np.ndarray, p_vis: np.ndarray,
                    k_pos: float = _K_POS, k_vel: float = _K_VEL) -> None:
    """Pull the live nav-state a bounded fraction toward the vision fix.

    Smooth complementary correction (NOT a hard ``p = p_vis`` snap): closes a
    per-frame fraction of the position/velocity/attitude error toward the
    fresh vision pose, then resets the anchor-interval accumulator so the next
    velocity-feedback term is scaled by the next inter-correction interval.
    Mutates ``nav`` in place. ``k_pos`` / ``k_vel`` default to the full gains;
    the track-continuity gate passes overlap-scaled translation gains (rotation
    ``_K_ROT`` is always full -- vision yaw is reliable).
    """
    R_new, p_new, v_new = complementary_correct(
        nav["R"], nav["p"], nav["v"], R_vis, p_vis,
        float(nav.get("anchor_dt", 0.0)), k_pos, k_vel, _K_ROT)
    nav["R"], nav["p"], nav["v"] = R_new, p_new, v_new
    nav["anchor_dt"] = 0.0


# --------------------------------------------------------------------------- #
# Closed-loop SLAM correction (LIVE + --tight only)
# --------------------------------------------------------------------------- #
def _finalize(state: dict, nav: dict, seq: int, is_kf: bool) -> np.ndarray:
    """Apply the smooth loop-correction blend, record the keyframe anchor, and
    return the published camera->world pose for this frame.

    Called from every nav-advancing exit path so the closed-loop correction +
    keyframe-pose recording happen exactly once per frame, AFTER the vision
    correction and IMU propagation have settled the nav-state. When the
    closed-loop feedback is disabled (``loop_correct`` unset, e.g. no slam
    endpoint wired) this is a thin wrapper that only re-serialises the pose --
    the existing tight behaviour is unchanged. ``state`` is the worker's shared
    state dict (was ``ctx.state``).
    """
    if state.get("loop_correct"):
        # 1. Drain any loop correction(s) that arrived since the last frame and
        #    queue the world-frame SE(3) delta to bleed in.
        _drain_loop_inbox(state, nav)
        # 2. Bleed a bounded fraction of the pending delta into the live pose.
        _apply_loop_blend(nav)
        # 3. Remember this keyframe's PRE-(next-)correction pose as the anchor
        #    a future loop closure measures its SE(3) delta against.
        if is_kf:
            _record_kf_pose(state, nav, seq)
    return np.linalg.inv(body_world_to_T_cw(nav["R"], nav["p"]))


def _record_kf_pose(state: dict, nav: dict, seq: int) -> None:
    """Stash the current live body->world pose under keyframe ``seq``.

    This is the PRE-correction anchor: when a loop closure later rewrites
    keyframe ``seq``'s pose, the world-frame delta is measured between this
    stored pose and the corrected one. Bounded to the most recent
    ``_LOOP_KF_POSE_KEEP`` keyframes so a long live session stays bounded.
    """
    if not state.get("loop_correct"):
        return
    store = nav["kf_pose_pre"]
    store[int(seq)] = (nav["R"].copy(), nav["p"].copy())
    if len(store) > _LOOP_KF_POSE_KEEP:
        # Evict the oldest keyframe anchors (smallest seqs) -- those are well
        # past any plausible loop-correction latency.
        for old in sorted(store)[:len(store) - _LOOP_KF_POSE_KEEP]:
            del store[old]


def _adopt_backend_bias(state: dict, nav: dict) -> None:
    """Fold the BA backend's optimised ``(bg, ba)`` into the live dead-reckon bias.

    The ``ba`` process publishes its latest optimised bias on the IPC ``ba.state``
    topic (a :class:`~vio.comms.messages.BackendState` dataclass); vio.main's
    ba-endpoint bridge re-hydrates it onto the local bus, where the odometry worker's
    inbox holds the freshest one. We take it here (latest-wins inbox) and fold it into
    ``nav["bg"]``/``nav["ba"]`` via a bounded per-keyframe LOW-PASS (``_K_BIAS``), so
    a single noisy solve cannot step the dead-reckon. HEALTH-GATED: adopt ONLY when
    the solve is NOT ``degraded`` AND has held healthy for ``_BACKEND_HEALTHY_HOLD``
    keyframes (hysteresis -- fast to distrust, slow to trust) AND the ``seq`` is fresh
    (newer than the last adopted). The seq staleness gate is what makes the async IPC
    hop tolerable: a state that arrives late / out of order is dropped. A diverging BA
    is therefore NEVER fed and the live pose falls back to the decoupled behaviour.
    LIVE + --tight only -- the loose / oracle path never wires the inbox.

    ``msg`` is the :class:`~vio.comms.messages.BackendState` DATACLASS off IPC (seq /
    bg / ba / degraded as attributes), not the old in-vio local-bus dict.
    """
    if not _BACKEND_FEEDBACK:
        return
    inbox = state.get("backend_inbox")
    if inbox is None:
        return
    msg = inbox.take()
    if msg is None:
        return
    seq = int(msg.seq)
    if seq <= nav["backend_bias_seq"]:        # stale / already adopted -> drop
        return
    nav["backend_bias_seq"] = seq
    if bool(msg.degraded):
        nav["backend_healthy_run"] = 0        # distrust immediately on a bad solve
        nav["backend_degraded_run"] += 1
        # Sustained divergence: the held last-good bias may now be stale (thermal
        # drift, with no backend correcting it). Leak it back toward the zero seed
        # -- the same state the decoupled fallback assumes -- so it cannot quietly
        # contaminate the dead-reckon. (safety-reviewer FMEA)
        if nav["backend_degraded_run"] >= _BACKEND_DECAY_HOLD:
            nav["bg"] = nav["bg"] * (1.0 - _BACKEND_DECAY)
            nav["ba"] = nav["ba"] * (1.0 - _BACKEND_DECAY)
        return
    nav["backend_degraded_run"] = 0
    nav["backend_healthy_run"] += 1
    if nav["backend_healthy_run"] < _BACKEND_HEALTHY_HOLD:
        return                                # not yet trusted (hysteresis)
    bg, ba = msg.bg, msg.ba
    if bg is None or ba is None:
        return
    nav["bg"] = nav["bg"] + _K_BIAS * (np.asarray(bg, dtype=np.float64) - nav["bg"])
    nav["ba"] = nav["ba"] + _K_BIAS * (np.asarray(ba, dtype=np.float64) - nav["ba"])


def _drain_loop_inbox(state: dict, nav: dict) -> None:
    """Consume queued ``LoopCorrection``s; set the REMAINING world-frame delta.

    ``vio.main`` (LIVE + --tight) hands each ``LoopCorrection`` from the slam
    endpoint into the thread-safe ``loop_inbox`` holder. We process them on the
    ODOMETRY thread here (so the nav-state is only ever touched by one thread).
    ``state`` is the worker's shared state dict (was ``ctx.state``).

    Each SLAM correction is a FULL pose-graph re-optimisation (``kf_poses`` =
    ``{seq: T_world_cam}`` for the WHOLE graph), so a newer correction
    SUPERSEDES an older one rather than stacking on it. The total world-frame
    correction the live pose should have, measured at a CONVERGED corrected
    keyframe we hold a STABLE pre-correction anchor for, is::

        D_target = T_corrected[seq] @ inv(T_pre[seq])

    ``T_pre[seq]`` is the live pose AS IT WAS when that keyframe passed (the
    accumulated drift) -- recorded once in ``kf_pose_pre`` and NEVER re-anchored,
    so ``D_target`` is always the true total drift to remove. Part of a PRIOR
    correction may already be blended into the live pose (tracked in
    ``loop_applied``); the still-to-apply remainder is therefore::

        loop_delta = D_target @ inv(loop_applied)

    (the freshest target, minus what is already in the live pose). The blend
    then bleeds this remainder in smoothly (``_apply_loop_blend``).

    TELEPORT/OSCILLATION FIX (two parts, both here):

    1. **Target a CONVERGED keyframe, not ``max(seq)``.** SLAM re-confirms the
       loop every keyframe while revisiting, so it emits a STREAM of corrections;
       the just-inserted keyframe's pose-graph pose is still settling (jumps
       cm-scale solve-to-solve), so targeting the newest keyframe chases a
       MOVING target -> the remainder flips sign at the keyframe cadence -> a
       sawtooth the bounded gain can't damp. We instead target the newest
       keyframe we hold an anchor for that is at least ``_LOOP_SETTLE_KF``
       keyframes BACK from the freshest corrected one -- an OLDER, converged
       (sub-mm solve-to-solve) target the blend actually settles onto.

    2. **Re-blend only on a real NET change (freeze once converged).** Once the
       correction is blended in, successive re-confirmations differ only by
       solve noise. If the NET remaining delta (``T_rem``) is below
       ``_LOOP_MIN_RECORRECT_M`` / ``_LOOP_MIN_RECORRECT_DEG``, DROP it -- do
       not re-queue -- so the live pose FREEZES after the loop converges (kills
       the residual sawtooth). A genuinely NEW loop (large net delta) still
       clears the floor and fires.
    """
    inbox = state.get("loop_inbox")
    if inbox is None:
        return
    corrections = inbox.drain()
    if not corrections:
        return
    store = nav["kf_pose_pre"]
    # Only the FRESHEST correction matters (full graph rewrite supersedes), but
    # walk all drained so the newest with a usable anchor wins.
    for corr in reversed(corrections):
        kf_poses = getattr(corr, "kf_poses", None)
        if not kf_poses:
            continue
        # --- (1) pick a CONVERGED, STATIONARY target keyframe -----------
        # ``newest_corr`` is the freshest keyframe in this re-optimisation (the
        # just-inserted, still-settling one). The settled candidates are the
        # anchored keyframes at least ``_LOOP_SETTLE_KF`` keyframes back from it
        # (converged pose-graph poses). To keep the blend target STATIONARY
        # across the stream of re-confirmations, LOCK onto one converged
        # keyframe and re-measure the SAME seq every time it is still present +
        # settled -- only re-lock onto the newest settled keyframe when the lock
        # is gone (evicted / dropped from the graph). A stationary target makes
        # the remainder shrink monotonically to zero instead of chasing the
        # advancing newest keyframe (the sawtooth).
        newest_corr = max(int(s) for s in kf_poses)
        cand = [int(s) for s in kf_poses
                if int(s) in store and int(s) <= newest_corr - _LOOP_SETTLE_KF]
        if not cand:
            # No settled keyframe >= _LOOP_SETTLE_KF back yet (early loop /
            # small graph): fall back to the best AVAILABLE anchored keyframe
            # so a one-off / early correction still bounds drift. The lock
            # (loop_target_seq) + the freeze-when-converged
            # _LOOP_MIN_RECORRECT_M/_DEG drop below still prevent the sawtooth
            # -- the oscillation came from CHASING the advancing newest kf
            # across a STREAM of re-confirmations, which the lock pins; a
            # fallback target that gets LOCKED is stationary too.
            cand = [int(s) for s in kf_poses if int(s) in store]
            if not cand:
                continue                 # genuinely no anchor -> can't apply
        locked = nav.get("loop_target_seq")
        if locked is not None and int(locked) in cand:
            seq = int(locked)            # keep the stationary lock
        else:
            seq = max(cand)              # (re-)lock onto newest settled kf
            nav["loop_target_seq"] = seq
        T_corr = np.asarray(kf_poses[seq], np.float64)
        R_corr, p_corr = T_cw_to_body_world(np.linalg.inv(T_corr))
        R_pre, p_pre = store[seq]
        # Total target world-frame correction (full drift to remove).
        R_t, p_t = loop_correction_delta(R_pre, p_pre, R_corr, p_corr)
        T_target = _se3(R_t, p_t)
        # Subtract what is already blended into the live pose -> remainder.
        T_applied = nav.get("loop_applied")
        T_rem = T_target if T_applied is None \
            else T_target @ _se3_inv(T_applied)
        # --- (2) freeze once converged: drop a negligible NET re-correction.
        # The remainder is what is STILL to apply on top of ``loop_applied``;
        # if it is below the floors the loop has already converged and this is
        # just solve jitter -- ignore it so the live pose stops oscillating.
        rem_trans = float(np.linalg.norm(T_rem[:3, 3]))
        rem_rot_deg = float(np.degrees(
            np.linalg.norm(so3_log(T_rem[:3, :3]))))
        if (rem_trans < _LOOP_MIN_RECORRECT_M
                and rem_rot_deg < _LOOP_MIN_RECORRECT_DEG):
            return        # converged: freeze (no re-queue), kill the sawtooth
        nav["loop_delta"] = (T_rem[:3, :3].copy(), T_rem[:3, 3].copy())
        # A loop closure is a RARE event; log the drift it removes so the
        # closed-loop feedback is observable in a live run (and provable).
        LOG.info("vio: closed-loop SLAM correction at kf seq=%d (n_loops=%d) "
                 "-- pulling %.1f cm / %.2f deg of accumulated drift back into "
                 "the live pose (smoothly)", seq,
                 int(getattr(corr, "n_loops", 0)),
                 rem_trans * 100.0, rem_rot_deg)
        return            # freshest usable correction wins; ignore older ones


def _apply_loop_blend(nav: dict) -> None:
    """Bleed a bounded fraction of the pending loop correction into the live
    pose (SMOOTH -- no hard snap), retiring it once negligible.

    Applies ``_LOOP_BLEND_GAIN`` of the REMAINING world-frame SE(3) delta this
    frame via geodesic interpolation, left-multiplies the partial step onto the
    live ``(R, p)``, reduces the remaining delta by the same step, and ACCRUES
    the step into ``loop_applied`` (the total correction now in the live pose,
    used by the next full-graph correction to compute its remainder). Velocity
    is left untouched (the correction is a position/attitude re-anchor, not a
    motion). When the remaining delta falls below the done floor it is cleared.
    """
    pend = nav.get("loop_delta")
    if pend is None:
        return
    R_rem, p_rem = pend
    # Retire a negligible remainder so we don't apply ever-tinier deltas.
    if (float(np.linalg.norm(p_rem)) < _LOOP_DONE_TRANS_M
            and float(np.linalg.norm(so3_log(R_rem))) < _LOOP_DONE_ROT_RAD):
        nav["loop_delta"] = None
        return
    # Partial (bounded) world-frame step to apply this frame.
    R_step, p_step = scale_se3_delta(R_rem, p_rem, _LOOP_BLEND_GAIN)
    nav["R"], nav["p"] = apply_se3_left(R_step, p_step, nav["R"], nav["p"])
    # Remaining = full_delta composed with the inverse of the step (so the
    # product of all per-frame steps converges to the full delta).
    T_step = _se3(R_step, p_step)
    T_rem_new = _se3(R_rem, p_rem) @ _se3_inv(T_step)
    nav["loop_delta"] = (T_rem_new[:3, :3].copy(), T_rem_new[:3, 3].copy())
    # Accrue the step into the total correction already in the live pose.
    T_applied = nav.get("loop_applied")
    nav["loop_applied"] = T_step if T_applied is None else T_step @ T_applied
