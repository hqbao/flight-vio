"""``propagate_imu`` step (TIGHT path only): IMU forward-propagate the live pose.

The gap this closes
-------------------
On the loose path the live displayed position is ``pose.odom``, published EVERY
frame by :class:`~vio.modules.publish_pose.PublishPose` from the per-frame
VISION-ONLY odometry (PnP). When vision is absent (covered camera) or too weak to
solve (white wall) the PnP either fails or freezes translation, so the live pose
FREEZES even while the device is physically moving -- the "covered camera + move =
stays still" symptom. Basalt does not freeze: it propagates the IMU every frame
(predictState) so the live pose keeps reacting to motion (and drifts) until vision
re-locks and pulls it back.

This step adds exactly that, but ONLY on the ``--tight`` path (gated on
``retain_imu`` -- the same flag that turns on per-frame IMU retention). It owns a
live body->world nav-state ``(R, p, v, bg, ba)`` plus the fixed world gravity, and
on EVERY frame:

1. **Re-anchor on a keyframe (vision pull-back).** When the current frame is a
   keyframe boundary, the vision (PnP/gyro-fused/BA) pose in ``step.pose`` is the
   freshest absolute fix, so the live nav-state's pose is RE-ANCHORED to it and the
   velocity is re-derived from the displacement since the previous anchor. This is
   the "vision pulls the drift back" half: between keyframes the pose dead-reckons
   from the IMU; at each keyframe the accumulated inertial drift is reset to the
   vision estimate. (The tight backend's drift-corrected pose arrives later on
   ``pose.refined``; re-anchoring to the per-frame vision pose here is the
   immediate, deterministic correction and keeps the live output in lock-step with
   the keyframe cadence.)

2. **ZUPT at rest (preserve the static-drift win).** If the frame's IMU block reads
   "at rest" (low gyro AND |accel| ~ g, see :func:`vio.mathlib.imu.imu.imu_at_rest`),
   a Zero-Velocity Update is applied: velocity is held at zero and the translation
   is frozen for that frame (rotation may still track a tiny residual but position
   does not walk). This is what keeps the at-rest drift better than a pure
   forward-integrating filter -- the pose only ever moves when there is real motion.

3. **Forward-propagate when moving.** Otherwise the retained raw IMU block for this
   frame (camera optical frame, from :class:`~vio.modules.preintegrate_prior.PreintegratePrior`)
   is integrated forward under gravity (:func:`vio.mathlib.imu.imu.predict_state`):
   gyro -> rotation, gravity-removed accel -> velocity -> position. The propagated
   pose then REPLACES ``step.pose`` so the downstream :class:`PublishPose` emits the
   IMU-propagated pose on ``pose.odom`` -- the live marker dead-reckons through the
   blind interval instead of freezing.

LOOSE path: ``retain_imu`` is False, so this step is a pass-through no-op (it never
allocates a nav-state and never touches ``step.pose``). The byte-parity oracle is
therefore untouched -- ``pose.odom`` stays the vision-only odometry pose.

Placement: this step runs AFTER ``CorrectTilt`` (so ``step.pose`` is the final
vision pose used for re-anchoring) and BEFORE ``PublishPose`` (so the published
``pose.odom`` is the IMU-propagated pose). It also OWNS the keyframe-cadence
counter and stamps ``ctx.state["is_kf_frame"]`` so the later ``EmitKeyframe`` does
not duplicate the cadence (single source of truth).
"""
from __future__ import annotations

import numpy as np

from vio.comms import Step as StepBase
from vio.mathlib.backend.vio_window import T_cw_to_body_world, body_world_to_T_cw
from vio.mathlib.imu.imu import imu_at_rest, predict_state
from .step import Step


class PropagateImu(StepBase):
    name = "propagate_imu"

    def run(self, ctx, step: Step):
        # LOOSE / oracle path: retain_imu is False -> pure pass-through. Never
        # allocate state, never touch step.pose (byte-identical pose.odom).
        if not ctx.state.get("retain_imu"):
            return step

        # --- keyframe-cadence (single source of truth, shared with EmitKeyframe)
        # PropagateImu runs FIRST in the tail of the chain, so it owns the kf
        # counter and stamps the boolean EmitKeyframe consumes. This avoids two
        # steps independently tracking kf_every (which would desync the re-anchor
        # from the actual keyframe emission).
        n = ctx.state.get("kf_count", 0) + 1
        is_kf = n >= ctx.state["kf_every"]
        ctx.state["kf_count"] = 0 if is_kf else n
        ctx.state["is_kf_frame"] = bool(is_kf)

        g_world = np.asarray(
            ctx.state.get("g_world", (0.0, 9.81, 0.0)), np.float64)

        # Live nav-state: body->world (R, p), world velocity v, biases bg/ba.
        nav = ctx.state.get("live_nav")
        # Vision pose for this frame (camera->world == body->world here, body ==
        # camera optical frame) -> body->world (R, p) for the nav-state.
        R_vis, p_vis = T_cw_to_body_world(np.linalg.inv(step.pose))

        if nav is None:
            # First frame on the tight path: anchor the live state to the vision
            # pose with zero velocity and zero bias. From here on it dead-reckons
            # between keyframes and re-anchors on each keyframe.
            ctx.state["live_nav"] = {
                "R": R_vis, "p": p_vis, "v": np.zeros(3),
                "bg": np.zeros(3), "ba": np.zeros(3),
                "anchor_p": p_vis.copy(), "anchor_dt": 0.0,
            }
            return step

        # --- (1) re-anchor on a keyframe: vision pulls the inertial drift back --
        if is_kf:
            # Re-derive velocity from the displacement over the interval since the
            # previous anchor, so the dead-reckoning velocity stays continuous
            # across the re-anchor (no jump) instead of being thrown away. Falls
            # back to the integrated velocity when the interval is degenerate.
            dt_anchor = float(nav.get("anchor_dt", 0.0))
            if dt_anchor > 1e-6:
                nav["v"] = (p_vis - nav["anchor_p"]) / dt_anchor
            nav["R"] = R_vis
            nav["p"] = p_vis
            nav["anchor_p"] = p_vis.copy()
            nav["anchor_dt"] = 0.0
            # The live pose IS the vision keyframe pose this frame; publish it.
            return step

        # --- pull this frame's retained raw IMU block (camera optical frame) ----
        # PreintegratePrior stores an EMPTY segment (size-0 arrays) for a frame
        # whose packet carried no IMU samples, so guard on the sample count -- a
        # 0/1-sample block has no dt to integrate and is treated as "no IMU".
        seg = ctx.state["imu_segs"].get(step.frame.seq)
        if seg is None or np.asarray(seg[0]).size < 2:
            # No usable IMU for this frame: hold the current nav pose (cannot
            # dead-reckon without samples). Still write it out so pose.odom stays
            # the live nav pose rather than reverting to the vision pose
            # mid-interval.
            step.pose = np.linalg.inv(body_world_to_T_cw(nav["R"], nav["p"]))
            return step
        ts, gyro_cam, accel_cam = seg

        # --- (2) ZUPT at rest: hold velocity at zero, freeze translation --------
        # imu_at_rest uses raw |gyro|/|accel| magnitudes (frame-invariant), so the
        # camera-frame samples give the same verdict as the IMU-frame ones.
        if imu_at_rest(gyro_cam, accel_cam, gravity=float(np.linalg.norm(g_world))):
            nav["v"] = np.zeros(3)
            # Rotation may still integrate a tiny residual rate; translation is
            # frozen. Re-integrate rotation only (position unchanged) so a slow
            # at-rest yaw is still tracked without the position walking off.
            R_new, _, _ = predict_state(
                nav["R"], nav["p"], np.zeros(3), ts, gyro_cam, accel_cam,
                nav["bg"], nav["ba"], np.zeros(3))
            nav["R"] = R_new
            # accumulate the interval so the next re-anchor's dt is correct.
            nav["anchor_dt"] += (int(ts[-1]) - int(ts[0])) * 1e-9
            step.pose = np.linalg.inv(body_world_to_T_cw(nav["R"], nav["p"]))
            return step

        # --- (3) forward-propagate the IMU (real motion) ------------------------
        R_new, p_new, v_new = predict_state(
            nav["R"], nav["p"], nav["v"], ts, gyro_cam, accel_cam,
            nav["bg"], nav["ba"], g_world)
        nav["R"], nav["p"], nav["v"] = R_new, p_new, v_new
        nav["anchor_dt"] += (int(ts[-1]) - int(ts[0])) * 1e-9
        # Replace the published live pose with the IMU-propagated one (camera->world).
        step.pose = np.linalg.inv(body_world_to_T_cw(nav["R"], nav["p"]))
        return step
