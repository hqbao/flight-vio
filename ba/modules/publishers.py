"""Thin "emit one result on a bus topic" steps for the backend.

The two backend publishers the windowed-BA worker needs, lifted verbatim from
``vio.modules.publishers`` (the per-frame odometry publishers stay in ``vio`` --
``ba`` consumes a finished keyframe, it has no front-end). Each takes the pose
message flowing through the keyframe solve, publishes one message on a bus topic,
and forwards the carrier UNCHANGED so the chain continues. They hold no state and
never re-derive anything -- each is a faithful tap of a REAL backend output for
the UI / downstream subscribers.

* :func:`publish_refined`   -> ``pose.refined``   (BA-refined pose, terminal)
* :func:`publish_ba_window` -> ``ba.window``      (OPT-IN --ba-window)

``publish_ba_window`` is never wired on the default / oracle path, so the
byte-parity oracle is UNAFFECTED.
"""
from __future__ import annotations

from ba.comms import LocalPubSub, topics
from ba.comms.messages import BaWindow, PoseMsg
from ba.engine import Engine
from ba.engine.ba_capture import BaWindowSnap


def publish_refined(bus: LocalPubSub, msg: PoseMsg) -> None:
    """Publish the BA-refined pose on ``pose.refined`` (terminal step).

    Was ``PublishRefined(Step)``; identical publish, the bus passed explicitly.
    """
    bus.publish(topics.POSE_REFINED, msg)
    return None


def publish_ba_window(engine: Engine, bus: LocalPubSub, msg: PoseMsg) -> PoseMsg:
    """Publish the BA-window solve snapshot on ``ba.window``; forward the pose.

    Was ``PublishBaWindow(Step)``; the engine + bus are passed explicitly. The
    refined pose carrier is forwarded UNCHANGED so ``publish_refined`` emits it
    identically to the no-capture path.
    """
    snap = engine.poll_overlay()
    # The overlay is a BaWindowSnap ONLY on the capture engine + a keyframe
    # whose solve ran; anything else (None on warmup) is simply skipped.
    if isinstance(snap, BaWindowSnap):
        bus.publish(topics.BA_WINDOW, BaWindow(
            seq=int(snap.seq), ts_ns=int(snap.ts_ns),
            kf_ids=snap.kf_ids, kf_quat=snap.kf_quat, kf_pos=snap.kf_pos,
            lm_ids=snap.lm_ids, lm_xyz=snap.lm_xyz,
            obs_kf=snap.obs_kf, obs_lm=snap.obs_lm, obs_uv=snap.obs_uv,
            obs_reproj_px=snap.obs_reproj_px,
            ba_reproj_px=float(snap.ba_reproj_px),
            kf_quat_pre=snap.kf_quat_pre, kf_pos_pre=snap.kf_pos_pre,
            lm_xyz_pre=snap.lm_xyz_pre,
            n_kf=int(snap.n_kf), n_lm=int(snap.n_lm)))
    # Forward the refined pose UNCHANGED so publish_refined emits it identically.
    return msg
