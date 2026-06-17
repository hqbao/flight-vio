"""Windowed bundle adjustment over the ``keyframe`` stream.

The back-end half of the old in-VIO ``vio.modules.backend`` / ``vio.modules.pipeline``
pair, extracted into the ``ba`` process. ``ba`` is a pure CONSUMER of keyframes:
``emit_keyframe`` stays in ``vio`` (it rides vio's odometry thread and produces the
``keyframe`` carrier), and ``ba`` ingests the resulting :class:`~ba.comms.messages.Keyframe`
over IPC and runs the sliding-window solve.

* :func:`run_ba` -- submit a keyframe's snapshot to the swappable
  :class:`~ba.engine.base.Engine` and forward any refined pose. The ``ba`` process
  uses the in-process engine (it IS its own process), so the solve runs
  synchronously in-thread per keyframe.

Lifted verbatim from ``vio.modules.backend.run_ba`` so the refined-pose output (and
the offline byte-parity argument that depends on the SAME frozen solve) is
unchanged; only the package the comms / engine come from moved (``vio`` -> ``ba``).
"""
from __future__ import annotations

import numpy as np

from ba.comms.messages import Keyframe, PoseMsg
from ba.engine import Engine


def run_ba(engine: Engine, tight: bool, kf: Keyframe):
    """Submit the keyframe's snapshot to the BA engine; return the refined pose.

    Was ``RunBA(Step)``; the engine + the ``tight`` snapshot selector are passed
    explicitly. Returns ``(PoseMsg, backend_state)`` -- the refined pose + the TIGHT
    backend's latest optimised ``(bg, ba)`` for the live feed-forward
    (``backend_state`` is ``None`` on the loose path) -- or bare ``None`` (chain
    short-circuit) when the keyframe has no tracks or the engine has no refined pose
    yet.
    """
    if kf.track_ids is None or kf.track_px is None:
        return None
    T_cw = np.linalg.inv(kf.T_world_cam)
    # Submit the snapshot shaped for whichever backend the worker built.
    # LOOSE (default): the historical 5-tuple ``ba_step`` consumes -- the
    # keyframe's at-rest gravity accel. TIGHT (``--tight``): the SUPERSET
    # 6-tuple ``vio_step`` consumes -- the keyframe timestamp + the raw
    # inter-keyframe IMU block (camera optical frame) for preintegration.
    if tight:
        engine.submit((T_cw, kf.track_ids, kf.track_px, kf.depth_m,
                       kf.ts_ns, kf.imu_seg))
    else:
        engine.submit((T_cw, kf.track_ids, kf.track_px, kf.depth_m,
                       kf.accel))
    post = engine.poll()                     # refined latest T_cw, or None
    if post is None:
        return None
    # The TIGHT step (``vio_step``) returns ``(T_cw, health)`` so the divergence
    # guard's verdict reaches the published pose; the LOOSE step (``ba_step``)
    # returns the bare ``T_cw``. Merge the tight health fields (``vio_degraded``
    # etc.) into the SAME info dict the FC already reads (alongside ``refined``
    # and the ``pos_sigma_m`` position-noise field), so a downstream / FC consumer
    # sees "estimator degraded this keyframe" next to the pose it acts on. On the
    # loose path ``post`` is a bare array -> ``info`` stays ``{"refined": True}``
    # exactly as before (the ``vio_degraded`` key is simply absent).
    info = {"refined": True}
    backend_state = None
    if isinstance(post, tuple):
        # TIGHT ``vio_step`` returns (T_cw, health, backend_bias); the LOOSE
        # ``ba_step`` returns a bare array. Unpack defensively (a 2-tuple still
        # works). ``backend_state`` = the latest keyframe's optimised (bg, ba)
        # for the live feed-forward (PLAN P1) -- None on the loose path.
        backend_state = post[2] if len(post) >= 3 else None
        post, health = post[0], post[1]
        info.update(health)
    return PoseMsg(kf.seq, 0, np.linalg.inv(post), info), backend_state
