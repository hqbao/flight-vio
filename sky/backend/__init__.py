"""``sky.backend`` -- the shared optimisation backend (bundle adjustment core).

This is the home for the project-agnostic *optimiser engines* the VIO / SLAM
backends call into: the canonical sliding-window bundle adjustment solver, the
loose sliding-window map that drives it, and its keyframe marginalization.

* :mod:`sky.backend.bundle` -- :func:`~sky.backend.bundle.optimize`, the
  Levenberg-Marquardt + Schur-complement BA core, plus its
  :class:`~sky.backend.bundle.BAConfig` / :class:`~sky.backend.bundle.BAResult`.
  The solver is FACTOR-AGNOSTIC: reprojection, depth, gravity-leveling,
  marginalization-prior and VO-relative factors are all passed in as arrays.
  WHICH factors get assembled (VIO reprojection + IMU/gravity, etc.) is decided
  by the CALLER -- the sliding-window glue in :mod:`sky.backend.windowed`
  builds them. That is why the core deduped cleanly: ``vio`` and ``slam`` shipped
  token-identical copies of this file (``slam``'s was in fact dead code -- nothing
  in ``slam`` imported it; ``slam`` loop closure runs its own pose-graph in
  :mod:`sky.slam.posegraph`), and the only behavioural divergence between
  VIO BA and SLAM loop-closure BA lives in those callers, never here.
* :mod:`sky.backend.windowed` -- :class:`~sky.backend.windowed.WindowedBAMap` /
  :class:`~sky.backend.windowed.WindowedRGBDOdometry`, the LOOSE sliding-window
  keyframe map: it runs frame-to-frame PnP (:mod:`sky.front.odometry`), inserts
  keyframes, builds the reprojection/depth/VO-prior factors and feeds them to
  :func:`sky.backend.bundle.optimize`. Single-copy in ``vio`` (VIO was the only
  consumer); RELOCATED here so the VIO process holds no BA-map math (R3).
* :mod:`sky.backend.marginalize` -- :func:`~sky.backend.marginalize.marginalize_keyframe`
  + :class:`~sky.backend.marginalize.MargPrior`, the Schur marginalization of a
  dropped keyframe into a pose prior over the survivors (carries gauge / yaw /
  scale forward). Consumed only by :mod:`sky.backend.windowed`; relocated with it.

It imports only :mod:`sky.math` (SE(3) ``se3_exp`` / ``se3_log`` / ``skew``),
:mod:`sky.front` (the windowed map's VO front-end) and ``numpy`` -- no process /
comms / io module -- so it stays a leaf and movable (maps onto the C ``libskyba``
/ backend layer in ``docs/C_PORT_PLAN.md``).

NOTE -- the tight variant: the tight-coupled VIO window optimiser (formerly
``vio.mathlib.backend.vio_window``) is the SEPARATE :mod:`sky.vio.window` package,
NOT part of this loose backend. It was consolidated into :mod:`sky.vio` in S7 once
Phase 4 reached its OAK-D ceiling; the loose sliding-window map here and the tight
window solver there share the factor-agnostic :func:`sky.backend.bundle.optimize`
core but build different factors (see ``docs/CONSOLIDATION_PLAN.md``).
"""
