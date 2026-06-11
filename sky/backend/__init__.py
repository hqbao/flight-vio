"""``sky.backend`` -- the shared optimisation backend (bundle adjustment core).

This is the home for the project-agnostic *optimiser engines* the VIO / SLAM
backends call into. So far it holds the ONE canonical sliding-window bundle
adjustment solver.

* :mod:`sky.backend.bundle` -- :func:`~sky.backend.bundle.optimize`, the
  Levenberg-Marquardt + Schur-complement BA core, plus its
  :class:`~sky.backend.bundle.BAConfig` / :class:`~sky.backend.bundle.BAResult`.
  The solver is FACTOR-AGNOSTIC: reprojection, depth, gravity-leveling,
  marginalization-prior and VO-relative factors are all passed in as arrays.
  WHICH factors get assembled (VIO reprojection + IMU/gravity, etc.) is decided
  by the CALLER -- the sliding-window glue in :mod:`vio.mathlib.backend.windowed`
  builds them. That is why the core deduped cleanly: ``vio`` and ``slam`` shipped
  token-identical copies of this file (``slam``'s was in fact dead code -- nothing
  in ``slam`` imported it; ``slam`` loop closure runs its own pose-graph in
  :mod:`slam.mathlib.loop.posegraph`), and the only behavioural divergence between
  VIO BA and SLAM loop-closure BA lives in those callers, never here.

It imports only :mod:`sky.math` (SE(3) ``se3_exp`` / ``se3_log`` / ``skew``) and
``numpy`` -- no process / comms / io module -- so it stays a leaf and movable
(maps onto the C ``libskyba`` / backend layer in ``docs/C_PORT_PLAN.md``).

NOTE -- variant deferral: ``vio.mathlib.backend.vio_window`` (the tight-coupled
Phase-4 VIO window optimiser) and the sliding-window glue / marginalization
(``windowed.py`` / ``marginalize.py``) are NOT consolidated here yet; they are
either the live research surface or scheduled for a later step (see
``docs/CONSOLIDATION_PLAN.md``).
"""
