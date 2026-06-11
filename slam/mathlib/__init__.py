"""``slam.mathlib`` -- the math the SLAM project OWNS.

Ported VERBATIM from ``ours.lib.{loop,engine}`` plus the FORCED dependencies of
the loop-closure import graph (only the cross-package import roots + the doc
cross-references were re-rooted at ``slam.mathlib`` / ``slam.comms``; no algorithm
changed, so the numerical output is byte-identical to the reference oracle --
proved by :mod:`slam.tests.loop_closure_selftest`).

* :mod:`sky.slam` -- the loop-closure frontend + backend (RELOCATED into the
  shared :mod:`sky` leaf library; SLAM was the only consumer, R4): the
  from-scratch ORB detector/descriptor + Hamming matcher + fundamental-matrix
  RANSAC (:mod:`~sky.slam.orb`), the appearance + geometric loop detector
  (:mod:`~sky.slam.loopclosure`), the SE(3) pose-graph optimiser
  (:mod:`~sky.slam.posegraph`), and the persistent-keyframe SLAM map
  orchestrator (:mod:`~sky.slam.slam`: ``SlamMap`` / ``SlamConfig``).
* :mod:`~slam.mathlib.engine` -- the swappable in-process / subprocess runners for
  the heavy keyframe optimiser (SLAM carries its OWN engine copy).

SHARED-LIBRARY dependencies (the loop import graph reaches into ``sky.*``, the
one consolidated algorithm library):

* The SE(3) / SO(3) Lie-group helpers :mod:`~sky.slam.posegraph` needs
  come from the shared :mod:`sky.math` kernel (e.g. ``so3_log_robust``), NOT from
  any per-project ``backend`` copy.
* The PnP RANSAC that :mod:`~sky.slam.loopclosure`'s metric geometric
  verification needs is the shared :func:`sky.front.pnp.solve_pnp_ransac` (one
  canonical copy, deduped out of the old per-project ``mathlib/odometry/pnp.py``).
* The bundle-adjustment core is the shared :mod:`sky.backend.bundle`; SLAM's old
  vendored ``backend/bundle.py`` was DEAD (nothing in SLAM imported it -- loop
  closure runs the standalone pose-graph in :mod:`~sky.slam.posegraph`),
  so it was deleted outright when the core was consolidated.

The ARCHITECTURE RULE lives here too: the math-coupled config builder
(:mod:`~slam.mathlib.resolution_build`) lives in ``mathlib`` -- NOT in the
vendored, generic, bit-identical :mod:`slam.comms` -- because it imports SLAM's
own math (:class:`~sky.slam.loopclosure.LoopConfig`). SLAM has NO numba
JIT kernel (its ORB frontend is pure NumPy), so -- unlike VIO -- there is no
``warmup`` module to pre-compile.
"""
