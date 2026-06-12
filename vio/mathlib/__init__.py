"""``vio.mathlib`` -- the math the VIO project OWNS.

Ported VERBATIM from ``ours.lib.{frontend,odometry,backend,engine,imu}`` (only the
cross-package import roots + the doc cross-references were re-rooted at
``vio.mathlib`` / ``vio.comms``; no algorithm changed, so the numerical output is
byte-identical to the reference oracle -- proved by
:mod:`vio.tests.vio_ba_selftest`).

After the consolidation (``docs/CONSOLIDATION_PLAN.md``) almost all the VIO
algorithm code has been relocated into the shared :mod:`sky` leaf library; what
remains here is the per-project glue that the C IPC port discards (the engine) plus
the math-coupled config builders. The relocated algorithm lives in:

* :mod:`sky.front` -- the from-scratch KLT optical-flow tracker + Shi-Tomasi
  corner detector + RGB-D PnP visual odometry (R1/R2).
* :mod:`sky.backend` -- the LOOSE sliding-window map + marginalization + the
  factor-agnostic BA core (R3 / S5).
* :mod:`sky.vio` -- the TIGHT-coupled visual-inertial window optimiser
  (``sky.vio.window``, formerly ``vio_window.py``) + the tight IMU SUPERSET
  (``sky.vio.imu``, the covariance + forward-propagation + loop machinery),
  relocated in S7 once Phase 4 reached its OAK-D ceiling.
* :mod:`sky.imu` -- the LOOSE IMU preintegration + gyro prior the oracle path uses
  (S4).

What still lives under ``vio.mathlib``:

* :mod:`~vio.mathlib.engine` -- the swappable in-process / subprocess runners for
  the heavy keyframe optimisers (VIO carries its OWN engine copy; S6 left
  per-project as a Python-GIL artifact the C port replaces wholesale).

The ARCHITECTURE RULE lives here too: the math-coupled config builders
(:mod:`~vio.mathlib.resolution_build`) and the JIT warmup
(:mod:`~vio.mathlib.warmup`) live in ``mathlib`` -- NOT in the vendored, generic,
bit-identical :mod:`vio.comms` -- because they import VIO's own math.
"""
