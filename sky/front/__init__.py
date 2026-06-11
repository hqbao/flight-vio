"""``sky.front`` -- the shared visual front-end (PnP + KLT tracker + corners).

This is the ONE canonical home for the from-scratch, library-free visual
front-end the VIO/SLAM pipeline needs.

* :mod:`sky.front.pnp` -- :func:`~sky.front.pnp.solve_pnp_ransac`, a pure-NumPy
  drop-in for the subset of ``cv2.solvePnPRansac`` the from-scratch VIO/SLAM need
  (RANSAC over minimal-point DLT hypotheses + robust Gauss-Newton refinement). It
  used to be vendored byte-identically in ``vio/mathlib/odometry/pnp.py`` and
  ``slam/mathlib/odometry/pnp.py``; both copies were byte-for-byte identical, so
  consolidating to one import removed the duplication outright (S2).
* :mod:`sky.front.frontend` -- :class:`~sky.front.frontend.KLTFrontend`, the
  persistent-id KLT optical-flow tracker that drives the odometry; built on
  :mod:`sky.front.klt` (pyramidal Lucas-Kanade, numba-accelerated via
  :mod:`sky.front.klt_numba`) and :mod:`sky.front.corners` (Shi-Tomasi
  good-features-to-track). These were single-copy in ``vio/mathlib/frontend/``
  (VIO was the only consumer) and were RELOCATED here so the VIO process stays a
  thin IPC shell (R1).

It imports only :mod:`sky.math` (the SO(3) exp helper), ``numpy`` and -- for the
KLT kernel -- ``numba`` (optional). No process / comms / io module is reachable,
so it stays a leaf and movable (maps onto the C ``libskyfront`` layer in
``docs/C_PORT_PLAN.md``).
"""
