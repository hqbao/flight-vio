"""``sky.vio`` -- the shared TIGHT-coupled visual-inertial odometry.

This is the ONE canonical home for the tight RGB-D VIO the ``vio`` process runs on
the ``--tight`` path. It used to live in ``vio/mathlib/{backend/vio_window,imu/imu}.py``;
``vio`` was the only consumer, so the two modules were RELOCATED here (S7 -- the
final consolidation step, taken once Phase 4 reached its OAK-D ceiling and the
tight code stopped churning) to make the ``vio`` process a thin IPC shell that
just calls into :mod:`sky.vio` (loose path) / :mod:`sky.vio` (tight path).

* :mod:`sky.vio.window` -- the Basalt-style tight window optimiser:
  :class:`~sky.vio.window.WindowedVIOMap` / :class:`~sky.vio.window.WindowedVIORGBDOdometry`,
  :func:`~sky.vio.window.optimize_vio`, the IMU residual, the shipped
  ``vel_cv_prior`` / ``vel_zupt`` velocity stabilisers (Phase-4), and the opt-in
  dense-ICP relative-pose factor (:class:`~sky.vio.window.IcpFactor` /
  ``icp_factor``). It puts reprojection + metric-depth + IMU-preintegration factors
  into ONE non-linear least-squares solve over the keyframe window (poses,
  velocities, biases, landmarks). This is the BA-window-independent tight solve.
* :mod:`sky.vio.imu` -- the TIGHT IMU SUPERSET. It carries the same loose
  preintegration core as :mod:`sky.imu.imu` PLUS the tight-only extensions the
  window optimiser + the live forward-propagation need:
  preintegration COVARIANCE (:class:`~sky.vio.imu.ImuNoise`,
  the 9x9 ``cov`` / ``sqrt_info`` propagated inside :func:`~sky.vio.imu.preintegrate_imu`),
  the per-frame dead-reckoning :func:`~sky.vio.imu.predict_state` + ZUPT gate
  :func:`~sky.vio.imu.imu_at_rest`, the complementary vision correction
  :func:`~sky.vio.imu.complementary_correct`, and the loop-closure SE(3) deltas
  (:func:`~sky.vio.imu.loop_correction_delta` / :func:`~sky.vio.imu.scale_se3_delta`
  / :func:`~sky.vio.imu.apply_se3_left`).

Reconciliation note (tight superset vs the loose :mod:`sky.imu.imu`)
-------------------------------------------------------------------
:mod:`sky.imu.imu` (moved in S4) is the LOOSE preintegration and feeds the
byte-parity oracle indirectly -- it is left byte-untouched. The tight
:mod:`sky.vio.imu` is a 707-line SUPERSET that DIVERGED: its
:func:`~sky.vio.imu.preintegrate_imu` interleaves the 9-state covariance recursion
(``A_k``/``B_k``/``Q/dt``) INSIDE the same per-segment loop as the delta update, so
the loose core is NOT extractable cleanly without re-ordering the float arithmetic
(which would risk the tight numerics). Per the consolidation rule
"*correctness + the tight-numerics gate over DRY*", the superset was moved WHOLE
into :mod:`sky.vio.imu` as a documented tight variant. The overlap with
:mod:`sky.imu.imu` (the loose ``ImuPreintegration`` / ``preintegrate_imu`` /
``GyroPreintegrator`` / ``integrate_gyro_camera`` / ``gravity_aligned_R0``) is
deliberate and accepted; the byte-identical members are why the oracle stays
``gap = 0`` after repointing its imports from ``vio.mathlib.imu`` to here.

Every module imports only ``numpy`` + other :mod:`sky.*` (``sky.math`` Lie
helpers, ``sky.front`` front-end + odometry, ``sky.depth.icp``) -- no process /
comms / io module -- so the package stays a leaf and movable (maps onto the C
``libskyvio`` layer in ``docs/C_PORT_PLAN.md``). The process-level glue (the
engine / pipeline / ``propagate_imu`` step) stays in the ``vio`` process and calls
into here.
"""
