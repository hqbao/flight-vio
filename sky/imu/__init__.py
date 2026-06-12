"""``sky.imu`` -- the shared inertial layer (loose preint + buffer + DR filter).

This is the ONE canonical home for the device-free inertial math the acquisition
/ VIO pipelines compose.

* :mod:`sky.imu.imu` -- the LOOSE preintegration:
  :func:`~sky.imu.imu.preintegrate_imu` (on-manifold rotation/velocity/position
  preintegration, Forster et al. TRO 2017) + :class:`~sky.imu.imu.GyroPreintegrator`
  / :func:`~sky.imu.imu.integrate_gyro_camera` (the cheap gyro-only rotation prior
  used to seed PnP) + :func:`~sky.imu.imu.gravity_aligned_R0` (level the first
  frame from accel). It used to be vendored byte-identically in
  ``imu_camera/mathlib/imu/imu.py`` and ``slam/mathlib/imu/imu.py``; both copies
  were byte-for-byte identical, so consolidating to one import deduped it (S4).
* :mod:`sky.imu.timed_buffer` -- :class:`~sky.imu.timed_buffer.TimedImuBuffer`,
  the thread-safe ring buffer of timestamped IMU samples the acquisition pipeline
  drains per camera trigger (so each frame carries the IMU interval that bridges
  it to the previous one). Single-copy in ``imu_camera``; relocated here (R6).
* :mod:`sky.imu.inertial_filter` -- :class:`~sky.imu.inertial_filter.InertialTranslationFilter`
  + :class:`~sky.imu.inertial_filter.InertialFilterConfig`, the accel-driven
  dead-reckoning translation filter. Single-copy in ``imu_camera``; relocated here
  (R6).

It imports only :mod:`sky.math` (SO(3) exp / right-Jacobian / skew), ``numpy`` and
the standard library (``threading`` / ``time`` / ``collections`` for the buffer)
-- no process / comms / io module -- so it stays a leaf and movable (maps onto the
C ``libskyimu`` layer in ``docs/C_PORT_PLAN.md``).

NOTE -- the tight variant: :mod:`sky.vio.imu` is a SUPERSET of this loose copy (it
adds the preintegration COVARIANCE + the tight-only forward-propagation / loop /
complementary-correction machinery for the tight-coupled VIO window optimiser).
Because that superset's ``preintegrate_imu`` interleaves the covariance recursion
INSIDE the same per-segment loop as the delta update, the loose core could not be
extracted cleanly without re-ordering the float arithmetic, so the superset was
moved WHOLE into :mod:`sky.vio.imu` as a documented tight VARIANT (S7) -- this
loose copy stays the byte-untouched oracle-feeding base. The two share the
byte-identical loose members (``GyroPreintegrator`` / ``integrate_gyro_camera`` /
``gravity_aligned_R0`` / the loose ``ImuPreintegration`` core) by design (see
``docs/CONSOLIDATION_PLAN.md``).
"""
