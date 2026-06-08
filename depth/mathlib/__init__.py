"""``depth.mathlib`` -- the math this project OWNS (the SGM stereo source of truth).

depth is the SOURCE-OF-TRUTH for the stereo math: :mod:`depth.mathlib.stereo` is
the canonical from-scratch SGM dense-stereo matcher + rectifiers (numba). The
capture project (:mod:`imu_camera`) vendors a BYTE-IDENTICAL copy at
``imu_camera/mathlib/stereo`` because depth runs INLINE on the capture process's
``imu_cam`` thread today; a ``diff -r depth/mathlib/stereo
imu_camera/mathlib/stereo`` gate keeps the two copies in lock-step.

* :mod:`~depth.mathlib.stereo` -- :class:`~depth.mathlib.stereo.stereo.SGMStereoMatcher`
  + ``SGMConfig`` (semi-global block matching with built-in left/right
  rectification) and the sparse :class:`~depth.mathlib.stereo.stereo.StereoMatcher`
  used by the self-test.
"""
