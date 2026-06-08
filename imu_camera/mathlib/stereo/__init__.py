"""``imu_camera.mathlib.stereo`` -- the from-scratch SGM dense-stereo matcher.

A vendored copy of the depth project's stereo math (depth runs INLINE on the
``imu_cam`` module's thread, not as a separate process), so this copy must stay
byte-identical to ``depth/mathlib/stereo``.

* :class:`~imu_camera.mathlib.stereo.stereo.SGMStereoMatcher` + ``SGMConfig`` --
  semi-global block matching with built-in left/right rectification (numba).
* :class:`~imu_camera.mathlib.stereo.stereo.StereoMatcher` + ``StereoConfig`` --
  the sparse block matcher used by the stereo self-test.
"""
