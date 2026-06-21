"""``lidar.tools`` -- bench / bring-up tools for the rangefinder (I2C).

* :mod:`lidar.tools.characterize` -- ``--characterize``: stream the VL53L1X dist +
  range_status + signal over I2C and, when the rig is on the ground, print the
  recommended FC ``disarm_range`` (measured ground floor + margin).
"""
