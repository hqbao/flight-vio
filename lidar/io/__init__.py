"""``lidar.io`` -- the I2C rangefinder reader (real device + a host MOCK).

A single small interface (:class:`lidar.io.vl53l1x_reader.RangeReader`) with two
implementations:

* :class:`~lidar.io.vl53l1x_reader.VL53L1XReader` -- the real I2C reader (lazy
  ``pimoroni-vl53l1x`` + ``smbus2`` import, only the flight Pi needs them).
* :class:`~lidar.io.vl53l1x_reader.MockRangeReader` -- a deterministic, hardware-
  free reader for host tests (no I2C bus, no device).

The reader is deliberately SWAPPABLE because the TOF400F's exact I2C
address/mode is HIL-unknown until the rig is on the bench -- the rest of the
``lidar`` process depends only on the :class:`RangeReader` interface.
"""
