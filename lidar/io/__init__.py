"""``lidar.io`` -- the I2C rangefinder reader (real device + a host MOCK).

A single small interface (:class:`lidar.io.vl53l1x_reader.RangeReader`) with two
implementations:

* :class:`~lidar.io.vl53l1x_reader.VL53L1XReader` -- the real I2C reader: a bare
  VL53L1X driven register-level with ``smbus2`` ONLY (lazy import, only the flight
  Pi needs it).
* :class:`~lidar.io.vl53l1x_reader.MockRangeReader` -- a deterministic, hardware-
  free reader for host tests (no I2C bus, no device).

The reader is deliberately SWAPPABLE (the rest of the ``lidar`` process depends
only on the :class:`RangeReader` interface), so the real device and the host mock
are interchangeable and a future sensor swap is contained here.
"""
