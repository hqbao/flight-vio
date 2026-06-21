"""``lidar`` -- the downward-rangefinder PROJECT (VL53L1X over I2C -> lidar.range).

A STANDALONE, independently-runnable source tree (its own vendored ``comms``
copy, like the other split projects). It reads a downward-facing VL53L1X
time-of-flight rangefinder over I2C and publishes each gated reading as a
:class:`~lidar.comms.wire.WireRange` POD on the ``lidar.range`` IPC topic, served
on its own ``oak.lidar`` endpoint.

The range is consumed by the ``fc`` UART sender, which keeps the freshest reading
and BUNDLES it into the dblink VIO-pose frame (``sky.fc.dblink``: the trailing
``range_m`` + the ``range_valid`` flag bit) -- it is NOT a separate dblink message.

Layers
------
* :mod:`lidar.comms` -- the FROZEN vendored comms contract (byte-identical to the
  other copies; this project only consumes its client/server API).
* :mod:`lidar.io` -- the swappable I2C reader: :class:`~lidar.io.vl53l1x_reader.
  VL53L1XReader` (real ``pimoroni-vl53l1x`` + ``smbus2``) and
  :class:`~lidar.io.vl53l1x_reader.MockRangeReader` (hardware-free, for host tests).
* :mod:`lidar.main` -- the standalone lidar process: read loop -> publish
  ``lidar.range`` on an :class:`~lidar.comms.IPCPubSub` server.
* :mod:`lidar.tools.characterize` -- an I2C bench tool: stream dist + range_status
  + signal and, on the ground, print the recommended FC ``disarm_range``.

cv2-free: only ``smbus2`` + ``pimoroni-vl53l1x`` (both pure-Python) -- nothing
here imports OpenCV, so the lean Pi flight image stays clean.
"""
