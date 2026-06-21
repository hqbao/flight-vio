"""sky.fc -- flight-controller link primitives (self-owned, portable).

Leaf package: numpy + :mod:`sky.math` only (no third-party runtime dependency, no
pymavlink), so it maps 1:1 onto the roadmap's future C ``fc_link_dblink.c`` and
keeps the lean Pi flight image. Contents:

* :mod:`dblink` -- the self-owned ``dblink`` serializer for the FC's in-house wire
  protocol: ``build_db_frame`` + the ``DB_CMD_VIO_POSE`` VIO-pose packer
  (stdlib ``struct`` only).
* :mod:`fc_earth_pose` -- the PURE optical-world ``T_world_cam`` -> NED earth-pose
  conversion (the SSOT shared by the UI viewer and the ``fc`` UART sender).
"""
