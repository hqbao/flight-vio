"""sky.fc -- flight-controller link primitives (self-owned, portable).

Leaf package: stdlib-only, no third-party runtime dependency (no pymavlink), so it
maps 1:1 onto the roadmap's future C ``fc_link_mavlink.c`` and keeps the lean Pi
flight image. The single message the VIO->FC link needs lives in :mod:`mavlink_vpe`.
"""
