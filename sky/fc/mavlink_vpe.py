"""Self-owned MAVLink v2 ``VISION_POSITION_ESTIMATE`` (#102) serializer.

Flight-runtime, dependency-free (stdlib ``struct`` only) packer for the ONE MAVLink
message the VIO->FC link needs. We deliberately do NOT pull pymavlink into the
flight runtime -- it keeps the lean Pi image and the project's self-owned ethos,
and this module maps 1:1 onto the roadmap's future C ``fc_link_mavlink.c``.
pymavlink is used ONLY as a dev-time GOLD cross-check in the selftest, never here.

Wire format = MAVLink v2 (STX ``0xFD``). ``VISION_POSITION_ESTIMATE`` payload, in
MAVLink's size-descending field order (the on-wire order, NOT the XML declaration
order -- here they coincide because the only sub-4-byte field is the trailing u8)::

    uint64 usec | float x, y, z, roll, pitch, yaw | float covariance[21] | uint8 reset_counter

- ``covariance`` is the row-major UPPER-triangular of the 6x6 pose covariance
  (order x, y, z, roll, pitch, yaw). ``covariance[0] = NaN`` is the MAVLink
  sentinel for "covariance unknown". Position variance sigma**2 goes at indices
  0 / 6 / 11 (the x/y/z diagonal).
- ``reset_counter`` MUST be bumped on a SLAM loop-closure JUMP so the FC ESKF
  resets its origin instead of fusing the discontinuity (a fused jump injects
  phantom velocity). It is the trailing byte.

CRC = CRC-16/MCRF4XX (MAVLink's "x25") over ``[len .. end-of-payload]`` then the
message ``CRC_EXTRA`` (158 for #102), appended little-endian. The payload is
trailing-zero-truncated per MAVLink v2 (the receiver zero-pads back to 117).

This module is a LEAF: stdlib only, imports nothing from the rest of the tree.
"""
from __future__ import annotations

import math
import struct

# MAVLink v2 framing + message constants ------------------------------------- #
STX_V2 = 0xFD
MSGID_VISION_POSITION_ESTIMATE = 102
#: ``CRC_EXTRA`` for VISION_POSITION_ESTIMATE, from the MAVLink ``common`` dialect.
#: The selftest's pymavlink GOLD cross-check is the authoritative confirmation.
CRC_EXTRA_VPE = 158
#: Full (untruncated) #102 payload size: 8 + 6*4 + 21*4 + 1.
PAYLOAD_LEN_VPE = 117
#: Standard companion-computer component id (MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY).
COMP_ID_VIO = 197

_PAYLOAD_STRUCT = struct.Struct("<Q6f21fB")  # usec, x/y/z/roll/pitch/yaw, cov[21], reset


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX (MAVLink "x25"): poly 0x1021 reflected, init 0xFFFF, no xorout.

    Independent check vector: ``x25_crc(b"123456789") == 0x6F91``.
    """
    for byte in data:
        tmp = (byte ^ crc) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_vision_position_estimate(
    usec: int,
    x: float, y: float, z: float,
    roll: float, pitch: float, yaw: float,
    covariance=None,
    reset_counter: int = 0,
    *,
    seq: int = 0,
    sysid: int = 1,
    compid: int = COMP_ID_VIO,
) -> bytes:
    """Serialize one VISION_POSITION_ESTIMATE into a complete MAVLink v2 frame.

    Args:
        usec: timestamp, microseconds (monotonic boot time the FC can align to).
        x, y, z: position in the FC frame (NED), metres. The caller is responsible
            for the WORLD(optical)->NED rotation -- this packer is frame-agnostic.
        roll, pitch, yaw: attitude in the FC frame, radians.
        covariance: 21 floats (upper-tri 6x6 pose cov), or None -> ``cov[0]=NaN``
            ("unknown"). Use :func:`pose_covariance_from_sigma` to fill the position
            diagonal from a scalar sigma.
        reset_counter: 0..255; bump on a loop-closure jump (see module docstring).
        seq: MAVLink sequence byte (wraps mod 256); the sender increments it.
        sysid, compid: MAVLink source ids.

    Returns:
        The framed packet bytes (STX .. CRC), ready to write to the UART.
    """
    if covariance is None:
        cov = [math.nan] + [0.0] * 20
    else:
        cov = [float(c) for c in covariance]
        if len(cov) != 21:
            raise ValueError("covariance must have exactly 21 elements")

    payload = _PAYLOAD_STRUCT.pack(
        int(usec) & 0xFFFFFFFFFFFFFFFF,
        float(x), float(y), float(z),
        float(roll), float(pitch), float(yaw),
        *cov,
        int(reset_counter) & 0xFF,
    )
    # MAVLink v2 trailing-zero truncation (payload length never below 1).
    plen = len(payload)
    while plen > 1 and payload[plen - 1] == 0:
        plen -= 1
    payload = payload[:plen]

    # header (after STX): len, incompat_flags, compat_flags, seq, sysid, compid
    header = struct.pack("<BBBBBB", plen, 0, 0, seq & 0xFF, sysid & 0xFF, compid & 0xFF)
    msgid3 = struct.pack("<I", MSGID_VISION_POSITION_ESTIMATE)[:3]  # 24-bit LE
    frame_wo_crc = header + msgid3 + payload

    crc = x25_crc(frame_wo_crc)
    crc = x25_crc(bytes((CRC_EXTRA_VPE,)), crc)  # MAVLink mixes in CRC_EXTRA
    return bytes((STX_V2,)) + frame_wo_crc + struct.pack("<H", crc)


def pose_covariance_from_sigma(pos_sigma_m, ang_sigma_rad=None):
    """Build the 21-float upper-tri pose covariance from scalar sigmas.

    Position variance ``pos_sigma_m**2`` on the x/y/z diagonal (cov idx 0/6/11).
    Attitude: if ``ang_sigma_rad`` is given, ``ang_sigma_rad**2`` on the
    roll/pitch/yaw diagonal (idx 15/18/20); otherwise those stay 0.0. NOTE: the
    correct attitude-covariance semantics for a position-only VIO fix is a
    math/safety-reviewer question (0 means "perfectly certain", which is wrong);
    this helper just exposes the knob. Off-diagonals are 0 (axis-independent).
    """
    cov = [0.0] * 21
    var_p = float(pos_sigma_m) ** 2
    cov[0] = var_p   # (x, x)
    cov[6] = var_p   # (y, y)
    cov[11] = var_p  # (z, z)
    if ang_sigma_rad is not None:
        var_a = float(ang_sigma_rad) ** 2
        cov[15] = var_a  # (roll, roll)
        cov[18] = var_a  # (pitch, pitch)
        cov[20] = var_a  # (yaw, yaw)
    return cov
