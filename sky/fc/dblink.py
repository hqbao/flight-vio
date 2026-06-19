"""Self-owned ``dblink`` serializer for the VIO->FC vision-pose message.

``dblink`` is the in-house wire protocol the drone flight controller (FC) speaks
on its UART (sibling repo ``../flight-controller``). This is the flight-runtime,
dependency-free (stdlib :mod:`struct` only) packer for the ONE dblink message the
VIO->FC link emits: the vision pose. It maps 1:1 onto the roadmap's future C
``fc_link_dblink.c`` and keeps the lean Pi flight image (no pymavlink, no third-
party serializer).

Wire frame (host -> FC)
-----------------------
Every dblink frame is::

    'd' 'b' | CMD(1B) | CLASS(1B, =0x00) | LEN(2B LE) | payload | checksum(2B LE)

* ``CMD`` selects the message. The FC routes purely by this byte (``data[0]``
  after the ``'db'`` magic); see :data:`DB_CMD_VISION_POSE`.
* ``CLASS`` is the log/message class, fixed ``0x00`` for the host->FC commands.
* ``LEN`` is the payload byte count, little-endian.
* ``checksum`` = ``(cmd + class + len_lo + len_hi + sum(payload)) & 0xFFFF``,
  little-endian. The FC's ``db_reader.c`` now VERIFIES this dblink checksum and
  DROPS any frame that fails it (the same gate it already applied to UBX), so a
  correct checksum is REQUIRED for the FC to route the frame on the CMD byte and
  decode the payload. This packer emits the correct checksum, so the VIO pose
  always passes the gate; the framing + checksum here are VERBATIM the FC's own
  ``build_db_frame`` (``flight-controller/tools/_dblink.py``).

Vision-pose payload (38 bytes, little-endian ``struct '<8fIBB'``)
-----------------------------------------------------------------
=====  ============  =======================================================
off    field         meaning
=====  ============  =======================================================
0      pos_n         NED North position, f32 metres
4      pos_e         NED East  position, f32 metres
8      pos_d         NED Down  position, f32 metres
12     q_w           attitude quaternion (body->NED), Hamilton, w-first, unit
16     q_x
20     q_y
24     q_z
28     pos_sigma_m   1-sigma position noise, f32 m (the FC uses it as sqrt(R))
32     age_us        measurement age, u32 microseconds (capture->send elapsed)
36     reset_counter u8 -- bump on a pose discontinuity (re-lock / jump)
37     flags         u8 -- bit0 pos_valid, bit1 att_valid, bit2 degraded
=====  ============  =======================================================

The pose carries the FULL attitude quaternion, NOT a heading scalar: the FC
extracts heading (and roll/pitch) from it itself, which is gimbal-lock-free (a
scalar yaw is undefined near pitch = +/-90 deg). Heading is still RELATIVE -- the
optical world's gravity-aligned X axis defines "North" -- but that is the
quaternion's reference frame, not a property of the encoding.

This module is a LEAF: stdlib :mod:`struct` only, NO time / I/O / serial, imports
nothing from the rest of the tree -- so it is trivially testable and matches the
``sky.*`` import-lint.
"""
from __future__ import annotations

import math
import struct

# dblink framing + message constants ----------------------------------------- #
#: dblink frame magic ('d', 'b') -- the first two bytes of every frame.
DB_MAGIC = b"db"
#: Host->FC message class for the command frames (fixed 0x00).
DB_CLASS = 0x00
#: CMD byte for the vision-pose message. The FC routes on this byte; the FC
#: header owns the final value (proposed 0x0C, kept in sync with the FC).
DB_CMD_VISION_POSE = 0x0C

#: Vision-pose payload layout: pos_n/e/d, q_w/x/y/z, pos_sigma_m (8x f32),
#: age_us (u32), reset_counter (u8), flags (u8). Little-endian.
_PAYLOAD_STRUCT = struct.Struct("<8fIBB")
#: Vision-pose payload size in bytes (8*4 + 4 + 1 + 1 = 38).
VISION_POSE_LEN = _PAYLOAD_STRUCT.size

#: u32 / u8 ceilings: age_us clamps (saturates) to u32; reset_counter / flags are
#: MASKED to u8 (``& 0xFF`` -- a wrap, not a clamp; both fields are designed to wrap).
_U32_MAX = 0xFFFFFFFF
_U8_MAX = 0xFF

#: IEEE-754 binary32 finite ceiling (largest representable f32 magnitude). A float
#: of larger magnitude is NOT representable as f32 and makes ``struct '<f'`` raise
#: OverflowError -- so :func:`_safe_f32` clamps to +/- this.
_F32_MAX = 3.4028234663852886e38
#: "Unknown / untrusted" position sigma (m) substituted when a sigma field is non-
#: finite. Large + finite so the FC, using it as sqrt(R), down-weights the fix to
#: ~zero gain -- a non-finite sigma must NEVER reach the wire as NaN/inf.
_SIGMA_UNKNOWN = 1.0e4


def _safe_f32(x, default: float) -> float:
    """Coerce ``x`` to a finite float inside the f32 range -- NEVER NaN/inf/overflow.

    The wire is ``struct '<f'`` (IEEE-754 binary32). Two failure modes must be
    eliminated *here*, at the leaf, so the packer is authoritative and can never
    raise nor emit a poisoned float regardless of what the caller passes:

    * **Non-finite** (NaN / +/-inf): some FC float parsers choke on these and a NaN
      measurement silently corrupts an EKF -- so substitute ``default`` (a finite,
      caller-chosen sentinel: 0.0 for pos/quat, a large "unknown" sigma for sigma).
    * **Out of f32 range** (``|x| > _F32_MAX``, e.g. the codebase's known
      diverged/exploding poses at 1e300): ``struct.pack('<f', x)`` raises
      ``OverflowError`` -- so saturate the magnitude to ``+/- _F32_MAX``.

    Finite in-range values pass through unchanged (exact same f32 they would have
    packed to before). This is a pure, total function: it returns a float for every
    input and never raises.
    """
    xf = float(x)
    if not math.isfinite(xf):
        return float(default)
    if xf > _F32_MAX:
        return _F32_MAX
    if xf < -_F32_MAX:
        return -_F32_MAX
    return xf


def build_db_frame(cmd_id: int, payload: bytes) -> bytes:
    """Frame a dblink message: ``'db' | cmd | class | len | payload | checksum``.

    VERBATIM the FC's own ``build_db_frame`` (``flight-controller/tools/
    dblink_test.py``): the 6-byte header is ``struct '<2sBBH'`` (magic, cmd,
    class=0x00, len) and the trailing checksum is the little-endian 16-bit sum
    ``(cmd + class + len_lo + len_hi + sum(payload)) & 0xFFFF``.

    Args:
        cmd_id: the dblink CMD byte (the FC routes on it), e.g.
            :data:`DB_CMD_VISION_POSE`.
        payload: the message body bytes (already serialized).

    Returns:
        The complete framed packet, ready to write to the UART.
    """
    msg_class = DB_CLASS
    length = len(payload)
    header = struct.pack("<2sBBH", DB_MAGIC, cmd_id, msg_class, length)
    cksum = (cmd_id + msg_class + (length & 0xFF) + ((length >> 8) & 0xFF)
             + sum(payload)) & 0xFFFF
    return header + payload + struct.pack("<H", cksum)


def pack_vision_pose(
    pos_ned,
    q_wxyz,
    pos_sigma_m: float,
    age_us: int,
    reset_counter: int,
    flags: int,
) -> bytes:
    """Serialize one vision-pose into a complete dblink frame -- NEVER raises.

    Builds the 38-byte little-endian payload (see the module docstring) and wraps
    it with :func:`build_db_frame` under :data:`DB_CMD_VISION_POSE`. This packer is
    AUTHORITATIVE about wire well-formedness and is a total function -- it cannot
    raise and cannot put a poisoned value on the wire, regardless of the caller:

    * **Floats** are passed through :func:`_safe_f32`: a non-finite (NaN/inf) field
      becomes a finite sentinel (pos/quat -> 0.0; ``pos_sigma_m`` -> the large
      :data:`_SIGMA_UNKNOWN` so the FC down-weights to ~zero gain) and a magnitude
      beyond the f32 range (the codebase's known exploding poses, |x| > ~3.4e38) is
      saturated to ``+/- _F32_MAX`` -- so ``struct '<f'`` can never OverflowError and
      NaN/inf can never reach the wire. NaN/inf-handling here is a LAST-RESORT
      backstop; the caller (``fc.main.send_once``) is expected to detect a non-finite
      pose first and mark the frame explicitly INVALID (clear pos_valid, set degraded).
    * **age_us** SATURATES (clamps) to ``[0, 2**32-1]`` so a runaway age pins at the
      u32 max instead of wrapping.
    * **reset_counter / flags** are MASKED to ``[0, 255]`` (``& 0xFF``) -- by design a
      WRAP, not a clamp: reset_counter is a free-running mod-256 counter and flags is
      a bitfield, so masking the low byte is the intended behaviour.

    Args:
        pos_ned: NED position ``(north, east, down)`` in metres (any 3-sequence;
            the FC frame mapping is the caller's responsibility -- this packer is
            frame-agnostic). Non-finite components are zeroed (see above).
        q_wxyz: attitude quaternion ``(w, x, y, z)``, body->NED, Hamilton w-first,
            expected unit-norm. Passed straight from the SSOT -- NOT re-normalized
            or re-ordered here. Non-finite components are zeroed.
        pos_sigma_m: 1-sigma position noise in metres -- the FC uses it as the
            measurement ``sqrt(R)``. The caller MUST inflate this when the fix is
            degraded so the FC down-weights it; if it arrives non-finite the packer
            substitutes the large :data:`_SIGMA_UNKNOWN` (never NaN on the wire).
        age_us: measurement age in microseconds (capture -> send elapsed); the FC
            anchors it to its own clock. Saturates (clamps) to the u32 range.
        reset_counter: bumped on a pose discontinuity so the FC ESKF resets its
            origin instead of fusing the jump. Masked to u8 (``& 0xFF``, wraps).
        flags: bit0 pos_valid, bit1 att_valid, bit2 degraded. Masked to u8.

    Returns:
        The complete framed dblink packet (``'db' .. checksum``).
    """
    age_u32 = int(min(max(int(age_us), 0), _U32_MAX))
    rc_u8 = int(reset_counter) & _U8_MAX
    flags_u8 = int(flags) & _U8_MAX
    payload = _PAYLOAD_STRUCT.pack(
        _safe_f32(pos_ned[0], 0.0), _safe_f32(pos_ned[1], 0.0),
        _safe_f32(pos_ned[2], 0.0),
        _safe_f32(q_wxyz[0], 0.0), _safe_f32(q_wxyz[1], 0.0),
        _safe_f32(q_wxyz[2], 0.0), _safe_f32(q_wxyz[3], 0.0),
        _safe_f32(pos_sigma_m, _SIGMA_UNKNOWN),
        age_u32,
        rc_u8,
        flags_u8,
    )
    return build_db_frame(DB_CMD_VISION_POSE, payload)
