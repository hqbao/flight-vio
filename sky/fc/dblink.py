"""Self-owned ``dblink`` serializer for the VIO->FC pose message.

``dblink`` is the in-house wire protocol the drone flight controller (FC) speaks
on its UART (sibling repo ``../flight-controller``). This is the flight-runtime,
dependency-free (stdlib :mod:`struct` only) packer for the ONE dblink message the
VIO->FC link emits: the VIO pose. It maps 1:1 onto the roadmap's future C
``fc_link_dblink.c`` and keeps the lean Pi flight image (no pymavlink, no third-
party serializer).

Wire frame (host -> FC)
-----------------------
Every dblink frame is::

    'd' 'b' | CMD(1B) | CLASS(1B, =0x00) | LEN(2B LE) | payload | checksum(2B LE)

* ``CMD`` selects the message. The FC routes purely by this byte (``data[0]``
  after the ``'db'`` magic); see :data:`DB_CMD_VIO_POSE`.
* ``CLASS`` is the log/message class, fixed ``0x00`` for the host->FC commands.
* ``LEN`` is the payload byte count, little-endian.
* ``checksum`` = ``(cmd + class + len_lo + len_hi + sum(payload)) & 0xFFFF``,
  little-endian. The FC's ``db_reader.c`` now VERIFIES this dblink checksum and
  DROPS any frame that fails it (the same gate it already applied to UBX), so a
  correct checksum is REQUIRED for the FC to route the frame on the CMD byte and
  decode the payload. This packer emits the correct checksum, so the VIO pose
  always passes the gate; the framing + checksum here are VERBATIM the FC's own
  ``build_db_frame`` (``flight-controller/tools/_dblink.py``).

VIO-pose payload (42 bytes, little-endian ``struct '<8fIBBf'``)
---------------------------------------------------------------
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
37     flags         u8 -- bit0 pos_valid, bit1 att_valid, bit2 degraded,
                     bit3 range_valid (the downward range below is meaningful)
38     range_m       f32 m -- downward rangefinder (VL53L1X) range, BUNDLED into
                     this message (NOT a separate dblink frame). Meaningful ONLY
                     when flags bit3 (range_valid) is set; 0.0 otherwise.
=====  ============  =======================================================

The downward LIDAR range rides INSIDE this VIO-pose message (the FC reads it from
the same 42-byte payload, gated on the ``range_valid`` flag bit) -- there is no
separate range channel on the wire. The Pi's ``lidar`` process publishes the gated
range on the ``lidar.range`` IPC topic; the ``fc`` sender grabs the freshest one
each cadence tick and folds it (+ its validity) in here.

The VIO pose carries the FULL attitude quaternion, NOT a heading scalar: the FC
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
#: CMD byte for the VIO-pose message. The FC routes on this byte; the FC
#: header owns the final value (LOCKED 0x0C, kept in sync with the FC).
DB_CMD_VIO_POSE = 0x0C

#: VIO-pose payload layout: pos_n/e/d, q_w/x/y/z, pos_sigma_m (8x f32),
#: age_us (u32), reset_counter (u8), flags (u8), range_m (f32). Little-endian.
#: The trailing ``range_m`` carries the downward LIDAR range BUNDLED into this
#: message (NOT a separate frame); it is meaningful only when the flags
#: :data:`VIO_FLAG_RANGE_VALID` bit is set. LOCKED with the FC byte-for-byte.
_PAYLOAD_STRUCT = struct.Struct("<8fIBBf")
#: VIO-pose payload size in bytes (8*4 + 4 + 1 + 1 + 4 = 42).
VIO_LEN = _PAYLOAD_STRUCT.size

#: flags-byte bit set when the bundled ``range_m`` is valid (the sensor-side gate
#: passed). Matches the FC's ``VIO_FLAG_RANGE_VALID``. bit0 pos_valid, bit1
#: att_valid, bit2 degraded are owned by the caller (``fc.main``); this packer only
#: folds bit3 from ``range_valid`` so the range field can never look valid by
#: accident.
VIO_FLAG_RANGE_VALID = 0x08

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
            :data:`DB_CMD_VIO_POSE`.
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


def pack_vio_pose(
    pos_ned,
    q_wxyz,
    pos_sigma_m: float,
    age_us: int,
    reset_counter: int,
    flags: int,
    range_m: float = 0.0,
    range_valid: bool = False,
) -> bytes:
    """Serialize one VIO pose into a complete dblink frame -- NEVER raises.

    Builds the 42-byte little-endian payload (see the module docstring) and wraps
    it with :func:`build_db_frame` under :data:`DB_CMD_VIO_POSE`. This packer is
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
        flags: bit0 pos_valid, bit1 att_valid, bit2 degraded. Masked to u8. The
            :data:`VIO_FLAG_RANGE_VALID` bit (bit3) is OWNED by this packer -- it is
            derived from ``range_valid`` below and OR-ed in (any bit3 the caller set
            is overwritten), so the range field can never be advertised valid by
            accident.
        range_m: downward rangefinder (VL53L1X) range in METRES, BUNDLED into this
            VIO-pose message (NOT a separate frame). Written ONLY when ``range_valid``
            is true; when invalid the field is forced to 0.0 (and bit3 cleared) so a
            stale / rejected reading never reaches the FC as a live range. Non-finite
            -> 0.0 via :func:`_safe_f32`.
        range_valid: True iff the sensor-side gate passed (range_status == 0x09 and the
            distance is within the configured min/max). Sets/clears bit3 and gates
            whether ``range_m`` is written. Default False (no range source attached
            -- e.g. the lidar process is absent / ``--no-lidar``).

    Returns:
        The complete framed dblink packet (``'db' .. checksum``).
    """
    age_u32 = int(min(max(int(age_us), 0), _U32_MAX))
    rc_u8 = int(reset_counter) & _U8_MAX
    # bit3 (range_valid) is packer-owned: clear whatever the caller passed in that
    # position, then OR it back ONLY from range_valid -- so the FC can trust bit3 to
    # mean "the range field below is meaningful" regardless of caller hygiene.
    flags_u8 = (int(flags) & _U8_MAX) & ~VIO_FLAG_RANGE_VALID
    if range_valid:
        flags_u8 |= VIO_FLAG_RANGE_VALID
        range_out = _safe_f32(range_m, 0.0)
    else:
        # Invalid -> zero the field (defence in depth: the FC gates on bit3, but a
        # zeroed payload means a dropped flag can never expose a stale range value).
        range_out = 0.0
    payload = _PAYLOAD_STRUCT.pack(
        _safe_f32(pos_ned[0], 0.0), _safe_f32(pos_ned[1], 0.0),
        _safe_f32(pos_ned[2], 0.0),
        _safe_f32(q_wxyz[0], 0.0), _safe_f32(q_wxyz[1], 0.0),
        _safe_f32(q_wxyz[2], 0.0), _safe_f32(q_wxyz[3], 0.0),
        _safe_f32(pos_sigma_m, _SIGMA_UNKNOWN),
        age_u32,
        rc_u8,
        flags_u8,
        range_out,
    )
    return build_db_frame(DB_CMD_VIO_POSE, payload)
