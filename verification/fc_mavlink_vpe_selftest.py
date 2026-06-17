#!/usr/bin/env python3
"""Byte-correctness selftest for the self-owned MAVLink v2 VPE packer.

The packer (``sky/fc/mavlink_vpe.py``) ships in the flight runtime WITHOUT
pymavlink, so its correctness is anchored three independent ways here:

  1. CRC algorithm vs the published CRC-16/MCRF4XX check vector
     (``"123456789" -> 0x6F91``) -- proves the "x25" accumulate is right.
  2. A self-contained MAVLink v2 frame parser (written here, independent of the
     packer) round-trips every field, including the v2 trailing-zero truncation /
     zero-pad and the CRC_EXTRA mix-in.
  3. GOLD: if pymavlink is importable (dev-only, NOT a flight dep), feed the
     self-made bytes to its canonical parser -- which verifies framing + CRC +
     CRC_EXTRA + field order against the reference. SKIPPED (loudly) if absent.

  .venv/bin/python -m verification.fc_mavlink_vpe_selftest
"""
from __future__ import annotations

import math
import struct
import sys

from sky.fc import mavlink_vpe as vpe


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


def _f32(v: float) -> float:
    """The float32 value the wire carries for a python float input."""
    return struct.unpack("<f", struct.pack("<f", v))[0]


def _parse_v2(frame: bytes):
    """Independent MAVLink v2 parser -> (msgid, seq, sysid, compid, payload117).

    Verifies STX, recomputes the CRC (with CRC_EXTRA), zero-pads the truncated
    payload back to the full length. Raises on any inconsistency.
    """
    if frame[0] != vpe.STX_V2:
        raise ValueError("bad STX")
    plen = frame[1]
    incompat, _, seq, sysid, compid = frame[2], frame[3], frame[4], frame[5], frame[6]
    if incompat != 0:
        raise ValueError("unexpected incompat flags (no signing here)")
    msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
    payload = frame[10:10 + plen]
    crc_lo, crc_hi = frame[10 + plen], frame[11 + plen]
    got_crc = crc_lo | (crc_hi << 8)
    crc = vpe.x25_crc(frame[1:10 + plen])
    crc = vpe.x25_crc(bytes((vpe.CRC_EXTRA_VPE,)), crc)
    if crc != got_crc:
        raise ValueError("CRC mismatch (%#06x != %#06x)" % (crc, got_crc))
    payload = payload + b"\x00" * (vpe.PAYLOAD_LEN_VPE - len(payload))  # v2 zero-pad
    return msgid, seq, sysid, compid, payload


def main() -> int:
    print("[1] CRC-16/MCRF4XX check vector")
    _check(vpe.x25_crc(b"123456789") == 0x6F91,
           'x25_crc("123456789") == 0x6F91')

    print("[2] self-parse round-trip (all fields, with a non-zero reset_counter)")
    usec = 1234567890123456
    pos = (1.5, -2.25, 0.125)
    rpy = (0.1, -0.2, 3.0)
    cov = vpe.pose_covariance_from_sigma(0.07)          # pos sigma 7 cm
    rc = 5
    frame = vpe.pack_vision_position_estimate(
        usec, *pos, *rpy, covariance=cov, reset_counter=rc, seq=42, sysid=1, compid=197)
    _check(frame[0] == 0xFD, "STX is 0xFD (MAVLink v2)")
    _check((frame[7] | (frame[8] << 8) | (frame[9] << 16)) == 102, "msgid == 102")
    msgid, seq, sysid, compid, payload = _parse_v2(frame)
    _check(seq == 42 and sysid == 1 and compid == 197, "seq/sysid/compid preserved")
    f = struct.Struct("<Q6f21fB").unpack(payload)
    _check(f[0] == usec, "usec exact")
    _check(f[1:4] == tuple(_f32(v) for v in pos), "x/y/z == float32(input)")
    _check(f[4:7] == tuple(_f32(v) for v in rpy), "roll/pitch/yaw == float32(input)")
    _check(f[7] == _f32(0.07 ** 2) and f[13] == _f32(0.07 ** 2)
           and f[18] == _f32(0.07 ** 2), "pos variance sigma^2 at cov idx 0/6/11")
    _check(f[28] == rc, "reset_counter trailing byte preserved")

    print("[3] covariance=None -> cov[0]=NaN sentinel + trailing-zero truncation")
    f0 = vpe.pack_vision_position_estimate(1, 0, 0, 0, 0, 0, 0,
                                           covariance=None, reset_counter=0)
    # all-zero pose + NaN cov[0] + rc 0: everything after cov[0] is zero -> truncated.
    plen0 = f0[1]
    _check(plen0 < vpe.PAYLOAD_LEN_VPE, "v2 truncates trailing zeros (len %d < 117)" % plen0)
    _, _, _, _, pay0 = _parse_v2(f0)
    g = struct.Struct("<Q6f21fB").unpack(pay0)
    _check(math.isnan(g[7]), "cov[0] is NaN (unknown-covariance sentinel)")
    _check(g[28] == 0, "zero-padded reset_counter recovers 0")

    print("[4] GOLD cross-check via pymavlink (dev-only; SKIP if absent)")
    try:
        from pymavlink.dialects.v20 import common as mav2
    except Exception as e:  # noqa: BLE001
        print("  SKIP -- pymavlink not importable (%s). `pip install pymavlink`"
              " as a DEV dep to run the canonical cross-check." % type(e).__name__)
    else:
        link = mav2.MAVLink(None, srcSystem=1, srcComponent=197)
        decoded = []
        for byte in frame:
            m = link.parse_char(bytes((byte,)))
            if m is not None:
                decoded.append(m)
        _check(len(decoded) == 1 and decoded[0].get_type() == "VISION_POSITION_ESTIMATE",
               "pymavlink accepts our bytes as VISION_POSITION_ESTIMATE (CRC+CRC_EXTRA ok)")
        d = decoded[0]
        _check(d.usec == usec and d.reset_counter == rc
               and d.x == _f32(pos[0]) and d.yaw == _f32(rpy[2]),
               "pymavlink-decoded fields match")

    print("\nPASS -- self-owned MAVLink v2 VPE packer is byte-correct "
          "(CRC anchor + independent round-trip + truncation).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
