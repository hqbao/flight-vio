#!/usr/bin/env python3
"""Byte-correctness selftest for the self-owned ``dblink`` VIO-pose packer.

The packer (``sky/fc/dblink.py``) ships in the flight runtime, so its correctness
is anchored three independent ways here:

  1. FRAMING: a self-contained copy of the FC's own ``parse_db_stream``
     (``flight-controller/tools/dblink_test.py``) splits the packed bytes back out
     -- proving the ``'db' | cmd | class | len | payload | checksum`` framing is
     exactly what the FC reads, including the CMD byte the FC routes on.
  2. CHECKSUM: an INDEPENDENT recomputation of the dblink 16-bit sum checksum over
     ``cmd + class + len_lo + len_hi + sum(payload)`` matches the trailing bytes.
  3. ROUND-TRIP: every payload field (pos NED, quaternion, pos_sigma_m, age_us,
     reset_counter, flags, range_m) unpacks back to the input, including the
     integer-field saturation (age_us -> u32, reset_counter / flags -> u8) and the
     BUNDLED downward range (the trailing f32 + the range_valid flag bit 0x08).

  .venv/bin/python -m verification.fc_dblink_selftest
"""
from __future__ import annotations

import math
import struct
import sys

from sky.fc.dblink import (
    DB_CMD_VIO_POSE, VIO_FLAG_RANGE_VALID, VIO_LEN, build_db_frame, pack_vio_pose,
)


def _check(cond: bool, msg: str) -> None:
    if not cond:
        print("  FAIL --", msg)
        raise SystemExit(1)
    print("  ok --", msg)


def _f32(v: float) -> float:
    """The float32 value the wire carries for a python float input."""
    return struct.unpack("<f", struct.pack("<f", v))[0]


def parse_db_stream(buf):
    """VERBATIM copy of the FC's ``parse_db_stream`` (dblink_test.py:105).

    Returns ``(frames, tail)`` where each frame is ``(msg_id, payload_bytes)``.
    """
    out = []
    i = 0
    while True:
        j = buf.find(b"db", i)
        if j < 0:
            tail = buf[-1:] if buf.endswith(b"d") else b""
            return out, tail
        if len(buf) - j < 6:
            return out, bytes(buf[j:])
        length = int.from_bytes(buf[j + 4:j + 6], "little")
        if length > 1024:
            i = j + 2
            continue
        frame_total = 6 + length + 2
        if len(buf) - j < frame_total:
            return out, bytes(buf[j:])
        msg_id = buf[j + 2]
        payload = bytes(buf[j + 6:j + 6 + length])
        out.append((msg_id, payload))
        i = j + frame_total


def _checksum(cmd_id: int, payload: bytes) -> int:
    """Independent recompute of the dblink checksum (class fixed 0x00)."""
    length = len(payload)
    return (cmd_id + 0x00 + (length & 0xFF) + ((length >> 8) & 0xFF)
            + sum(payload)) & 0xFFFF


def main() -> int:
    print("[1] pack_vio_pose -> parse_db_stream framing round-trip")
    pos = (1.5, -2.25, 0.125)
    quat = (0.9961947, 0.0871557, 0.0, 0.0)     # ~10 deg roll, unit
    sigma = 0.07
    age_us = 4321
    rc = 5
    flags = 0b101                                # pos_valid + degraded
    frame = pack_vio_pose(pos, quat, sigma, age_us, rc, flags)

    _check(frame[:2] == b"db", "frame starts with the 'db' magic")
    _check(frame[2] == DB_CMD_VIO_POSE == 0x0C, "CMD byte == DB_CMD_VIO_POSE (0x0C)")
    _check(frame[3] == 0x00, "CLASS byte == 0x00")
    wire_len = frame[4] | (frame[5] << 8)
    _check(wire_len == VIO_LEN == 42, "LEN field == 42 (payload size)")

    frames, tail = parse_db_stream(frame)
    _check(len(frames) == 1 and tail == b"", "exactly one frame parses, no tail")
    msg_id, payload = frames[0]
    _check(msg_id == DB_CMD_VIO_POSE, "parsed msg_id == DB_CMD_VIO_POSE")
    _check(len(payload) == VIO_LEN, "parsed payload is 42 bytes")

    print("[2] checksum == independent recompute")
    got_cksum = frame[-2] | (frame[-1] << 8)
    _check(got_cksum == _checksum(DB_CMD_VIO_POSE, payload),
           "trailing checksum matches the independent 16-bit sum")

    print("[3] all payload fields round-trip (struct '<8fIBBf')")
    f = struct.Struct("<8fIBBf").unpack(payload)
    _check(f[0:3] == tuple(_f32(v) for v in pos), "pos_n/e/d == float32(input)")
    _check(f[3:7] == tuple(_f32(v) for v in quat), "q_w/x/y/z == float32(input)")
    _check(f[7] == _f32(sigma), "pos_sigma_m == float32(input)")
    _check(f[8] == age_us, "age_us exact (u32)")
    _check(f[9] == rc, "reset_counter exact (u8)")
    # No range passed (default range_valid=False) -> bit3 clear, range field 0.0.
    _check(f[10] == flags, "flags exact (u8) -- range_valid bit clear by default")
    _check(f[11] == 0.0, "range_m == 0.0 when range_valid defaults False")

    print("[4] integer-field saturation (age u32, reset/flags u8)")
    big = pack_vio_pose((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0),
                        sigma, 10 ** 12, 300, 0x1FF)
    big_frames, _ = parse_db_stream(big)
    g = struct.Struct("<8fIBBf").unpack(big_frames[0][1])
    _check(g[8] == 0xFFFFFFFF, "age_us 1e12 saturates to u32 max")
    _check(g[9] == (300 & 0xFF) == 44, "reset_counter 300 wraps to u8 (44)")
    # 0x1FF masks to u8 = 0xFF, but range_valid (bit3) is packer-owned: with the
    # default range_valid=False it is FORCE-CLEARED, so the wire byte is 0xFF & ~0x08.
    _check(g[10] == ((0x1FF & 0xFF) & ~VIO_FLAG_RANGE_VALID) == 0xF7,
           "flags 0x1FF masks to u8 then range bit force-cleared (0xF7)")

    print("[5] BUNDLED downward range -- the trailing f32 + the 0x08 flag bit")
    # range VALID: range_valid=True -> bit3 (VIO_FLAG_RANGE_VALID) SET + range_m
    # written. The caller's flags (without bit3) are preserved.
    rng_m = 0.842
    fr_v = pack_vio_pose(pos, quat, sigma, age_us, rc, flags,
                         range_m=rng_m, range_valid=True)
    pv = parse_db_stream(fr_v)[0][0][1]
    gv = struct.Struct("<8fIBBf").unpack(pv)
    _check(gv[10] & VIO_FLAG_RANGE_VALID,
           "range_valid=True -> flags bit3 (0x08) SET")
    _check(gv[10] & ~VIO_FLAG_RANGE_VALID == flags,
           "the caller's other flag bits are preserved alongside bit3")
    _check(gv[11] == _f32(rng_m), "range_m == float32(input) when valid")
    # The range f32 lands at byte offset 38 of the payload (matches the FC layout).
    _check(struct.unpack("<f", pv[38:42])[0] == _f32(rng_m),
           "range_m occupies payload bytes [38:42] (FC offset)")

    # range INVALID: range_valid=False -> bit3 FORCE-CLEARED + range field zeroed,
    # EVEN when the caller pre-set bit3 + passed a non-zero range (the packer owns
    # bit3 so a stale range can never be advertised valid by accident).
    fr_i = pack_vio_pose(pos, quat, sigma, age_us, rc, flags | VIO_FLAG_RANGE_VALID,
                         range_m=9.99, range_valid=False)
    pi = parse_db_stream(fr_i)[0][0][1]
    gi = struct.Struct("<8fIBBf").unpack(pi)
    _check(not (gi[10] & VIO_FLAG_RANGE_VALID),
           "range_valid=False -> flags bit3 FORCE-CLEARED even if caller set it")
    _check(gi[11] == 0.0,
           "range_m FORCED to 0.0 when invalid (stale range never on the wire)")

    print("[6] build_db_frame is the framing primitive (arbitrary payload)")
    raw = bytes(range(7))
    fr = build_db_frame(DB_CMD_VIO_POSE, raw)
    frs, _ = parse_db_stream(fr)
    _check(frs == [(DB_CMD_VIO_POSE, raw)], "build_db_frame round-trips a raw payload")
    _check((fr[-2] | (fr[-1] << 8)) == _checksum(DB_CMD_VIO_POSE, raw),
           "build_db_frame checksum matches the independent recompute")

    print("[7] NON-FINITE / out-of-f32-range fuzz: packer never raises, wire all finite")
    # This codebase genuinely produces exploding / NaN poses (--tight on shake,
    # --direct divergence). pack_vio_pose is a LEAF backstop: every float field
    # must come out finite no matter what the caller passes, and it must NOT raise
    # (raw float() into struct '<f' would OverflowError on |x| > ~3.4e38). The
    # BUNDLED range_m is also passed through the leaf when range_valid is True.
    poison = [
        float("nan"), float("inf"), float("-inf"),
        1e300, -1e300, 3.5e38, -3.5e38,        # all out of f32 range / non-finite
    ]
    for bad in poison:
        # bad in all 9 float slots at once (pos x3, quat x4, sigma, range), with
        # range_valid=True so the poisoned range_m goes through _safe_f32 too.
        frame_bad = pack_vio_pose((bad, bad, bad), (bad, bad, bad, bad),
                                  bad, age_us, rc, flags,
                                  range_m=bad, range_valid=True)
        bf, btail = parse_db_stream(frame_bad)
        _check(len(bf) == 1 and btail == b"",
               f"poison={bad!r}: still exactly one well-formed frame")
        _check(len(bf[0][1]) == VIO_LEN,
               f"poison={bad!r}: payload is 42 bytes")
        # All 9 wire floats (8 pose + the bundled range) must be finite.
        u = struct.Struct("<8fIBBf").unpack(bf[0][1])
        floats = u[0:8] + (u[11],)
        _check(all(math.isfinite(v) for v in floats),
               f"poison={bad!r}: all 9 wire floats are finite ({floats})")
        # The checksum must still be self-consistent (a NaN byte pattern would still
        # be summed, but the frame must remain parseable + correctly summed).
        got = frame_bad[-2] | (frame_bad[-1] << 8)
        _check(got == _checksum(DB_CMD_VIO_POSE, bf[0][1]),
               f"poison={bad!r}: checksum still matches the independent recompute")
    # Mixed case: one good + several poison fields in the same frame.
    mixed = pack_vio_pose((1.0, float("nan"), float("inf")),
                          (float("-inf"), 0.0, 1e300, -3.5e38),
                          float("nan"), age_us, rc, flags,
                          range_m=float("inf"), range_valid=True)
    mf, _ = parse_db_stream(mixed)
    mu = struct.Struct("<8fIBBf").unpack(mf[0][1])
    mfloats = mu[0:8] + (mu[11],)
    _check(all(math.isfinite(v) for v in mfloats),
           f"mixed good/poison frame: all 9 wire floats finite ({mfloats})")
    _check(mfloats[0] == _f32(1.0),
           "mixed frame: the FINITE field (pos_n=1.0) passes through unchanged")

    print("\nPASS -- self-owned dblink VIO-pose packer is byte-correct "
          "(42-byte '<8fIBBf' FC-framing round-trip + independent checksum + field "
          "saturation + BUNDLED range (0x08 set/clear, byte [38:42]) + "
          "non-finite/overflow fuzz never raises and never emits NaN/inf).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
