"""Reflective, class-path-INDEPENDENT binary codec for the IPC wire boundary.

WHY a codec at all
------------------
pickle bakes the publisher's MODULE PATH into the bytes, so a decoder living in a
DIFFERENT vendored copy of ``comms`` (``imu_camera.comms.wire.WirePoseMsg`` vs
``vio.comms.wire.WirePoseMsg``) could fail to resolve the class or build an
object whose identity / type does not match the consumer's own ``Wire*``. This
codec is keyed instead by ``(topic -> Wire* class, dataclass-field-ORDER)`` from
:data:`comms.wire.TOPIC_WIRE` -- never the module path -- so any copy can decode
any other copy's bytes bit-identically.

Frame layout (big-endian everywhere; LEB128 varints for all metadata)
---------------------------------------------------------------------
Message::

    [varint topic_len][topic UTF-8 bytes]
    then, for the topic's Wire* class, each field in @dataclass DEFINITION ORDER:
        [type_tag:1B][varint value_len][value bytes]

WireEnd is encoded out-of-band as a single top-level field of tag 0x0A (it
carries its own ``topic``), so decode of any topic whose first/only frame is a
0x0A returns a reconstructed :class:`comms.wire.WireEnd`.

Type tags & value encodings::

    0x01 int           signed LEB128 zig-zag (handles negatives)
    0x02 float         struct.pack('>d', x)  (8B IEEE754 BE; NaN/Inf preserved)
    0x03 str           UTF-8 bytes
    0x04 bytes         raw bytes
    0x05 bool          b'\\x00' | b'\\x01'
    0x06 None          length 0, no value bytes
    0x07 dict          [varint n_items][ encode(k) encode(v) ]*  (sorted items)
    0x08 ndarray       [str-frame dtype.name][varint ndim][dims...][C-order bytes]
    0x09 SharedArrayRef[str ring_name][varint slot][varint ndim][shape...][str dtype]
    0x0A WireEnd       [str topic]

Each dict key/value is itself a FULL [tag|len|bytes] frame; ndarray inner dtype
and SharedArrayRef inner strings are written as full frames too, so a decoder
parses recursively with no out-of-band schema.

Forward-compat: a decoder reads exactly ``len(fields)`` frames; an unknown tag is
skipped via its length prefix (append-only fields are tolerated). Optional fields
encode as 0x06 None.
"""
from __future__ import annotations

import struct
from dataclasses import fields as dataclass_fields
from typing import Any

import numpy as np

from .shared_array import SharedArrayRef
from .wire import TOPIC_WIRE, WireEnd

# --------------------------------------------------------------------------- #
# Type tags (0x00 reserved).
# --------------------------------------------------------------------------- #
TAG_INT = 0x01
TAG_FLOAT = 0x02
TAG_STR = 0x03
TAG_BYTES = 0x04
TAG_BOOL = 0x05
TAG_NONE = 0x06
TAG_DICT = 0x07
TAG_NDARRAY = 0x08
TAG_SHAREDREF = 0x09
TAG_WIREEND = 0x0A


# --------------------------------------------------------------------------- #
# LEB128 varints (unsigned + zig-zag signed)
# --------------------------------------------------------------------------- #
def _write_uvarint(out: bytearray, value: int) -> None:
    """Append ``value`` (>= 0) as an unsigned LEB128 varint to ``out``."""
    if value < 0:
        raise ValueError(f"unsigned varint cannot encode negative {value}")
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return


def _read_uvarint(data: bytes, pos: int) -> tuple[int, int]:
    """Read an unsigned LEB128 varint from ``data`` at ``pos``.

    Returns ``(value, new_pos)``.
    """
    result = 0
    shift = 0
    while True:
        byte = data[pos]
        pos += 1
        result |= (byte & 0x7F) << shift
        if not (byte & 0x80):
            return result, pos
        shift += 7


def _svarint_bytes(value: int) -> bytes:
    """Encode a signed int as zig-zag LEB128 (variable length, no overall cap).

    Zig-zag maps signed -> unsigned (0,-1,1,-2,2 -> 0,1,2,3,4) then LEB128. Works
    for arbitrarily large Python ints (not bound to 64 bits).
    """
    if value >= 0:
        zz = value << 1
    else:
        zz = (abs(value) << 1) - 1
    out = bytearray()
    _write_uvarint(out, zz)
    return bytes(out)


def _svarint_read(data: bytes, pos: int) -> tuple[int, int]:
    """Read a zig-zag LEB128 signed int. Returns ``(value, new_pos)``."""
    zz, pos = _read_uvarint(data, pos)
    if zz & 1:
        value = -((zz + 1) >> 1)
    else:
        value = zz >> 1
    return value, pos


# --------------------------------------------------------------------------- #
# Frame helpers: every value is written as [tag:1B][uvarint len][value bytes].
# --------------------------------------------------------------------------- #
def _emit_frame(out: bytearray, tag: int, value: bytes) -> None:
    out.append(tag)
    _write_uvarint(out, len(value))
    out += value


def _read_frame(data: bytes, pos: int) -> tuple[int, bytes, int]:
    """Read one [tag][len][value] frame. Returns ``(tag, value_bytes, new_pos)``."""
    tag = data[pos]
    pos += 1
    length, pos = _read_uvarint(data, pos)
    value = data[pos:pos + length]
    return tag, value, pos + length


# --------------------------------------------------------------------------- #
# Value encoders -> raw value bytes (the bytes that go between len and the next
# frame). Each pairs with a decoder keyed by the tag.
# --------------------------------------------------------------------------- #
def _encode_ndarray_value(arr: np.ndarray) -> bytes:
    """[str-frame dtype.name][uvarint ndim][ndim x uvarint dims][C-order bytes].

    dtype is canonicalised via ``np.dtype(x).name`` (e.g. 'uint8' / 'float32',
    never '<u1') and the data is forced C-contiguous + that exact dtype via
    ``astype`` so the raw bytes are identical on any host / endianness. The
    BIG-ENDIAN intent is realised by writing the canonical native bytes plus the
    dtype.name tag: a decoder rebuilds with the SAME ``np.dtype(name)`` so the
    interpretation matches; the cross-copy hex gate enforces that every copy
    canonicalises identically.
    """
    name = np.dtype(arr.dtype).name
    canonical = np.ascontiguousarray(arr).astype(np.dtype(name), copy=False)
    out = bytearray()
    _emit_frame(out, TAG_STR, name.encode("utf-8"))
    _write_uvarint(out, canonical.ndim)
    for dim in canonical.shape:
        _write_uvarint(out, int(dim))
    out += canonical.tobytes(order="C")
    return bytes(out)


def _decode_ndarray_value(value: bytes) -> np.ndarray:
    pos = 0
    tag, name_bytes, pos = _read_frame(value, pos)
    if tag != TAG_STR:
        raise ValueError(f"ndarray dtype expected str frame, got tag {tag:#x}")
    name = name_bytes.decode("utf-8")
    ndim, pos = _read_uvarint(value, pos)
    shape = []
    for _ in range(ndim):
        dim, pos = _read_uvarint(value, pos)
        shape.append(dim)
    raw = value[pos:]
    arr = np.frombuffer(raw, dtype=np.dtype(name))
    # `.copy()` so the result owns its memory (the input `value` slice is a view
    # into the inbound buffer that the caller may reuse / free).
    return arr.reshape(shape).copy()


def _encode_sharedref_value(ref: SharedArrayRef) -> bytes:
    """[str ring_name][uvarint slot][uvarint ndim][shape...][str dtype].

    Metadata ONLY -- the actual slot pixels never travel here; they stay in the
    SharedArrayRing shared memory and are read out by the subscriber bridge.
    """
    out = bytearray()
    _emit_frame(out, TAG_STR, ref.ring_name.encode("utf-8"))
    _write_uvarint(out, int(ref.slot))
    shape = tuple(int(s) for s in ref.shape)
    _write_uvarint(out, len(shape))
    for dim in shape:
        _write_uvarint(out, int(dim))
    _emit_frame(out, TAG_STR, str(ref.dtype).encode("utf-8"))
    return bytes(out)


def _decode_sharedref_value(value: bytes) -> SharedArrayRef:
    pos = 0
    tag, name_bytes, pos = _read_frame(value, pos)
    if tag != TAG_STR:
        raise ValueError(f"SharedArrayRef name expected str frame, got {tag:#x}")
    ring_name = name_bytes.decode("utf-8")
    slot, pos = _read_uvarint(value, pos)
    ndim, pos = _read_uvarint(value, pos)
    shape = []
    for _ in range(ndim):
        dim, pos = _read_uvarint(value, pos)
        shape.append(dim)
    tag, dt_bytes, pos = _read_frame(value, pos)
    if tag != TAG_STR:
        raise ValueError(f"SharedArrayRef dtype expected str frame, got {tag:#x}")
    return SharedArrayRef(ring_name=ring_name, slot=int(slot),
                          shape=tuple(shape), dtype=dt_bytes.decode("utf-8"))


def _encode_dict_value(d: dict) -> bytes:
    """[uvarint n_items][ encode_field(k) encode_field(v) ]* in sorted-item order.

    Keys must be ``str`` or ``int`` (validated): they are the only key types the
    wire dicts use (``PoseMsg.info`` string keys, ``LoopCorrection.kf_poses`` int
    keys). Sorting makes the encoding deterministic regardless of insertion order.
    """
    items = list(d.items())
    for k, _ in items:
        if not isinstance(k, (str, int)) or isinstance(k, bool):
            raise TypeError(
                f"dict key must be str or int (got {type(k).__name__})")
    items.sort(key=lambda kv: kv[0])
    out = bytearray()
    _write_uvarint(out, len(items))
    for key, val in items:
        _encode_field(out, key)
        _encode_field(out, val)
    return bytes(out)


def _decode_dict_value(value: bytes) -> dict:
    pos = 0
    n_items, pos = _read_uvarint(value, pos)
    result: dict = {}
    for _ in range(n_items):
        key, pos = _decode_field(value, pos)
        val, pos = _decode_field(value, pos)
        result[key] = val
    return result


# --------------------------------------------------------------------------- #
# Generic field encode / decode (one [tag|len|value] frame for ANY supported
# Python value). Dispatch by Python type at encode; by tag at decode.
# --------------------------------------------------------------------------- #
def _encode_field(out: bytearray, value: Any) -> None:
    """Append one fully framed value to ``out`` (recursive for dict / nested)."""
    # NOTE: bool is a subclass of int -> check bool BEFORE int.
    if value is None:
        _emit_frame(out, TAG_NONE, b"")
    elif isinstance(value, bool):
        _emit_frame(out, TAG_BOOL, b"\x01" if value else b"\x00")
    elif isinstance(value, (int, np.integer)):
        _emit_frame(out, TAG_INT, _svarint_bytes(int(value)))
    elif isinstance(value, (float, np.floating)):
        _emit_frame(out, TAG_FLOAT, struct.pack(">d", float(value)))
    elif isinstance(value, str):
        _emit_frame(out, TAG_STR, value.encode("utf-8"))
    elif isinstance(value, (bytes, bytearray)):
        _emit_frame(out, TAG_BYTES, bytes(value))
    elif isinstance(value, np.ndarray):
        _emit_frame(out, TAG_NDARRAY, _encode_ndarray_value(value))
    elif isinstance(value, SharedArrayRef):
        _emit_frame(out, TAG_SHAREDREF, _encode_sharedref_value(value))
    elif isinstance(value, WireEnd):
        _emit_frame(out, TAG_WIREEND, value.topic.encode("utf-8"))
    elif isinstance(value, dict):
        _emit_frame(out, TAG_DICT, _encode_dict_value(value))
    else:
        raise TypeError(
            f"codec cannot encode value of type {type(value).__name__!r}")


def _decode_field(data: bytes, pos: int) -> tuple[Any, int]:
    """Decode one framed value at ``pos``. Returns ``(value, new_pos)``."""
    tag, value, new_pos = _read_frame(data, pos)
    if tag == TAG_NONE:
        return None, new_pos
    if tag == TAG_BOOL:
        return (value == b"\x01"), new_pos
    if tag == TAG_INT:
        decoded, _ = _svarint_read(value, 0)
        return decoded, new_pos
    if tag == TAG_FLOAT:
        return struct.unpack(">d", value)[0], new_pos
    if tag == TAG_STR:
        return value.decode("utf-8"), new_pos
    if tag == TAG_BYTES:
        return bytes(value), new_pos
    if tag == TAG_NDARRAY:
        return _decode_ndarray_value(value), new_pos
    if tag == TAG_SHAREDREF:
        return _decode_sharedref_value(value), new_pos
    if tag == TAG_WIREEND:
        return WireEnd(value.decode("utf-8")), new_pos
    if tag == TAG_DICT:
        return _decode_dict_value(value), new_pos
    # Unknown tag: skip it via its length prefix (append-only forward-compat).
    return _SKIP, new_pos


#: Marker returned by :func:`_decode_field` for an unknown tag that was skipped.
_SKIP = object()


# --------------------------------------------------------------------------- #
# Public API
# --------------------------------------------------------------------------- #
def encode(topic: str, msg: Any) -> bytes:
    """Encode ``(topic, msg)`` into the wire byte string.

    Layout: ``[uvarint topic_len][topic UTF-8]`` then either:

    * a single 0x0A WireEnd frame (when ``msg`` is a :class:`comms.wire.WireEnd`),
      or
    * one frame per ``Wire*`` field in @dataclass DEFINITION ORDER.

    ``msg`` for a data topic must be the topic's ``Wire*`` dataclass instance
    (looked up in :data:`comms.wire.TOPIC_WIRE`); the codec writes only its
    fields, never its class path, so any vendored copy can decode the result.
    """
    topic_bytes = topic.encode("utf-8")
    out = bytearray()
    _write_uvarint(out, len(topic_bytes))
    out += topic_bytes

    if isinstance(msg, WireEnd):
        # END rides out-of-band: a single 0x0A frame, class-path-independent.
        _emit_frame(out, TAG_WIREEND, msg.topic.encode("utf-8"))
        return bytes(out)

    wire_cls = TOPIC_WIRE.get(topic)
    if wire_cls is None:
        raise KeyError(f"no Wire* class registered for topic {topic!r}")
    if not isinstance(msg, wire_cls):
        raise TypeError(
            f"topic {topic!r} expects {wire_cls.__name__}, "
            f"got {type(msg).__name__}")
    # Fields in @dataclass definition order (the FROZEN positional contract).
    for f in dataclass_fields(wire_cls):
        _encode_field(out, getattr(msg, f.name))
    return bytes(out)


def decode(data: bytes) -> tuple[str, Any]:
    """Decode a wire byte string back into ``(topic, msg)``.

    Looks the ``Wire*`` class up by TOPIC in :data:`comms.wire.TOPIC_WIRE` and
    reconstructs it POSITIONALLY (field-definition order), so the result is the
    DECODER's own ``Wire*`` type -- independent of which copy produced the bytes.
    A leading 0x0A frame decodes to a :class:`comms.wire.WireEnd`.
    """
    pos = 0
    topic_len, pos = _read_uvarint(data, pos)
    topic = data[pos:pos + topic_len].decode("utf-8")
    pos += topic_len

    # Peek at the first frame's tag: a WireEnd rides as a single 0x0A frame.
    if pos < len(data) and data[pos] == TAG_WIREEND:
        value, pos = _decode_field(data, pos)
        return topic, value

    wire_cls = TOPIC_WIRE.get(topic)
    if wire_cls is None:
        raise KeyError(f"no Wire* class registered for topic {topic!r}")

    fields = dataclass_fields(wire_cls)
    values: list[Any] = []
    for _ in fields:
        if pos >= len(data):
            # Fewer frames than fields: the rest stay at their dataclass default
            # (append-only forward-compat -- an older encoder omitted new fields).
            break
        value, pos = _decode_field(data, pos)
        if value is _SKIP:
            # Unknown tag for this slot: treat as "field absent" -> default.
            values.append(_SKIP)
        else:
            values.append(value)

    # Build kwargs in field order; honour defaults for absent / skipped fields.
    kwargs: dict[str, Any] = {}
    for f, val in zip(fields, values):
        if val is _SKIP:
            continue
        kwargs[f.name] = val
    return topic, wire_cls(**kwargs)
