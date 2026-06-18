# `comms/` — the canonical, vendored comms contract

This package is the **single source of truth** for the cross-project comms layer
of the split. The anchor is `imu_camera/comms`; it is **copied bit-identically**
into every other project (`depth`, `vio`, `ba`, `slam`, `ui`, `launcher`, and
`netbridge` — a 7th copy). CI diffs the copies (`diff -r <proj>/comms
imu_camera/comms` must be empty) and compares the codec digest oracle
byte-for-byte. Because of that:

- **All internal imports are RELATIVE** (`from .codec import …`), so the package
  drops into any project unchanged.
- **It imports NO `depthai` and NO `PyQt6` (and no `cv2`)** — it is headless-safe.
  Verified: importing every module in `comms/` pulls none of those.

It is the merge of the pre-split `ours.lib.flow` + `ours.lib.ipc` +
`ours.flows.bridge`, plus the foundational `ours.lib.{misc,config}`, renamed for
the split.

## Two transports

| Transport | Class | Serialization | Use |
|-----------|-------|---------------|-----|
| In-process | `LocalPubSub` (`pubsub.py`) | **none** — passes Python objects directly | offline deterministic replay / oracle; byte-for-byte unchanged from pre-split |
| Cross-process | `IPCPubSub(endpoint, role="server"\|"client")` (`ipc.py`) | the class-path-independent `codec` (NOT pickle) | the live multi-process topology (`capture → vio → ba → slam → ui`) |

`IPCPubSub` merges the old `IpcServerBus` (`role="server"` → publish) and
`IpcClientBus` (`role="client"` → subscribe + recv thread). All the old logic is
preserved (accept/fanout/recv threads, retained topics, bounded-outbox
latest-wins/blocking back-pressure, handshake). The **only** wire change:

```
# OLD (implicit pickle):  conn.send(("M", topic, msg)) / conn.recv()
# NEW (raw codec bytes):  conn.send_bytes(codec.encode(topic, msg))
#                         topic, msg = codec.decode(conn.recv_bytes())
```

`send_bytes`/`recv_bytes` already length-frame on the socket, so the codec body
carries no extra length prefix. The handshake (subscribed-topics) rides a JSON
byte frame; `BYE` is a fixed 3-byte sentinel.

## Why a codec (not pickle) on the cross-project seam

`pickle` bakes the publisher's **module path** into the bytes, so a decoder in a
*different* vendored copy (`imu_camera.comms.wire.WirePoseMsg` vs
`vio.comms.wire.WirePoseMsg`) could fail to resolve or mismatch identity. The
codec (`codec.py`) is keyed by **`(topic → Wire* class, dataclass-field-ORDER)`**
from `wire.TOPIC_WIRE` — never the module path — so any copy decodes any other
copy's bytes bit-identically, into the *decoder's own* `Wire*` type.

### Byte layout (big-endian; LEB128 varints for all metadata)

Message: `[uvarint topic_len][topic UTF-8]` then, for the topic's `Wire*` class,
one frame per field in `@dataclass` **definition order**. Each frame is
`[type_tag:1B][uvarint value_len][value bytes]`.

| Tag | Type | Value encoding |
|-----|------|----------------|
| `0x01` | int | signed LEB128 **zig-zag** (handles negatives, unbounded) |
| `0x02` | float | `struct.pack('>d', x)` — 8B IEEE754 BE, NaN/Inf preserved |
| `0x03` | str | UTF-8 bytes |
| `0x04` | bytes | raw bytes |
| `0x05` | bool | `\x00` / `\x01` |
| `0x06` | None | length 0, no value |
| `0x07` | dict | `[uvarint n][encode(k) encode(v)]*`, items `sorted()`; keys str/int |
| `0x08` | ndarray | `[str dtype.name][uvarint ndim][dims…][C-order `astype`-canonical bytes]` |
| `0x09` | SharedArrayRef | `[str ring_name][uvarint slot][uvarint ndim][shape…][str dtype]` — metadata only, pixels stay in shared memory |
| `0x0A` | WireEnd | `[str topic]` — END sentinel, out-of-band |

Forward-compat: decode reads exactly `len(fields)` frames; an unknown tag is
skipped via its length prefix; absent trailing frames fall back to the
dataclass default. Public API: `encode(topic, msg) -> bytes`,
`decode(data) -> (topic, msg)`.

## Files

```
comms/
├── __init__.py        re-exports LocalPubSub, IPCPubSub, Module, SourceModule,
│                       Step, SharedArrayRing/Ref, RingRegistry, IPCPublisher,
│                       IPCSubscriber, topics, encode/decode
├── pubsub.py          LocalPubSub                       (was Bus)
├── module.py          Module / SourceModule / ModuleContext  (was Flow/SourceFlow/FlowContext)
├── step.py            Step                              (was Task)
├── runtime.py         NUMBA_PARALLEL_LOCK               (verbatim)
├── messages.py        local carriers + END sentinel
├── topics.py          topic string constants            (UNCHANGED — the contract)
├── wire.py            ALL Wire* dataclasses + TOPIC_WIRE (FROZEN names + field order)
├── codec.py           NEW class-path-independent binary codec
├── shared_array.py    SharedArrayRing + SharedArrayRef  (UNCHANGED binary layout)
├── ring_registry.py   RingRegistry + RingSpec + default_*_specs
├── converters.py      to_wire / to_local local↔wire bridge
├── ipc.py             IPCPubSub(role=…)                 (was IpcServerBus + IpcClientBus)
├── bridge.py          IPCPublisher + IPCSubscriber      (was IpcPublisherFlow/IpcSubscriberFlow)
└── lib/
    ├── misc/          frames, geometry, pose, pngio, warmup  (numpy + stdlib)
    └── config/        resolution                        (project-math glue; NOT imported by comms)
```

### Rename map applied

`Bus → LocalPubSub` · `Flow/SourceFlow/FlowContext → Module/SourceModule/ModuleContext`
· `Task → Step` · `IpcServerBus + IpcClientBus → IPCPubSub(role=…)` ·
`IpcPublisherFlow → IPCPublisher` · `IpcSubscriberFlow → IPCSubscriber`.
`SharedArrayRing`/`SharedArrayRef` and the `Wire*` names + field order are
**UNCHANGED** (frozen contract). Topic STRINGS are **UNCHANGED**.

## `TOPIC_WIRE` registry (the codec key)

Includes the **retained / read-directly** topics that have no converter
(`calib.bundle → WireCalibBundle`, `vio.map → WireVioMap`) so consumers that read
them straight off the wire can still decode. `WireEnd` is handled out-of-band by
tag `0x0A`.

## Self-test (the cross-copy byte-parity oracle)

`imu_camera/tests/codec_roundtrip_selftest.py` builds one fixed vector for every
`Wire*` (incl. SharedArrayRef-bearing, dict-bearing, Optional-None, empty-ndarray
IMU interval `M=0`, NaN/Inf floats, `WireEnd`), asserts
`decode(encode(topic, msg)) == msg` (array-aware deep equality), and freezes the
sha256 of each `encode()` into `codec_vectors.json`. On later runs it asserts the
digests still match — so any field-order / dtype / endianness drift fails loudly.
The same test, vendored into each copy, must produce an identical
`codec_vectors.json`.

```sh
.venv/bin/python -m imu_camera.tests.codec_roundtrip_selftest
```
