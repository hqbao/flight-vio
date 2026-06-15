# netbridge — cross-machine live-data bridge (Pi → Mac over TCP/WiFi)

The 5-project split (`imu_camera` → `vio` → `slam` → `ui`) runs as a graph of
processes on **one** host, wired by the in-host `comms.ipc.IPCPubSub` over AF_UNIX
sockets + POSIX shared-memory rings. That works because every process shares the
same kernel: a `SharedArrayRef` published by capture is `read_copy`-ed straight out
of shared memory by the UI.

`netbridge` lifts that boundary across the network so the **Pi** runs the whole
flight stack (capture + vio + slam) and a **Mac** runs only the UI — live, over
TCP/WiFi. The UI is **byte-for-byte unchanged**: the Mac re-serves the same
abstract endpoints the UI already consumes.

```
  PI (flight stack)                                  MAC (UI)
  ┌──────────────────────────┐                       ┌────────────────────────┐
  │ capture / vio / slam      │                       │ ui.main (UNCHANGED)     │
  │   AF_UNIX + shm rings     │                       │   AF_UNIX + shm rings   │
  │            │              │                       │            ▲           │
  │            ▼              │   TCP / WiFi          │            │           │
  │  netbridge.forward  ──────┼──── 0x08 frames ──────┼──► netbridge.receive   │
  │  (resolve refs → ndarray) │   (HMAC authkey)      │  (decode → Mac rings)  │
  └──────────────────────────┘                       └────────────────────────┘
```

## Files

| File | Role |
|------|------|
| `comms/` | The vendored comms contract — a **7th bit-identical copy** (`cp -R ui/comms`). sha256-gated by `verification/ipc_comms_selftest.py` alongside the other 6. **Never hand-edited.** netbridge is a *consumer* of comms. |
| `tcp_transport.py` | AF_INET frame transport: `TcpServer` / `TcpClient` with an HMAC authkey (`OAKD_NETBRIDGE_KEY` if set, else a built-in **default key** so no setup is needed on a trusted LAN), a `_BYE` sentinel, and retained-topic replay on connect. The network analogue of `IPCPubSub`. |
| `topics_allowlist.py` | The single source of truth for **which** topics cross the wire (UI-needed only), split into POD / image / retained, plus the direct-wire (retained config) set. Forward and receive both import it. |
| `wire_full.py` | The ref-free `Wire*` ⇄ local-dataclass converter for the four image topics — puts **full ndarrays** where the in-host wire would hold a `SharedArrayRef`. |
| `forward.py` | Runs on the **Pi**. The local IPC → TCP pump. **The only re-encode point.** |
| `receive.py` | Runs on the **Mac**. The TCP → local IPC re-serve. |

## The data path & the `0x09 → 0x08` re-materialisation

The one tricky thing about going off-host is shared memory. The wire `Wire*`
classes for image topics (`WireCamSync` / `WireDepthFrame` / `WireImuCamPacket` /
`WireKeyframe`) carry a `SharedArrayRef` (codec tag **0x09**) in their image/depth
slots — metadata only; the pixels stay in a `SharedArrayRing`. The Mac cannot
`read_copy` the Pi's shared memory, so those pixels must travel inline.

**forward** is where that conversion happens, and it is the **only** place the
bridge re-encodes:

1. A `comms.bridge.IPCSubscriber` reads each Pi endpoint and runs
   `comms.converters.to_local`, which `read_copy`-s every `SharedArrayRef` out of
   the Pi's rings into a **real ndarray** and rebuilds the local dataclass.
2. forward taps that, builds the **ref-free** wire form (`wire_full` for the four
   image topics; `comms.converters.to_wire` for POD), and `codec.encode`s it →
   the image arrays now ride as full ndarrays (tag **0x08**) on the TCP wire.
3. A **defensive assert** (`_assert_no_shared_ref`) refuses to encode if any field
   is still a `SharedArrayRef` — shipping a ref the Mac can't read would corrupt
   the UI silently, so it **fails loud** instead.

**receive** mirrors it: `codec.decode` (all 0x08) → local dataclass → a standard
`comms.bridge.IPCPublisher` writes the arrays into **Mac-local rings** (0x09 over
AF_UNIX) → the UI `read_copy`s them exactly as if it were on the Pi.

Retained config (`calib.bundle` / `calib.stereo` / `vio.map`) has no converter; it
travels as its raw `Wire*` form (forward subscribes it on a second raw IPC client;
the TCP server caches + replays it; receive publishes it straight onto the
re-served server).

### Ring sizing from calib (critical)

receive must size the Mac rings at the **same resolution** the Pi produced (54×42
for a ToF run, 640×400 otherwise) — a hardcoded 640×400 would corrupt a 54×42
stream. So receive **awaits the forwarded `calib.bundle` first** (like `ui.main`
does), reads `width`/`height`, and only then creates the rings + serves.

## Backpressure (never stall the flight stack)

Image topics forward **non-blocking latest-wins** (drop the stale frame on a WiFi
stall — never back-pressure capture/vio). POD + retained topics forward
**reliably** (a one-shot calib or a pose is never silently dropped). The policy
lives in `TcpServer` (`image_topics` set).

## Security (HONEST)

`OAKD_NETBRIDGE_KEY` is a shared HMAC secret (`multiprocessing.connection`
challenge-response, done **once at connect**, not per frame). It **authenticates**
the peer — a wrong key is refused. It does **NOT encrypt** the stream — built for a
trusted LAN. For an untrusted network, tunnel it through **Wireguard** or an
**SSH `-L` forward** (netbridge then sees only loopback and the tunnel encrypts).

**No key to manage (trusted LAN).** If `OAKD_NETBRIDGE_KEY` is **unset**, both ends
fall back to the same built-in **default key** — so the bridge connects with zero
setup (handy for testing). That default is **public** (it's in the source), so it is
convenience auth, not a secret: export a real `OAKD_NETBRIDGE_KEY` on both hosts for
security on an untrusted network. It is the same speed either way (one-time handshake).

## Run

```bash
# on the Pi — additive to the normal flight launch:
./run.sh --no-ui --vl53l9cx --direct --forward 0.0.0.0:8787   # no key -> default key

# on the Mac — connects with the same default key, no setup:
./deploy/pi-ui.sh --connect <pi-host>:8787

# OR with a real shared secret (export the SAME value on BOTH hosts):
export OAKD_NETBRIDGE_KEY=$(openssl rand -hex 32)   # Pi  (and the same on the Mac)
./run.sh --no-ui --vl53l9cx --direct --forward 0.0.0.0:8787
```

## Gate

`verification/netbridge_loopback_selftest.py` — single Mac, two TCP hops over
127.0.0.1 (fake producer → forward → TCP → receive → headless subscriber). Proves:
`frame.depth` pixels bit-identical end-to-end (0x09→0x08→0x09 through **both** ring
sets), `pose.odom` + `ba.window` arrays bit-identical, retained calib replayed to a
subscriber connecting **after** the producer stops, a **wrong authkey → refused**,
and an offscreen `ui.main` smoke that gets past `_await_calib_bundle` and renders.

`verification/ipc_comms_selftest.py` additionally proves `netbridge/comms` is
sha256-identical to the other 6 copies.
```bash
.venv/bin/python verification/netbridge_loopback_selftest.py   # OAKD_NETBRIDGE_KEY auto-set to "test"
.venv/bin/python verification/ipc_comms_selftest.py
```
