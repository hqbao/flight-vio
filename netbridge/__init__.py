"""``netbridge`` -- the cross-machine live-data bridge (Pi flight stack -> Mac UI).

The 5-project split (imu_camera / vio / slam / ui) runs as a graph of processes
on ONE host, wired together by the in-host :class:`comms.ipc.IPCPubSub` over
AF_UNIX sockets + POSIX shared-memory rings. That works because every process
shares the same kernel: a ``SharedArrayRef`` published by capture can be
``read_copy``-ed straight out of shared memory by the UI.

``netbridge`` lifts that boundary across the network so the Pi runs the WHOLE
flight stack (capture + vio + slam) and a Mac runs ONLY the UI, live over
TCP/WiFi. It is a thin two-process pump that vendors ``comms`` as a 7th
bit-identical copy (sha256-gated by ``verification/ipc_comms_selftest.py``) and
adds its own TCP transport on top:

* :mod:`netbridge.tcp_transport` -- AF_INET frame transport (HMAC authkey,
  ``_BYE`` sentinel, retained-topic replay on connect) -- the network analogue of
  the AF_UNIX ``IPCPubSub`` handshake/fan-out.
* :mod:`netbridge.forward` -- runs on the **Pi**: subscribes capture/vio/slam over
  the local AF_UNIX ``IPCPubSub`` (resolving every ``SharedArrayRef`` to a REAL
  ndarray via ``to_local``), then re-encodes the FULL ndarray onto the TCP server.
  This is the ONLY re-encode point: it guarantees full-ndarray (0x08) on the wire
  (a defensive assert refuses to ship any surviving ``SharedArrayRef``).
* :mod:`netbridge.receive` -- runs on the **Mac**: decodes the TCP stream and
  RE-SERVES the canonical ``oak.capture`` / ``oak.vio`` / ``oak.slam`` AF_UNIX
  endpoints, writing arrays back into MAC-LOCAL rings sized from the forwarded
  ``calib.bundle``. The UI then attaches EXACTLY as if it were on the Pi -- it is
  byte-for-byte unchanged.

Threat model (HONEST): the HMAC authkey AUTHENTICATES the peer (wrong key ->
connection refused) but does NOT encrypt the stream. It is built for a trusted
LAN. For an untrusted network, tunnel it through Wireguard or SSH (``ssh -L``) --
netbridge then sees only loopback and the tunnel provides the encryption.
"""
from __future__ import annotations
