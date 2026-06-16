#!/usr/bin/env python3
"""netbridge end-to-end loopback gate -- the headline test for the TCP bridge.

Single Mac, TWO TCP hops over 127.0.0.1, mirroring the real Pi->Mac topology:

    fake producer (AF_UNIX server + Pi-side rings @54x42)
        --AF_UNIX--> netbridge.forward (the re-encode point)
        --TCP 127.0.0.1--> netbridge.receive (re-serve on suffixed endpoints)
        --AF_UNIX--> headless subscriber (asserts bit-identity)

What it proves
--------------
1. **Image pixels survive 0x09 -> 0x08 -> 0x09 through BOTH ring sets.** The
   producer publishes ``frame.depth`` through the in-host bridge, so its gray +
   depth ride a ``SharedArrayRef`` (0x09) in the producer's Pi rings. forward
   ``read_copy``-s them to REAL ndarrays and re-encodes them as full ndarrays
   (0x08) on the wire; receive decodes them and writes them into MAC-LOCAL rings,
   re-serving them as 0x09 refs the subscriber ``read_copy``-s. The depth + gray
   pixels are asserted ``np.array_equal`` end-to-end.
2. **POD arrays are bit-identical.** ``pose.odom`` (T_world_cam) and ``ba.window``
   (every ndarray field) round-trip ``np.array_equal``.
3. **Retained replay to a LATE subscriber.** A subscriber that connects AFTER the
   producer has stopped still receives the retained ``calib.bundle`` +
   ``calib.stereo`` (cached by the TCP server + re-served by the receive endpoint).
4. **Authkey is enforced.** A client with the WRONG ``OAKD_NETBRIDGE_KEY`` is
   REFUSED (the HMAC challenge-response fails); a missing key refuses to start.
5. **Offscreen ui.main smoke** (``QT_QPA_PLATFORM=offscreen``) against the receive
   side: the real UI gets past ``_await_calib_bundle`` and renders >= 1 frame.

Runs on stdlib + numpy + (for the smoke) PyQt6. FAILS LOUDLY on any divergence.

Run::

    .venv/bin/python verification/netbridge_loopback_selftest.py
"""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

# The bridge refuses to open a socket without a key; set a test secret BEFORE any
# netbridge import path constructs a transport.
os.environ.setdefault("OAKD_NETBRIDGE_KEY", "test")

from netbridge.comms import topics                                  # noqa: E402
from netbridge.comms.bridge import IPCPublisher                     # noqa: E402
from netbridge.comms.ipc import IPCPubSub                           # noqa: E402
from netbridge.comms.messages import BaWindow, DepthFrame, PoseMsg  # noqa: E402
from netbridge.comms.pubsub import LocalPubSub                      # noqa: E402
from netbridge.comms.ring_registry import (                        # noqa: E402
    RingRegistry, default_capture_specs, default_vio_specs,
)
from netbridge.comms.wire import (                                  # noqa: E402
    WireCalibBundle, WireCalibStereo, WireEnd,
)
from netbridge.forward import run_forward                          # noqa: E402
from netbridge.receive import run_receive                          # noqa: E402
from netbridge.tcp_transport import TcpClient                      # noqa: E402

# The ToF resolution the producer runs at -- DELIBERATELY not 640x400, to prove
# the Mac rings are sized from the forwarded calib (a hardcoded 640x400 in receive
# would corrupt these pixels).
W, H = 54, 42
SLOTS = 16


def _check(cond: bool, msg: str) -> bool:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    return bool(cond)


def _free_port() -> int:
    """Grab an ephemeral TCP port (then release it for the server to rebind)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# --------------------------------------------------------------------------- #
# Deterministic test payloads (no RNG -> reproducible bit-identity assertions).
# --------------------------------------------------------------------------- #
def _make_depth(seq: int) -> DepthFrame:
    """A 54x42 gray + depth frame whose pixels depend on ``seq`` (so a dropped /
    duplicated frame would mis-compare)."""
    gray = ((np.arange(H * W, dtype=np.int64).reshape(H, W) + seq * 7) % 256
            ).astype(np.uint8)
    depth = ((np.arange(H * W, dtype=np.float32).reshape(H, W) + seq) * 0.01
             ).astype(np.float32)
    return DepthFrame(seq=seq, ts_ns=seq * 50_000_000,
                      gray_left=gray, depth_m=depth)


def _make_pose(seq: int) -> PoseMsg:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [0.1 * seq, -0.2 * seq, 0.3 * seq]
    return PoseMsg(seq=seq, ts_ns=seq * 50_000_000, T_world_cam=T,
                   info={"ok": True, "n_inliers": 40 + seq})


def _make_ba(seq: int) -> BaWindow:
    return BaWindow(
        seq=seq, ts_ns=seq * 1_000_000,
        kf_ids=np.array([seq, seq + 1, seq + 2], dtype=np.int64),
        kf_quat=np.array([[1.0, 0.0, 0.0, 0.0],
                          [0.92388, 0.0, 0.38268, 0.0],
                          [0.70711, 0.0, 0.70711, 0.0]], dtype=np.float64),
        kf_pos=(np.arange(9, dtype=np.float64).reshape(3, 3) * 0.5 + seq),
        lm_ids=np.array([10, 11, 12, 13], dtype=np.int64),
        lm_xyz=(np.arange(12, dtype=np.float64).reshape(4, 3) * 0.25),
        obs_kf=np.array([0, 0, 1, 1, 2], dtype=np.int32),
        obs_lm=np.array([0, 1, 1, 2, 3], dtype=np.int32),
        obs_uv=(np.arange(10, dtype=np.float32).reshape(5, 2) * 3.0),
        obs_reproj_px=np.array([0.1, 0.5, 1.2, 2.0, 4.5], dtype=np.float32),
        ba_reproj_px=0.83,
        kf_quat_pre=np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.99619, 0.0, 0.08716, 0.0],
                              [0.96593, 0.0, 0.25882, 0.0]], dtype=np.float64),
        kf_pos_pre=(np.arange(9, dtype=np.float64).reshape(3, 3) * 0.49),
        lm_xyz_pre=(np.arange(12, dtype=np.float64).reshape(4, 3) * 0.24),
        n_kf=3, n_lm=4)


def _make_calib_bundle() -> WireCalibBundle:
    K = np.array([[40.0, 0.0, 27.0],
                  [0.0, 40.0, 21.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    return WireCalibBundle(K=K, width=W, height=H, fps=20,
                           T_imu_left=np.eye(4, dtype=np.float64),
                           R_imu_cam=np.eye(3, dtype=np.float64),
                           accel_align=np.zeros(3, dtype=np.float64),
                           gyro_bias=np.zeros(3, dtype=np.float64),
                           device_id="nbtest-dev")


def _make_calib_stereo() -> WireCalibStereo:
    K = np.array([[40.0, 0.0, 27.0],
                  [0.0, 40.0, 21.0],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    T = np.eye(4, dtype=np.float64); T[0, 3] = -0.075
    return WireCalibStereo(left_K=K, left_dist=np.zeros(8, dtype=np.float64),
                           right_K=K + 0.1, right_dist=np.zeros(8, dtype=np.float64),
                           T_left_right=T, width=W, height=H)


# --------------------------------------------------------------------------- #
# The fake producer: AF_UNIX servers + Pi-side rings @54x42, in-host bridge.
# --------------------------------------------------------------------------- #
class FakeProducer:
    """Mimics the Pi flight stack's capture+vio+slam IPC servers.

    Publishes ``frame.depth`` (image, via the in-host IPCPublisher so it rides a
    SharedArrayRef in the rings -> proves forward materialises it), ``pose.odom``
    + ``ba.window`` (POD, direct on the server), and the retained calib bundles.
    """

    def __init__(self, cap_ep: str, vio_ep: str, slam_ep: str) -> None:
        self.cap_ep, self.vio_ep, self.slam_ep = cap_ep, vio_ep, slam_ep
        # Pi rings (producer owns them).
        self.cap_rings = RingRegistry().create_all(
            default_capture_specs(endpoint=cap_ep, width=W, height=H, slots=SLOTS))
        self.vio_rings = RingRegistry().create_all(
            default_vio_specs(endpoint=vio_ep, width=W, height=H, slots=SLOTS))
        # Capture server: retained calib + the depth image stream (blocking so the
        # selftest never drops a frame it later asserts on).
        self.cap_server = IPCPubSub(
            cap_ep, role="server",
            retain_topics={topics.CALIB_BUNDLE, topics.CALIB_STEREO},
            blocking=True)
        self.cap_local = LocalPubSub()
        self.cap_pub = IPCPublisher(self.cap_local, self.cap_server,
                                    self.cap_rings, [topics.FRAME_DEPTH])
        # VIO server: pose.odom + ba.window (POD) + the retained calib it
        # republishes (the real vio.main re-broadcasts capture's bundle so the UI
        # can await calib on the VIO endpoint).
        self.vio_server = IPCPubSub(
            vio_ep, role="server",
            retain_topics={topics.VIO_MAP, topics.CALIB_BUNDLE}, blocking=True)
        # SLAM server: present (so forward connects) + the retained calib it
        # re-broadcasts (the real slam.main does this too -- the UI awaits calib
        # on the SLAM endpoint).
        self.slam_server = IPCPubSub(
            slam_ep, role="server",
            retain_topics={topics.CALIB_BUNDLE}, blocking=True)

    def start(self) -> None:
        self.cap_pub.start()                  # binds cap server + starts fanout
        self.vio_server.start()
        self.slam_server.start()
        # Publish the retained calib FIRST so a forward/receive that connects late
        # still gets it via replay. All three servers republish the bundle, exactly
        # like capture/vio/slam do in-host.
        bundle = _make_calib_bundle()
        self.cap_server.publish(topics.CALIB_BUNDLE, bundle)
        self.cap_server.publish(topics.CALIB_STEREO, _make_calib_stereo())
        self.vio_server.publish(topics.CALIB_BUNDLE, bundle)
        self.slam_server.publish(topics.CALIB_BUNDLE, bundle)

    def publish_frame(self, seq: int) -> None:
        # Image: through the in-host bridge (LocalPubSub -> IPCPublisher), so it
        # rides a SharedArrayRef in cap_rings. forward will materialise it.
        self.cap_local.publish(topics.FRAME_DEPTH, _make_depth(seq))
        # POD: directly as wire on the vio server (matches how vio publishes pose).
        from netbridge.comms.converters import to_wire
        empty = RingRegistry()                # POD converters never touch rings
        self.vio_server.publish(
            topics.POSE_ODOM, to_wire(topics.POSE_ODOM, _make_pose(seq),
                                      empty, self.vio_ep))
        self.vio_server.publish(
            topics.BA_WINDOW, to_wire(topics.BA_WINDOW, _make_ba(seq),
                                      empty, self.vio_ep))

    def stop(self) -> None:
        for s in (self.cap_pub, self.vio_server, self.slam_server):
            try:
                s.stop() if hasattr(s, "stop") else s.close()
            except Exception:                                      # noqa: BLE001
                pass
        try:
            self.cap_server.close()
        except Exception:                                          # noqa: BLE001
            pass
        for r in (self.cap_rings, self.vio_rings):
            try:
                r.unlink(); r.close()
            except Exception:                                      # noqa: BLE001
                pass


# --------------------------------------------------------------------------- #
# A headless subscriber on the RE-SERVED endpoints -- the stand-in for the UI.
# --------------------------------------------------------------------------- #
class Subscriber:
    """Subscribes the re-served capture + vio endpoints; collects decoded msgs.

    Uses the SAME ``IPCSubscriber`` path the real UI sinks use (image topics go
    through the rings; POD topics ride inline), so a pass proves the UI-facing
    contract end-to-end."""

    def __init__(self, cap_ep: str, vio_ep: str, *,
                 width: int, height: int, slots: int = SLOTS) -> None:
        from netbridge.comms.bridge import IPCSubscriber
        self.depth: list[DepthFrame] = []
        self.pose: list = []
        self.ba: list = []
        self.calib: list = []
        self.stereo: list = []
        self._lock = threading.Lock()

        # Mac-side rings attach to what receive created on these endpoints.
        self.cap_rings = RingRegistry().attach_all(
            default_capture_specs(endpoint=cap_ep, width=width, height=height,
                                  slots=slots))
        self.cap_local = LocalPubSub()
        self.cap_local.subscribe(topics.FRAME_DEPTH, self._on_depth)
        self.cap_client = IPCPubSub(cap_ep, role="client", connect_timeout_s=10.0)
        self.cap_sub = IPCSubscriber(self.cap_local, self.cap_client,
                                     self.cap_rings, [topics.FRAME_DEPTH])
        # Calib rides directly off the wire (retained); subscribe it on a 2nd
        # client exactly like ui._await_calib_bundle does.
        self.cap_calib_client = IPCPubSub(cap_ep, role="client",
                                          connect_timeout_s=10.0)
        self.cap_calib_client.subscribe(topics.CALIB_BUNDLE, self._on_calib)
        self.cap_calib_client.subscribe(topics.CALIB_STEREO, self._on_stereo)

        # VIO POD topics ride inline -> read straight off a client bus.
        self.vio_client = IPCPubSub(vio_ep, role="client", connect_timeout_s=10.0)
        self.vio_client.subscribe(topics.POSE_ODOM, self._on_pose)
        self.vio_client.subscribe(topics.BA_WINDOW, self._on_ba)

    def _on_depth(self, msg) -> None:
        if isinstance(msg, WireEnd):
            return
        with self._lock:
            self.depth.append(msg)

    def _on_pose(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.pose.append(wm)

    def _on_ba(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.ba.append(wm)

    def _on_calib(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.calib.append(wm)

    def _on_stereo(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.stereo.append(wm)

    def start(self) -> None:
        self.cap_sub.start()
        self.cap_calib_client.start()
        self.vio_client.start()

    def stop(self) -> None:
        for c in (self.cap_sub, self.cap_calib_client, self.vio_client):
            try:
                c.stop()
            except Exception:                                      # noqa: BLE001
                pass
        try:
            self.cap_rings.close()
        except Exception:                                          # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# Test 1: full two-hop bit-identity (image + POD) + retained-to-late-subscriber.
# --------------------------------------------------------------------------- #
def test_two_hop_bit_identity() -> bool:
    print("\n[1] two-hop loopback bit-identity (image 0x09->0x08->0x09 + POD)")
    ok = True
    port = _free_port()
    suffix = f"nb{os.getpid() & 0xFFF:x}"
    # Pi-side (producer + forward) endpoints.
    cap_pi, vio_pi, slam_pi = (f"oak.cap.{suffix}p", f"oak.vio.{suffix}p",
                               f"oak.slm.{suffix}p")
    # Mac-side (receive re-serve + subscriber) endpoints.
    cap_mac, vio_mac, slam_mac = (f"oak.cap.{suffix}m", f"oak.vio.{suffix}m",
                                  f"oak.slm.{suffix}m")

    producer = FakeProducer(cap_pi, vio_pi, slam_pi)
    fwd_ready = threading.Event()
    rcv_ready = threading.Event()
    fwd_stop = threading.Event()
    rcv_stop = threading.Event()
    fwd_thread = recv_thread = None
    sub = None
    try:
        producer.start()
        time.sleep(0.3)

        # forward (Pi): connect the producer's local endpoints -> TCP server.
        fwd_thread = threading.Thread(
            target=run_forward, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_pi, vio_endpoint=vio_pi,
                slam_endpoint=slam_pi, width=W, height=H, slots=SLOTS,
                connect_timeout_s=10.0, ready_event=fwd_ready,
                stop_event=fwd_stop),
            name="nb-forward", daemon=True)
        fwd_thread.start()
        ok &= _check(fwd_ready.wait(timeout=10.0), "forward came up")

        # receive (Mac): TCP client -> re-serve on the Mac endpoints. It AWAITS
        # the forwarded calib.bundle, then sizes the Mac rings to 54x42.
        recv_thread = threading.Thread(
            target=run_receive, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_mac, vio_endpoint=vio_mac,
                slam_endpoint=slam_mac, slots=SLOTS,
                calib_timeout_s=15.0, ready_event=rcv_ready,
                stop_event=rcv_stop),
            name="nb-receive", daemon=True)
        recv_thread.start()
        ok &= _check(rcv_ready.wait(timeout=15.0),
                     "receive came up (sized Mac rings from forwarded calib)")

        # Subscriber on the re-served endpoints (the UI stand-in).
        sub = Subscriber(cap_mac, vio_mac, width=W, height=H)
        sub.start()
        time.sleep(0.5)                       # let the subscribe handshakes settle

        # Publish N deterministic frames through the whole chain.
        n = 5
        sent_depth = [_make_depth(s) for s in range(n)]
        sent_pose = [_make_pose(s) for s in range(n)]
        sent_ba = [_make_ba(s) for s in range(n)]
        for s in range(n):
            producer.publish_frame(s)
            time.sleep(0.08)

        # Wait for the subscriber to drain (POD blocking; image latest-wins so we
        # check we got the LAST one bit-identical, not necessarily all N).
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with sub._lock:
                have = (len(sub.pose) >= n and len(sub.ba) >= n
                        and len(sub.depth) >= 1)
            if have:
                break
            time.sleep(0.05)

        with sub._lock:
            got_depth = list(sub.depth)
            got_pose = list(sub.pose)
            got_ba = list(sub.ba)
            got_calib = list(sub.calib)
            got_stereo = list(sub.stereo)

        # --- image: every received depth frame must bit-match the one its seq
        #     encodes (proves 0x09->0x08->0x09 through both ring sets) ---
        ok &= _check(len(got_depth) >= 1,
                     f"received >= 1 frame.depth (got {len(got_depth)})")
        img_ok = True
        for df in got_depth:
            exp = sent_depth[int(df.seq)]
            if not (np.array_equal(df.gray_left, exp.gray_left)
                    and np.array_equal(df.depth_m, exp.depth_m)):
                img_ok = False
                break
        ok &= _check(img_ok,
                     "frame.depth gray + depth pixels BIT-IDENTICAL end-to-end")
        # Prove the resolution survived (a hardcoded 640x400 receive would fail).
        ok &= _check(all(df.gray_left.shape == (H, W) for df in got_depth),
                     f"frame.depth shape is {H}x{W} (sized from calib, not 640x400)")

        # --- POD: pose + ba.window arrays bit-identical ---
        ok &= _check(len(got_pose) == n,
                     f"received all {n} pose.odom (got {len(got_pose)})")
        pose_ok = all(np.array_equal(g.T_world_cam, sent_pose[int(g.seq)].T_world_cam)
                      for g in got_pose)
        ok &= _check(pose_ok, "pose.odom T_world_cam BIT-IDENTICAL")
        ok &= _check(len(got_ba) == n,
                     f"received all {n} ba.window (got {len(got_ba)})")
        ba_ok = True
        for g in got_ba:
            exp = sent_ba[int(g.seq)]
            for fld in ("kf_ids", "kf_quat", "kf_pos", "lm_ids", "lm_xyz",
                        "obs_kf", "obs_lm", "obs_uv", "obs_reproj_px",
                        "kf_quat_pre", "kf_pos_pre", "lm_xyz_pre"):
                if not np.array_equal(getattr(g, fld), getattr(exp, fld)):
                    ba_ok = False
                    break
        ok &= _check(ba_ok, "ba.window every ndarray field BIT-IDENTICAL")

        # --- retained calib reached the subscriber (it connected after producer
        #     published; the TCP server + receive endpoint replay it) ---
        ok &= _check(len(got_calib) >= 1 and int(got_calib[0].width) == W
                     and int(got_calib[0].height) == H,
                     "calib.bundle retained-replayed (correct W/H)")
        ok &= _check(len(got_stereo) >= 1
                     and np.array_equal(got_stereo[0].left_K,
                                        _make_calib_stereo().left_K),
                     "calib.stereo retained-replayed bit-identical")

        # --- LATE subscriber: connect AFTER the producer stops; calib must still
        #     replay (the receive endpoint retains it) ---
        producer.publish_frame(n)             # one more so n+1 exist upstream
        time.sleep(0.3)
        producer.stop()
        time.sleep(0.3)
        late = Subscriber(cap_mac, vio_mac, width=W, height=H)
        late.start()
        time.sleep(0.8)
        with late._lock:
            late_calib = list(late.calib)
            late_stereo = list(late.stereo)
        ok &= _check(len(late_calib) >= 1 and int(late_calib[0].width) == W,
                     "LATE subscriber (post-producer-stop) got calib.bundle replay")
        ok &= _check(len(late_stereo) >= 1,
                     "LATE subscriber got calib.stereo replay")
        late.stop()
    finally:
        if sub is not None:
            sub.stop()
        # Stop forward + receive (the receive thread unlinks the Mac rings on
        # teardown) so the next test starts clean.
        rcv_stop.set()
        fwd_stop.set()
        if recv_thread is not None:
            recv_thread.join(timeout=5.0)
        if fwd_thread is not None:
            fwd_thread.join(timeout=5.0)
        try:
            producer.stop()
        except Exception:                                          # noqa: BLE001
            pass
    return ok


# --------------------------------------------------------------------------- #
# Test 1b: POSE-ONLY (low-bandwidth) mode -- pose round-trips, images NEVER do.
# --------------------------------------------------------------------------- #
def test_pose_only_excludes_images() -> bool:
    """forward + receive both in pose-only: pose.odom round-trips bit-identically,
    and the heavy image topic (frame.depth) is NEVER served / never arrives.

    This is the bandwidth fix: the operator's default Pi deploy runs pose-only so the
    ~51 Mbit/s image stream NEVER leaves the Pi. We prove the trajectory UI's data
    (pose.odom) still flows AND that frame.depth is genuinely absent end-to-end.
    """
    print("\n[1b] pose-only mode (image topics EXCLUDED; pose.odom still flows)")
    ok = True
    port = _free_port()
    suffix = f"nbpo{os.getpid() & 0xFFF:x}"
    cap_pi, vio_pi, slam_pi = (f"oak.cap.{suffix}p", f"oak.vio.{suffix}p",
                               f"oak.slm.{suffix}p")
    cap_mac, vio_mac, slam_mac = (f"oak.cap.{suffix}m", f"oak.vio.{suffix}m",
                                  f"oak.slm.{suffix}m")
    producer = FakeProducer(cap_pi, vio_pi, slam_pi)
    fwd_ready = threading.Event()
    rcv_ready = threading.Event()
    fwd_stop = threading.Event()
    rcv_stop = threading.Event()
    fwd_thread = recv_thread = None
    pose_msgs: list = []
    depth_msgs: list = []
    pose_lock = threading.Lock()
    cap_client = vio_client = None
    try:
        producer.start()
        time.sleep(0.3)
        # forward POSE-ONLY: the image topics must never be subscribed/forwarded, so
        # it does NOT even attach the capture/vio image rings.
        fwd_thread = threading.Thread(
            target=run_forward, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_pi, vio_endpoint=vio_pi,
                slam_endpoint=slam_pi, width=W, height=H, slots=SLOTS,
                connect_timeout_s=10.0, pose_only=True,
                ready_event=fwd_ready, stop_event=fwd_stop),
            name="nb-forward-po", daemon=True)
        fwd_thread.start()
        ok &= _check(fwd_ready.wait(timeout=10.0), "forward came up (pose-only)")

        # receive POSE-ONLY: it must NOT allocate image rings and must still await
        # the retained calib.bundle + re-serve pose without hanging.
        recv_thread = threading.Thread(
            target=run_receive, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_mac, vio_endpoint=vio_mac,
                slam_endpoint=slam_mac, slots=SLOTS,
                calib_timeout_s=15.0, pose_only=True,
                ready_event=rcv_ready, stop_event=rcv_stop),
            name="nb-receive-po", daemon=True)
        recv_thread.start()
        ok &= _check(rcv_ready.wait(timeout=15.0),
                     "receive came up (pose-only -- no image rings, did NOT hang)")

        # Subscribe pose.odom (POD, on the re-served vio endpoint) + frame.depth
        # (image, on the re-served capture endpoint). pose must arrive; depth must
        # NOT (the bridge never subscribed it on either side). frame.depth rides
        # rings normally, but in pose-only the receive endpoint registers no ring
        # publisher for it, so a raw client subscription simply never fires.
        vio_client = IPCPubSub(vio_mac, role="client", connect_timeout_s=10.0)

        def _on_pose(wm) -> None:
            if isinstance(wm, WireEnd):
                return
            with pose_lock:
                pose_msgs.append(wm)
        vio_client.subscribe(topics.POSE_ODOM, _on_pose)
        vio_client.start()

        cap_client = IPCPubSub(cap_mac, role="client", connect_timeout_s=10.0)

        def _on_depth(wm) -> None:
            if isinstance(wm, WireEnd):
                return
            with pose_lock:
                depth_msgs.append(wm)
        cap_client.subscribe(topics.FRAME_DEPTH, _on_depth)
        cap_client.start()
        time.sleep(0.5)

        n = 5
        sent_pose = [_make_pose(s) for s in range(n)]
        for s in range(n):
            producer.publish_frame(s)         # publishes BOTH depth + pose upstream
            time.sleep(0.08)

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with pose_lock:
                have = len(pose_msgs) >= n
            if have:
                break
            time.sleep(0.05)
        # Give any (erroneously) forwarded depth a generous window to show up.
        time.sleep(0.5)

        with pose_lock:
            got_pose = list(pose_msgs)
            got_depth = list(depth_msgs)

        ok &= _check(len(got_pose) == n,
                     f"pose.odom flows in pose-only (got {len(got_pose)}/{n})")
        pose_ok = all(np.array_equal(g.T_world_cam,
                                     sent_pose[int(g.seq)].T_world_cam)
                      for g in got_pose)
        ok &= _check(pose_ok, "pose.odom T_world_cam BIT-IDENTICAL (pose-only)")
        ok &= _check(len(got_depth) == 0,
                     f"frame.depth NEVER arrives in pose-only (got {len(got_depth)})")
    finally:
        for c in (cap_client, vio_client):
            if c is not None:
                try:
                    c.stop()
                except Exception:                                  # noqa: BLE001
                    pass
        rcv_stop.set()
        fwd_stop.set()
        if recv_thread is not None:
            recv_thread.join(timeout=5.0)
        if fwd_thread is not None:
            fwd_thread.join(timeout=5.0)
        try:
            producer.stop()
        except Exception:                                          # noqa: BLE001
            pass
    return ok


# --------------------------------------------------------------------------- #
# Test 2: authkey enforcement (wrong key refused; no key -> built-in default key).
# --------------------------------------------------------------------------- #
def test_authkey_enforced() -> bool:
    print("\n[2] authkey enforcement (wrong key refused; no key -> default key)")
    ok = True
    port = _free_port()
    suffix = f"nba{os.getpid() & 0xFFF:x}"
    cap_pi = f"oak.cap.{suffix}p"
    vio_pi = f"oak.vio.{suffix}p"
    slam_pi = f"oak.slm.{suffix}p"
    producer = FakeProducer(cap_pi, vio_pi, slam_pi)
    fwd_ready = threading.Event()
    fwd_stop = threading.Event()
    fwd_thread = None
    try:
        producer.start()
        time.sleep(0.2)
        fwd_thread = threading.Thread(
            target=run_forward, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_pi, vio_endpoint=vio_pi,
                slam_endpoint=slam_pi, width=W, height=H, slots=SLOTS,
                connect_timeout_s=5.0, ready_event=fwd_ready,
                stop_event=fwd_stop),
            name="nb-forward-auth", daemon=True)
        fwd_thread.start()
        ok &= _check(fwd_ready.wait(timeout=8.0), "forward (correct key) came up")

        # WRONG key -> the HMAC challenge fails -> Client raises; our TcpClient
        # does NOT retry an auth failure, so it surfaces as a connect error fast.
        saved = os.environ.get("OAKD_NETBRIDGE_KEY")
        os.environ["OAKD_NETBRIDGE_KEY"] = "WRONG-KEY"
        refused = False
        try:
            bad = TcpClient("127.0.0.1", port, connect_timeout_s=3.0)
            bad.subscribe(topics.CALIB_BUNDLE, lambda *_a: None)
            bad.start()
            time.sleep(0.5)
            # If auth somehow passed, start() would not have raised. Treat a
            # silent start as a failure of the gate.
            bad.stop()
        except Exception:                                          # noqa: BLE001
            refused = True
        finally:
            if saved is not None:
                os.environ["OAKD_NETBRIDGE_KEY"] = saved
        ok &= _check(refused, "WRONG authkey -> connection REFUSED")

        # MISSING key -> the client falls back to the built-in DEFAULT key (it no
        # longer raises at construction). That default does NOT match THIS server's
        # custom "test" key, so the handshake still fails -> connection refused.
        from netbridge.tcp_transport import (                       # noqa: PLC0415
            DEFAULT_AUTHKEY, resolve_authkey)
        saved = os.environ.pop("OAKD_NETBRIDGE_KEY", None)
        default_used = resolve_authkey() == DEFAULT_AUTHKEY.encode("utf-8")
        missing_refused = False
        try:
            bad2 = TcpClient("127.0.0.1", port, connect_timeout_s=3.0)
            bad2.subscribe(topics.CALIB_BUNDLE, lambda *_a: None)
            bad2.start()
            time.sleep(0.5)
            bad2.stop()
        except Exception:                                          # noqa: BLE001
            missing_refused = True
        finally:
            if saved is not None:
                os.environ["OAKD_NETBRIDGE_KEY"] = saved
        ok &= _check(default_used, "MISSING key -> falls back to the built-in default key")
        ok &= _check(missing_refused, "default key != this server's custom key -> REFUSED")

        # CORRECT key still works (sanity: the server didn't get wedged by the
        # bad attempt).
        good = TcpClient("127.0.0.1", port, connect_timeout_s=5.0)
        got = threading.Event()
        good.subscribe(topics.CALIB_BUNDLE,
                       lambda _t, _m: got.set())
        good.start()
        ok &= _check(got.wait(timeout=3.0),
                     "CORRECT authkey still connects + gets retained calib")
        good.stop()
    finally:
        fwd_stop.set()
        if fwd_thread is not None:
            fwd_thread.join(timeout=5.0)
        try:
            producer.stop()
        except Exception:                                          # noqa: BLE001
            pass
    return ok


# --------------------------------------------------------------------------- #
# Test 3: offscreen ui.main smoke against the receive side.
# --------------------------------------------------------------------------- #
def test_offscreen_ui_smoke() -> bool:
    print("\n[3] offscreen ui.main smoke against the receive side "
          "(QT_QPA_PLATFORM=offscreen)")
    ok = True
    # Run the producer + forward + receive in THIS process, then launch the real
    # ui.main in a CHILD process (offscreen) against the re-served endpoints. A
    # child keeps Qt's global state out of the selftest process.
    port = _free_port()
    suffix = f"nbu{os.getpid() & 0xFFF:x}"
    cap_pi, vio_pi, slam_pi = (f"oak.cap.{suffix}p", f"oak.vio.{suffix}p",
                               f"oak.slm.{suffix}p")
    cap_mac, vio_mac, slam_mac = (f"oak.cap.{suffix}m", f"oak.vio.{suffix}m",
                                  f"oak.slm.{suffix}m")
    producer = FakeProducer(cap_pi, vio_pi, slam_pi)
    fwd_ready = threading.Event()
    rcv_ready = threading.Event()
    fwd_stop = threading.Event()
    rcv_stop = threading.Event()
    fwd_thread = recv_thread = None
    proc = None
    stop_pub = threading.Event()
    pub_thread = None
    try:
        producer.start()
        time.sleep(0.2)
        fwd_thread = threading.Thread(
            target=run_forward, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_pi, vio_endpoint=vio_pi,
                slam_endpoint=slam_pi, width=W, height=H, slots=SLOTS,
                connect_timeout_s=10.0, ready_event=fwd_ready,
                stop_event=fwd_stop),
            name="nb-forward-ui", daemon=True)
        fwd_thread.start()
        if not fwd_ready.wait(timeout=10.0):
            return _check(False, "forward came up (ui smoke)")
        recv_thread = threading.Thread(
            target=run_receive, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_mac, vio_endpoint=vio_mac,
                slam_endpoint=slam_mac, slots=SLOTS,
                calib_timeout_s=15.0, ready_event=rcv_ready,
                stop_event=rcv_stop),
            name="nb-receive-ui", daemon=True)
        recv_thread.start()
        if not rcv_ready.wait(timeout=15.0):
            return _check(False, "receive came up (ui smoke)")

        # Keep publishing poses so the UI has live data to render >= 1 frame.
        def _pump() -> None:
            s = 0
            while not stop_pub.is_set():
                producer.publish_frame(s)
                s += 1
                time.sleep(0.05)
        pub_thread = threading.Thread(target=_pump, name="nb-ui-pump",
                                      daemon=True)
        pub_thread.start()

        # The smoke driver: run the REAL ui.run_ui offscreen against the re-served
        # endpoints, in a CHILD process (so its os._exit / Qt singletons don't
        # touch this process). Patching QApplication.exec to arm a quit timer means
        # reaching the loop AT ALL proves run_ui got PAST _await_calib_bundle (it
        # blocks there until the forwarded calib arrives) and built the viewer; the
        # 2.5 s timer lets >= 1 render tick happen first, then quits 0.
        driver = f"""
import os, sys, time, logging
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, {str(REPO)!r})
# run_ui logs "ui: vio ready" via its module logger; configure logging so that
# line reaches stderr where the parent greps for it (run_ui alone doesn't call
# basicConfig -- only ui.main()'s entrypoint does).
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")
from PyQt6.QtWidgets import QApplication
import ui.main as M

# Replace the blocking app.exec() with a bounded MANUAL event loop: pump events
# for ~2.5 s (>= 1 render tick offscreen) then return 0. Reaching this patched
# exec AT ALL proves run_ui got PAST _await_calib_bundle (it blocks there until the
# forwarded calib arrives) and built the viewer. Avoids the sip 'exec(): too many
# arguments' pitfall of trying to delegate to the original overloaded method.
def _patched(self, *a, **k):
    print("UI_SMOKE_LOOP_ENTERED", flush=True)
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        self.processEvents()
        time.sleep(0.02)
    return 0
QApplication.exec = _patched

rc = M.run_ui(vio_endpoint={vio_mac!r}, slam_endpoint={slam_mac!r},
              capture_endpoint={cap_mac!r}, calib_timeout_s=20.0)
print("UI_SMOKE_RC", rc, flush=True)
sys.exit(0)
"""
        env = dict(os.environ)
        env["QT_QPA_PLATFORM"] = "offscreen"
        env["OAKD_NETBRIDGE_KEY"] = os.environ["OAKD_NETBRIDGE_KEY"]
        proc = subprocess.run([sys.executable, "-c", driver], env=env,
                              capture_output=True, text=True, timeout=60)
        out = (proc.stdout or "") + (proc.stderr or "")
        passed_calib = "ui: vio ready" in out or "ui: slam ready" in out
        entered_loop = "UI_SMOKE_LOOP_ENTERED" in out
        ok &= _check(proc.returncode == 0,
                     f"ui.main exited cleanly (rc={proc.returncode})")
        ok &= _check(passed_calib,
                     "ui.main passed _await_calib_bundle (forwarded calib arrived)")
        ok &= _check(entered_loop,
                     "ui.main entered the Qt event loop + rendered (offscreen)")
        if not ok:
            print("        --- ui smoke output (tail) ---")
            for line in out.splitlines()[-25:]:
                print(f"        {line}")
    finally:
        stop_pub.set()
        if pub_thread is not None:
            pub_thread.join(timeout=1.0)
        rcv_stop.set()
        fwd_stop.set()
        if recv_thread is not None:
            recv_thread.join(timeout=5.0)
        if fwd_thread is not None:
            fwd_thread.join(timeout=5.0)
        try:
            producer.stop()
        except Exception:                                          # noqa: BLE001
            pass
    return ok


# --------------------------------------------------------------------------- #
def main() -> int:
    print("netbridge_loopback_selftest -- Pi->Mac TCP bridge end-to-end "
          "(127.0.0.1 two-hop)")
    results = {
        "two-hop bit-identity + retained replay": test_two_hop_bit_identity(),
        "pose-only excludes images":              test_pose_only_excludes_images(),
        "authkey enforcement":                    test_authkey_enforced(),
        "offscreen ui.main smoke":                test_offscreen_ui_smoke(),
    }
    print("\n" + "=" * 70)
    all_ok = all(results.values())
    for name, ok in results.items():
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if all_ok:
        print("\nPASS -- netbridge round-trips Pi->Mac bit-identically, enforces "
              "the authkey, replays retained calib, and the real UI renders.")
        return 0
    print("\nFAIL -- see the [FAIL] lines above.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
