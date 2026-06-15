#!/usr/bin/env python3
"""netbridge REAL-LAUNCHER replay gate -- the regression for the forward crash.

This is the test that was MISSING. The pre-existing ``netbridge_loopback_selftest``
drives forward/receive with a FAKE producer that PRE-CREATES every Pi-side ring at
a fixed 54x42 before forward attaches. That short-circuited two things the REAL
launcher does and forward got WRONG, so the loopback passed while the real path
(``./run.sh --no-ui --vl53l9cx --direct --forward ...``) CRASHED:

  (a) **calib-driven ToF resolution.** The launcher passes forward the RAW camera
      res (``--width 640 --height 400``), but under ``--vl53l9cx`` capture creates
      its rings at the ToF grid (54x42). The old forward attached at 640x400 -> a
      too-small shm buffer ("buffer is too small for requested array"). The fix
      awaits capture's retained ``calib.bundle`` and attaches at the ACTUAL grid.

  (b) **async ring creation by separate processes.** capture/vio/slam + forward are
      INDEPENDENT processes booting concurrently; vio allocates its
      ``kf_gray``/``kf_depth`` rings only AFTER it comes up and awaits calib, i.e.
      AFTER forward (which raced ahead on the same calib) tries to attach. The old
      forward did a single ``attach_all`` -> ``FileNotFoundError`` startup-race
      crash. The fix retries the attach until the producer has created the ring.

So this gate runs the ACTUAL launcher (replay -- no device) end to end:

    launcher.main --no-ui --direct [--vl53l9cx] --forward 127.0.0.1:<port>
        --(spawns)--> imu_camera + vio + slam + netbridge.forward (Pi side)
        --TCP 127.0.0.1--> netbridge.receive (Mac side, in THIS process)
        --AF_UNIX--> headless subscriber (the UI stand-in)

and ASSERTS the subscriber actually RECEIVES live data: >= 1 ``pose.odom``, >= 1
image frame (``frame.depth`` AND ``imucam.sample``) at the RIGHT shape per mode,
and >= 1 ``slam.map`` -- plus a CLEAN teardown (launcher exits 0, receive thread
joins, no leftover sockets/shm). It covers BOTH ring-resolution paths:

  * ``--vl53l9cx``  -> 54x42 (the ToF path -- the one that crashed)
  * full res        -> 640x400 (the no-ToF path)

so a future regression in EITHER attach path is caught. Replay-only + headless ->
runs on the dev box AND on the Pi. Also runs ONE offscreen ``ui.main`` smoke
against the receive side (the "tester must verify UI" rule).

Run::

    OAKD_NETBRIDGE_KEY=test .venv/bin/python \\
        verification/netbridge_live_replay_selftest.py
"""
from __future__ import annotations

import glob
import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

# The bridge refuses to open a socket without a key; set a test secret BEFORE any
# netbridge import constructs a transport (and BEFORE we spawn the launcher, which
# inherits this env and validates --forward needs the key).
os.environ.setdefault("OAKD_NETBRIDGE_KEY", "test")

from netbridge.comms import topics                                  # noqa: E402
from netbridge.comms.bridge import IPCSubscriber                    # noqa: E402
from netbridge.comms.ipc import IPCPubSub                           # noqa: E402
from netbridge.comms.pubsub import LocalPubSub                      # noqa: E402
from netbridge.comms.ring_registry import (                        # noqa: E402
    RingRegistry, default_capture_specs,
)
from netbridge.comms.wire import WireEnd                            # noqa: E402
from netbridge.receive import run_receive                          # noqa: E402

#: The gold session every replay path runs against (short loop -> a slam.map).
SESSION = "sessions/gold/lab_loop_30s"
#: Cap the replay so a CI run is bounded; >= a few keyframes so slam.map fires.
MAX_FRAMES = 60


def _check(cond: bool, msg: str) -> bool:
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    return bool(cond)


def _free_port() -> int:
    """Grab an ephemeral TCP port (then release it for the launcher to rebind)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


# --------------------------------------------------------------------------- #
# A headless subscriber on the RE-SERVED Mac endpoints -- the UI stand-in.
# --------------------------------------------------------------------------- #
class Subscriber:
    """Subscribes the re-served capture/vio/slam endpoints; counts decoded msgs.

    Uses the SAME ``IPCSubscriber`` path the real UI sinks use (image topics go
    through the Mac rings; POD topics ride inline), so the counts prove the
    UI-facing contract end-to-end. Records the first depth + imucam shape so the
    test can assert the resolution survived the bridge (a wrong-res attach in
    forward/receive would corrupt or drop these).
    """

    def __init__(self, cap_ep: str, vio_ep: str, slam_ep: str, *,
                 width: int, height: int, slots: int = 64) -> None:
        self.n_pose = 0
        self.n_depth = 0
        self.n_imucam = 0
        self.n_slam = 0
        self.depth_shape: tuple[int, int] | None = None
        self.imucam_shape: tuple[int, int] | None = None
        self._lock = threading.Lock()

        # Mac-side capture rings attach to what receive created on this endpoint.
        # (vio's kf rings exist too but we don't subscribe keyframe here.)
        self.cap_rings = RingRegistry().attach_all(
            default_capture_specs(endpoint=cap_ep, width=width, height=height,
                                  slots=slots))
        self.cap_local = LocalPubSub()
        self.cap_local.subscribe(topics.FRAME_DEPTH, self._on_depth)
        self.cap_local.subscribe(topics.IMUCAM_SAMPLE, self._on_imucam)
        self.cap_client = IPCPubSub(cap_ep, role="client", connect_timeout_s=15.0)
        self.cap_sub = IPCSubscriber(
            self.cap_local, self.cap_client, self.cap_rings,
            [topics.FRAME_DEPTH, topics.IMUCAM_SAMPLE])

        # POD topics ride inline -> read straight off a client bus.
        self.vio_client = IPCPubSub(vio_ep, role="client", connect_timeout_s=15.0)
        self.vio_client.subscribe(topics.POSE_ODOM, self._on_pose)
        self.slam_client = IPCPubSub(slam_ep, role="client", connect_timeout_s=15.0)
        self.slam_client.subscribe(topics.SLAM_MAP, self._on_slam)

    def _on_depth(self, m) -> None:
        if isinstance(m, WireEnd):
            return
        with self._lock:
            self.n_depth += 1
            if self.depth_shape is None:
                self.depth_shape = tuple(m.depth_m.shape)

    def _on_imucam(self, m) -> None:
        if isinstance(m, WireEnd):
            return
        with self._lock:
            self.n_imucam += 1
            g = getattr(m, "gray_left", None)
            if self.imucam_shape is None and g is not None:
                self.imucam_shape = tuple(g.shape)

    def _on_pose(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.n_pose += 1

    def _on_slam(self, wm) -> None:
        if isinstance(wm, WireEnd):
            return
        with self._lock:
            self.n_slam += 1

    def start(self) -> None:
        self.cap_sub.start()
        self.vio_client.start()
        self.slam_client.start()

    def snapshot(self) -> dict:
        with self._lock:
            return dict(pose=self.n_pose, depth=self.n_depth,
                        imucam=self.n_imucam, slam=self.n_slam,
                        depth_shape=self.depth_shape,
                        imucam_shape=self.imucam_shape)

    def stop(self) -> None:
        for c in (self.cap_sub, self.vio_client, self.slam_client):
            try:
                c.stop()
            except Exception:                                      # noqa: BLE001
                pass
        try:
            self.cap_rings.close()
        except Exception:                                          # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
def _run_one(*, vl53: bool) -> bool:
    """Real launcher -> forward -> receive -> subscriber for ONE resolution mode.

    Spawns the ACTUAL launcher (replay) with ``--forward``; runs ``receive`` + the
    subscriber in this process; asserts frames cross at the right shape + a clean
    teardown. ``vl53`` selects the 54x42 ToF attach path vs the 640x400 full-res
    path -- both must work (the bug only crashed the ToF path, but a future change
    could break either).
    """
    tag = "vl53l9cx (54x42 ToF)" if vl53 else "full-res (640x400)"
    print(f"\n[{'A' if vl53 else 'B'}] real launcher --forward replay -- {tag}")
    ok = True
    exp_h, exp_w = (42, 54) if vl53 else (400, 640)
    port = _free_port()
    # Unique endpoint suffix per mode/PID so two runs (or both modes) never collide
    # on the same AF_UNIX socket / shm name. Keep it short (macOS shm-name limit).
    suffix = f"nlr{os.getpid() & 0xFF:x}{'v' if vl53 else 'f'}"
    # Mac-side (receive re-serve + subscriber) endpoints -- distinct from the Pi
    # ones the launcher creates (we add an 'm' so they never clash with the
    # launcher's own '.<suffix>' sockets).
    cap_mac = f"oak.cap.{suffix}m"
    vio_mac = f"oak.vio.{suffix}m"
    slam_mac = f"oak.slm.{suffix}m"

    env = dict(os.environ)
    env["OAKD_NETBRIDGE_KEY"] = os.environ["OAKD_NETBRIDGE_KEY"]

    launch_argv = [sys.executable, "-m", "launcher.main",
                   "--no-ui", "--direct",
                   "--session", SESSION,
                   "--max-frames", str(MAX_FRAMES),
                   "--endpoint-suffix", suffix,
                   "--forward", f"127.0.0.1:{port}"]
    if vl53:
        launch_argv.append("--vl53l9cx")

    launcher: subprocess.Popen | None = None
    sub: Subscriber | None = None
    rcv_ready = threading.Event()
    rcv_stop = threading.Event()
    recv_thread: threading.Thread | None = None
    try:
        launcher = subprocess.Popen(launch_argv, cwd=str(REPO), env=env)

        # receive (Mac): connect to the launcher's forward TCP port, AWAIT the
        # forwarded calib.bundle, size the Mac rings to the ACTUAL grid, re-serve.
        recv_thread = threading.Thread(
            target=run_receive, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_mac, vio_endpoint=vio_mac,
                slam_endpoint=slam_mac, slots=64,
                calib_timeout_s=90.0, ready_event=rcv_ready,
                stop_event=rcv_stop),
            name="nlr-receive", daemon=True)
        recv_thread.start()

        # If the launcher's forward crashed (the OLD bug), receive never gets
        # calib -> this times out, which is the regression signal.
        ok &= _check(rcv_ready.wait(timeout=90.0),
                     "receive came up (forward served the real launcher path)")
        if not rcv_ready.is_set():
            return ok                                  # nothing else to assert

        # Subscriber on the re-served endpoints (the UI stand-in).
        sub = Subscriber(cap_mac, vio_mac, slam_mac, width=exp_w, height=exp_h)
        sub.start()

        # Let the launcher's BOUNDED replay (--max-frames) run to COMPLETION and
        # exit on its OWN -- do NOT terminate it mid-replay, or its --no-ui path
        # never reaches its rc=0 (we'd see rc=143 from the SIGTERM). We keep
        # receive + the subscriber alive the whole time so the forward->receive
        # hop stays connected and the subscriber accumulates the full stream. The
        # clean-exit assertion below then proves the WHOLE chain shuts down clean.
        deadline = time.monotonic() + 120.0
        while time.monotonic() < deadline and launcher.poll() is None:
            time.sleep(0.2)
        # Grace period for the LAST replayed frames to traverse the TCP hop after
        # the launcher process has exited (its drain pushes the tail through).
        time.sleep(2.0)

        s = sub.snapshot()
        print(f"      counts: pose={s['pose']} depth={s['depth']} "
              f"imucam={s['imucam']} slam={s['slam']}  "
              f"depth_shape={s['depth_shape']} imucam_shape={s['imucam_shape']}")

        ok &= _check(s["pose"] >= 1,
                     f"received >= 1 pose.odom (got {s['pose']})")
        ok &= _check(s["depth"] >= 1,
                     f"received >= 1 frame.depth (got {s['depth']})")
        ok &= _check(s["imucam"] >= 1,
                     f"received >= 1 imucam.sample (got {s['imucam']})")
        ok &= _check(s["slam"] >= 1,
                     f"received >= 1 slam.map (got {s['slam']})")
        # The resolution MUST have survived the bridge: a wrong-res attach in
        # forward/receive (the (a) bug) would have crashed or corrupted these.
        ok &= _check(s["depth_shape"] == (exp_h, exp_w),
                     f"frame.depth shape is {exp_h}x{exp_w} "
                     f"(got {s['depth_shape']})")
        ok &= _check(s["imucam_shape"] == (exp_h, exp_w),
                     f"imucam.sample shape is {exp_h}x{exp_w} "
                     f"(got {s['imucam_shape']})")
    finally:
        # Tear DOWN in order: subscriber, receive (unlinks Mac rings), launcher.
        if sub is not None:
            sub.stop()
        rcv_stop.set()
        if recv_thread is not None:
            recv_thread.join(timeout=10.0)
        if launcher is not None:
            # The replay should have exited on its own (--max-frames). If it is
            # still alive (e.g. an assertion already failed early), terminate it.
            if launcher.poll() is None:
                launcher.terminate()
                try:
                    launcher.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    launcher.kill()
                    launcher.wait(timeout=5.0)

    # --- clean teardown assertions ---
    lrc = launcher.poll() if launcher is not None else None
    # The launcher exits 0 on a completed replay (its mains os._exit(0)).
    ok &= _check(lrc == 0, f"launcher exited cleanly (rc={lrc})")
    ok &= _check(recv_thread is not None and not recv_thread.is_alive(),
                 "receive thread joined (clean shutdown)")
    # No leftover AF_UNIX sockets for OUR suffix (launcher + receive both clean up).
    leftover = (glob.glob(f"/tmp/*{suffix}*")
                + glob.glob(f"/tmp/*{suffix}m*"))
    ok &= _check(not leftover, f"no leftover sockets for suffix {suffix!r} "
                               f"({len(leftover)} found)")
    return ok


# --------------------------------------------------------------------------- #
def test_vl53_path() -> bool:
    return _run_one(vl53=True)


def test_fullres_path() -> bool:
    return _run_one(vl53=False)


# --------------------------------------------------------------------------- #
# Offscreen ui.main smoke against the receive side (the "tester must verify UI"
# rule): run the REAL ui.run_ui offscreen against the re-served endpoints of a
# live launcher+forward+receive chain, in a CHILD process.
# --------------------------------------------------------------------------- #
def test_offscreen_ui_smoke() -> bool:
    print("\n[C] offscreen ui.main smoke against the receive side "
          "(real launcher --vl53l9cx --forward)")
    ok = True
    port = _free_port()
    suffix = f"nlu{os.getpid() & 0xFF:x}"
    cap_mac = f"oak.cap.{suffix}m"
    vio_mac = f"oak.vio.{suffix}m"
    slam_mac = f"oak.slm.{suffix}m"

    env = dict(os.environ)
    env["OAKD_NETBRIDGE_KEY"] = os.environ["OAKD_NETBRIDGE_KEY"]
    # A longer replay so the UI has live data through its whole bounded loop.
    launch_argv = [sys.executable, "-m", "launcher.main",
                   "--no-ui", "--direct", "--vl53l9cx",
                   "--session", SESSION, "--max-frames", "200",
                   "--endpoint-suffix", suffix,
                   "--forward", f"127.0.0.1:{port}"]

    launcher: subprocess.Popen | None = None
    rcv_ready = threading.Event()
    rcv_stop = threading.Event()
    recv_thread: threading.Thread | None = None
    proc = None
    try:
        launcher = subprocess.Popen(launch_argv, cwd=str(REPO), env=env)
        recv_thread = threading.Thread(
            target=run_receive, kwargs=dict(
                host="127.0.0.1", port=port,
                capture_endpoint=cap_mac, vio_endpoint=vio_mac,
                slam_endpoint=slam_mac, slots=64,
                calib_timeout_s=90.0, ready_event=rcv_ready,
                stop_event=rcv_stop),
            name="nlu-receive", daemon=True)
        recv_thread.start()
        if not rcv_ready.wait(timeout=90.0):
            return _check(False, "receive came up (ui smoke)")

        # Run the REAL ui.run_ui offscreen against the re-served endpoints in a
        # CHILD process. Patching QApplication.exec to a bounded manual loop means
        # reaching it AT ALL proves run_ui got PAST _await_calib_bundle (it blocks
        # there until the forwarded calib arrives) and built the viewer; the timer
        # lets >= 1 render tick happen, then quits 0. Same pattern as the loopback
        # selftest's ui smoke.
        driver = f"""
import os, sys, time, logging
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, {str(REPO)!r})
logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")
from PyQt6.QtWidgets import QApplication
import ui.main as M

def _patched(self, *a, **k):
    print("UI_SMOKE_LOOP_ENTERED", flush=True)
    deadline = time.monotonic() + 2.5
    while time.monotonic() < deadline:
        self.processEvents()
        time.sleep(0.02)
    return 0
QApplication.exec = _patched

rc = M.run_ui(vio_endpoint={vio_mac!r}, slam_endpoint={slam_mac!r},
              capture_endpoint={cap_mac!r}, calib_timeout_s=30.0)
print("UI_SMOKE_RC", rc, flush=True)
sys.exit(0)
"""
        denv = dict(env)
        denv["QT_QPA_PLATFORM"] = "offscreen"
        proc = subprocess.run([sys.executable, "-c", driver], env=denv,
                              capture_output=True, text=True, timeout=90)
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
        rcv_stop.set()
        if recv_thread is not None:
            recv_thread.join(timeout=10.0)
        if launcher is not None and launcher.poll() is None:
            launcher.terminate()
            try:
                launcher.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                launcher.kill()
                launcher.wait(timeout=5.0)
    return ok


# --------------------------------------------------------------------------- #
def main() -> int:
    print("netbridge_live_replay_selftest -- REAL launcher --forward replay "
          "end-to-end (the regression for the forward crash)")
    results = {
        "vl53l9cx (54x42 ToF) real-path e2e": test_vl53_path(),
        "full-res (640x400) real-path e2e":   test_fullres_path(),
        "offscreen ui.main smoke":            test_offscreen_ui_smoke(),
    }
    print("\n" + "=" * 70)
    all_ok = all(results.values())
    for name, ok in results.items():
        print(f"  [{'ok' if ok else 'FAIL'}] {name}")
    if all_ok:
        print("\nPASS -- the real launcher --forward path serves both the 54x42 "
              "ToF and the 640x400 full-res streams; pose/image/slam.map all cross "
              "at the right shape; clean teardown; the UI renders.")
        return 0
    print("\nFAIL -- see the [FAIL] lines above.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
