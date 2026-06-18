"""Multi-process launcher: boot imu_camera + vio + ba + slam in background, run ui
foreground.

The launcher's only job is process lifecycle management:

1. Spawn ``imu_camera`` (capture) in background (it owns the OAK-D, or replays a
   session).
2. Spawn ``vio``, ``ba`` and ``slam`` in background (they connect to capture's /
   vio's retained ``calib.bundle`` over IPC, then start their own IPC endpoints).
   ``ba`` (the windowed-BA backend) consumes vio's keyframe + publishes
   ``pose.refined``, which vio re-emits on its endpoint (UI keeps one endpoint).
3. Run the ``ui`` process in the FOREGROUND so the Qt event loop has the GUI
   focus and Ctrl-C / window-close cleanly tears everything down.
4. On UI exit (clean or crash), send SIGTERM to capture / vio / ba / slam, wait for
   them to drain (each has a SIGTERM handler that runs the same finally block
   the replay-end path uses), then SIGKILL stragglers.

This is a behaviour-for-behaviour port of the pre-split ``ours.proc.launcher``,
retargeted onto the four split projects' ``<project>.main`` entrypoints
(``imu_camera.main`` / ``vio.main`` / ``slam.main`` / ``ui.main``). The only
wire-level change is that the new ``imu_camera.main`` DEFAULTS to replay and
takes an explicit ``--live`` flag for hardware (the old ``ours.proc.capture``
defaulted to live), so the live branch passes ``--live`` and the replay branch
passes ``--session``.

Endpoint naming
---------------
By default the launcher uses the canonical endpoint names ``oak.capture``,
``oak.vio``, ``oak.slam`` so external tools (calibration / visualize tools that
subscribe via IPC) work without configuration.  ``--endpoint-suffix SUFFIX`` (or
``--auto-suffix``) uniquifies them per launcher PID so two launchers can co-exist
(e.g. dev vs CI on the same machine).

Run::

    python -m launcher.main                                       # live, default
    python -m launcher.main --session sessions/gold/lab_loop_30s  # replay
    python -m launcher.main --width 1280 --height 800 --fps 15
"""
from __future__ import annotations

import argparse
import contextlib
import glob
import logging
import os
import signal
import subprocess
import sys
import tempfile
import time
from multiprocessing import shared_memory
from pathlib import Path

# Import-safe: ui.main imports PyQt6 LAZILY inside run_ui, so pulling in this one
# constant does NOT drag Qt into the launcher (verified by the QT-FREE check).
from ui.main import RESTART_EXIT_CODE

LOG = logging.getLogger("launcher.main")


# Endpoint roles + the ring names each role's process owns. Mirrors
# `imu_camera.comms.ring_registry.default_capture_specs()` (capture) and
# `default_vio_specs()` (vio); slam attaches but owns no rings. Used by
# `_cleanup_orphans` to unlink every stale POSIX shm segment from prior crashed
# runs so a fresh launch doesn't trip macOS's per-process fd / shm caps with
# EMFILE.
#
# Each live ring is now ONE block named `{ep}.{ring}` (slots are byte offsets --
# see SharedArrayRing). `_RING_SLOTS` is retained only for the legacy cleanup
# pass that reclaims the OLD per-slot `{ep}.{ring}.{i}` names left by crashed
# PRE-upgrade runs during the transition.
_RING_NAMES_BY_ROLE = {
    "cap": ("gray_left", "gray_right", "depth_m"),
    "vio": ("kf_gray", "kf_depth"),
    "slm": (),
    "ba": (),
}
_RING_SLOTS = 64


def _endpoints_history_path() -> Path:
    """File where every launcher persists its endpoint trio. cleanup reads it
    so a prior run whose sock files were already deleted (manual cleanup,
    /tmp reaper, etc.) is still recoverable next launch."""
    return Path(tempfile.gettempdir()) / "ours_ipc" / ".endpoints_seen"


def _record_endpoints(eps: list[str]) -> None:
    p = _endpoints_history_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    try:
        seen: set[str] = set()
        if p.is_file():
            seen = {ln.strip() for ln in p.read_text().splitlines() if ln.strip()}
        seen.update(eps)
        # Cap so the file doesn't grow forever; recent entries first.
        capped = sorted(seen)[:1024]
        p.write_text("\n".join(capped) + "\n")
    except OSError:
        pass


def _cleanup_orphans() -> None:
    """Best-effort: unlink every stale `oak.*` SHM segment + IPC socket file.

    macOS has no public listing API for POSIX shared memory, so we union
    endpoint candidates from THREE sources, in order of recency:
      1. The launcher's IPC socket directory (`oak.*.sock`).
      2. A persistent endpoints-seen file (every launcher records itself,
         so prior endpoints whose sock files were already deleted are still
         covered).
      3. (fallback) Brute-force the 4096 `l<hex>` PID-suffix space when both
         of the above turn up nothing -- runs on the EMFILE recovery path
         only, since it adds ~2.6 s on macOS.
    Missing segments are silently skipped -- this is a guard against
    accumulation, not a correctness operation.
    """
    sock_dir = Path(tempfile.gettempdir()) / "ours_ipc"
    endpoints: set[str] = set()
    if sock_dir.is_dir():
        for p in glob.glob(str(sock_dir / "oak.*.sock")):
            endpoints.add(Path(p).name[:-len(".sock")])
    hist = _endpoints_history_path()
    if hist.is_file():
        try:
            for ln in hist.read_text().splitlines():
                ln = ln.strip()
                if ln.startswith("oak."):
                    endpoints.add(ln)
        except OSError:
            pass
    if not endpoints:
        return
    unlinked = 0
    for ep in sorted(endpoints):
        parts = ep.split(".")
        if len(parts) != 3:
            continue
        role = parts[1]
        for ring in _RING_NAMES_BY_ROLE.get(role, ()):
            # Current layout: each ring is ONE block named exactly `{ep}.{ring}`
            # (slots are byte offsets inside it -- see SharedArrayRing). Plus a
            # legacy pass over the OLD per-slot names `{ep}.{ring}.{i}` so
            # segments leaked by a PRE-upgrade crashed run are still reclaimed
            # during the transition. Both are best-effort; missing names skipped.
            candidates = [f"{ep}.{ring}"]
            candidates += [f"{ep}.{ring}.{i}" for i in range(_RING_SLOTS)]
            for shm_name in candidates:
                try:
                    shm = shared_memory.SharedMemory(name=shm_name,
                                                     create=False)
                    shm.close()
                    shm.unlink()
                    unlinked += 1
                except FileNotFoundError:
                    pass
                except Exception:                                  # noqa: BLE001
                    pass
    sock_removed = 0
    if sock_dir.is_dir():
        for p in glob.glob(str(sock_dir / "oak.*.sock")):
            try:
                os.unlink(p)
                sock_removed += 1
            except FileNotFoundError:
                pass
    if unlinked or sock_removed:
        LOG.info("launcher: cleanup_orphans freed %d stale SHM segments + "
                 "%d socket files from %d prior endpoints",
                 unlinked, sock_removed, len(endpoints))


# --------------------------------------------------------------------------- #
def build_capture_args(args, cap_ep: str) -> list[str]:
    """Build the ``imu_camera.main`` argv from the parsed launcher ``args``.

    Pure (no I/O, no spawning) so the flag-forwarding contract is unit-testable
    without launching subprocesses. ``imu_camera.main`` defaults to REPLAY and takes
    an explicit ``--live`` for hardware (inverse of the old ours.proc.capture, which
    defaulted to live):

    * replay -> ``--session PATH [--max-frames N]``
    * live   -> ``--live [--no-gyro] [--recalibrate-bias] [--use-camera-calib]``

    ``--use-camera-calib`` is forwarded ONLY in the live branch and ONLY when set --
    it opts the capture process into the operator's saved stereo calib (default OFF =
    factory). ``--model`` is forwarded the same way (live + set only): it selects
    which OAK device capture opens when several are connected. Only capture needs both;
    vio/slam get whatever calib capture publishes on the retained ``calib.bundle``.
    ``--vl53l9cx`` applies to both modes, so it is appended after the mode branch.
    """
    capture_args: list[str] = ["--endpoint", cap_ep,
                               "--width", str(args.width),
                               "--height", str(args.height),
                               "--fps", str(args.fps)]
    if args.session:
        capture_args += ["--session", args.session]
        if args.max_frames > 0:
            capture_args += ["--max-frames", str(args.max_frames)]
    else:
        capture_args += ["--live"]
        if args.no_gyro:
            capture_args += ["--no-gyro"]
        if args.recalibrate_bias:
            capture_args += ["--recalibrate-bias"]
        if args.use_camera_calib:
            capture_args += ["--use-camera-calib"]
        # --model picks WHICH OAK device capture opens when several are plugged in;
        # live-only (replay has no device) and forwarded only when the operator set
        # it, mirroring --use-camera-calib above.
        if args.model:
            capture_args += ["--model", args.model]
    if args.vl53l9cx:
        capture_args += ["--vl53l9cx"]
    return capture_args


def resolve_ba_window(args) -> bool:
    """Effective BA-Window capture state (pure -> unit-testable).

    The BA Window is a UI diagnostic tool, so it is ON by default whenever the UI
    runs on the loose path ("just works", no flag needed), and OFF when headless
    (lean flight path) or under ``--tight`` (there is no loose BA window there).
    ``--ba-window`` forces it on (e.g. headless, for the PNG smoke); ``--no-ba-window``
    forces it off. Oracle-safe either way: the capture runs the SAME frozen
    ``run_ba`` and ``oracle_replay_selftest`` never goes through this launcher path.
    """
    return (not args.no_ba_window) and (
        args.ba_window or (not args.no_ui and not args.tight))


def resolve_frontend_viz(args) -> bool:
    """Effective Frontend-Internals capture state (pure -> unit-testable).

    The Frontend Internals view is a UI diagnostic, so it is ON by default
    whenever the UI runs ("just works", no flag needed) and OFF when headless
    (lean flight path). Unlike the BA Window it is NOT tight-only: the KLT
    frontend is identical on the loose and tight paths, so --frontend-viz works on
    both. ``--frontend-viz`` forces it on (e.g. headless, for the smoke);
    ``--no-frontend-viz`` forces it off. Oracle-safe either way: the
    CaptureKLTFrontend returns BYTE-IDENTICAL tracks and the oracle never goes
    through this launcher path.
    """
    return (not args.no_frontend_viz) and (
        args.frontend_viz or not args.no_ui)


def build_vio_args(args, cap_ep: str, vio_ep: str, slam_ep: str,
                   ba_ep: str | None = None,
                   ba_spawned: bool = False) -> list[str]:
    """Build the ``vio.main`` argv from the parsed launcher ``args``.

    Pure (no I/O, no spawning) so the flag-forwarding contract is unit-testable
    without launching subprocesses -- the same discipline as ``build_capture_args``.

    ``--tight`` selects the tight-coupled VIO path; only on that path do we wire the
    ``--slam-endpoint`` (closed-loop SLAM->VIO feedback). The windowed-BA backend
    knobs (``--stabilize-velocity`` / ``--depth-icp`` / ``--backend-window`` /
    ``--backend-iters`` / ``--ba-window``) are NO LONGER forwarded here -- the backend
    moved to the ``ba`` process, so :func:`build_ba_args` routes them to ``ba.main``
    instead (they were inert on VIO once the backend left). VIO keeps only the
    front-end + live-dead-reckon flags.

    ``ba_ep`` / ``ba_spawned``: when the launcher spawns the ``ba`` process (NOT
    ``--no-ba``), pass ``--ba-endpoint ba_ep`` so VIO opens the read-only backend
    pass-through client (pose.refined + ba.window re-emit + ba.state bias feed). Under
    ``--no-ba`` no ``ba`` is spawned, so ``ba_spawned`` is False and ``--ba-endpoint``
    is omitted -- VIO runs with no refined pose + inert bias feed. The windowed-BA
    backend now lives in the ``ba`` process, so ``--no-ba`` is a launcher SPAWN gate
    (mirror ``--no-slam``), NOT a vio flag any more.
    """
    vio_args: list[str] = ["--capture-endpoint", cap_ep, "--endpoint", vio_ep,
                           "--kf-every", str(args.kf_every)]
    if args.no_gyro:
        vio_args += ["--no-gyro"]
    # BACKEND PASS-THROUGH: VIO subscribes the ba endpoint's pose.refined + ba.window
    # (both re-emitted on the VIO endpoint) + ba.state (the --tight bias feed). Only
    # when the launcher actually spawned ba (not --no-ba); else omit it (no refined
    # pose, inert feed).
    if ba_spawned and ba_ep:
        vio_args += ["--ba-endpoint", ba_ep]
    if args.tight:
        vio_args += ["--tight"]
        # CLOSED-LOOP feedback (slam -> vio): give VIO the slam endpoint so its
        # --tight live pose subscribes loop.correction and the SLAM pose-graph
        # correction is fed back into the live pose (drift bounded on revisits).
        # Only on the --tight path; the loose pipeline never wires it. And NOT under
        # --no-slam (no SLAM process is spawned -> nothing to subscribe; loop_correct
        # stays off in VIO).
        if not getattr(args, "no_slam", False):
            vio_args += ["--slam-endpoint", slam_ep]
        # Opt-out: keep the slam endpoint wired (map still built) but disable the
        # SLAM->live-pose loop pull (diagnostic / Lite escape hatch).
        if getattr(args, "no_live_loop_correct", False):
            vio_args += ["--no-live-loop-correct"]
    # Dense DIRECT RGB-D VO odometry mode: opt-in, default OFF. Forwarded ONLY when
    # --direct is set, on BOTH live AND replay (direct is a real odometry mode, not
    # a viz, and is exercised on replay too -- the target recipe is the replay
    # ``--vl53l9cx --direct`` smoke). It is a THIRD front-end selected only by this
    # flag, so the loose/tight paths and the offline byte-parity oracle (which never
    # passes --direct and never goes through this launcher) stay gap=0. Independent
    # of --tight (direct owns its own IMU seed), so it is NOT gated on it.
    if args.direct:
        vio_args += ["--direct"]
    # Frontend-internals snapshot stream: opt-in, works on BOTH loose AND tight
    # (the KLT frontend is identical), and on BOTH live AND replay (the view
    # scrubs a replay segment too). Oracle-safe: the CaptureKLTFrontend returns
    # byte-identical tracks and the oracle never goes through this launcher path.
    if args.frontend_viz:
        vio_args += ["--frontend-viz"]
    return vio_args


def build_ba_args(args, vio_ep: str, ba_ep: str) -> list[str]:
    """Build the ``ba.main`` argv from the parsed launcher ``args`` (pure).

    The ``ba`` process is a pure CONSUMER of VIO's keyframe output: it subscribes the
    VIO endpoint (``--vio-endpoint``), runs the windowed BA, and publishes
    ``pose.refined`` (+ ``ba.state`` under ``--tight``, + ``ba.window`` under
    ``--ba-window``) on its own endpoint (``--endpoint``). ``--tight`` selects the
    tight-coupled backend, mirroring ``vio.main --tight`` -- forwarded ONLY when set.

    The windowed-BA backend knobs route HERE (the backend lives in ``ba`` now), with
    the SAME gating the pre-split in-VIO forward used:

    * ``--stabilize-velocity`` / ``--depth-icp`` -- the Phase-4 tight knobs, appended
      ONLY when ``args.tight AND <flag>`` (the loose backend has no velocity / window
      factor to regularise, so they are dropped off the tight path -- the caller warns).
    * ``--ba-window`` -- the LOOSE-only visualiser. ``args.ba_window`` is already the
      RESOLVED effective state by the time we run (``main`` sets it via
      :func:`resolve_ba_window`, which is False under ``--tight`` / ``--no-ui`` /
      ``--no-ba-window``), so we forward it verbatim.
    * ``--backend-window`` / ``--backend-iters`` -- the loose solve size; forwarded
      only when non-default so the common argv stays minimal.
    """
    ba_args: list[str] = ["--vio-endpoint", vio_ep, "--endpoint", ba_ep]
    if args.tight:
        ba_args += ["--tight"]
        # Phase-4 tight knobs: tight-only, opt-in (same contract as the old in-VIO
        # forward). The loose backend has no velocity / window factor to regularise.
        if getattr(args, "stabilize_velocity", False):
            ba_args += ["--stabilize-velocity"]
        if getattr(args, "depth_icp", False):
            ba_args += ["--depth-icp"]
    # BA-window visualiser: LOOSE-only, opt-in. args.ba_window is the resolved
    # effective state (resolve_ba_window already returns False under --tight), so this
    # is never both --tight and --ba-window.
    if getattr(args, "ba_window", False):
        ba_args += ["--ba-window"]
    # Loose windowed-BA solve size: forward only when the operator overrode the
    # default (keeps the argv minimal; ba.main defaults match these).
    if getattr(args, "backend_window", 6) != 6:
        ba_args += ["--backend-window", str(args.backend_window)]
    if getattr(args, "backend_iters", 5) != 5:
        ba_args += ["--backend-iters", str(args.backend_iters)]
    return ba_args


# --------------------------------------------------------------------------- #
def _numba_thread_caps(cap: bool) -> dict[str, int]:
    """Per-role numba thread budget when ``--cap-numba-threads`` is set.

    The flight stack runs capture / vio / ba / slam as SEPARATE OS processes, and
    nothing sets ``NUMBA_NUM_THREADS`` -- so by default EVERY process spins a
    numba pool of all cores. When capture's SGM burst and vio's KLT burst
    overlap (they pipeline), that is 2x ncores runnable threads fighting over
    ncores cores: oversubscription thrash, which on the Pi5 reads as "cores look
    cool" (idle between bursts, thrashing during them) while wall-clock is set by
    serial glue. Capping each hot process to ~half the cores keeps two
    overlapping bursts at <= ncores total. SLAM + BA are off the per-frame path
    (they run on the keyframe cadence), so they get the remainder. Returns {} when
    capping is off (full cores, dev hosts).
    """
    if not cap:
        return {}
    n = os.cpu_count() or 4
    half = max(1, n // 2)
    rest = max(1, n // 4)
    return {"imu_camera": half, "vio": half, "slam": rest, "ba": rest}


def _role_env(base_env: dict[str, str], threads: int | None) -> dict[str, str]:
    """A child env with NUMBA_NUM_THREADS pinned to ``threads`` for this role.

    A user-set NUMBA_NUM_THREADS always wins (never override an explicit choice);
    ``threads`` None / 0 leaves the env untouched.
    """
    if not threads or "NUMBA_NUM_THREADS" in os.environ:
        return base_env
    e = dict(base_env)
    e["NUMBA_NUM_THREADS"] = str(threads)
    return e


def _spawn(py: str, mod: str, args: list[str], *, env: dict[str, str],
           name: str) -> subprocess.Popen:
    """Spawn a child python process; stdout / stderr inherited from launcher."""
    cmd = [py, "-m", mod, *args]
    p = subprocess.Popen(cmd, env=env)
    LOG.info("launcher: spawned %s pid=%d -> %s", name, p.pid, " ".join(cmd))
    return p


@contextlib.contextmanager
def _ignore_sigint():
    """Make the wrapped teardown atomic against Ctrl-C: a SECOND SIGINT during
    process shutdown can't abort it half-way (which would orphan children or
    surface a raw KeyboardInterrupt traceback). Restores the prior handler on
    exit so a later pipeline generation stays interruptible."""
    prev = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, prev)


def _terminate(procs: list[subprocess.Popen], *, deadline_s: float = 10.0,
               step_s: float = 0.2) -> None:
    """SIGTERM all procs, wait for clean exit, SIGKILL any straggler."""
    for p in procs:
        if p.poll() is None:
            try:
                p.terminate()
            except Exception:                                      # noqa: BLE001
                pass
    deadline = time.monotonic() + float(deadline_s)
    while time.monotonic() < deadline and any(p.poll() is None for p in procs):
        time.sleep(step_s)
    for p in procs:
        if p.poll() is None:
            LOG.warning("launcher: SIGKILL on pid %d (clean shutdown timeout)",
                        p.pid)
            try:
                p.kill()
            except Exception:                                      # noqa: BLE001
                pass


# Overload watchdog tuning: warn when the end-to-end pose.odom rate stays below
# _OVL_FRAC of the target across _OVL_HOLD windows of _OVL_WIN s (throttled).
_OVL_FRAC, _OVL_WIN, _OVL_HOLD, _OVL_THROTTLE = 0.6, 3.0, 2, 8.0


def _pose_overload_rate(st: dict, now: float, n: int, target_fps: int):
    """Update the pose-rate overload state on a pose arrival.

    ``st`` carries ``rate_t / rate_n / warn_t / low`` across calls. Returns the
    measured rate (Hz) when a fresh OVERLOAD warning is DUE (the rate has held
    below ``_OVL_FRAC * target_fps`` for ``_OVL_HOLD`` windows), else ``None``.
    Pure + state-explicit so the launcher's overload watchdog is unit-testable
    WITHOUT a live device (the live path can only be exercised on real hardware).
    The clock anchors to the FIRST pose so VIO's startup wait never reads as slow.
    """
    if not target_fps:
        return None
    if n <= 1:                                  # first pose -> start the clock here
        st["rate_t"], st["rate_n"], st["low"] = now, n, 0
        return None
    if now - st["rate_t"] < _OVL_WIN:           # window not elapsed yet
        return None
    rate = (n - st["rate_n"]) / (now - st["rate_t"])
    st["rate_t"], st["rate_n"] = now, n
    if rate >= _OVL_FRAC * target_fps:          # keeping up -> reset the run
        st["low"] = 0
        return None
    st["low"] += 1                              # a slow window
    if st["low"] >= _OVL_HOLD and now - st["warn_t"] >= _OVL_THROTTLE:
        st["warn_t"] = now
        return rate
    return None


def _start_pose_logger(vio_ep: str, target_fps: int = 0, width: int = 0,
                       height: int = 0, live: bool = False):
    """``--no-ui`` FC-output preview: subscribe to the vio pose, print pos + quat.

    A READ-ONLY consumer on the vio endpoint -- the ours FC-output HOOK, the
    single place a real MAVLink ``VISION_POSITION_ESTIMATE`` send will go (mirrors
    baseline's ``_on_pose``). Prints the RAW pose (position + quaternion; the FC
    derives heading itself), throttled to ~2 Hz. Best-effort: returns the
    ``IPCPubSub`` client to ``stop()`` on teardown, or ``None`` if it cannot
    attach (logging must never break the flight run). Pose is ``T_world_cam`` in
    the pipeline's gravity-aligned WORLD frame; the FC NED / body-extrinsic
    mapping is the (future) FC-link step, not done here.

    Also an OVERLOAD watchdog (live only): when the chosen resolution is too heavy
    for the box (e.g. 640x400 on a 4-core Pi), nothing ERRORS -- the pipeline just
    can't keep real-time, the pose.odom rate collapses, and the (remote) UI looks
    FROZEN with nothing in the log. So it compares the actual end-to-end pose rate
    to ``target_fps`` (pose.odom is the LAST stage, so this catches a bottleneck in
    EITHER capture or vio) and, when it stays well below, logs a LOUD WARNING
    telling the operator to lower the resolution.
    """
    from launcher.comms import IPCPubSub, topics
    from launcher.comms.lib.misc.frames import rot_to_quat

    st = {"last": 0.0, "n": 0, "rate_t": 0.0, "rate_n": 0, "warn_t": 0.0,
          "low": 0}

    def _on_pose(wm) -> None:
        # === FC OUTPUT HOOK (ours) -- wire MAVLink VISION_POSITION_ESTIMATE here.
        st["n"] += 1
        now = time.monotonic()
        # --- OVERLOAD watch: end-to-end pose rate vs the target fps (live only).
        if live:
            r = _pose_overload_rate(st, now, st["n"], target_fps)
            if r is not None:
                LOG.warning(
                    "pipeline OVERLOADED: pose.odom only ~%.1f Hz vs %d target at "
                    "%dx%d -- the box can't keep up at this resolution; lower it "
                    "(e.g. --width 320 --height 200).", r, target_fps, width, height)
        if now - st["last"] < 0.5:
            return
        st["last"] = now
        T = wm.T_world_cam
        qw, qx, qy, qz = rot_to_quat(T[:3, :3])
        sig = wm.info.get("pos_sigma_m") if isinstance(wm.info, dict) else None
        sig_s = "  sig_pos=%.3fm" % sig if sig is not None else ""
        LOG.info("pose: pos WORLD=(%+.3f %+.3f %+.3f) m  "
                 "quat wxyz=(%+.4f %+.4f %+.4f %+.4f)%s  n=%d",
                 T[0, 3], T[1, 3], T[2, 3], qw, qx, qy, qz, sig_s, st["n"])

    try:
        # 30 s, not 5 s: this logger is started right after spawning VIO, whose
        # IPC server only comes up ~7 s later (calib wait + frontend build). At
        # 5 s the connect timed out and the --no-ui run had NO pose log at all
        # (blind). The client retries within the timeout, so 30 s simply waits
        # for VIO to be ready.
        client = IPCPubSub(vio_ep, role="client", connect_timeout_s=30.0)
        client.subscribe(topics.POSE_ODOM, _on_pose)
        client.start()
    except Exception as e:                                          # noqa: BLE001
        LOG.warning("launcher: --no-ui pose logger could not attach to %r: %s",
                    vio_ep, e)
        return None
    LOG.info("launcher: --no-ui pose logger on %r (pose.odom -> pos+quat; "
             "the FC-output hook)", vio_ep)
    return client


def build_forward_args(host: str, port: int, args, cap_ep: str, vio_ep: str,
                       slam_ep: str) -> list[str]:
    """Build the ``netbridge.forward`` argv (pure -> unit-testable).

    The forward process runs ON THE PI alongside the flight stack: it subscribes
    the local capture/vio/slam IPC endpoints and re-serves them over TCP for a
    remote Mac UI. It must attach to the SAME ring resolution capture created, so
    the launcher's ``--width`` / ``--height`` (and the ToF override they imply) are
    forwarded verbatim. The endpoints are the launcher's resolved (possibly
    suffixed) names so a ``--auto-suffix`` run still bridges the right sockets.

    Bandwidth mode: the bridge is POSE-ONLY by DEFAULT (low-bandwidth -- the heavy
    uncompressed camera/depth/keyframe image topics, ~51 Mbit/s at 320x200, are NOT
    bridged), because the main trajectory + map UI never displays the camera image,
    and a congested 2.4 GHz WiFi link delivers only ~1.6 Mbit/s. ``--bridge-frames``
    opts back into forwarding the image topics (full behaviour as before), for an
    operator who wants the camera Visualize windows over a fast link. So we append
    ``--pose-only`` UNLESS ``args.bridge_frames`` is set; the Mac-side receive must
    be run with the MATCHING intent (``deploy/pi-ui.sh --frames`` to keep frames).
    """
    # --no-slam: no SLAM process exists, so tell the forward NOT to bridge slam
    # (empty endpoint -> skip). Passing the real slam_ep here made the forward
    # block on `oak.slam` and crash after the 30 s connect timeout, taking the
    # remote UI down with it. Mirrors build_vio_args' --slam-endpoint gating.
    fwd_slam_ep = "" if getattr(args, "no_slam", False) else slam_ep
    forward_args = ["--listen", f"{host}:{port}",
                    "--capture-endpoint", cap_ep,
                    "--vio-endpoint", vio_ep,
                    "--slam-endpoint", fwd_slam_ep,
                    "--width", str(args.width),
                    "--height", str(args.height)]
    if not args.bridge_frames:
        forward_args += ["--pose-only"]
    return forward_args


def parse_host_port(s: str, *, default_host: str = "0.0.0.0") -> tuple[str, int]:
    """Parse a ``--forward HOST:PORT`` (or bare ``:PORT`` / ``PORT``) value.

    Pure helper so the launcher CLI parsing is testable. A bare port binds the
    forward server on ``0.0.0.0`` (all interfaces) so a Mac on the LAN can reach
    it; an explicit host narrows the bind (e.g. a Wireguard interface address).
    """
    s = str(s).strip()
    if ":" in s:
        host, _, port = s.rpartition(":")
        host = host or default_host
    else:
        host, port = default_host, s
    return host, int(port)


# --------------------------------------------------------------------------- #
def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(name)s %(levelname)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session", default=None,
                    help="replay this session directory instead of opening the OAK-D")
    ap.add_argument("--live", action="store_true",
                    help="open the OAK-D (the default when no --session is given; "
                         "accepted explicitly for convenience). Ignored if --session is set.")
    ap.add_argument("--max-frames", type=int, default=0,
                    help="cap replay frames (0 = all)")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=400)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--kf-every", type=int, default=5)
    ap.add_argument("--no-gyro", action="store_true",
                    help="live: disable IMU gyro use (pure-vision)")
    ap.add_argument("--recalibrate-bias", action="store_true",
                    help="live: re-measure gyro bias instead of using the cached one")
    ap.add_argument("--use-camera-calib", action="store_true",
                    help="live: apply the operator's SAVED per-device stereo calib "
                         "(from the wizard) instead of the FACTORY calib. Default "
                         "OFF -- factory is the trusted reference. Forwarded to the "
                         "capture subprocess.")
    ap.add_argument("--model", default=None,
                    help="live: select which OAK device to open when several are "
                         "connected, by product-name substring (e.g. 'lite') or "
                         "deviceId. Forwarded to the capture subprocess. Device "
                         "capabilities (IMU, mono resolution) are auto-detected.")
    ap.add_argument("--cap-numba-threads", action="store_true",
                    help="cap each child's numba thread pool so the per-stage "
                         "parallel bursts don't oversubscribe a small core count "
                         "(e.g. on the 4-core Pi5: capture=2, vio=2, slam=1 "
                         "instead of every process grabbing all 4 cores and "
                         "thrashing). Derived from os.cpu_count(); a user-set "
                         "NUMBA_NUM_THREADS in the environment always wins. "
                         "Off by default (full cores) for big dev hosts.")
    ap.add_argument("--endpoint-suffix", default="",
                    help="append SUFFIX to canonical endpoint names so two "
                         "launchers can co-exist (e.g. 'dev', 'ci')")
    ap.add_argument("--auto-suffix", action="store_true",
                    help="derive endpoint suffix from this launcher's PID")
    ap.add_argument("--no-ui", action="store_true",
                    help="don't open the UI -- useful for capture-only headless runs")
    ap.add_argument("--forward", default=None, metavar="HOST:PORT",
                    help="ALSO run the cross-machine bridge: spawn "
                         "netbridge.forward bound to HOST:PORT (bare :PORT or PORT "
                         "binds 0.0.0.0) so a remote Mac UI can connect over "
                         "TCP/WiFi (run `deploy/pi-ui.sh` there). Additive -- the "
                         "local UI / --no-ui path is unaffected. Authenticates with "
                         "OAKD_NETBRIDGE_KEY if set, else a built-in default key "
                         "(no setup needed on a trusted LAN). DEFAULT = pose-only "
                         "(low-bandwidth); see --bridge-frames.")
    ap.add_argument("--bridge-frames", action="store_true",
                    help="with --forward, ALSO bridge the heavy camera/depth/"
                         "keyframe IMAGE topics (~51 Mbit/s uncompressed). OFF by "
                         "default: the bridge is POSE-ONLY (only the small pose + "
                         "map + overlay topics cross the wire), because the main "
                         "trajectory + map UI never shows the camera image and a "
                         "congested WiFi link is easily oversubscribed by the raw "
                         "frames. Use this only to feed the opt-in camera Visualize "
                         "windows over a FAST link -- and run the Mac side with the "
                         "matching `deploy/pi-ui.sh --frames`.")
    ap.add_argument("--vl53l9cx", action="store_true",
                    help="simulate a VL53L9CX-class ToF camera in the capture "
                         "process: compute depth at the source resolution then "
                         "downsample gray + depth to 54x42 (works live + replay)")
    ap.add_argument("--tight", action="store_true",
                    help="run the VIO process with its TIGHT-coupled backend "
                         "(joint visual + IMU window optimiser) instead of the "
                         "default loose windowed-BA backend. Forwarded to "
                         "vio.main --tight; loose stays the default.")
    ap.add_argument("--no-ba", action="store_true",
                    help="LEAN flight: don't spawn the BA process (the windowed-BA "
                         "backend) -- no pose.refined (the VIO-BA line) + no "
                         "backend->live bias feed-forward. pose.odom (live VIO) is "
                         "unaffected; VIO is also not given --ba-endpoint. A launcher "
                         "SPAWN gate (mirror --no-slam). Frees a process on the Pi.")
    ap.add_argument("--no-slam", action="store_true",
                    help="LEAN flight: don't spawn the SLAM process -- no map, no "
                         "loop-closure (so no loop-correction feedback into "
                         "pose.odom; bounded-on-revisit drift is forgone). For a "
                         "one-way flight that never revisits. Pairs with --no-ba "
                         "for the lightest stack.")
    ap.add_argument("--loop-search-radius", type=float, nargs="?", const=5.0,
                    default=0.0,
                    help="metres: make SLAM LIGHTER while KEEPING loop-closure -- "
                         "spatial-gate the loop search to keyframes within this "
                         "radius of the current pose instead of brute-force against "
                         "ALL of them (the O(N)-growing per-keyframe cost that is "
                         "the live SLAM CPU hog on the Pi). 0 = exact (default); the "
                         "bare flag = 5m, or pass a value (e.g. --loop-search-radius "
                         "3). Forwarded to slam.main. Alternative to --no-slam.")
    ap.add_argument("--direct", action="store_true",
                    help="run the VIO process in DENSE DIRECT RGB-D VO odometry "
                         "mode: replace the sparse corner/KLT->PnP front-end with "
                         "dense direct photometric frame-to-keyframe alignment + "
                         "live IMU seed + divergence guard. Forwarded to "
                         "vio.main --direct. Opt-in (default OFF); meant to pair "
                         "with --vl53l9cx (the 54x42 ToF recipe where the sparse "
                         "VIO scale-collapses). Oracle byte-identical (gap=0).")
    ap.add_argument("--no-live-loop-correct", action="store_true",
                    help="tight only: DISABLE feeding the SLAM loop-correction into "
                         "the live pose (the SLAM map is still built). Diagnostic / "
                         "escape hatch; forwarded to vio.main only with --tight.")
    ap.add_argument("--backend-window", type=int, default=6,
                    help="LOOSE windowed-BA sliding-window size (keyframes). "
                         "Forwarded to ba.main (the windowed-BA backend lives in the "
                         "ba process); inert on the tight path. Default 6.")
    ap.add_argument("--backend-iters", type=int, default=5,
                    help="LOOSE windowed-BA max Gauss-Newton iterations per solve. "
                         "Forwarded to ba.main; inert on the tight path. Default 5.")
    ap.add_argument("--stabilize-velocity", action="store_true",
                    help="tight only: enable Phase-4 velocity regularisation "
                         "(CV prior + gated ZUPT) to curb 54x42/shake velocity "
                         "divergence. Forwarded to ba.main --stabilize-velocity "
                         "only with --tight; ignored (warned) on the loose path.")
    ap.add_argument("--depth-icp", action="store_true",
                    help="tight only: enable the Phase-4 dense-ICP relative-pose "
                         "factor (anchors inter-keyframe translation at 54x42). "
                         "Forwarded to ba.main --depth-icp only with --tight; "
                         "ignored (warned) on the loose path.")
    ap.add_argument("--ba-window", action="store_true",
                    help="force the BA Window visualiser ON (ba publishes ba.window "
                         "solve snapshots: window keyframe poses + 3D landmarks + "
                         "observation rays + reprojection error, which vio re-emits on "
                         "the VIO endpoint; UI exposes Visualize > BA Window). It is a "
                         "UI tool, so it is ALREADY ON by default whenever the UI runs "
                         "on the loose path -- this flag only forces it on headless "
                         "(e.g. for the PNG smoke). Loose-only; ignored under --tight; "
                         "oracle byte-identical.")
    ap.add_argument("--no-ba-window", action="store_true",
                    help="force the BA Window capture OFF even when the UI is shown "
                         "(skip its small per-keyframe snapshot/publish cost).")
    ap.add_argument("--frontend-viz", action="store_true",
                    help="force the Frontend Internals view ON (VIO publishes "
                         "frame.frontend: Shi-Tomasi response heatmap + accepted "
                         "corners + KLT flow field coloured by forward-backward "
                         "error; UI exposes Visualize > Frontend Internals). It is "
                         "a UI tool, so it is ALREADY ON by default whenever the UI "
                         "runs -- this flag only forces it on headless (e.g. for "
                         "the smoke). Works on loose AND tight; oracle byte-identical "
                         "(CaptureKLTFrontend returns byte-identical tracks).")
    ap.add_argument("--no-frontend-viz", action="store_true",
                    help="force the Frontend Internals capture OFF even when the UI "
                         "is shown (skip its small per-frame snapshot/publish cost).")
    ap.add_argument("--corrected-vio", action="store_true",
                    help="UI: enable the SLAM-corrected VIO overlay (the dense VIO "
                         "trail rubber-sheeted to the SLAM graph). OFF by default -- "
                         "its warp is recomputed EVERY GUI tick even when the line "
                         "is hidden, so it is opt-in for a lighter UI. Forwarded to "
                         "ui.main; needs SLAM running to be non-empty.")
    args = ap.parse_args()

    args.ba_window = resolve_ba_window(args)
    args.frontend_viz = resolve_frontend_viz(args)

    # ---- Endpoint names (computed ONCE, identical across restarts) --------
    # The auto-suffix is derived from THIS launcher's PID, so re-spawning the
    # pipeline (Restart button) re-creates the same-named endpoints + rings.
    # _cleanup_orphans + SharedArrayRing.create's cleanup_stale reclaim any
    # leftovers from the prior generation each iteration.
    if args.auto_suffix:
        # `oak.cap.l<pidhex>` -- the ring shm name is `{ep}.{ring}` and must fit
        # inside macOS's 30-char POSIX shm name limit. See `SharedArrayRing.create`
        # for the gate.
        suffix = f".l{os.getpid() & 0xFFF:x}"
    elif args.endpoint_suffix:
        suffix = "." + args.endpoint_suffix
    else:
        suffix = ""
    cap_ep = f"oak.cap{suffix}" if suffix else "oak.capture"
    vio_ep = f"oak.vio{suffix}" if suffix else "oak.vio"
    slam_ep = f"oak.slm{suffix}" if suffix else "oak.slam"
    # ``ba`` owns no rings (it ATTACHES to vio's kf_* rings), so its endpoint name
    # has no shm-name length constraint -- one form for both suffixed + default.
    ba_ep = f"oak.ba{suffix}"
    LOG.info("launcher: endpoints cap=%r vio=%r ba=%r slam=%r",
             cap_ep, vio_ep, ba_ep, slam_ep)
    # Persist our endpoints so the NEXT launcher's `_cleanup_orphans` can
    # recover them even if our sock files are deleted between runs.
    _record_endpoints([cap_ep, vio_ep, ba_ep, slam_ep])

    py = sys.executable
    env = dict(os.environ)
    # Per-role numba thread budget (--cap-numba-threads). Empty -> full cores.
    caps = _numba_thread_caps(args.cap_numba_threads)
    if caps:
        LOG.info("launcher: capping numba threads per process %s "
                 "(ncores=%d, --cap-numba-threads)", caps, os.cpu_count() or 0)

    # ---- Cross-machine bridge (--forward) --------------------------------
    # Resolve + validate the forward endpoint ONCE, before spawning, so a bad
    # value (or a missing authkey) fails fast instead of crashing a child later.
    forward_hostport: tuple[str, int] | None = None
    if args.forward:
        forward_hostport = parse_host_port(args.forward)
        # The bridge is always authenticated: OAKD_NETBRIDGE_KEY if set, else a
        # built-in default key (netbridge.tcp_transport.resolve_authkey) so a remote
        # UI connects with no setup on a trusted LAN. Just log which is in use.
        LOG.info("launcher: --forward bridge will listen on %s:%d (remote Mac UI "
                 "connects via deploy/pi-ui.sh)%s", *forward_hostport,
                 "" if env.get("OAKD_NETBRIDGE_KEY")
                 else "  [using the built-in default bridge key]")

    # ---- Build per-proc argv ---------------------------------------------
    # Capture argv (mode branch + flag forwarding) lives in build_capture_args so
    # the contract is unit-testable without spawning subprocesses.
    capture_args = build_capture_args(args, cap_ep)

    # VIO argv (front-end + live dead-reckon flags: --tight / --slam-endpoint /
    # --direct / --frontend-viz) lives in build_vio_args. The windowed-BA backend
    # knobs route to ba.main via build_ba_args (the backend moved to the ba process).
    if args.stabilize_velocity and not args.tight:
        # --stabilize-velocity only affects the tight backend's velocity state;
        # the loose path has no velocity to regularise, so warn + drop it
        # (build_ba_args gates it behind --tight; this just tells the operator).
        LOG.warning("launcher: --stabilize-velocity has no effect without "
                    "--tight (loose path has no velocity state); ignoring it")
    if args.depth_icp and not args.tight:
        # --depth-icp only affects the tight backend's window solve; the loose
        # path has no relative-pose factor, so warn + drop it (build_ba_args gates
        # it behind --tight; this just tells the operator).
        LOG.warning("launcher: --depth-icp has no effect without --tight "
                    "(loose path has no window factor graph); ignoring it")
    # --no-ba is a launcher SPAWN gate (mirror --no-slam): the windowed-BA backend
    # now lives in the ``ba`` process, so --no-ba simply skips spawning it (no
    # pose.refined / no backend->live bias feed-forward; pose.odom is unaffected).
    spawn_ba = not getattr(args, "no_ba", False)
    # VIO gets --ba-endpoint only when ba is actually spawned (else no refined pose,
    # inert bias feed -- exactly the old --no-ba behaviour, now launcher-gated).
    vio_args = build_vio_args(args, cap_ep, vio_ep, slam_ep,
                              ba_ep=ba_ep, ba_spawned=spawn_ba)

    # BA argv (a pure consumer of VIO's keyframe output) lives in build_ba_args so
    # the contract is unit-testable without spawning.
    ba_args = build_ba_args(args, vio_ep, ba_ep)

    # NB: the new `slam.main` is a PURE consumer of VIO's output and -- unlike the
    # pre-split `ours.proc.slam` -- intentionally DROPPED `--capture-endpoint`
    # (its docstring: "We deliberately don't subscribe to capture at all"). So we
    # wire only `--vio-endpoint` / `--endpoint` here; passing the old
    # `--capture-endpoint` would make slam's argparse abort on startup.
    slam_args = ["--vio-endpoint", vio_ep,
                 "--endpoint", slam_ep]
    # Spatial-gate the loop search (caps the O(N) per-keyframe SLAM cost -- the live
    # CPU hog on the Pi). 0 = exact (default); forwarded to slam.main.
    if getattr(args, "loop_search_radius", 0.0) and args.loop_search_radius > 0.0:
        slam_args += ["--loop-search-radius", str(args.loop_search_radius)]

    # The UI's calib + visualise windows subscribe capture directly (IMU /
    # imucam.sample / frame.depth), so it must know the suffixed live endpoint
    # under an --auto-suffix run.
    ui_args = ["--capture-endpoint", cap_ep,
               "--vio-endpoint", vio_ep, "--slam-endpoint", slam_ep]
    # BA Window: tell the UI to expose Visualize > BA Window (live follow-latest)
    # only when the operator asked for it (and not under --tight, where ba never
    # publishes ba.window). args.ba_window is the RESOLVED effective state (False
    # under --tight already), so the menu is honest about what the pipeline emits:
    # ba publishes ba.window and vio re-emits it on the VIO endpoint the UI reads.
    if args.ba_window and not args.tight:
        ui_args += ["--ba-window"]
    # Frontend Internals: tell the UI to expose Visualize > Frontend Internals
    # (live follow-latest) when the operator asked for it. Works on loose AND
    # tight (the frontend is identical), so it is NOT gated on --tight.
    if args.frontend_viz:
        ui_args += ["--frontend-viz"]
    # SLAM-corrected VIO overlay: opt-in (its rubber-sheet warp runs every GUI tick
    # even when hidden). Default OFF for a lighter UI; needs SLAM running to show.
    if args.corrected_vio:
        ui_args += ["--corrected-vio"]
    # --no-slam: tell the UI the SLAM endpoint won't open, so it doesn't block
    # waiting for its calib.bundle (would time out + crash the UI).
    if args.no_slam:
        ui_args += ["--no-slam"]

    # ---- SIGTERM handler (registered ONCE) -------------------------------
    # `kill <launcher_pid>` from outside must clean up the whole tree, not just
    # the launcher itself. The handler reads a STABLE mutable `procs` holder
    # that the restart loop clear()s + repopulates each generation, so it always
    # signals the CURRENT generation's children regardless of how many restarts
    # have happened.
    #
    # CRITICAL: do NOT call `_terminate(procs)` here -- `_terminate` polls each
    # `Popen.poll()`, which calls `os.waitpid(pid, WNOHANG)` on the same pid the
    # main thread is blocked in `ui_proc.wait()` on. The two waitpid callers race
    # for the single reap event, leaving Popen's `returncode` stuck at None on
    # the loser, so the handler's `_terminate` loop spins the full 10 s deadline
    # and SIGKILLs the UI even though it already exited cleanly. Instead just
    # forward SIGTERM to each child (they likely already got it from the
    # process-group signal anyway) and `os._exit` immediately; children either
    # finish their own shutdown or get reaped by init when launcher dies.
    procs: list[subprocess.Popen] = []
    # Named handles for the procs the --no-ui drain path needs by ROLE (not by a
    # fragile procs[] index -- ba spawns BETWEEN vio and slam, so positional
    # indexing of slam would shift). _spawn_pipeline clears + repopulates this each
    # generation alongside `procs`.
    named: dict[str, subprocess.Popen] = {}

    def _on_sigterm(_signo, _frame):
        LOG.info("launcher: SIGTERM -> forwarding to children + exiting")
        for p in procs:
            try:
                p.terminate()
            except Exception:                                      # noqa: BLE001
                pass
        os._exit(143)                                             # 128 + SIGTERM
    signal.signal(signal.SIGTERM, _on_sigterm)

    # ---- Boot order ------------------------------------------------------
    # capture FIRST so the retained `calib.bundle` is published as soon as it
    # builds the frontend. vio / ba / slam connect with retried `IpcClientBus.start`
    # so booting them after capture is fine; this just minimises the connect retry
    # noise in the log. Order: capture -> vio -> ba -> slam (ba + slam both consume
    # vio's keyframe; vio's BA pass-through client + slam's loop-correction client
    # retry until ba / vio are up, so the exact order is cosmetic).
    def _spawn_pipeline() -> None:
        """Clear `procs` in place and spawn a fresh capture+vio+ba+slam generation.

        Mutates the SHARED `procs` holder (clear + append) + the `named` role map so
        the once-registered SIGTERM handler always sees the live generation. Best-
        effort SHM cleanup runs FIRST so the prior generation's segments are reclaimed
        before the same-named rings are re-created (macOS POSIX shm persists past
        SIGKILL; a stale namespace eventually trips capture's shm_open() with EMFILE).
        """
        _cleanup_orphans()
        procs.clear()
        named.clear()
        cap_proc = _spawn(py, "imu_camera.main", capture_args,
                          env=_role_env(env, caps.get("imu_camera")),
                          name="imu_camera")
        procs.append(cap_proc)
        named["capture"] = cap_proc
        # tiny sleep so capture's IPC server is listening before vio / ba / slam
        # try their first connect (vio retries so this is cosmetic, but it
        # gives a clean first-attempt success in the log).
        time.sleep(0.2)
        vio_proc = _spawn(py, "vio.main", vio_args,
                          env=_role_env(env, caps.get("vio")), name="vio")
        procs.append(vio_proc)
        named["vio"] = vio_proc
        # --no-ba (lean flight): skip the BA process entirely (no pose.refined, no
        # backend->live bias feed-forward). pose.odom (live VIO) runs unaffected;
        # build_vio_args also omits --ba-endpoint so VIO never wires the pass-through.
        # Spawn ba AFTER vio (it subscribes vio's keyframe) + BEFORE slam.
        if spawn_ba:
            ba_proc = _spawn(py, "ba.main", ba_args,
                             env=_role_env(env, caps.get("ba")), name="ba")
            procs.append(ba_proc)
            named["ba"] = ba_proc
        # --no-slam (lean flight): skip the SLAM process entirely (no map, no
        # loop-closure). pose.odom (live VIO) runs unaffected; build_vio_args also
        # omits --slam-endpoint so VIO never wires the loop-correction feedback.
        if not args.no_slam:
            slam_proc = _spawn(py, "slam.main", slam_args,
                               env=_role_env(env, caps.get("slam")), name="slam")
            procs.append(slam_proc)
            named["slam"] = slam_proc
        # Cross-machine bridge: spawn netbridge.forward LAST (it connects to the
        # capture/vio/slam IPC servers, which retry, so order is cosmetic) when
        # --forward is set. It joins `procs`, so _terminate tears it down with the
        # rest. Additive -- a missing --forward leaves the pipeline unchanged.
        if forward_hostport is not None:
            fwd_host, fwd_port = forward_hostport
            forward_args = build_forward_args(fwd_host, fwd_port, args,
                                              cap_ep, vio_ep, slam_ep)
            procs.append(_spawn(py, "netbridge.forward", forward_args, env=env,
                                name="netbridge.forward"))

    if args.no_ui:
        # No restart button without a UI -- the --no-ui path runs exactly ONCE,
        # independent of the restart loop below.
        pose_client = None
        try:
            _spawn_pipeline()
            # Resolve the procs the drain needs by ROLE, not a fragile procs[] index
            # (ba spawns between vio and slam). capture + vio always exist; ba / slam
            # are absent under --no-ba / --no-slam (named.get -> None).
            cap_proc = named["capture"]
            vio_proc = named["vio"]
            ba_proc = named.get("ba")
            slam_proc = named.get("slam")
            # FC-output preview: print the live pose (pos + quat) to the log, plus
            # the OVERLOAD watchdog (live only: a session is replay-paced, not
            # real-time, so its rate is not a "can't keep up" signal).
            pose_client = _start_pose_logger(
                vio_ep, target_fps=args.fps, width=args.width,
                height=args.height, live=not bool(args.session))
            LOG.info("launcher: --no-ui set; waiting for capture to exit "
                     "(Ctrl-C to stop)")
            interrupted = False
            try:
                cap_proc.wait()
            except KeyboardInterrupt:
                LOG.info("launcher: SIGINT -> stopping")
                interrupted = True
            rc = cap_proc.returncode if cap_proc.returncode is not None else 0
            # After capture exits NATURALLY, vio + ba + slam see END on their inputs
            # (capture's publisher bridge converts each Flow's `_emit_end` to a
            # wire END then drains them onto the socket before close; vio re-emits END
            # so ba + slam drain in turn). Give them a natural-exit window BEFORE
            # `_terminate` SIGKILLs them: each one has its own 120 s drain ceiling so a
            # busy back-end won't lose data. On Ctrl-C, skip this wait -- END never
            # arrives, so `_terminate`'s SIGTERM (fast teardown) is the clean path.
            if not interrupted:
                LOG.info("launcher: waiting for vio + ba + slam to drain "
                         "naturally ...")
                for child in (p for p in (vio_proc, ba_proc, slam_proc)
                              if p is not None):
                    try:
                        child.wait(timeout=30.0)
                    except subprocess.TimeoutExpired:
                        LOG.warning("launcher: %s pid=%d still running after 30 s; "
                                    "_terminate will SIGTERM/SIGKILL",
                                    child.args, child.pid)
                    except KeyboardInterrupt:
                        # Ctrl-C during the drain -> stop waiting; the `finally`
                        # _terminate() SIGTERMs both children (fast teardown).
                        LOG.info("launcher: SIGINT during drain -> terminating")
                        break
        finally:
            # Teardown is bounded + must complete to release SHM rings / reap
            # children -- ignore a second Ctrl-C so it can't abort us mid-way.
            with _ignore_sigint():
                if pose_client is not None:
                    try:
                        pose_client.stop()
                    except Exception:                              # noqa: BLE001
                        pass
                LOG.info("launcher: shutting down background procs ...")
                _terminate(procs)
                LOG.info("launcher: bye")
        return int(rc)

    # ---- Restart loop ----------------------------------------------------
    # Each iteration spawns a FRESH capture+vio+ba+slam+ui generation, blocks on the
    # UI, then tears that generation down. The IPC bus is one-way (server->client)
    # so the UI cannot reset vio/ba/slam in place; the robust "chay lai tu dau" is a
    # full respawn, which the UI requests via the RESTART_EXIT_CODE return code.
    rc = 0
    try:
        while True:
            _spawn_pipeline()
            # UI in foreground -- inherits stdout / stderr / stdin so the user
            # sees Qt warnings and can Ctrl-C cleanly.
            ui_proc = subprocess.Popen([py, "-m", "ui.main", *ui_args],
                                       env=env)
            procs.append(ui_proc)
            try:
                rc = ui_proc.wait()
            except KeyboardInterrupt:
                LOG.info("launcher: SIGINT -> stopping UI")
                try:
                    ui_proc.terminate()
                except Exception:                                  # noqa: BLE001
                    pass
                rc = ui_proc.wait(timeout=5.0)
            # Tear down THIS generation on the main thread (no waitpid race: the
            # UI is already reaped by `ui_proc.wait()` above). Ignore a second
            # Ctrl-C so the teardown can't be aborted half-way (the restored
            # handler keeps the NEXT generation interruptible).
            LOG.info("launcher: shutting down background procs ...")
            with _ignore_sigint():
                _terminate(procs)
            if rc == RESTART_EXIT_CODE:
                LOG.info("launcher: restart requested -> respawning pipeline")
                continue
            break
    finally:
        LOG.info("launcher: bye")

    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())
