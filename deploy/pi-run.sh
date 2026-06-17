#!/usr/bin/env bash
# pi-run.sh -- TASK 3 (run): start the flight VIO on the Pi, headless and detached,
# from the Mac. The stack keeps running after this script (and the SSH session)
# exits; logs go to ~/flight-vio/run.log and the PID to ~/flight-vio/run.pid on the
# Pi. Stop it with ./deploy/pi-stop.sh.
#
#   ./deploy/pi-run.sh                       # run.sh --no-ui  (default)
#   ./deploy/pi-run.sh --vl53l9cx --direct   # the 54x42 ToF flight recipe
#   ./deploy/pi-run.sh --ui [args]           # ALSO open the netbridge so you can
#                                            #   watch live from the Mac:
#                                            #     ./deploy/pi-ui.sh
#   ./deploy/pi-run.sh --ui --frames [args]  # ...and ALSO bridge the camera frames
#                                            #   (watch pi-ui.sh --frames on the Mac)
#   ./deploy/pi-run.sh --logs                # just tail the live run.log, no start
#
# Pi defaults (no flag needed): --worker ON (the bursty windowed-BA solve runs in
# its own process so it can't stall the vio/frontend core) and --cap-numba-threads
# (per-process numba thread caps so capture+vio don't oversubscribe the 4 cores).
#   --no-worker  keep the BA/SLAM solve in-thread (reproduce the stall / spare-core host)
#   --viz        with --ui, keep the Frontend-Internals + BA-Window UI captures ON
#                (OFF by default on the Pi: they drag the vio process below real-time)
#   --frames     with --ui, ALSO bridge the heavy camera/depth/keyframe IMAGE topics
#                (~51 Mbit/s). DEFAULT is POSE-ONLY: only the small pose+map+overlay
#                topics cross the WiFi, because the main trajectory+map UI never shows
#                the camera image and a congested 2.4GHz link (~1.6 Mbit/s) is easily
#                oversubscribed by the raw frames. Use --frames only on a FAST link
#                to feed the opt-in camera Visualize windows -- and run the Mac side
#                with the MATCHING `./deploy/pi-ui.sh --frames` (both default pose-only).
#
# --ui: if OAKD_NETBRIDGE_KEY is exported on the Mac it is forwarded to the Pi run
# (authenticated; the SAME secret pi-ui.sh needs). If it is NOT set, the bridge runs
# UNAUTHENTICATED on a trusted LAN (a warning is printed) -- run pi-ui.sh the same way.
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh

DEST="flight-vio"
WANT_UI=0
LOGS_ONLY=0
WANT_VIZ=0
WANT_FRAMES=0          # default OFF: the bridge is POSE-ONLY (low-bandwidth). Pass
                       # --frames to also bridge the heavy camera/depth/keyframe
                       # image topics (only worth it on a fast link).
WANT_WORKER=1          # default ON: the Pi is 4-core, so move the bursty BA/SLAM
                       # solve off the frontend core (measured: loose 320x200 vio
                       # process drops from a 80ms 1-in-5 hitch to ~33ms/frame).
                       # Pass --no-worker to keep it in-thread.
PORT="8787"
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    --ui)        WANT_UI=1; shift ;;
    --logs)      LOGS_ONLY=1; shift ;;
    --viz)       WANT_VIZ=1; shift ;;
    --frames)    WANT_FRAMES=1; shift ;;   # bridge camera frames too (default pose-only)
    --worker)    WANT_WORKER=1; shift ;;   # explicit (already the default)
    --no-worker) WANT_WORKER=0; shift ;;   # keep BA/SLAM in-thread
    --port) PORT="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done

pi_require

if [ "$LOGS_ONLY" -eq 1 ]; then
  pi_say "tailing ~/$DEST/run.log (Ctrl-C to stop) ..."
  pi_ssh "tail -n 40 -f ~/$DEST/run.log"   # pi_ssh is a function -> no exec
  exit 0
fi

# Refuse to double-start: a live PID means a stack is already running.
if pi_ssh "test -f ~/$DEST/run.pid && kill -0 \$(cat ~/$DEST/run.pid) 2>/dev/null"; then
  pi_die "a flight stack is already running on the Pi (PID $(pi_ssh "cat ~/$DEST/run.pid")). \
Stop it first:  ./deploy/pi-stop.sh"
fi

# Sweep ORPHANS from a crashed/interrupted previous run. We're past the live-PID
# guard above, so any lingering capture/vio/slam/launcher procs are orphans -- and
# a stray capture still HOLDS the OAK USB device, so the new run opens "no OAK
# device found" (the classic "it worked, then after a crash it stopped working").
# pi-stop does this on teardown; do it here too so a forgotten stop never blocks.
# Include netbridge.forward in the sweep: a stale forward keeps holding port 8787,
# so this run's forward can't bind and pi-ui connects to the DEAD one (UI shows no
# pose / "samples 0") even though the live stack publishes fine.
_ORPH='python -m ((launcher|imu_camera|vio|ba|slam)\\.main|netbridge\\.forward)'
if pi_ssh "pgrep -f '$_ORPH' >/dev/null 2>&1"; then
  pi_warn "sweeping orphan procs from a previous run (they hold the OAK device / port 8787) ..."
  pi_ssh "pkill -TERM -f '$_ORPH' 2>/dev/null || true; sleep 1; pkill -KILL -f '$_ORPH' 2>/dev/null || true"
  sleep 1   # give the kernel a moment to release the USB handle + the 8787 socket
fi

# Build the remote run.sh args. --ui adds the cross-machine bridge.
RUN_ARGS=(--no-ui)
# On the 4-core Pi: run the heavy solves in worker processes (--worker) and cap
# each child's numba pool so overlapping SGM (capture) + KLT (vio) bursts don't
# oversubscribe the 4 cores (--cap-numba-threads -> capture=2, vio=2, slam=1).
# Both are no-ops on a big dev host and gap-safe (no math changes).
[ "$WANT_WORKER" -eq 1 ] && RUN_ARGS+=(--worker)
RUN_ARGS+=(--cap-numba-threads)
[ "${#EXTRA[@]}" -gt 0 ] && RUN_ARGS+=("${EXTRA[@]}")
ENV_PREFIX=""
if [ "$WANT_UI" -eq 1 ]; then
  RUN_ARGS+=(--forward "0.0.0.0:$PORT")
  # Bandwidth mode: the bridge is POSE-ONLY by DEFAULT (launcher appends
  # --pose-only unless --bridge-frames is set) -- only the small pose/map/overlay
  # topics cross the WiFi, not the ~51 Mbit/s of uncompressed camera/depth/keyframe
  # frames the main trajectory+map UI never displays. --frames opts back into
  # bridging those image topics (for the camera Visualize windows over a fast link);
  # the Mac side must then run `pi-ui.sh --frames` to match.
  [ "$WANT_FRAMES" -eq 1 ] && RUN_ARGS+=(--bridge-frames)
  # The UI's "Frontend Internals" + "BA Window" diagnostic CAPTURES run in the VIO
  # process and measurably drag it BELOW real-time on the Pi (measured 160x100:
  # pose.odom 24.6 fps without them vs 16.6 with -> the difference between keeping
  # up with a 20 fps camera and lagging). They are deep-debug views, not needed to
  # watch the live trajectory/image/depth, so the Pi defaults them OFF; pass --viz
  # to keep them (e.g. when debugging on a fast host).
  if [ "$WANT_VIZ" -eq 0 ]; then
    RUN_ARGS+=(--no-frontend-viz --no-ba-window)
  fi
  if [ -n "${OAKD_NETBRIDGE_KEY:-}" ]; then
    # Custom secret: forward it to the Pi run's environment (both ends authenticate
    # with it). Run pi-ui.sh on the Mac with the SAME secret exported.
    ENV_PREFIX="OAKD_NETBRIDGE_KEY=$(printf '%q' "$OAKD_NETBRIDGE_KEY") "
  else
    # No secret exported -> the bridge falls back to the built-in default key on BOTH
    # ends, so pi-ui.sh connects with no setup. Fine for a trusted LAN.
    pi_say "OAKD_NETBRIDGE_KEY not set -> using the built-in default bridge key "
    pi_say "(trusted LAN; run pi-ui.sh the same way -- no key needed)."
  fi
fi

pi_say "starting flight stack on the Pi: run.sh ${RUN_ARGS[*]}"
# Delegate the detach + pid/log bookkeeping to the on-Pi helper (avoids fragile
# nested SSH quoting; it captures the REAL launcher PID from inside the new session).
if ! pi_ssh "${ENV_PREFIX}~/$DEST/deploy/pi/run-flight.sh ${RUN_ARGS[*]}"; then
  pi_die "the flight stack failed to start (see the run.log lines above)."
fi

pi_say "running (PID $(pi_ssh "cat ~/$DEST/run.pid" 2>/dev/null)). Logs:  ./deploy/pi-run.sh --logs"

# Catch the #1 confusion: a LIVE run with no camera on the Pi. capture then fails
# ("no OAK device found"), so there is no data -- pose is empty and (with --ui) the
# forward never gets a calib.bundle, so it never opens its port and pi-ui hangs.
sleep 1
if pi_ssh "grep -q 'live build failed: no OAK device' ~/$DEST/run.log 2>/dev/null"; then
  pi_warn "capture could NOT open a camera on the Pi (no OAK device found)."
  pi_warn "  -> the stack has NO data. Plug the OAK-D into the PI, or replay a"
  pi_warn "     recorded session:  ./deploy/pi-run.sh ${WANT_UI:+--ui }--vl53l9cx --direct \\"
  pi_warn "                          --session sessions/gold/lab_static_10s"
  [ "$WANT_UI" -eq 1 ] && pi_warn "  (with no data the --ui bridge never opens; pi-ui.sh would hang.)"
fi

if [ "$WANT_UI" -eq 1 ]; then
  pi_rule
  pi_say "when the stack has data (camera attached or --session), watch from the Mac:"
  if [ "$WANT_FRAMES" -eq 1 ]; then
    pi_say "    ./deploy/pi-ui.sh --frames --connect $PI_IP:$PORT   (frames bridged -> Mac must match)"
  else
    pi_say "    ./deploy/pi-ui.sh --connect $PI_IP:$PORT     (or just ./deploy/pi-ui.sh; pose-only)"
  fi
  pi_rule
fi
