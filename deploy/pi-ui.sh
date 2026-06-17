#!/usr/bin/env bash
# Mac-side launcher for the cross-machine live bridge.
#
# Runs the UI on THIS Mac against a flight stack running on a remote Pi. The Pi
# must be running the pipeline WITH the bridge, e.g.:
#
#   # on the Pi:
#   export OAKD_NETBRIDGE_KEY=<shared-secret>
#   ./run.sh --vl53l9cx --forward 0.0.0.0:8787      # (or --no-ui --forward ...)
#
# This script:
#   1. starts `netbridge.receive` (TCP client -> re-serves oak.capture / oak.vio /
#      oak.slam on THIS Mac's AF_UNIX sockets, ring-sized from the forwarded calib),
#   2. waits for those three sockets to appear,
#   3. runs the UNCHANGED `ui.main` on the canonical endpoints.
#
# OAKD_NETBRIDGE_KEY is OPTIONAL: export the SAME secret on both hosts for real auth,
# or leave it unset and both ends use a built-in default key (trusted LAN, no setup).
#
# Bandwidth mode: this defaults to POSE-ONLY (passes --pose-only to netbridge.receive)
# -- only the small pose/map/overlay topics cross the WiFi, not the heavy camera/depth/
# keyframe frames. The Pi side MUST be running the SAME way (pi-run.sh --ui defaults
# pose-only too). The trajectory + map UI works fully; only the opt-in camera Visualize
# windows have no frames. Pass --frames to include the camera frames -- then the Pi
# side MUST also have been started with `./deploy/pi-run.sh --ui --frames`.
#
# Threat model (HONEST): the authkey AUTHENTICATES the peer but does NOT encrypt
# the stream. For an untrusted network, tunnel it: e.g. `ssh -L 8787:localhost:8787
# pi@<pi-host>` then `./deploy/pi-ui.sh --connect 127.0.0.1:8787` -- the SSH
# forward provides the encryption and netbridge sees only loopback.
#
# Usage:
#   ./deploy/pi-ui.sh --connect <pi-host>:8787 [extra ui.main args]   # pose-only
#   ./deploy/pi-ui.sh --frames --connect <pi-host>:8787               # incl. frames
#   ./deploy/pi-ui.sh --connect 192.168.1.50:8787 --ba-window
set -e
cd "$(dirname "$0")/.."          # deploy/pi-ui.sh -> repo root (where .venv lives)

if [ ! -d .venv ]; then
  echo "[pi-ui.sh] .venv missing — bootstrap with:" >&2
  echo "  python3.13 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
  exit 1
fi

# Auth needs no setup: with OAKD_NETBRIDGE_KEY set the bridge uses that shared
# secret; without it BOTH ends fall back to the same built-in default key (so this
# must match how the Pi side ran -- pi-run.sh --ui defaults the same way). The stream
# is never encrypted either way; tunnel over SSH on an untrusted network.
if [ -z "${OAKD_NETBRIDGE_KEY:-}" ]; then
  echo "[pi-ui.sh] OAKD_NETBRIDGE_KEY not set — using the built-in default bridge key" >&2
  echo "  (trusted LAN; the Pi side must run the same way). Export the SAME secret on" >&2
  echo "  both hosts for real auth on an untrusted network." >&2
fi

# --- parse --connect HOST:PORT + --frames; default to the cached Pi IP : 8787 ---
# WANT_FRAMES default OFF -> receive runs --pose-only (low-bandwidth). --frames drops
# it so the heavy camera/depth/keyframe topics are subscribed (Pi side must match).
CONNECT=""
WANT_FRAMES=0
UI_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --connect) CONNECT="$2"; shift 2 ;;
    --connect=*) CONNECT="${1#--connect=}"; shift ;;
    --frames) WANT_FRAMES=1; shift ;;
    *) UI_ARGS+=("$1"); shift ;;
  esac
done

# No explicit --connect? use the Pi from the deploy connection cache (pi-discover.sh),
# so `./deploy/pi-ui.sh` "just works" after a `./deploy/pi-run.sh --ui`.
if [ -z "$CONNECT" ] && [ -f .cache/pi_connection.env ]; then
  # shellcheck disable=SC1091
  . .cache/pi_connection.env
  [ -n "${PI_IP:-}" ] && CONNECT="$PI_IP:8787"
fi

if [ -z "$CONNECT" ]; then
  echo "[pi-ui.sh] no Pi to connect to. Pass --connect <pi-host>:PORT, or run" >&2
  echo "  ./deploy/pi-discover.sh first so the cached IP is used automatically." >&2
  exit 2
fi

# Pose-only by default (must match the Pi side); --frames includes the camera frames.
RECV_MODE_ARGS=()
if [ "$WANT_FRAMES" -eq 1 ]; then
  echo "[pi-ui.sh] connecting UI to $CONNECT  (FRAMES mode -- Pi must run pi-run.sh --ui --frames)"
else
  RECV_MODE_ARGS+=(--pose-only)
  echo "[pi-ui.sh] connecting UI to $CONNECT  (pose-only -- low bandwidth; Pi defaults to match)"
fi

# Canonical endpoints the receive side re-serves + the UI connects to.
CAP_EP="oak.capture"
VIO_EP="oak.vio"
SLAM_EP="oak.slam"

# Resolve the IPC socket directory the SAME way comms.ipc does ($TMPDIR/ours_ipc).
SOCK_DIR="$(.venv/bin/python -c 'import tempfile, os; print(os.path.join(tempfile.gettempdir(), "ours_ipc"))')"

# Sweep ORPHAN receivers + STALE sockets from a previous watch session. This is the
# Mac-side analog of the Pi forward-sweep in pi-run.sh/pi-stop.sh, and it is the real
# safety net: closing the UI terminal sends SIGHUP, and a hard kill sends SIGKILL --
# neither runs the cleanup trap below, so the background netbridge.receive LEAKS and
# keeps owning oak.{capture,vio,slam}.sock. A new run then sees those leftover sockets,
# its wait-loop below false-positives ("sockets up"), and ui.main attaches to the DEAD
# receiver -> the UI shows no pose ("src fps 0.0 / samples 0"). Kill every stray
# receiver and delete the re-served sockets so the fresh receiver below binds clean.
# (pi-ui is the only thing that runs netbridge.receive on the Mac, so a blanket sweep
# is safe; you watch one Pi at a time.)
# Match on the module name only -- the Mac's Homebrew interpreter is
# ".../Python.app/Contents/MacOS/Python" (no lowercase "python" substring), so a
# "python -m netbridge.receive" pattern would NOT match here. "netbridge.receive" is
# unambiguous (pi-ui is the only thing that runs it on the Mac) and matches any
# interpreter path/case.
if pgrep -f 'netbridge\.receive' >/dev/null 2>&1; then
  echo "[pi-ui.sh] sweeping orphan netbridge.receive from a previous watch session ..." >&2
  pkill -TERM -f 'netbridge\.receive' 2>/dev/null || true
  sleep 1
  pkill -KILL -f 'netbridge\.receive' 2>/dev/null || true
fi
rm -f "$SOCK_DIR/$CAP_EP.sock" "$SOCK_DIR/$VIO_EP.sock" "$SOCK_DIR/$SLAM_EP.sock"

echo "[pi-ui.sh] starting netbridge.receive (connect $CONNECT) ..."
.venv/bin/python -m netbridge.receive \
    --connect "$CONNECT" \
    --capture-endpoint "$CAP_EP" \
    --vio-endpoint "$VIO_EP" \
    --slam-endpoint "$SLAM_EP" \
    "${RECV_MODE_ARGS[@]}" &
RECV_PID=$!

# Tear the receiver down whenever this script exits (clean quit, error/Ctrl-C, OR the
# terminal window being closed -- HUP). A leaked receiver keeps owning the re-served
# sockets and breaks the NEXT run (see the sweep above), so we also unlink them here.
cleanup() {
  if kill -0 "$RECV_PID" 2>/dev/null; then
    echo "[pi-ui.sh] stopping netbridge.receive (pid $RECV_PID) ..."
    kill "$RECV_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5; do kill -0 "$RECV_PID" 2>/dev/null || break; sleep 0.3; done
    kill -KILL "$RECV_PID" 2>/dev/null || true
    wait "$RECV_PID" 2>/dev/null || true
  fi
  rm -f "$SOCK_DIR/$CAP_EP.sock" "$SOCK_DIR/$VIO_EP.sock" "$SOCK_DIR/$SLAM_EP.sock"
}
trap cleanup EXIT INT TERM HUP

# Wait for the three re-served sockets to appear (receive creates them only AFTER
# it has learned the resolution from the forwarded calib.bundle -- so a socket
# present means the bridge handshake + ring sizing completed).
echo "[pi-ui.sh] waiting for re-served sockets in $SOCK_DIR ..."
DEADLINE=$(( $(date +%s) + 90 ))
while :; do
  if [ -S "$SOCK_DIR/$CAP_EP.sock" ] && \
     [ -S "$SOCK_DIR/$VIO_EP.sock" ] && \
     [ -S "$SOCK_DIR/$SLAM_EP.sock" ]; then
    break
  fi
  if ! kill -0 "$RECV_PID" 2>/dev/null; then
    echo "[pi-ui.sh] could not connect to $CONNECT — no bridge is serving there." >&2
    echo "  Make sure './deploy/pi-run.sh --ui ...' is RUNNING on the Pi AND has a" >&2
    echo "  data source: a camera attached, or a '--session <gold>' replay. A LIVE" >&2
    echo "  run with no camera produces no calib, so the forward never opens its" >&2
    echo "  port. (If you set OAKD_NETBRIDGE_KEY on one side, set the SAME on both.)" >&2
    exit 1
  fi
  if [ "$(date +%s)" -ge "$DEADLINE" ]; then
    echo "[pi-ui.sh] timed out waiting for re-served sockets (90s) — is" >&2
    echo "  './deploy/pi-run.sh --ui ...' running on the Pi with a camera or a" >&2
    echo "  --session? (a live run with no camera never opens the bridge port.)" >&2
    exit 1
  fi
  sleep 0.3
done

echo "[pi-ui.sh] sockets up — launching ui.main ..."
exec .venv/bin/python -m ui.main \
    --capture-endpoint "$CAP_EP" \
    --vio-endpoint "$VIO_EP" \
    --slam-endpoint "$SLAM_EP" \
    "${UI_ARGS[@]}"
