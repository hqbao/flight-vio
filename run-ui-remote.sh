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
# The same shared HMAC secret must be exported on BOTH hosts:
#   export OAKD_NETBRIDGE_KEY=<shared-secret>
#
# Threat model (HONEST): the authkey AUTHENTICATES the peer but does NOT encrypt
# the stream. For an untrusted network, tunnel it: e.g. `ssh -L 8787:localhost:8787
# pi@<pi-host>` then `./run-ui-remote.sh --connect 127.0.0.1:8787` -- the SSH
# forward provides the encryption and netbridge sees only loopback.
#
# Usage:
#   ./run-ui-remote.sh --connect <pi-host>:8787 [extra ui.main args]
#   ./run-ui-remote.sh --connect 192.168.1.50:8787 --ba-window
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "[run-ui-remote.sh] .venv missing — bootstrap with:" >&2
  echo "  python3.13 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
  exit 1
fi

if [ -z "${OAKD_NETBRIDGE_KEY:-}" ]; then
  echo "[run-ui-remote.sh] OAKD_NETBRIDGE_KEY is unset — export the SAME shared" >&2
  echo "  secret you exported on the Pi, e.g.:" >&2
  echo "    export OAKD_NETBRIDGE_KEY=<shared-secret>" >&2
  exit 1
fi

# --- parse --connect HOST:PORT (required); everything else flows to ui.main ----
CONNECT=""
UI_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --connect) CONNECT="$2"; shift 2 ;;
    --connect=*) CONNECT="${1#--connect=}"; shift ;;
    *) UI_ARGS+=("$1"); shift ;;
  esac
done

if [ -z "$CONNECT" ]; then
  echo "[run-ui-remote.sh] --connect <pi-host>:PORT is required" >&2
  exit 2
fi

# Canonical endpoints the receive side re-serves + the UI connects to.
CAP_EP="oak.capture"
VIO_EP="oak.vio"
SLAM_EP="oak.slam"

# Resolve the IPC socket directory the SAME way comms.ipc does ($TMPDIR/ours_ipc).
SOCK_DIR="$(.venv/bin/python -c 'import tempfile, os; print(os.path.join(tempfile.gettempdir(), "ours_ipc"))')"

echo "[run-ui-remote.sh] starting netbridge.receive (connect $CONNECT) ..."
.venv/bin/python -m netbridge.receive \
    --connect "$CONNECT" \
    --capture-endpoint "$CAP_EP" \
    --vio-endpoint "$VIO_EP" \
    --slam-endpoint "$SLAM_EP" &
RECV_PID=$!

# Tear the receiver down whenever this script exits (clean quit OR error/Ctrl-C).
cleanup() {
  if kill -0 "$RECV_PID" 2>/dev/null; then
    echo "[run-ui-remote.sh] stopping netbridge.receive (pid $RECV_PID) ..."
    kill "$RECV_PID" 2>/dev/null || true
    wait "$RECV_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Wait for the three re-served sockets to appear (receive creates them only AFTER
# it has learned the resolution from the forwarded calib.bundle -- so a socket
# present means the bridge handshake + ring sizing completed).
echo "[run-ui-remote.sh] waiting for re-served sockets in $SOCK_DIR ..."
DEADLINE=$(( $(date +%s) + 90 ))
while :; do
  if [ -S "$SOCK_DIR/$CAP_EP.sock" ] && \
     [ -S "$SOCK_DIR/$VIO_EP.sock" ] && \
     [ -S "$SOCK_DIR/$SLAM_EP.sock" ]; then
    break
  fi
  if ! kill -0 "$RECV_PID" 2>/dev/null; then
    echo "[run-ui-remote.sh] netbridge.receive exited before serving — check the" >&2
    echo "  Pi-side --forward + the shared OAKD_NETBRIDGE_KEY." >&2
    exit 1
  fi
  if [ "$(date +%s)" -ge "$DEADLINE" ]; then
    echo "[run-ui-remote.sh] timed out waiting for re-served sockets (90s)." >&2
    exit 1
  fi
  sleep 0.3
done

echo "[run-ui-remote.sh] sockets up — launching ui.main ..."
exec .venv/bin/python -m ui.main \
    --capture-endpoint "$CAP_EP" \
    --vio-endpoint "$VIO_EP" \
    --slam-endpoint "$SLAM_EP" \
    "${UI_ARGS[@]}"
