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
#   ./deploy/pi-run.sh --logs                # just tail the live run.log, no start
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
PORT="8787"
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    --ui)   WANT_UI=1; shift ;;
    --logs) LOGS_ONLY=1; shift ;;
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

# Build the remote run.sh args. --ui adds the cross-machine bridge.
RUN_ARGS=(--no-ui)
[ "${#EXTRA[@]}" -gt 0 ] && RUN_ARGS+=("${EXTRA[@]}")
ENV_PREFIX=""
if [ "$WANT_UI" -eq 1 ]; then
  RUN_ARGS+=(--forward "0.0.0.0:$PORT")
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
  pi_say "    ./deploy/pi-ui.sh --connect $PI_IP:$PORT     (or just ./deploy/pi-ui.sh)"
  pi_rule
fi
