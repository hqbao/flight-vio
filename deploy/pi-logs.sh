#!/usr/bin/env bash
# pi-logs.sh -- fetch flight-vio pose logs (flight_logs/vio_*.csv) from the Pi to this Mac.
#
# A live flight stack (deploy/pi-run.sh) writes one CSV per run under
# ~/flight-vio/flight_logs/ on the Pi: EVERY VIO pose at full rate (pos, quat,
# sigma, ok/degraded, sensor_gap). This pulls them down so VIO drift / noise /
# dropouts can be analysed offline and handed off.
#
#   ./deploy/pi-logs.sh                 # rsync logs -> ./flight_logs/ (this repo)
#   ./deploy/pi-logs.sh --dest DIR      # rsync into DIR instead
#   ./deploy/pi-logs.sh --list          # list the remote logs, fetch nothing
#   ./deploy/pi-logs.sh --clean         # after a successful fetch, delete them on the Pi
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh
case "${1:-}" in -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;; esac

DEST="flight_logs"
LIST=0; CLEAN=0
while [ $# -gt 0 ]; do
  case "$1" in
    --dest)  DEST="${2:?--dest needs a directory}"; shift 2 ;;
    --list)  LIST=1; shift ;;
    --clean) CLEAN=1; shift ;;
    *) pi_die "unknown arg: $1 (try --help)" ;;
  esac
done

pi_require
REMOTE="flight-vio/flight_logs"

if [ "$LIST" = 1 ]; then
  pi_say "remote logs on $PI_USER@$PI_IP:~/$REMOTE :"
  pi_ssh "ls -lh ~/$REMOTE/vio_*.csv 2>/dev/null || echo '  (none yet — fly a live run)'"
  exit 0
fi

if ! pi_ssh "ls ~/$REMOTE/vio_*.csv >/dev/null 2>&1"; then
  pi_say "no logs on the Pi yet (~/$REMOTE/vio_*.csv). Fly a live run first (deploy/pi-run.sh)."
  exit 0
fi

mkdir -p "$DEST"
pi_say "fetching VIO flight logs -> $DEST/ ..."
pi_sync "$PI_USER@$PI_IP:$REMOTE/vio_*.csv" "$DEST/"
pi_say "got:"
ls -lht "$DEST"/vio_*.csv 2>/dev/null | head

if [ "$CLEAN" = 1 ]; then
  pi_say "deleting fetched logs on the Pi ..."
  pi_ssh "rm -f ~/$REMOTE/vio_*.csv" && pi_say "remote cleaned."
fi
pi_say "done. (column header is the first lines of each CSV.)"
