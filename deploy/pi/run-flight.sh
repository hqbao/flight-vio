#!/usr/bin/env bash
# run-flight.sh -- (runs ON the Pi) start the flight stack DETACHED and record its
# PID + log, then report. Driven by deploy/pi-run.sh from the Mac; not meant to be
# called by hand. All arguments are forwarded verbatim to ./run.sh.
#
# Detachment: setsid puts run.sh in its own session so it survives the SSH session
# closing; the real launcher PID is captured from INSIDE the new session (echo $$),
# not from setsid's $! (setsid forks, so its $! is the wrong, already-exited PID).
set -eu
cd "$(dirname "$0")/../.."          # deploy/pi/run-flight.sh -> repo root
rm -f run.pid run.log

# New session; the inner shell writes its OWN pid (which exec ./run.sh inherits)
# then becomes run.sh. stdin from /dev/null, all output to run.log.
setsid bash -c 'echo $$ > run.pid; exec ./run.sh "$@"' _ "$@" \
  > run.log 2>&1 < /dev/null &

# Give it a moment to write the pid + get past an immediate crash, then verify.
sleep 2
if [ -f run.pid ] && kill -0 "$(cat run.pid)" 2>/dev/null; then
  echo "started pid=$(cat run.pid)"
else
  echo "FAILED to start -- first lines of run.log:"
  sed -n '1,30p' run.log 2>/dev/null
  exit 1
fi
