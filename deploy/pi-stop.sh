#!/usr/bin/env bash
# pi-stop.sh -- TASK 3 (stop): stop the flight VIO running on the Pi, from the Mac.
#
# Sends the recorded run to a clean shutdown: SIGTERM the launcher PID (the launcher
# tears down imu_camera/vio/slam + the bridge on SIGTERM), wait, then SIGKILL any
# straggler and sweep leftover python -m {launcher,imu_camera,vio,slam}.main procs.
#
#   ./deploy/pi-stop.sh
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh
case "${1:-}" in -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;; esac

pi_require
DEST="flight-vio"

pi_say "stopping the flight stack on $PI_USER@$PI_IP ..."
pi_ssh "
  set -e
  cd ~/$DEST 2>/dev/null || exit 0
  if [ -f run.pid ]; then
    pid=\$(cat run.pid)
    if kill -0 \"\$pid\" 2>/dev/null; then
      echo '  SIGTERM launcher pid='\$pid
      kill -TERM \"\$pid\" 2>/dev/null || true
      for _ in \$(seq 1 25); do kill -0 \"\$pid\" 2>/dev/null || break; sleep 0.4; done
      kill -0 \"\$pid\" 2>/dev/null && { echo '  SIGKILL straggler'; kill -KILL \"\$pid\" 2>/dev/null || true; }
    else
      echo '  recorded PID not running'
    fi
    rm -f run.pid
  else
    echo '  no run.pid'
  fi
  # Sweep any leftover stack processes (e.g. from a crash that left no pid file).
  # Include netbridge.forward: a stale forward keeps holding port 8787, so the next
  # run's forward can't bind and pi-ui ends up on the DEAD one (UI shows no pose).
  left=\$(pgrep -f 'python -m ((launcher|imu_camera|vio|ba|slam)\\.main|netbridge\\.forward)' || true)
  if [ -n \"\$left\" ]; then echo '  sweeping leftover procs:' \$left; pkill -TERM -f 'python -m ((launcher|imu_camera|vio|ba|slam)\\.main|netbridge\\.forward)' || true; sleep 1; pkill -KILL -f 'python -m ((launcher|imu_camera|vio|ba|slam)\\.main|netbridge\\.forward)' || true; fi
  echo '  stopped.'
"
pi_say "done."
