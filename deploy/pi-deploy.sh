#!/usr/bin/env bash
# pi-deploy.sh -- TASK 3 (install): push the flight-vio repo to the Pi and build
# its flight venv, entirely from the Mac. The operator never logs into the Pi.
#
# Steps (all over the cached, key-authenticated connection):
#   1. ensure the apt prerequisites (python3.13-venv / -dev / build-essential /
#      rsync) are installed on the Pi (sudo, idempotent);
#   2. rsync the repo to ~/flight-vio (code + gold session; NOT .venv/.git/.cache);
#   3. run deploy/pi/setup_pi.sh on the Pi -> create .venv + install the lean
#      flight deps (requirements-flight.txt) + the validation smoke.
#
# Idempotent: re-running re-syncs changed files and reuses the existing venv.
#
#   ./deploy/pi-deploy.sh             # rsync + bootstrap + smoke
#   ./deploy/pi-deploy.sh --no-smoke  # rsync + bootstrap, skip the validation run
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh

RUN_CHECK=1
case "${1:-}" in
  -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
  --no-smoke) RUN_CHECK=0 ;;
esac

pi_require
DEST="flight-vio"     # ~/flight-vio on the Pi

pi_rule
pi_say "deploying flight-vio -> $PI_USER@$PI_IP:~/$DEST"
pi_rule

# 1. apt prerequisites (idempotent). Refresh the index first -- a stale index 404s
#    on point releases (the cached .deb version is gone from the mirror).
pi_say "[1/3] ensuring apt prerequisites on the Pi (apt-get update + install) ..."
pi_sudo "apt-get update -qq" || pi_warn "apt-get update reported an issue (continuing)."
pi_sudo "apt-get install -y python3.13-venv python3.13-dev build-essential rsync" \
  || pi_warn "apt install reported an issue -- setup_pi.sh will flag any genuinely \
missing prereq below."

# 1b. OAK udev rules (Linux-only, and the reason a fresh Pi reports "no OAK device
#     found" even with the camera plugged in): without a rule granting the login
#     user access to the Movidius USB device (vendor 03e7), depthai only sees it as
#     root. Install + reload; idempotent. The rule body is PIPED (no remote quoting),
#     and the cp+reload+trigger run under ONE `sudo bash -c` (pi_sudo only elevates
#     the first word of a chain).
pi_say "[1b/3] ensuring OAK udev rules (depthai USB access as the login user) ..."
printf 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"\n' \
  | pi_ssh "cat > /tmp/80-movidius.rules"
pi_sudo "bash -c 'cp /tmp/80-movidius.rules /etc/udev/rules.d/80-movidius.rules && \
udevadm control --reload-rules && udevadm trigger'" \
  || pi_warn "could not install the OAK udev rule -- live capture may need 'sudo'."

# 2. rsync the repo (lean: code only -- NOT venv/git/cache/logs/baseline, and NOT
#    sessions/: those are multi-GB recorded replays the LIVE flight Pi never needs).
pi_say "[2/3] rsync repo (code only, no sessions) -> ~/$DEST ..."
pi_sync --delete \
  --exclude='.git/' --exclude='.venv/' --exclude='.cache/' \
  --exclude='__pycache__/' --exclude='*.pyc' --exclude='*.log' \
  --exclude='.vscode/' --exclude='baseline/' --exclude='sessions/' \
  ./ "$PI_USER@$PI_IP:$DEST/"
pi_say "rsync done."

# 3. bootstrap the venv + flight deps ON the Pi (--no-smoke: the smoke replays a
#    multi-GB session we deliberately did NOT ship; we validate session-free below).
pi_say "[3/3] bootstrap venv + flight deps on the Pi (setup_pi.sh) ..."
pi_ssh "cd ~/$DEST && ./deploy/pi/setup_pi.sh --no-smoke"

# 3b. Lean, session-free validation -- proves the aarch64 venv actually works:
#     the flight deps import (numpy/numba/llvmlite/pyserial/depthai) and the IPC
#     codec round-trips byte-for-byte. (Full live perf is measured by pi-run.sh.)
if [ "$RUN_CHECK" -eq 1 ]; then
  pi_say "validating aarch64 venv (imports + codec round-trip) ..."
  pi_ssh "cd ~/$DEST && .venv/bin/python -c '
import importlib, sys
for m in (\"numpy\",\"numba\",\"llvmlite\",\"serial\",\"depthai\"):
    try: importlib.import_module(m); print(\"  import OK:\", m)
    except Exception as e: print(\"  import FAIL:\", m, \"->\", e)
'"
  pi_ssh "cd ~/$DEST && .venv/bin/python -m imu_camera.tests.codec_roundtrip_selftest >/dev/null 2>&1 \
    && echo '  codec round-trip: PASS' || echo '  codec round-trip: FAIL'"
fi

pi_rule
pi_say "deploy complete. Next:"
pi_say "  ./deploy/pi-optimize.sh                    # reduce boot time"
pi_say "  ./deploy/pi-run.sh --vl53l9cx --direct     # run the flight VIO (headless)"
pi_rule
