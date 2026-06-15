#!/usr/bin/env bash
# pi-optimize.sh -- TASK 2: cut the Pi's boot time by DISABLING (reversibly) the
# services/timers a headless flight build does not need -- driven entirely from the
# Mac. Wraps deploy/pi/optimize_pi.sh (which runs on the Pi, is idempotent, records
# what it changed, and can roll everything back).
#
# KEEPS working: WiFi/NetworkManager, ssh, mDNS (so <name>.local still resolves),
# clock sync, the camera + UART stack. Frees the UART for a flight controller.
#
#   ./deploy/pi-optimize.sh             # preview (dry-run) then APPLY
#   ./deploy/pi-optimize.sh --reboot    # apply, reboot the Pi, re-measure boot time
#   ./deploy/pi-optimize.sh --dry-run   # preview only, change nothing
#   ./deploy/pi-optimize.sh --rollback  # re-enable everything it disabled
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh

MODE="apply"
case "${1:-}" in
  -h|--help)  awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
  --dry-run)  MODE="dry" ;;
  --rollback) MODE="rollback" ;;
  --reboot)   MODE="reboot" ;;
  "")         MODE="apply" ;;
  *) pi_die "unknown arg: ${1} (try --help)" ;;
esac

pi_require
REMOTE=/tmp/fvio_optimize_pi.sh

# optimize_pi.sh is standalone (only systemctl) -> push to /tmp; no full deploy needed.
pi_say "uploading optimiser to the Pi ..."
pi_sync deploy/pi/optimize_pi.sh "$PI_USER@$PI_IP:$REMOTE"

pi_show_boot() { pi_ssh 'systemd-analyze 2>/dev/null | head -1; systemd-analyze blame 2>/dev/null | head -n 8'; }

case "$MODE" in
  rollback)
    pi_say "rolling back boot optimisations (re-enabling services) ..."
    pi_sudo "bash $REMOTE --rollback"
    pi_say "rollback done. The re-enabled services start on the next boot."
    ;;
  dry)
    pi_say "DRY-RUN preview (no changes):"
    pi_sudo "bash $REMOTE --dry-run"
    ;;
  apply|reboot)
    pi_rule; pi_say "BEFORE (boot time):"; pi_show_boot; pi_rule
    pi_say "preview:"; pi_sudo "bash $REMOTE --dry-run"
    pi_say "applying ..."; pi_sudo "bash $REMOTE"
    if [ "$MODE" = "reboot" ]; then
      pi_say "rebooting the Pi to re-measure ..."
      pi_sudo "reboot" || true
      pi_say "waiting for the Pi to come back ..."
      sleep 30
      back=0
      for _ in $(seq 1 30); do
        if pi_key_works; then back=1; break; fi
        sleep 4
      done
      [ "$back" -eq 1 ] || { pi_warn "Pi not back yet -- re-check later with: ./deploy/pi-discover.sh --status"; exit 0; }
      pi_rule; pi_say "AFTER (boot time):"; pi_show_boot; pi_rule
      pi_say "essentials still up:"
      pi_ssh 'for s in NetworkManager ssh; do printf "  %-16s %s\n" "$s" "$(systemctl is-active "$s")"; done; printf "  %-16s %s\n" "UART /dev/serial0" "$(ls /dev/serial0 2>/dev/null || echo missing)"'
    else
      pi_say "applied. Reboot to take effect:  ./deploy/pi-optimize.sh --reboot  (or it persists on next boot)."
    fi
    ;;
esac
