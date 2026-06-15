#!/usr/bin/env bash
#
# optimize_pi.sh -- reduce flight-vio Raspberry Pi 5 boot time by DISABLING
# (reversible) services/timers that a headless flight build does not need.
#
# KEEPS working: NetworkManager + wpa_supplicant (WiFi), ssh, avahi-daemon
# (mDNS -- so bao.local keeps resolving), systemd-timesyncd (clock sync, needed
# for sane VIO timestamps) and the camera / UART device stack.
#
# It also FREES the UART for the flight controller by masking the serial login
# console (serial-getty@ttyAMA10) -- that console otherwise holds /dev/serial0
# and would fight MAVLink.
#
# Runs ON the Pi, needs sudo, is idempotent, and is fully reversible:
#   sudo ./optimize_pi.sh --dry-run    # preview only, change nothing
#   sudo ./optimize_pi.sh              # apply
#   sudo ./optimize_pi.sh --rollback   # re-enable everything this script touched
#   sudo reboot                        # then `systemd-analyze` to compare
#
set -eu

DRY=0
ROLLBACK=0
for a in "$@"; do
  case "$a" in
    --dry-run)  DRY=1 ;;
    --rollback) ROLLBACK=1 ;;
    -h|--help)  awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    *) printf 'unknown arg: %s (try --help)\n' "$a" >&2; exit 2 ;;
  esac
done

say()  { printf '[optimize_pi] %s\n' "$*"; }
warn() { printf '[optimize_pi] WARNING: %s\n' "$*" >&2; }
die()  { printf '[optimize_pi] ERROR: %s\n' "$*" >&2; exit 1; }
rule() { printf -- '----------------------------------------------------------------------\n'; }

[ "$(id -u)" -eq 0 ] || die "must run as root -- use: sudo $0 $*"

STATE_DIR=/var/lib/flight-vio
STATE="$STATE_DIR/boot-optimize.state"
CLOUD_DISABLED=/etc/cloud/cloud-init.disabled
SERIAL_GETTY=serial-getty@ttyAMA10.service

# Services / timers a headless flight build does not need (heaviest first).
# All reversible via `systemctl enable`. The serial console is handled separately
# (masked, below) because it can be re-spawned from the kernel cmdline.
UNITS="
NetworkManager-wait-online.service
dpkg-db-backup.timer
e2scrub_all.timer
e2scrub_reap.service
apt-daily.timer
apt-daily-upgrade.timer
man-db.timer
rpi-eeprom-update.service
bluetooth.service
keyboard-setup.service
console-setup.service
"

rule
say "flight-vio boot-time optimiser (headless, reversible)"
[ "$DRY" -eq 1 ]      && say "MODE: dry-run (no changes will be made)"
[ "$ROLLBACK" -eq 1 ] && say "MODE: rollback"
rule

# ----------------------------- ROLLBACK ------------------------------------- #
if [ "$ROLLBACK" -eq 1 ]; then
  [ -f "$STATE" ] || die "no state file at $STATE -- nothing to roll back."
  while IFS= read -r line; do
    [ -n "$line" ] || continue
    case "$line" in
      cloud-init:disabled-file)
        if [ -e "$CLOUD_DISABLED" ]; then say "re-enable cloud-init (rm $CLOUD_DISABLED)"; rm -f "$CLOUD_DISABLED"; fi ;;
      mask:*)
        u=${line#mask:}; say "unmask + enable $u"
        systemctl unmask "$u" >/dev/null 2>&1 || true
        systemctl enable "$u" >/dev/null 2>&1 || warn "could not enable $u" ;;
      *)
        say "re-enable $line"
        systemctl enable "$line" >/dev/null 2>&1 || warn "could not enable $line" ;;
    esac
  done < "$STATE"
  rm -f "$STATE"
  say "rollback complete -- reboot to restore original boot behaviour."
  exit 0
fi

# ------------------------------- APPLY -------------------------------------- #
[ "$DRY" -eq 0 ] && mkdir -p "$STATE_DIR"
[ "$DRY" -eq 0 ] && : > "${STATE}.tmp"

disable_unit() {
  u="$1"
  st=$(systemctl is-enabled "$u" 2>/dev/null || true)   # enabled/disabled/masked/static/not-found/""
  act=$(systemctl is-active  "$u" 2>/dev/null || true)   # active/inactive/failed/""
  if [ "$st" = "not-found" ] || { [ -z "$st" ] && [ "$act" != "active" ]; }; then
    say "skip  $u (absent)"; return 0
  fi
  if { [ "$st" = "disabled" ] || [ "$st" = "masked" ]; } && [ "$act" != "active" ]; then
    say "skip  $u (already off)"; return 0
  fi
  if [ "$DRY" -eq 1 ]; then
    say "WOULD disable $u (enabled=$st active=$act)"; return 0
  fi
  if systemctl disable --now "$u" >/dev/null 2>&1; then
    say "disabled $u"; printf '%s\n' "$u" >> "${STATE}.tmp"
  elif systemctl mask --now "$u" >/dev/null 2>&1; then
    say "masked $u (static unit)"; printf '%s\n' "mask:$u" >> "${STATE}.tmp"
  else
    warn "could not disable/mask $u"
  fi
}

for u in $UNITS; do disable_unit "$u"; done

# Free the UART for the flight controller: MASK the serial login console.
sg_st=$(systemctl is-enabled "$SERIAL_GETTY" 2>/dev/null || true)
sg_act=$(systemctl is-active "$SERIAL_GETTY" 2>/dev/null || true)
if [ "$sg_st" = "masked" ]; then
  say "skip  $SERIAL_GETTY (already masked)"
elif [ "$sg_st" = "not-found" ] && [ "$sg_act" != "active" ]; then
  say "skip  $SERIAL_GETTY (absent)"
elif [ "$DRY" -eq 1 ]; then
  say "WOULD mask $SERIAL_GETTY (free UART /dev/serial0 for flight controller)"
elif systemctl mask --now "$SERIAL_GETTY" >/dev/null 2>&1; then
  say "masked $SERIAL_GETTY (UART /dev/serial0 freed)"; printf '%s\n' "mask:$SERIAL_GETTY" >> "${STATE}.tmp"
else
  warn "could not mask $SERIAL_GETTY"
fi

# cloud-init: disable cleanly via the documented sentinel file.
if [ ! -d /etc/cloud ]; then
  say "skip  cloud-init (not installed)"
elif [ -e "$CLOUD_DISABLED" ]; then
  say "skip  cloud-init (already disabled)"
elif [ "$DRY" -eq 1 ]; then
  say "WOULD disable cloud-init (touch $CLOUD_DISABLED)"
else
  touch "$CLOUD_DISABLED"; say "disabled cloud-init ($CLOUD_DISABLED)"
  printf '%s\n' "cloud-init:disabled-file" >> "${STATE}.tmp"
fi

if [ "$DRY" -eq 1 ]; then
  rule; say "dry-run complete -- re-run WITHOUT --dry-run to apply."; rule; exit 0
fi

# Merge with any prior state so --rollback always restores everything.
if [ -s "${STATE}.tmp" ]; then
  [ -f "$STATE" ] && cat "$STATE" >> "${STATE}.tmp"
  sort -u "${STATE}.tmp" > "$STATE"
  say "recorded $(wc -l < "$STATE" | tr -d ' ') changes in $STATE"
else
  say "nothing to change -- already optimised."
fi
rm -f "${STATE}.tmp"

rule
say "Applied. Next steps:"
say "  1) sudo reboot"
say "  2) systemd-analyze            # compare total"
say "  3) systemd-analyze blame | head -n 15"
say "Revert any time:  sudo $0 --rollback"
rule
