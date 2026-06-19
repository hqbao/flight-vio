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
      config:*)
        rest=${line#config:}; val=${rest%%:*}; file=${rest#*:}
        if [ -f "$file" ] && grep -q "^${val}$" "$file"; then
          say "remove '$val' from $file"; sed -i "\|^${val}$|d" "$file"
        fi ;;
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

# cloud-init: DO NOT disable. The Imager-provisioned headless WiFi/SSH is a
# cloud-init nocloud datasource (boot-partition network-config + user-data) that
# renders the network at boot. Disabling it (this script's old behaviour) dropped
# that per-boot WiFi rendering, so the FIRST reboot afterwards came up with NO
# network and was unreachable -- recoverable only via HDMI + keyboard (a brownout
# reboot hit exactly this). The few seconds saved at boot are not worth bricking
# remote access. (--rollback above still re-enables cloud-init on any Pi a previous
# version of this script disabled.) If you ever DO want it off, first persist the
# WiFi as a standalone NetworkManager profile so boot connectivity no longer
# depends on cloud-init.
say "keep  cloud-init (owns headless WiFi/SSH; disabling it breaks boot networking)"

# OAK camera USB power: the Pi5 default-caps total USB current at 600mA, which
# browns out an OAK-D under load -- ESPECIALLY the OAK-D Lite (narrower-FOV,
# USB-powered) running stereo+IMU -> the device firmware crashes in a loop
# ("Device ... has crashed" / "Couldn't read data from stream (X_LINK_ERROR)").
# usb_max_current_enable=1 lifts the cap to ~1.6A; it needs a capable 5V/5A PSU
# (check `vcgencmd get_throttled` stays 0x0). ALSO plug the OAK into a USB3 (blue)
# port with a USB3 cable -- a USB2 port/cable (480Mbps) starves bandwidth+power.
# Idempotent; --rollback removes the line. Reboot to apply.
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if [ ! -f "$CONFIG_TXT" ]; then
  say "skip  usb_max_current_enable (no config.txt found)"
elif grep -q '^usb_max_current_enable=1' "$CONFIG_TXT"; then
  say "skip  usb_max_current_enable (already set in $CONFIG_TXT)"
elif [ "$DRY" -eq 1 ]; then
  say "WOULD set usb_max_current_enable=1 in $CONFIG_TXT (OAK USB power; reboot needed)"
else
  printf 'usb_max_current_enable=1\n' >> "$CONFIG_TXT"
  say "set usb_max_current_enable=1 in $CONFIG_TXT (OAK USB power; reboot needed)"
  printf '%s\n' "config:usb_max_current_enable=1:$CONFIG_TXT" >> "${STATE}.tmp"
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
