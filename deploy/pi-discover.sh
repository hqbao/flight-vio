#!/usr/bin/env bash
# pi-discover.sh -- TASK 1: find the Raspberry Pi on the LAN, authenticate ONCE,
# and cache the connection so every later deploy script connects automatically.
#
# First run: asks for the Pi SSH user / mDNS hostname / password, discovers the IP
# (mDNS -> Raspberry-Pi MAC vendor -> port-22 scan), installs your SSH public key
# on the Pi (passwordless from then on), and writes .cache/pi_connection.env (0600).
# Later runs (and every other deploy/pi-*.sh) just reuse that cache silently.
#
#   ./deploy/pi-discover.sh            # discover + cache (prompts on first run)
#   ./deploy/pi-discover.sh --reset    # forget the cached Pi, then re-discover
#   ./deploy/pi-discover.sh --status   # show the cached connection, change nothing
#
# Non-interactive (CI / scripting): preset PI_USER / PI_HOST / PI_PASS in the env
# and they are used instead of prompting.
set -u
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh

case "${1:-}" in
  -h|--help)
    awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
  --reset)  pi_reset_cache ;;
  --status)
    pi_load_cache
    [ -n "${PI_IP:-}" ] || { pi_say "no Pi cached -- run ./deploy/pi-discover.sh"; exit 0; }
    pi_say "cached: $PI_USER@$PI_IP (host '${PI_HOST:-?}')"
    if pi_key_works; then pi_say "key auth: OK"; else pi_warn "key auth: FAILED (re-run discover)"; fi
    exit 0 ;;
esac

pi_load_cache    # seed defaults from any prior cache

# Already connected and not forced to redo? report + exit.
if [ "${1:-}" != "--reset" ] && [ -n "${PI_IP:-}" ] && [ -n "${PI_USER:-}" ] && pi_key_works; then
  pi_say "already connected: $PI_USER@$PI_IP (key auth OK). Use --reset to re-discover."
  exit 0
fi

pi_rule
pi_say "flight-vio Raspberry Pi discovery + one-time authentication"
pi_rule

# 1. Credentials -- username + password ONLY (no hostname; discovery is scan+pick).
def_user="${PI_USER:-bao}"
if [ -z "${PI_USER:-}" ] || [ -t 0 ]; then
  read -rp "Pi SSH username [$def_user]: " in_user || true; PI_USER="${in_user:-$def_user}"
fi
PI_USER="${PI_USER:-$def_user}"; PI_HOST="${PI_HOST:-$PI_USER}"
if [ -z "${PI_PASS:-}" ]; then
  read -rsp "Pi SSH password (stored 0600 for sudo; entered once): " PI_PASS; echo
fi
[ -n "$PI_PASS" ] || pi_die "a password is required to install the SSH key the first time."

# 2+3. Reuse a cached IP that still answers with the key; otherwise SCAN the LAN and
# let the OPERATOR PICK the Pi from the list (no MAC/vendor guessing -- works for any
# model, and you pick the right box when several devices answer). The chosen host
# gets the key installed + verified; on failure you pick another.
if [ -n "${PI_IP:-}" ] && pi_key_works; then
  pi_say "reusing cached $PI_USER@$PI_IP (key auth OK)."
elif [ ! -t 0 ]; then
  pi_die "no working cached Pi and not a terminal -- run interactively to pick, or: PI_IP=<ip> $0"
else
  while :; do
    pi_say "scanning the LAN for SSH (port 22) hosts ..."
    hosts="$(pi_scan_hosts)"
    if [ -z "$hosts" ]; then
      pi_warn "no SSH host found -- is the Pi powered + on this WiFi?"
      read -rp "  Enter = rescan, Ctrl-C = quit ... " _ || pi_die "aborted."; continue
    fi
    pi_rule; pi_say "devices answering SSH (pick yours):"
    ips=(); n=0
    while IFS="$(printf '\t')" read -r ip label; do
      [ -n "$ip" ] || continue
      n=$((n + 1)); ips+=("$ip")
      printf '   [%d] %-15s %s\n' "$n" "$ip" "$label"
    done <<<"$hosts"
    pi_rule
    read -rp "Pick the Pi [1-$n] (r = rescan): " sel || true
    [ "$sel" = r ] && continue
    case "$sel" in '' | *[!0-9]*) pi_warn "enter a number."; continue ;; esac
    { [ "$sel" -ge 1 ] && [ "$sel" -le "$n" ]; } || { pi_warn "out of range."; continue; }
    PI_IP="${ips[$((sel - 1))]}"
    pi_say "connecting to $PI_USER@$PI_IP ..."
    pi_key_works && { pi_say "key auth already works."; break; }
    pi_install_key && pi_key_works && { pi_say "key installed + verified."; break; }
    pi_warn "could not log in to $PI_IP (wrong device or password?) -- pick another."
  done
fi

# 4. Cache + confirm.
pi_save_cache
pi_rule
pi_say "Pi ready: $PI_USER@$PI_IP"
pi_ssh 'echo "  remote: $(tr -d "\0" </proc/device-tree/model 2>/dev/null) | $(. /etc/os-release; echo "$PRETTY_NAME") | $(uname -m)"' || true
pi_say "next:  ./deploy/pi-deploy.sh   (rsync repo + bootstrap venv)"
pi_say "       ./deploy/pi-optimize.sh (reduce boot time)"
pi_rule
