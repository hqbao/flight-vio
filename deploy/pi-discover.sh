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

# 1. Credentials (env override -> prompt -> prior cache default).
def_user="${PI_USER:-bao}"; def_host="${PI_HOST:-$def_user}"
if [ -z "${PI_USER:-}" ] || [ -t 0 ]; then
  read -rp "Pi SSH username [$def_user]: " in_user || true; PI_USER="${in_user:-$def_user}"
  read -rp "Pi mDNS hostname (for <name>.local) [$def_host]: " in_host || true; PI_HOST="${in_host:-$def_host}"
fi
PI_USER="${PI_USER:-$def_user}"; PI_HOST="${PI_HOST:-$def_host}"
if [ -z "${PI_PASS:-}" ]; then
  read -rsp "Pi SSH password (stored 0600 for sudo; entered once): " PI_PASS; echo
fi
[ -n "$PI_PASS" ] || pi_die "a password is required to install the SSH key the first time."

# 2. Discover the IP.
pi_say "locating $PI_USER@$PI_HOST on the LAN ..."
PI_IP="$(pi_resolve_ip)"
[ -n "$PI_IP" ] || pi_die "could not find the Pi. Check it is powered + on this WiFi/LAN, \
or set its IP:  PI_IP=<ip> ./deploy/pi-discover.sh"
pi_say "found Pi at $PI_IP"

# 3. Install the SSH key (passwordless from now on) + verify.
if pi_key_works; then
  pi_say "key auth already works -- no key install needed."
else
  pi_say "installing SSH key (one-time, uses the password) ..."
  pi_install_key || pi_die "SSH key install failed -- wrong password, or SSH refused."
  pi_key_works || pi_die "key installed but key-auth still fails -- check the Pi's sshd config."
  pi_say "key auth verified."
fi

# 4. Cache + confirm.
pi_save_cache
pi_rule
pi_say "Pi ready: $PI_USER@$PI_IP"
pi_ssh 'echo "  remote: $(tr -d "\0" </proc/device-tree/model 2>/dev/null) | $(. /etc/os-release; echo "$PRETTY_NAME") | $(uname -m)"' || true
pi_say "next:  ./deploy/pi-deploy.sh   (rsync repo + bootstrap venv)"
pi_say "       ./deploy/pi-optimize.sh (reduce boot time)"
pi_rule
