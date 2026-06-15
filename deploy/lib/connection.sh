#!/usr/bin/env bash
# connection.sh -- shared Raspberry Pi connection layer for the flight-vio deploy
# scripts. SOURCED (not executed) by every deploy/pi-*.sh host-side script, so the
# operator enters the Pi user/host/password ONCE (deploy/pi-discover.sh) and every
# other script reuses the cached, KEY-authenticated connection with no prompts.
#
# What it provides:
#   pi_load_cache / pi_save_cache / pi_reset_cache   -- the .cache/pi_connection.env
#   pi_resolve_ip                                    -- mDNS / ARP / port-22 scan
#   pi_install_key / pi_key_works                    -- passwordless SSH bootstrap
#   pi_ssh  CMD...    -- run a command on the Pi (key auth)
#   pi_sudo CMD...    -- run a command as root on the Pi (cached password -> sudo -S)
#   pi_sync SRC DST   -- rsync over the keyed SSH
#   pi_require        -- assert a working cached connection or tell the operator
#                        to run pi-discover.sh first
#
# The cache file holds the Pi password (for sudo) so the operator never re-types
# it; it is written 0600 and lives under the gitignored .cache/ dir. SSH itself is
# key-based after the first connect -- the password is used only to install the key
# and to feed `sudo -S` on the Pi.

# Repo root from this lib's own path: deploy/lib/connection.sh -> repo root.
_PI_LIB_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$_PI_LIB_DIR/../.." && pwd)
PI_CACHE="$REPO_ROOT/.cache/pi_connection.env"
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_ed25519}"
# BatchMode=yes => never prompt (key-only); discovery/key-install use sshpass.
SSH_O="-o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 -o BatchMode=yes \
-o ControlMaster=auto -o ControlPath=/tmp/cm-fvio-%r@%h:%p -o ControlPersist=600"

pi_say()  { printf '[pi] %s\n' "$*"; }
pi_warn() { printf '[pi] WARNING: %s\n' "$*" >&2; }
pi_die()  { printf '[pi] ERROR: %s\n' "$*" >&2; exit 1; }
pi_rule() { printf -- '----------------------------------------------------------------------\n'; }

# --------------------------------------------------------------------------- #
# Cache                                                                         #
# --------------------------------------------------------------------------- #
pi_load_cache() { [ -f "$PI_CACHE" ] && . "$PI_CACHE"; return 0; }

pi_save_cache() {
  mkdir -p "$(dirname "$PI_CACHE")"
  ( umask 077
    {
      echo "# flight-vio Pi connection cache (0600, gitignored). Edit via pi-discover.sh."
      echo "PI_USER=$(_pi_q "$PI_USER")"
      echo "PI_HOST=$(_pi_q "$PI_HOST")"
      echo "PI_IP=$(_pi_q "$PI_IP")"
      echo "PI_PASS=$(_pi_q "$PI_PASS")"
    } > "$PI_CACHE" )
  chmod 600 "$PI_CACHE"
  pi_say "saved connection -> $PI_CACHE (0600)"
}

pi_reset_cache() {
  if [ -f "$PI_CACHE" ]; then rm -f "$PI_CACHE"; pi_say "cleared $PI_CACHE"
  else pi_say "no cache to clear ($PI_CACHE)"; fi
}

# single-quote a value safely for the env file
_pi_q() { printf "'%s'" "$(printf '%s' "$1" | sed "s/'/'\\\\''/g")"; }

# --------------------------------------------------------------------------- #
# IP discovery (mac-side): mDNS hostname -> ARP Pi-vendor -> port-22 scan       #
# --------------------------------------------------------------------------- #
pi_resolve_ip() {
  # Echoes the discovered IP (empty on failure). Honours an already-set PI_IP if
  # it still answers on SSH. PI_HOST is the mDNS label (e.g. "bao" -> bao.local).
  if [ -n "${PI_IP:-}" ] && nc -z -G 3 "$PI_IP" 22 >/dev/null 2>&1; then
    printf '%s' "$PI_IP"; return 0
  fi
  local ip host
  for host in "${PI_HOST:-raspberrypi}.local" raspberrypi.local pi.local; do
    ip=$(ping -c1 -t2 "$host" 2>/dev/null | sed -n 's/.*(\([0-9.]*\)).*/\1/p' | head -1)
    [ -n "$ip" ] && { printf '%s' "$ip"; return 0; }
  done
  # Subnet ping-sweep to populate ARP, then match a Raspberry Pi MAC vendor (OUI).
  local iface myip subnet
  iface=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
  myip=$(ipconfig getifaddr "$iface" 2>/dev/null || true)
  [ -z "$myip" ] && return 0
  subnet=$(printf '%s' "$myip" | cut -d. -f1-3)
  local i; for i in $(seq 1 254); do ping -c1 -t1 "$subnet.$i" >/dev/null 2>&1 & done; wait
  ip=$(arp -an | grep -iE '2c:cf:67|d8:3a:dd|dc:a6:32|e4:5f:01|b8:27:eb|28:cd:c1' \
       | sed -n 's/.*(\([0-9.]*\)).*/\1/p' | head -1)
  [ -n "$ip" ] && { printf '%s' "$ip"; return 0; }
  # Fallback: single host (other than us) with SSH open.
  local tmp cand; tmp=$(mktemp -d "${TMPDIR:-/tmp}/fvio-scan.XXXXXX")
  for i in $(seq 1 254); do
    [ "$subnet.$i" = "$myip" ] && continue
    ( nc -z -G1 -w1 "$subnet.$i" 22 >/dev/null 2>&1 && printf '%s\n' "$subnet.$i" >"$tmp/$i" ) &
  done; wait
  cand=$(cat "$tmp"/* 2>/dev/null); rm -rf "$tmp"
  set -- $cand; [ "$#" -eq 1 ] && printf '%s' "$1"
  return 0
}

# --------------------------------------------------------------------------- #
# SSH key bootstrap + run helpers                                              #
# --------------------------------------------------------------------------- #
pi_key_works() {
  ssh $SSH_O -i "$SSH_KEY" "$PI_USER@$PI_IP" true 2>/dev/null
}

pi_install_key() {
  command -v sshpass >/dev/null 2>&1 || pi_die \
    "sshpass not found -- install it: brew install sshpass"
  [ -f "$SSH_KEY" ] || ssh-keygen -t ed25519 -N '' -f "$SSH_KEY" -q
  sshpass -p "$PI_PASS" ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8 \
    "$PI_USER@$PI_IP" \
    "umask 077; mkdir -p ~/.ssh; cat >> ~/.ssh/authorized_keys; \
     sort -u ~/.ssh/authorized_keys -o ~/.ssh/authorized_keys" < "$SSH_KEY.pub"
}

pi_ssh()  { ssh $SSH_O -i "$SSH_KEY" "$PI_USER@$PI_IP" "$@"; }

# Run "$*" as root on the Pi: the cached password is piped to `sudo -S` (sudo reads
# it from the first stdin line, then runs the command). Call sites pass plain
# command words (no nested bash -c), e.g.  pi_sudo bash /tmp/optimize_pi.sh --dry-run
pi_sudo() {
  printf '%s\n' "$PI_PASS" | ssh $SSH_O -i "$SSH_KEY" "$PI_USER@$PI_IP" "sudo -S -p '' $*"
}

pi_sync() { rsync -az -e "ssh $SSH_O -i $SSH_KEY" "$@"; }

# Assert a usable cached connection, or stop with a clear pointer to discover.
pi_require() {
  pi_load_cache
  [ -n "${PI_IP:-}" ] && [ -n "${PI_USER:-}" ] || pi_die \
    "no Pi connection cached -- run:  ./deploy/pi-discover.sh"
  if ! pi_key_works; then
    # IP may have changed (DHCP). Try to re-resolve once before giving up.
    local newip; newip=$(pi_resolve_ip)
    if [ -n "$newip" ] && [ "$newip" != "$PI_IP" ]; then
      PI_IP="$newip"; pi_key_works && { pi_save_cache; return 0; }
    fi
    pi_die "Pi not reachable with key at $PI_USER@$PI_IP -- re-run: ./deploy/pi-discover.sh"
  fi
  return 0
}
