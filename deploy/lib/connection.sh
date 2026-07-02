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
# IP discovery (mac-side): SCAN for SSH hosts, operator PICKS (no MAC guessing)  #
# --------------------------------------------------------------------------- #
pi_resolve_ip() {
  # Non-interactive fast path ONLY: honour an already-set PI_IP that still answers
  # SSH. Real discovery is scan + operator-pick (pi_scan_hosts -> pi-discover.sh):
  # no MAC/vendor guessing, so ANY Pi model works and you choose the right box when
  # several devices answer (multiple Pis, a NAS, ...).
  if [ -n "${PI_IP:-}" ] && nc -z -G 3 "$PI_IP" 22 >/dev/null 2>&1; then
    printf '%s' "$PI_IP"; return 0
  fi
  return 0
}

# Best-effort, NO-auth label for a host so the operator can tell what it is:
# reverse-DNS name + the SSH banner's OS hint (e.g. "OpenSSH_9.2p1 Debian-...").
# Echoes the label, or "?" when nothing resolves.
pi_host_label() {
  local ip="$1" name banner
  name=$(dscacheutil -q host -a ip_address "$ip" 2>/dev/null | sed -n 's/^name: *//p' | head -1)
  banner=$(nc -w 2 "$ip" 22 2>/dev/null | head -1 | tr -d '\r' | sed -n 's/^SSH-[0-9.]*-//p')
  local label="$name"
  [ -n "$banner" ] && label="${label:+$label  }[$banner]"
  printf '%s' "${label:-?}"
}

# Scan the LAN for hosts with SSH (port 22) open. Echoes one "IP<TAB>LABEL" line per
# host, sorted by last octet. The port-22 filter keeps the list short (Pis/servers,
# not every phone) -- it drives the interactive picker in pi-discover.sh.
pi_scan_hosts() {
  local iface myip subnet tmp ip
  iface=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
  myip=$(ipconfig getifaddr "$iface" 2>/dev/null || true)
  [ -z "$myip" ] && return 0
  subnet=$(printf '%s' "$myip" | cut -d. -f1-3)
  tmp=$(mktemp -d "${TMPDIR:-/tmp}/fvio-scan.XXXXXX")
  local i
  for i in $(seq 1 254); do
    [ "$subnet.$i" = "$myip" ] && continue
    ( nc -z -G1 -w1 "$subnet.$i" 22 >/dev/null 2>&1 && printf '%s\n' "$subnet.$i" >"$tmp/$i" ) &
  done; wait
  for ip in $(cat "$tmp"/* 2>/dev/null | sort -t. -k4 -n); do
    printf '%s\t%s\n' "$ip" "$(pi_host_label "$ip")"
  done
  rm -rf "$tmp"
}

# --------------------------------------------------------------------------- #
# SSH key bootstrap + run helpers                                              #
# --------------------------------------------------------------------------- #
pi_key_works() {
  ssh $SSH_O -i "$SSH_KEY" "$PI_USER@$PI_IP" true 2>/dev/null
}

# A re-imaged Pi (or a DHCP IP reused by another box) presents a NEW host key, so
# SSH refuses with "REMOTE HOST IDENTIFICATION HAS CHANGED" and wedges discovery.
# pi_install_key is the deliberate re-trust step (operator picked this device and
# typed its password), so drop the stale known_hosts entry here instead of making
# the operator run ssh-keygen -R by hand. Only THIS IP is touched, and only on the
# changed-key error -- a matching key (or a merely-unknown host) is left alone.
pi_untrust_stale_host() {
  local ip="$1"
  ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=yes \
      "$PI_USER@$ip" true 2>&1 | grep -q "IDENTIFICATION HAS CHANGED" || return 0
  pi_warn "host key for $ip changed (Pi re-imaged or IP reused) -- refreshing known_hosts"
  ssh-keygen -R "$ip" >/dev/null 2>&1
}

pi_install_key() {
  command -v sshpass >/dev/null 2>&1 || pi_die \
    "sshpass not found -- install it: brew install sshpass"
  pi_untrust_stale_host "$PI_IP"
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
