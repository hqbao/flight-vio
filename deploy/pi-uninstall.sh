#!/usr/bin/env bash
# pi-uninstall.sh -- TASK 3 (uninstall): remove flight-vio from the Pi, from the Mac.
#
# Stops the stack, then removes ~/flight-vio (code + .venv). By default it LEAVES the
# boot optimisations and the SSH key in place; opt into reverting those:
#
#   ./deploy/pi-uninstall.sh                 # stop + remove ~/flight-vio
#   ./deploy/pi-uninstall.sh --restore-boot  # ALSO roll back pi-optimize (re-enable services)
#   ./deploy/pi-uninstall.sh --forget        # ALSO remove this Mac's SSH key from the Pi
#                                            #   and clear the local connection cache
#   ./deploy/pi-uninstall.sh --all           # everything above
#
# Asks for confirmation unless --yes is given.
set -eu
cd "$(dirname "$0")/.."
. deploy/lib/connection.sh

RESTORE_BOOT=0; FORGET=0; ASSUME_YES=0
for a in "$@"; do
  case "$a" in
    -h|--help) awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    --restore-boot) RESTORE_BOOT=1 ;;
    --forget) FORGET=1 ;;
    --all) RESTORE_BOOT=1; FORGET=1 ;;
    --yes|-y) ASSUME_YES=1 ;;
    *) pi_die "unknown arg: $a (try --help)" ;;
  esac
done

pi_require
DEST="flight-vio"

pi_warn "about to UNINSTALL flight-vio from $PI_USER@$PI_IP:"
echo   "    - stop the flight stack"
echo   "    - remove ~/$DEST (code + .venv)"
[ "$RESTORE_BOOT" -eq 1 ] && echo "    - roll back boot optimisations (re-enable services)"
[ "$FORGET" -eq 1 ]       && echo "    - remove this Mac's SSH key from the Pi + clear local cache"
if [ "$ASSUME_YES" -ne 1 ]; then
  read -rp "Proceed? [y/N] " ans; case "$ans" in y|Y|yes) : ;; *) pi_say "aborted."; exit 0 ;; esac
fi

# 1. stop (best-effort; reuse the stop logic).
pi_say "stopping the stack ..."
bash deploy/pi-stop.sh || true

# 2. optionally roll back the boot optimisations (needs the optimiser on the Pi).
if [ "$RESTORE_BOOT" -eq 1 ]; then
  pi_say "rolling back boot optimisations ..."
  pi_sync deploy/pi/optimize_pi.sh "$PI_USER@$PI_IP:/tmp/fvio_optimize_pi.sh"
  pi_sudo "bash /tmp/fvio_optimize_pi.sh --rollback" || pi_warn "rollback reported nothing to undo."
fi

# 3. remove the repo.
pi_say "removing ~/$DEST ..."
pi_ssh "rm -rf ~/$DEST /tmp/fvio_optimize_pi.sh"

# 4. optionally forget the Mac<->Pi auth.
if [ "$FORGET" -eq 1 ]; then
  pi_say "removing this Mac's SSH key from the Pi ..."
  pub=$(cat "$SSH_KEY.pub" 2>/dev/null | awk '{print $2}')
  if [ -n "$pub" ]; then
    pi_ssh "test -f ~/.ssh/authorized_keys && grep -v '$pub' ~/.ssh/authorized_keys > ~/.ssh/authorized_keys.tmp 2>/dev/null && mv ~/.ssh/authorized_keys.tmp ~/.ssh/authorized_keys || true"
  fi
  pi_reset_cache
fi

pi_rule
pi_say "uninstall complete."
[ "$FORGET" -ne 1 ] && pi_say "(connection cache kept -- ./deploy/pi-deploy.sh will re-install.)"
pi_rule
