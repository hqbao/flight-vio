#!/usr/bin/env bash
#
# setup_pi.sh -- idempotent RPi5 (Debian / aarch64) bootstrap for the flight-vio
# FLIGHT runtime.
#
# Goal: plug-and-run. On a fresh Raspberry Pi 5 (Debian "bookworm"/"trixie",
# aarch64) this script
#
#   1. CHECKS the apt prerequisites (python3.13 + venv + dev headers +
#      build-essential) and PRINTS the exact `sudo apt install ...` line if any
#      are missing -- it never runs sudo itself (the operator stays in control);
#   2. creates the project `.venv` with python3.13 and installs the lean flight
#      deps from requirements-flight.txt (numpy + numba + pyserial + depthai;
#      NO OpenCV, NO Qt);
#   3. runs the validation smoke -- codec round-trip + a headless `--no-ui`
#      replay + the cv2-absent litmus -- and prints a PASS/FAIL summary plus the
#      board-arrival next steps.
#
# It is IDEMPOTENT: re-running reuses an existing .venv and only re-installs /
# re-checks. Pure POSIX-friendly bash (no macOS-isms: no `realpath -m`, no BSD
# `readlink -f`, no `sed -i ''`); validates clean under `bash -n` and shellcheck.
#
# Usage (on the Pi, from the repo root or anywhere):
#   ./scripts/setup_pi.sh              # full bootstrap + smoke
#   ./scripts/setup_pi.sh --no-smoke   # bootstrap only (skip the validation run)
#   ./scripts/setup_pi.sh --help
#
set -eu

# --------------------------------------------------------------------------- #
# Locate the repo root from this script's own path (portable: no realpath/-f).  #
# --------------------------------------------------------------------------- #
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
cd "$REPO_ROOT"

PY_VERSION="python3.13"
VENV_DIR="$REPO_ROOT/.venv"
VENV_PY="$VENV_DIR/bin/python"
REQ_FLIGHT="$REPO_ROOT/requirements-flight.txt"
SMOKE_SESSION="sessions/gold/lab_loop_30s"

RUN_SMOKE=1
for arg in "$@"; do
  case "$arg" in
    --no-smoke) RUN_SMOKE=0 ;;
    -h|--help)
      # Print the leading header comment block only (up to the first blank /
      # non-comment line after the shebang), stripping the "# " prefix.
      awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"
      exit 0
      ;;
    *)
      echo "[setup_pi] unknown arg: $arg (try --help)" >&2
      exit 2
      ;;
  esac
done

# --------------------------------------------------------------------------- #
# Tiny logging helpers (no colour codes -- safe over ssh / in logs).           #
# --------------------------------------------------------------------------- #
say()  { printf '[setup_pi] %s\n' "$*"; }
warn() { printf '[setup_pi] WARNING: %s\n' "$*" >&2; }
die()  { printf '[setup_pi] ERROR: %s\n' "$*" >&2; exit 1; }
rule() { printf -- '----------------------------------------------------------------------\n'; }

# --------------------------------------------------------------------------- #
# 0. Sanity: we are in the right place.                                         #
# --------------------------------------------------------------------------- #
[ -f "$REQ_FLIGHT" ] || die "requirements-flight.txt not found at $REQ_FLIGHT (run from the flight-vio repo)."
[ -f "$REPO_ROOT/run.sh" ] || die "run.sh not found -- is $REPO_ROOT the flight-vio repo root?"

rule
say "flight-vio RPi5 flight bootstrap"
say "repo:   $REPO_ROOT"
say "python: $PY_VERSION   venv: $VENV_DIR"
rule

# --------------------------------------------------------------------------- #
# 1. apt prerequisites -- CHECK ONLY, print the install line if anything is     #
#    missing. We do NOT run sudo (the operator decides).                        #
# --------------------------------------------------------------------------- #
# Map each apt package to a presence check. python3.13-venv / python3.13-dev
# have no standalone binary, so we probe them via python3.13's own modules /
# headers once the interpreter exists; build-essential is represented by `gcc`.
APT_PKGS="python3.13 python3.13-venv python3.13-dev build-essential"
MISSING=""

if ! command -v "$PY_VERSION" >/dev/null 2>&1; then
  MISSING="$MISSING python3.13 python3.13-venv python3.13-dev"
else
  # python3.13 exists -- verify the venv module (python3.13-venv) is present.
  if ! "$PY_VERSION" -c 'import venv' >/dev/null 2>&1; then
    MISSING="$MISSING python3.13-venv"
  fi
  # verify the dev headers (python3.13-dev) -- needed if any wheel builds from
  # source (e.g. numba/llvmlite if no aarch64 wheel is published).
  if ! "$PY_VERSION" -c 'import sysconfig,os,sys; h=sysconfig.get_path("include"); sys.exit(0 if os.path.exists(os.path.join(h,"Python.h")) else 1)' >/dev/null 2>&1; then
    MISSING="$MISSING python3.13-dev"
  fi
fi

# build-essential -> gcc (proxy for the full compiler toolchain). Needed only
# for source builds; harmless to require on a deploy board.
if ! command -v gcc >/dev/null 2>&1; then
  MISSING="$MISSING build-essential"
fi

if [ -n "$MISSING" ]; then
  # De-duplicate the missing list while preserving order.
  DEDUP=""
  for p in $MISSING; do
    case " $DEDUP " in *" $p "*) : ;; *) DEDUP="$DEDUP $p" ;; esac
  done
  warn "missing apt prerequisites:$DEDUP"
  echo ""
  echo "    Run this (needs sudo), then re-run scripts/setup_pi.sh:"
  echo ""
  echo "        sudo apt update && sudo apt install -y$DEDUP"
  echo ""
  die "prerequisites missing -- install them and re-run."
fi
say "apt prerequisites present ($APT_PKGS)"

# --------------------------------------------------------------------------- #
# 2. Create / reuse the .venv and install the lean flight deps.                 #
# --------------------------------------------------------------------------- #
if [ -x "$VENV_PY" ]; then
  say ".venv already exists -- reusing ($($VENV_PY --version 2>&1))"
else
  say "creating .venv with $PY_VERSION ..."
  "$PY_VERSION" -m venv "$VENV_DIR" || die "venv creation failed (is python3.13-venv installed?)"
fi

say "upgrading pip ..."
"$VENV_PY" -m pip install -U pip >/dev/null || die "pip self-upgrade failed."

say "installing flight deps from requirements-flight.txt ..."
say "  (aarch64 note: numba/llvmlite + depthai wheels are the open question --"
say "   see docs/RPI5_DEPLOY.md troubleshooting if this step fails)"
if ! "$VENV_PY" -m pip install -r "$REQ_FLIGHT"; then
  warn "flight-deps install FAILED."
  warn "Most likely an aarch64 wheel issue (numba/llvmlite or depthai)."
  warn "See docs/RPI5_DEPLOY.md -> Troubleshooting for the per-package fixes:"
  warn "  - numba absent: the runtime has a pure-NumPy fallback (slower);"
  warn "    you can drop numba from the install and continue (perf, not function)."
  warn "  - depthai absent: only --live OAK-D capture needs it; replay + non-OAK"
  warn "    sources run without it (the VL53-ToF pivot makes depthai optional)."
  die "dependency install failed -- resolve per the runbook and re-run."
fi
say "flight deps installed:"
"$VENV_PY" -m pip list 2>/dev/null | grep -iE '^(numpy|numba|llvmlite|pyserial|depthai) ' \
  | sed 's/^/    /' || true

# --------------------------------------------------------------------------- #
# 3. Validation smoke -- codec round-trip + headless replay + cv2-absent litmus.#
#    Each prints PASS/FAIL; the script exits non-zero if any FAILS.             #
# --------------------------------------------------------------------------- #
if [ "$RUN_SMOKE" -eq 0 ]; then
  rule
  say "bootstrap complete (--no-smoke: validation skipped)."
  say "Run the smoke yourself with: ./scripts/setup_pi.sh   (no flag)"
  exit 0
fi

rule
say "VALIDATION SMOKE (3 checks)"
rule

SMOKE_FAILED=0

# 3a. Codec round-trip + cross-copy byte-parity (no hardware needed).
say "[1/3] codec round-trip + byte-parity ..."
if "$VENV_PY" -m imu_camera.tests.codec_roundtrip_selftest; then
  say "[1/3] codec round-trip: PASS"
else
  warn "[1/3] codec round-trip: FAIL"
  SMOKE_FAILED=1
fi
rule

# 3b. Headless --no-ui replay of a gold session -- proves the flight stack
#     (imu_camera -> vio -> slam) runs end-to-end with NO Qt/UI.
say "[2/3] headless --no-ui replay ($SMOKE_SESSION, 30 frames) ..."
if [ -d "$REPO_ROOT/$SMOKE_SESSION" ]; then
  if "$REPO_ROOT/run.sh" --no-ui --session "$SMOKE_SESSION" --max-frames 30; then
    say "[2/3] headless replay: PASS"
  else
    warn "[2/3] headless replay: FAIL"
    SMOKE_FAILED=1
  fi
else
  warn "[2/3] headless replay: SKIPPED (gold session $SMOKE_SESSION not present)"
fi
rule

# 3c. cv2-absent flight litmus -- the headline gate: the full --vl53l9cx
#     --direct flight runs with OpenCV unimportable (proves the lean install).
say "[3/3] cv2-absent flight litmus (--vl53l9cx --direct, cv2 BLOCKED) ..."
if "$VENV_PY" -m verification.cv2_absent_flight_litmus --max-frames 30; then
  say "[3/3] cv2-absent litmus: PASS"
else
  warn "[3/3] cv2-absent litmus: FAIL"
  SMOKE_FAILED=1
fi
rule

# --------------------------------------------------------------------------- #
# Summary + next steps.                                                         #
# --------------------------------------------------------------------------- #
if [ "$SMOKE_FAILED" -ne 0 ]; then
  warn "VALIDATION SMOKE: one or more checks FAILED (see above)."
  warn "See docs/RPI5_DEPLOY.md -> Troubleshooting."
  exit 1
fi

say "VALIDATION SMOKE: ALL PASS."
echo ""
say "Next steps on the board (see docs/RPI5_DEPLOY.md for the full checklist):"
echo "    1. MEASURE real-time perf -- does the live ToF pipeline hold ~20 Hz?"
echo "         ./run.sh --no-ui --vl53l9cx --direct      # with OAK-D/ToF attached"
echo "       (watch ms/frame in the logs; this is the open unknown the dev box"
echo "        cannot answer -- it must be measured ON the Pi.)"
echo "    2. Confirm live OAK-D capture works:  ./run.sh --no-ui --vl53l9cx"
echo "    3. UI is optional/remote -- keep --no-ui on the Pi (headless flight)."
echo ""
say "Bootstrap + validation complete."
