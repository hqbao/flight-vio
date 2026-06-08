#!/usr/bin/env bash
# Launcher for the 3D pose viewer of *our* from-scratch VIO.
# (For the DepthAI/Basalt baseline viewer, run baseline/tools/view_pose3d.py.)
#
# Modes:
#   ./run.sh ...                    -- single-process viewer (default; offline-safe)
#   ./run.sh --proc ...             -- 4-process pipeline (imu_camera + vio + slam + ui)
#                                      see docs/PROC4_ARCHITECTURE.md
#   ./run.sh --proc-old ...         -- the PRE-split reference pipeline (ours.proc),
#                                      kept reachable as the verification oracle
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "[run.sh] .venv missing — bootstrap with:" >&2
  echo "  python3.13 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
  exit 1
fi

# --proc selects the NEW 4-process launcher (launcher.main drives the four split
# projects imu_camera / vio / slam / ui); strip the flag and forward the rest.
if [ "${1:-}" = "--proc" ]; then
  shift
  exec .venv/bin/python -m launcher.main --auto-suffix "$@"
fi

# --proc-old keeps the PRE-split ours.proc.launcher reachable so the reference
# oracle stays runnable for the Phase 7 verification harness.
if [ "${1:-}" = "--proc-old" ]; then
  shift
  exec .venv/bin/python -m ours.proc.launcher --auto-suffix "$@"
fi

exec .venv/bin/python ours/tools/view_pose3d.py "$@"
