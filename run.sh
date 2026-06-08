#!/usr/bin/env bash
# Launcher for our from-scratch RGB-D VIO/SLAM pipeline.
#
# The implementation is the 5-project split + launcher:
#   imu_camera -> depth -> vio -> slam -> ui   (driven by launcher.main)
# (For the DepthAI/Basalt reference viewer, run baseline/tools/view_pose3d.py.)
#
# Usage:
#   ./run.sh                 -- live 4-process pipeline (default)
#   ./run.sh --proc          -- same as above (explicit)
#   ./run.sh --session X ...  -- replay a recorded session X instead of live
#   ./run.sh ... <extra args> -- any extra args are forwarded to launcher.main
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "[run.sh] .venv missing — bootstrap with:" >&2
  echo "  python3.13 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
  exit 1
fi

# --proc is accepted as an explicit alias for the (now sole) pipeline; strip it
# so the rest of the args still reach launcher.main unchanged.
if [ "${1:-}" = "--proc" ]; then
  shift
fi

# launcher.main drives the four split projects (imu_camera / depth+vio / slam / ui).
# Live by default; pass --session X for replay. --auto-suffix keeps run dirs unique.
exec .venv/bin/python -m launcher.main --auto-suffix "$@"
