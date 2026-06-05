#!/usr/bin/env bash
# Launcher for the 3D pose viewer of *our* from-scratch VIO.
# (For the DepthAI/Basalt baseline viewer, run baseline/tools/view_pose3d.py.)
set -e
cd "$(dirname "$0")"

if [ ! -d .venv ]; then
  echo "[run.sh] .venv missing — bootstrap with:" >&2
  echo "  python3.13 -m venv .venv && .venv/bin/pip install -r requirements.txt" >&2
  exit 1
fi

exec .venv/bin/python ours/tools/view_pose3d.py "$@"
