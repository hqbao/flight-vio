#!/usr/bin/env bash
# Launch the DepthAI/Basalt REFERENCE pipeline (compare against ./run.sh).
#   ./run-baseline.sh                         # Basalt VIO on the OAK-D
#   ./run-baseline.sh --source slam           # Basalt VIO + RTABMap SLAM
#   ./run-baseline.sh --width 80 --height 50  # low-res reference run
#   ./run-baseline.sh --source fake           # UI bring-up, no device
set -euo pipefail
cd "$(dirname "$0")"
[ -x .venv/bin/python ] || { echo "run-baseline.sh: .venv/bin/python not found" >&2; exit 1; }
# default to the real Basalt VIO unless the caller picked a --source
case " $* " in *" --source "*) ;; *) set -- --source oak "$@";; esac
exec .venv/bin/python baseline/tools/view_pose3d.py "$@"
