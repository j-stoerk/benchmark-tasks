#!/usr/bin/env bash
set -euo pipefail
source /data/IsaacLab-Arena/yam.env
export OMNI_KIT_ACCEPT_EULA=YES
uv pip install --no-deps -e "$1"
python "$(dirname "$0")/run_trial.py" "$@"
