#!/usr/bin/env bash
set -euo pipefail
source /data/IsaacLab-Arena/yam.env
export OMNI_KIT_ACCEPT_EULA=YES
uv pip install --no-deps -e "$1"
args=("$1/run.py" "$2" --output "$3" --max-steps "$5" --lift-force "$6")
if [[ "$2" == model ]]; then args+=(--bridge "$4"); fi
if [[ "${7:-0}" == 1 ]]; then args+=(--no-expand); fi
python "${args[@]}"
