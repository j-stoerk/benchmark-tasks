#!/usr/bin/env bash
set -euo pipefail
nvidia-smi
ls -ld /data/IsaacLab-Arena /data/IsaacLab-Arena/yam.env
source /data/IsaacLab-Arena/yam.env
python - <<'PY'
import importlib.metadata as m
from pathlib import Path
for name in ['inspect-robots','inspect-robots-yam','isaaclab-arena','isaacsim','numpy']:
    try: print('VERSION',name,m.version(name))
    except m.PackageNotFoundError: print('MISSING',name)
root=Path('/data/IsaacLab-Arena/submodules/IsaacLab/source/isaaclab/isaaclab')
for name in ['assets/rigid_object/rigid_object.py','assets/articulation/articulation_data.py','sensors/contact_sensor/contact_sensor_data.py']:
    p=root/name
    if not p.exists(): continue
    s=p.read_text()
    for term in ['def set_external_force_and_torque','def applied_torque','def net_forces_w','def computed_torque']:
        at=s.find(term)
        if at>=0: print('API',name,s[max(0,at-20):at+1500])
PY
