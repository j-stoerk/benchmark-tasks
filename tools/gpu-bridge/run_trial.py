"""Forward validated task-specific options as an argument array, never shell code."""
import json
from pathlib import Path
import subprocess
import sys

folder, mode, output, bridge, steps, options = sys.argv[1:]
options = json.loads(options)
command = [sys.executable, str(Path(folder) / "run.py"), mode, "--output", output, "--max-steps", steps]
if mode == "model":
    command += ["--bridge", bridge]
slug = Path(folder).name
if slug == "industrial-roll-handoff":
    command += ["--lift-force", str(options.get("lift_force", 6000))]
    if options.get("no_expand"):
        command += ["--no-expand"]
elif slug == "calender-loaded-nip-parallelism":
    command += ["--proof-scale", str(options.get("proof_scale", 1))]
    if options.get("unloaded_only"):
        command += ["--unloaded-only"]
    if options.get("calibration_bench"):
        command += ["--calibration-bench"]
else:
    command += ["--variant", options.get("variant", "normal")]
raise SystemExit(subprocess.call(command))
