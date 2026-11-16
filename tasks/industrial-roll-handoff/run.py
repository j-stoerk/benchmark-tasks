"""Run one recorded physical or genuine external-LLM trial; never overwrite evidence."""

import argparse
import datetime as dt
import hashlib
import importlib.metadata as metadata
import json
import shutil
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["smoke", "demo", "model"])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bridge", type=Path)
    parser.add_argument("--max-steps", type=int, default=2400)
    parser.add_argument("--lift-force", type=float, default=6000)
    parser.add_argument("--no-expand", action="store_true", help="Scripted negative control: keep the grip collapsed")
    args = parser.parse_args()
    folder = Path(__file__).resolve().parent
    spec = json.loads((folder / "task.json").read_text(encoding="utf-8"))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    policy = "industrial_llm" if args.mode == "model" else "industrial_demo"
    command = [
        shutil.which("inspect-robots") or "inspect-robots",
        "run",
        "--embodiment",
        "industrial_roll",
        "--policy",
        policy,
        "-E",
        f"trace_file={out / 'physics.jsonl'}",
        "-E",
        f"lift_force={args.lift_force}",
        "--instruction",
        spec["instruction"],
        "--scorer",
        "task_score",
        "--seed",
        "0",
        "--max-steps",
        str(args.max_steps),
        "--no-prompt",
        "--store-frames",
        "--log-dir",
        str(out),
    ]
    if policy == "industrial_llm":
        if args.no_expand:
            parser.error("--no-expand is only a scripted negative control")
        if args.bridge is None:
            parser.error("model needs --bridge directory for actual LLM decisions")
        command += ["-P", f"directory={args.bridge.resolve()}"]
    elif args.no_expand:
        command += ["-P", "expand=false"]
    manifest = {
        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "command": command,
        "policy": policy,
        "seed": 0,
        "max_steps": args.max_steps,
        "status": "running",
        "source_hashes": {
            str(p.relative_to(folder)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in folder.rglob("*")
            if p.is_file() and p.suffix in [".py", ".usda", ".json", ".toml"] and "__pycache__" not in p.parts
        },
        "versions": {
            name: metadata.version(name)
            for name in ["inspect-robots", "inspect-robots-yam", "isaacsim", "isaaclab-arena", "numpy"]
        },
    }
    receipt = out / "run_manifest.json"
    receipt.write_text(json.dumps(manifest, indent=2) + "\n")
    with (out / "console.txt").open("w", encoding="utf-8") as console:
        code = subprocess.call(command, stdout=console, stderr=subprocess.STDOUT)
    logs = list(out.glob("adhoc_*.json"))
    valid_trial = False
    if len(logs) == 1:
        result = json.loads(logs[0].read_text())["results"]
        valid_trial = result["errored_trials"] == 0 and result["total_trials"] == 1
    manifest.update(
        status="finished" if code == 0 and valid_trial else "failed",
        exit_code=code,
        valid_trial=valid_trial,
        finished_utc=dt.datetime.now(dt.timezone.utc).isoformat(),
    )
    receipt.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({k: manifest[k] for k in ["policy", "status", "exit_code", "started_utc", "finished_utc"]}))
    if code == 0 and valid_trial:
        subprocess.run([sys.executable, str(folder / "export.py"), str(out)], check=True)
    return code if code != 0 else (0 if valid_trial else 1)


if __name__ == "__main__":
    raise SystemExit(main())
