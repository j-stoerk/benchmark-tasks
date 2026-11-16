"""Run one recorded physical or genuine external-LLM trial; never overwrite evidence."""

import argparse
import datetime as dt
import hashlib
import importlib.metadata as metadata
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["smoke", "demo", "model"])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bridge", type=Path)
    parser.add_argument("--max-steps", type=int, default=1200)
    parser.add_argument("--proof-scale", type=float, default=1.0)
    parser.add_argument("--calibration-bench", action="store_true")
    parser.add_argument(
        "--unloaded-only", action="store_true", help="Scripted negative control: leave the physical proof load off"
    )
    args = parser.parse_args()
    folder = Path(__file__).resolve().parent
    spec = json.loads((folder / "task.json").read_text(encoding="utf-8"))
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    policy = "calender_llm" if args.mode == "model" else "calender_demo"
    command = [
        shutil.which("inspect-robots") or "inspect-robots",
        "run",
        "--embodiment",
        "calender_nip",
        "--policy",
        policy,
        "-E",
        f"trace_file={out / 'physics.jsonl'}",
        "-E",
        f"proof_scale={args.proof_scale}",
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
    if policy == "calender_llm":
        if args.unloaded_only or args.calibration_bench:
            parser.error("Scripted controls cannot be applied to the model policy")
        if args.bridge is None:
            parser.error("model needs --bridge directory for actual LLM decisions")
        command += ["-P", f"directory={args.bridge.resolve()}"]
    elif args.unloaded_only:
        command += ["-P", "unloaded_only=true"]
    elif args.calibration_bench:
        command += ["-P", "calibration_bench=true"]
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
        process = subprocess.Popen(command, stdout=console, stderr=subprocess.STDOUT)
        while process.poll() is None:
            time.sleep(10)
            trace = out / "physics.jsonl"
            if trace.exists():
                try:
                    with trace.open("rb") as stream:
                        stream.seek(max(0, trace.stat().st_size - 8000))
                        last = json.loads(stream.read().splitlines()[-1])
                    info = last["info"]
                    print(
                        json.dumps(
                            {
                                "simulation_s": last["t"],
                                "score": info["score"],
                                "proof_s": info["proof_seconds"],
                                "physics": info["physics"],
                            }
                        ),
                        flush=True,
                    )
                except (IndexError, json.JSONDecodeError):
                    pass  # A concurrent write is retried on the next progress tick.
        code = process.wait()
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
