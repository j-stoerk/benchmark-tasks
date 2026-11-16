"""Export every recorded native frame, at recorded time, into review media."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    folder = args.trial.resolve()
    logs = list(folder.glob("adhoc_*.json"))
    if len(logs) != 1:
        raise ValueError("Exactly one genuine Inspect log is required")
    log = json.loads(logs[0].read_text())
    manifest = json.loads((folder / "run_manifest.json").read_text())
    if not manifest["valid_trial"] or manifest["exit_code"] or log["results"]["errored_trials"]:
        raise ValueError("An errored attempt is not scored task evidence")
    trace = [json.loads(line) for line in (folder / "physics.jsonl").read_text().splitlines()]
    streams = folder / "streams"
    subprocess.run(["inspect-robots", "video", str(logs[0]), "--out", str(streams)], check=True)
    inputs = []
    counts = {}
    for camera in ("overview_cam", "detail_cam"):
        videos = list(streams.glob("*" + camera + "*.mp4"))
        if len(videos) != 1:
            raise ValueError("Missing recorded camera " + camera)
        inputs += ["-i", str(videos[0])]
        frames = sorted(folder.rglob("*" + camera + "*.npy"))
        if len(frames) != len(trace) + 1:
            raise ValueError("Incomplete camera stream: " + camera)
        counts[camera] = len(frames)
        for name, index in [("initial", 0), ("final", -1)]:
            Image.fromarray(np.load(frames[index])[:, :, :3]).save(folder / f"{camera}-{name}.png")
    video = folder / "trial.mp4"
    subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            *inputs,
            "-filter_complex",
            "[0:v][1:v]vstack=inputs=2[all]",
            "-map",
            "[all]",
            "-c:v",
            "libx264",
            "-crf",
            "21",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(video),
        ],
        check=True,
    )
    if video.stat().st_size > 50_000_000:
        raise ValueError("Review video exceeds 50 MB")
    summary = {
        "score": log["results"]["metrics"]["task_score"],
        "policy": manifest["policy"],
        "steps": len(trace),
        "frame_counts": counts,
        "final": trace[-1],
        "video_bytes": video.stat().st_size,
        "video_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
        "eval_log_sha256": hashlib.sha256(logs[0].read_bytes()).hexdigest(),
        "source_hashes": manifest["source_hashes"],
        "versions": manifest["versions"],
    }
    (folder / "evidence_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in ("score", "steps", "frame_counts", "video_bytes")}))


if __name__ == "__main__":
    main()
