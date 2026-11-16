"""Export complete recorded camera streams and measured gauges; never synthesize a run."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image


def clock(seconds):
    ticks = round(seconds * 100)
    return f"{ticks // 360000}:{ticks // 6000 % 60:02d}:{ticks // 100 % 60:02d}.{ticks % 100:02d}"


def gauges(trial, trace):
    """Render actual trace readings into a review display, never into policy input."""
    lines = [
        "[Script Info]",
        "ScriptType: v4.00+",
        "PlayResX: 1280",
        "PlayResY: 1620",
        "[V4+ Styles]",
        "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, "
        "Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, "
        "MarginL, MarginR, MarginV, Encoding",
        "Style: Default,DejaVu Sans Mono,26,&H0014243B,&H0014243B,&H00FFFFFF,&H00FFFFFF,"
        "0,0,0,0,100,100,0,0,1,0,0,7,8,8,8,1",
        "[Events]",
        "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text",
    ]
    for row in trace:
        info, t = row["info"], row["t"]
        p = info["physics"]
        parts = p["parts"]
        faces = sorted(
            (part["pos"][2] - 0.001 - 0.8) * 1000
            for name, part in parts.items()
            if name in ("part_1", "part_3", "part_5", "part_7")
        )
        values = "/".join(f"{x:.3f}" for x in faces)
        pads = "/".join(f"{x:.1f}" for x in p["jaw_forces"])
        text = (
            f"t={t:.2f}s score={info['score']:.2f} loaded hold={info['proof_seconds']:.2f}s"
            + r"\N"
            + f"Blade lower faces [mm]: {values}"
            + r"\N"
            + f"Clamp contact: {p['clamp_contact_N']:.1f} N | jaw normal: {pads} N"
            + r"\N"
            + f"Applied blade proof: {p['applied_proof'][0]:.0f} N / {p['applied_proof'][1]:.1f} Nm"
        )
        lines.append(f"Dialogue: 0,{clock(t)},{clock(t + 0.05)},Default,,0,0,0,,{text}")
    target = trial / "measured-gauges.ass"
    target.write_text("\n".join(lines) + "\n")
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    trial = args.trial.resolve()
    logs = list(trial.glob("adhoc_*.json"))
    if len(logs) != 1:
        raise RuntimeError(f"Expected one Inspect Robots evaluation log, got {len(logs)}")
    log = json.loads(logs[0].read_text())
    manifest = json.loads((trial / "run_manifest.json").read_text())
    if manifest.get("exit_code") != 0 or log["results"]["errored_trials"]:
        raise RuntimeError("An errored trial cannot be exported as valid model/demo evidence")
    trace = [json.loads(line) for line in (trial / "physics.jsonl").read_text().splitlines()]
    streams = trial / "streams"
    subprocess.run(["inspect-robots", "video", str(logs[0]), "--out", str(streams)], check=True)
    inputs, frame_counts = [], {}
    for camera in ["overview_cam", "arbor_cam"]:
        videos = list(streams.glob("*" + camera + "*.mp4"))
        if len(videos) != 1:
            raise RuntimeError(f"Expected one {camera} video, got {len(videos)}")
        inputs += ["-i", str(videos[0])]
        frames = sorted(trial.rglob("*" + camera + "*.npy"))
        frame_counts[camera] = len(frames)
        if not frames:
            raise RuntimeError("Missing actual recorded camera frames")
        for label, index in [("initial", 0), ("final", -1)]:
            Image.fromarray(np.load(frames[index])[:, :, :3]).save(trial / f"{camera}-{label}.png")
    # Full videos, at their recorded rate, with no frame selection or acceleration.
    video = trial / "trial.mp4"
    overlay = gauges(trial, trace)
    subprocess.run(
        [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            *inputs,
            "-filter_complex",
            f"[0:v][1:v]vstack=inputs=2,pad=1280:1620:0:180:color=white,subtitles={overlay}[all]",
            "-map",
            "[all]",
            "-c:v",
            "libx264",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(video),
        ],
        check=True,
    )
    if video.stat().st_size >= 50_000_000:
        raise RuntimeError("Video exceeds the board limit")
    summary = {
        "score": log["results"]["metrics"]["task_score"],
        "policy": manifest["policy"],
        "steps": len(trace),
        "frame_counts": frame_counts,
        "final": trace[-1] if trace else None,
        "video_bytes": video.stat().st_size,
        "video_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
        "eval_log_sha256": hashlib.sha256(logs[0].read_bytes()).hexdigest(),
        "source_hashes": manifest["source_hashes"],
        "versions": manifest["versions"],
    }
    (trial / "evidence_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: summary[k] for k in ["policy", "score", "steps", "frame_counts", "video_bytes"]}))


if __name__ == "__main__":
    main()
