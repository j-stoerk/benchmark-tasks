"""Gauge-feedback demo and genuine external LLM bridge with no scripted fallback."""

import json
import time
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, unloaded_only=False, calibration_bench=False):
        self.info = PolicyInfo(name="calender_demo", action_space=spec.action_space())
        self.unloaded_only = bool(unloaded_only)
        self.calibration_bench = bool(calibration_bench)

    def reset(self, scene):
        self.ready_seconds = 0.0
        self.proof_on = False

    def act(self, observation):
        if self.calibration_bench:
            command = spec.INITIAL.copy()
            command[2] = float(observation.state_time >= 4.0)
            return ActionChunk(actions=[Action(data=command)], control_hz=spec.HZ)
        g = np.asarray(observation.state["gaps_um"])
        rest = np.asarray(observation.state["rest_settings"])
        if not self.proof_on:
            self.ready_seconds = self.ready_seconds + 1 / spec.HZ if np.max(np.abs(g - spec.GAP)) < 3 else 0
            self.proof_on = self.ready_seconds >= 1.0 and not self.unloaded_only
        mean_error = spec.GAP - g[2]
        half_diff = (g[-1] - g[0]) * spec.BEARING_HALF_SPAN / (spec.STATIONS[-1] - spec.STATIONS[0])
        correction = np.array([mean_error + half_diff, mean_error - half_diff])
        command = np.r_[rest[:2] + 0.25 * correction, float(self.proof_on)]
        command = np.clip(command, spec.LOW, spec.HIGH)
        return ActionChunk(actions=[Action(data=command)], control_hz=spec.HZ)


class ExternalLLM(PolicyBase):
    def __init__(self, directory, controller="GPT-6 Codex interactive controller", timeout=1800):
        self.info = PolicyInfo(name="calender_llm", action_space=spec.action_space(), checkpoint=controller)
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.timeout = float(timeout)
        self.index = 0

    def reset(self, scene):
        self.index = 0

    def act(self, observation):
        from PIL import Image

        self.index += 1
        if self.index > 30:
            raise RuntimeError("The LLM call limit is 30")
        prefix = f"request-{self.index:03d}"
        images = {}
        for name, array in observation.images.items():
            path = self.directory / f"{prefix}-{name}.png"
            Image.fromarray(array).save(path)
            images[name] = path.name
        request = {
            "id": self.index,
            "t": observation.state_time,
            "instruction": observation.instruction,
            "operating_notes": spec.DOCS,
            "readouts": {name: np.asarray(value).tolist() for name, value in observation.state.items()},
            "images": images,
            "response_schema": {
                "action": "[left_um,right_um,proof_0_or_1]",
                "steps": "integer 1..200",
                "rationale": "brief decision explanation",
            },
        }
        (self.directory / f"{prefix}.json").write_text(json.dumps(request, indent=2) + "\n", encoding="utf-8")
        response = self.directory / f"response-{self.index:03d}.json"
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                data = json.loads(response.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(0.2)
                continue
            command, steps = np.asarray(data["action"], float), data["steps"]
            if (
                command.shape != (3,)
                or not np.isfinite(command).all()
                or not isinstance(steps, int)
                or isinstance(steps, bool)
                or not 1 <= steps <= 200
                or not str(data.get("rationale", "")).strip()
                or np.any(command < spec.LOW)
                or np.any(command > spec.HIGH)
                or command[2] not in (0, 1)
            ):
                raise ValueError("Invalid external LLM response")
            meta = {"llm_request": self.index, "rationale": data["rationale"]}
            return ActionChunk(
                actions=[Action(data=command.copy(), meta=meta) for _ in range(steps)],
                control_hz=spec.HZ,
            )
        raise TimeoutError("No external LLM response; no scripted replacement was used")


def calender_demo(**kwargs):
    return Demo(**kwargs)


def calender_llm(**kwargs):
    return ExternalLLM(**kwargs)
