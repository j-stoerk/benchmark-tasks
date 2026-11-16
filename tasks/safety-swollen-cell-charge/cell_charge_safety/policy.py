"""Bounded external LLM responses, independent of the task's scripted demo."""

import json
import time
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class ExternalLLM(PolicyBase):
    def __init__(self, directory, controller="GPT-6 Codex interactive controller", timeout=1800):
        self.info = PolicyInfo(name=spec.EMBODIMENT + "_llm", action_space=spec.action_space(), checkpoint=controller)
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.timeout = float(timeout)
        self.index = 0

    def reset(self, scene):
        self.index = 0

    def act(self, observation):
        from PIL import Image

        self.index += 1
        if self.index > spec.CALL_LIMIT:
            raise RuntimeError("The LLM call limit was reached")
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
            "images": images,
            "readouts": {key: np.asarray(value).tolist() for key, value in observation.state.items()},
            "response_schema": {
                "sequence": "[{action:[documented commands],steps:integer},...]",
                "limit": "1..400 total actions per call",
                "rationale": "Brief decision explanation",
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
            sequence = data.get("sequence", [data])
            if (
                not isinstance(sequence, list)
                or not 1 <= len(sequence) <= 20
                or not str(data.get("rationale", "")).strip()
            ):
                raise ValueError("An actual LLM sequence and rationale are required")
            actions = []
            for index, entry in enumerate(sequence):
                command = np.asarray(entry["action"], float)
                steps = entry["steps"]
                if (
                    command.shape != spec.INITIAL.shape
                    or not np.isfinite(command).all()
                    or np.any(command < spec.LOW)
                    or np.any(command > spec.HIGH)
                    or not isinstance(steps, int)
                    or isinstance(steps, bool)
                    or not 1 <= steps <= 400
                ):
                    raise ValueError("LLM waypoint outside the documented limits")
                meta = {"llm_request": self.index, "waypoint": index, "rationale": data["rationale"]}
                actions.extend(Action(data=command.copy(), meta=meta) for _ in range(steps))
            if not 1 <= len(actions) <= 400:
                raise ValueError("An LLM call may contain at most 400 actions")
            return ActionChunk(actions=actions, control_hz=spec.HZ)
        raise TimeoutError("No actual LLM response; no scripted fallback was used")


def demo(**kwargs):
    from .control import Demo

    return Demo(**kwargs)


def llm(**kwargs):
    return ExternalLLM(**kwargs)
