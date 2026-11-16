"""Independent scripted demo and an explicit external-LLM decision bridge."""

import json
import time
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, expand=True):
        self.info = PolicyInfo(name="industrial_demo", action_space=spec.action_space())
        self.expand = bool(expand)

    def reset(self, scene):
        pass

    def act(self, observation):
        t = observation.state_time
        command = [0, 0, 1, 0, 0, 0]
        if t >= 2:
            command[0] = 0.24
        if t >= 9 and self.expand:
            command[3] = 1
        if t >= 12:
            command[4] = 1
        if t >= 20:
            command[2] = 1.08
        if t >= 25:
            command[1] = 0.35
        if t >= 36:
            command[5] = 1
        return ActionChunk(actions=[Action(data=np.array(command, float))], control_hz=spec.HZ)


class ExternalLLM(PolicyBase):
    """No fallback: every action chunk requires an actual external LLM response.

    The calling Codex session receives only this explicit request protocol for
    control. Its code-aware, interactive nature must be reported with results.
    """

    def __init__(self, directory, controller="GPT-6 Codex interactive controller", timeout=1800):
        self.info = PolicyInfo(name="industrial_llm", action_space=spec.action_space(), checkpoint=controller)
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
            "machine_encoders": np.asarray(observation.state["machine_encoders"]).tolist(),
            "jaw_forces": np.asarray(observation.state["jaw_forces"]).tolist(),
            "images": images,
            "response_schema": {
                "action": "six numbers per operating notes",
                "steps": "integer 1..200",
                "rationale": "brief decision explanation",
            },
        }
        path = self.directory / f"{prefix}.json"
        path.write_text(json.dumps(request, indent=2) + "\n", encoding="utf-8")
        response = self.directory / f"response-{self.index:03d}.json"
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            try:
                data = json.loads(response.read_text(encoding="utf-8"))
            except (FileNotFoundError, json.JSONDecodeError):
                time.sleep(0.2)
                continue
            command = np.asarray(data["action"], float)
            steps = data["steps"]
            if (
                command.shape != (6,)
                or not np.isfinite(command).all()
                or not isinstance(steps, int)
                or not 1 <= steps <= 200
                or not str(data.get("rationale", "")).strip()
            ):
                raise ValueError("Invalid external LLM response")
            if np.any(command < spec.LOW) or np.any(command > spec.HIGH):
                raise ValueError("LLM command outside documented machine limits")
            meta = {"llm_request": self.index, "rationale": data["rationale"]}
            return ActionChunk(
                actions=[Action(data=command.copy(), meta=meta) for _ in range(steps)], control_hz=spec.HZ
            )
        raise TimeoutError("No external LLM response; no scripted replacement was used")


def industrial_demo(**kwargs):
    return Demo(**kwargs)


def industrial_llm(**kwargs):
    return ExternalLLM(**kwargs)
