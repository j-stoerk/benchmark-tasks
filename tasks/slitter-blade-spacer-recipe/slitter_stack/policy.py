"""Contact-feedback demonstration and bounded genuine external-LLM trajectories."""

import json
import time
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, variant="normal"):
        self.info = PolicyInfo(name="slitter_demo", action_space=spec.action_space())
        self.variant = variant

    def reset(self, scene):
        self.item, self.phase, self.entered = 0, "approach", 0.0
        if self.variant == "contact-bench":
            self.item = len(spec.PARTS)

    def act(self, observation):
        t = observation.state_time
        q = np.asarray(observation.state["machine_encoders"])
        if self.item >= len(spec.PARTS):
            command = [0.60, -0.40, 1.35, 0, 0, 0]
            if self.phase not in ("park", "clamp", "proof"):
                self.phase, self.entered = "park", t
            if self.phase == "park" and np.max(np.abs(q[:3] - command[:3])) < 0.00025:
                self.phase, self.entered = "clamp", t
            command[4] = float(self.phase in ("clamp", "proof"))
            if (
                self.phase == "clamp"
                and float(observation.state["clamp_force"][0]) >= 300
                and t - self.entered >= 1.2
            ):
                self.phase = "proof"
            command[5] = float(self.phase == "proof")
            return ActionChunk(actions=[Action(data=np.array(command, float))], control_hz=spec.HZ)
        part = spec.PARTS[self.item]
        source = np.asarray(part["source"])
        place_z = part["target_z"] + 0.001
        destinations = {
            "approach": [source[0], source[1], 1.35, 0, 0, 0],
            "lower": [*source, 0, 0, 0],
            "grip": [*source, 1, 0, 0],
            "lift": [source[0], source[1], 1.35, 1, 0, 0],
            "travel": [0, 0, 1.35, 1, 0, 0],
            "place": [0, 0, place_z, 1, 0, 0],
            "release": [0, 0, place_z, 0, 0, 0],
            "retract": [0, 0, 1.35, 0, 0, 0],
        }
        command = np.asarray(destinations[self.phase], float)
        close = np.max(np.abs(q[:3] - command[:3])) < 0.00025
        age = t - self.entered
        advance = close and age >= 0.10
        if self.phase == "grip":
            advance = age >= 1.20 and np.min(observation.state["jaw_forces"]) >= 5
        elif self.phase == "release":
            advance = age >= 1.20 and q[3] < 0.02
        if advance:
            phases = list(destinations)
            index = phases.index(self.phase)
            if index == len(phases) - 1:
                self.item += 1
                self.phase = "approach"
            else:
                self.phase = phases[index + 1]
            self.entered = t
        return ActionChunk(actions=[Action(data=command)], control_hz=spec.HZ)


class ExternalLLM(PolicyBase):
    def __init__(self, directory, controller="GPT-6 Codex interactive controller", timeout=1800):
        self.info = PolicyInfo(name="slitter_llm", action_space=spec.action_space(), checkpoint=controller)
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
            "readouts": {k: np.asarray(v).tolist() for k, v in observation.state.items()},
            "images": images,
            "response_schema": {
                "sequence": "List of {action:[x,y,z,grip,clamp,proof],steps:integer}; total 1..400 actions",
                "rationale": "Brief decision explanation; each waypoint is actually chosen by the LLM",
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
                raise ValueError("Invalid LLM sequence or rationale")
            actions = []
            for index, entry in enumerate(sequence):
                command = np.asarray(entry["action"], float)
                steps = entry["steps"]
                if (
                    command.shape != (6,)
                    or not np.isfinite(command).all()
                    or np.any(command < spec.LOW)
                    or np.any(command > spec.HIGH)
                    or not isinstance(steps, int)
                    or isinstance(steps, bool)
                    or not 1 <= steps <= 400
                ):
                    raise ValueError("LLM waypoint outside the documented controls")
                meta = {"llm_request": self.index, "waypoint": index, "rationale": data["rationale"]}
                actions.extend(Action(data=command.copy(), meta=meta) for _ in range(steps))
            if not 1 <= len(actions) <= 400:
                raise ValueError("An LLM call may select at most 400 control steps")
            return ActionChunk(actions=actions, control_hz=spec.HZ)
        raise TimeoutError("No actual LLM response; no scripted replacement was used")


def slitter_demo(**kwargs):
    return Demo(**kwargs)


def slitter_llm(**kwargs):
    return ExternalLLM(**kwargs)
