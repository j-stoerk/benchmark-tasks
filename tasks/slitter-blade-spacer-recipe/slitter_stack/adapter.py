"""Inspect Robots adapter; free-body poses are available only to the scorer."""

import atexit
import json
import os
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import EmbodimentBase, EmbodimentInfo
from inspect_robots.spaces import CameraSpec, ObservationSpace, StateField, StateSpec
from inspect_robots.types import Observation, StepResult

from . import spec

APP = None


def numpy(value):
    return getattr(value, "torch", value).detach().cpu().numpy().astype(float)


class SlitterStack(EmbodimentBase):
    def __init__(self, trace_file=None, device="cuda:0", variant="normal", **kwargs):
        if kwargs or variant not in ["normal", "no-grip", "unclamped"]:
            raise ValueError("Unknown assembly embodiment option or control variant")
        self.info = EmbodimentInfo(
            name="slitter_stack",
            action_space=spec.action_space(),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, 720, 1280) for name in ("overview_cam", "arbor_cam")),
                state=StateSpec(
                    fields=(
                        StateField("machine_encoders", (6,), "m,normalized"),
                        StateField("jaw_forces", (3,), "N"),
                        StateField("clamp_force", (1,), "N"),
                        StateField("blade_heights_mm", (4,), "mm"),
                    )
                ),
            ),
            control_hz=spec.HZ,
            is_simulated=True,
            capabilities=frozenset({"seedable", "resettable", "privileged_success"}),
            docs=spec.DOCS,
            environment_id="isaaclab-arena-slitter-stack",
            environment_revision=spec.revision(),
            morphology="three-axis-three-jaw-cassette-assembler",
        )
        self.device, self.variant = device, variant
        self.trace = Path(trace_file) if trace_file else None
        self.env = None
        self.command = spec.INITIAL.copy()
        self.proof = spec.Proof()
        self.t = 0.0

    def start(self):
        global APP
        if self.env is not None:
            return
        os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        if APP is None:
            from isaaclab.app import AppLauncher

            APP = AppLauncher(headless=True, device=self.device, enable_cameras=True).app
            atexit.register(lambda: APP.close())
        import torch

        from .native import make_env

        self.torch = torch
        self.env = make_env(self.device, grip_force=0 if self.variant == "no-grip" else 50)
        self.base = self.env.unwrapped
        self.robot = self.base.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.press_ids = self.robot.find_joints(["press"])[0]
        self.press_y_ids = self.robot.find_joints(["press_y"])[0]
        self.term = self.base.action_manager.get_term("machine")

    def native_action(self, command):
        return self.torch.tensor([command.tolist()], device=self.base.device, dtype=self.torch.float32)

    def readout(self):
        from yam_arena.scene import object_states

        states = object_states(self.env, [p["name"] for p in spec.PARTS])
        q = numpy(self.robot.data.joint_pos)[0, self.ids]
        tool = [*q[:3], -float(np.mean(q[3:])) / 0.023, *self.command[4:]]
        return {
            "parts": {
                name: {
                    "pos": obj.pos.tolist(),
                    "quat": obj.quat_xyzw.tolist(),
                    "lin_vel": obj.lin_vel.tolist(),
                    "ang_vel": obj.ang_vel.tolist(),
                }
                for name, obj in states.items()
            },
            "tool": tool,
            "jaw_forces": [
                float(np.linalg.norm(numpy(self.base.scene[f"pad_{i}"].data.net_normal_forces_w)[0])) for i in range(3)
            ],
            "clamp_contact_N": float(
                np.linalg.norm(numpy(self.base.scene["platen_contact"].data.net_normal_forces_w)[0])
            ),
            "clamp_z": 1.20 + float(numpy(self.robot.data.joint_pos)[0, self.press_ids[0]]),
            "clamp_y": float(numpy(self.robot.data.joint_pos)[0, self.press_y_ids[0]]),
            "applied_proof": numpy(self.term.applied_proof)[0].tolist(),
        }

    def observation(self):
        images = {}
        for name in ("overview_cam", "arbor_cam"):
            rgb = self.base.scene[name].data.output["rgb"]
            images[name] = np.ascontiguousarray(numpy(rgb)[0, :, :, :3], dtype=np.uint8)
        reading = self.readout()
        gauges = []
        for p in spec.PARTS:
            if not p["blade"]:
                continue
            pos = np.asarray(reading["parts"][p["name"]]["pos"])
            in_range = np.linalg.norm(pos[:2]) <= 0.001 and spec.BASE_Z - 0.005 <= pos[2] <= 1.20
            if in_range:
                height_mm = (pos[2] - spec.BLADE_H / 2 - spec.BASE_Z) * 1000
                gauges.append(float(np.round(height_mm / 0.005) * 0.005))
        gauges = sorted(gauges) + [-1.0] * (4 - len(gauges))
        return Observation(
            images=images,
            state={
                "machine_encoders": np.array(reading["tool"]),
                "jaw_forces": np.array(reading["jaw_forces"]),
                "clamp_force": np.array([reading["clamp_contact_N"]]),
                "blade_heights_mm": np.array(gauges),
            },
            instruction=self.instruction,
            state_time=self.t,
            image_times=dict.fromkeys(images, self.t),
        )

    def reset(self, scene, *, seed=None):
        self.start()
        self.instruction = scene.instruction
        self.env.reset(seed=seed)
        self.command = spec.INITIAL.copy()
        self.proof = spec.Proof()
        for _ in range(40):
            self.env.step(self.native_action(self.command))
        for _ in range(4):
            self.base.sim.render()
        self.t = 0.0
        if self.trace:
            self.trace.parent.mkdir(parents=True, exist_ok=True)
        return self.observation()

    def step(self, action):
        command = np.asarray(action.data, float)
        if command.shape != (6,) or not np.isfinite(command).all():
            raise ValueError("Expected six finite machine commands")
        self.command = np.clip(command, spec.LOW, spec.HIGH)
        if self.variant == "unclamped":
            self.command[4] = 0
        self.env.step(self.native_action(self.command))
        self.t += 1 / spec.HZ
        reading = self.readout()
        score, success = self.proof.update(reading, self.command[5] >= 0.5)
        info = {
            "score": score,
            "success": success,
            "invalid": self.proof.invalid,
            "proof_seconds": self.proof.elapsed,
            "physics": reading,
            "geometry": spec.geometry(reading),
        }
        if self.trace:
            with self.trace.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"t": self.t, "info": info}) + "\n")
        return StepResult(
            observation=self.observation(),
            terminated=success,
            termination_reason="success" if success else None,
            info=info,
        )

    def close(self):
        self.env = None


def slitter_stack(**kwargs):
    return SlitterStack(**kwargs)
