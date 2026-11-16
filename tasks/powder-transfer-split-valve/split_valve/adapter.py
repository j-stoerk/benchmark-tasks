"""Inspect Robots adapter for a task-specific native fixture and measured sampler."""

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


class Fixture(EmbodimentBase):
    def __init__(self, trace_file=None, device="cuda:0", variant="normal", **kwargs):
        if kwargs or variant not in spec.VARIANTS:
            raise ValueError("Unknown native fixture option/control variant")
        self.info = EmbodimentInfo(
            name=spec.EMBODIMENT,
            action_space=spec.action_space(),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, spec.HEIGHT, spec.WIDTH) for name in spec.CAMERAS),
                state=StateSpec(fields=tuple(StateField(*field) for field in spec.FIELDS)),
            ),
            control_hz=spec.HZ,
            is_simulated=True,
            capabilities=frozenset({"seedable", "resettable", "privileged_success"}),
            docs=spec.DOCS,
            environment_id="isaaclab-arena-" + spec.SLUG,
            environment_revision=spec.revision(),
            morphology=spec.MORPHOLOGY,
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

        from .native import Sampler, make_env

        self.torch = torch
        self.env = make_env(self.device, self.variant)
        self.base = self.env.unwrapped
        self.sampler = Sampler(self.base)

    def native_action(self):
        return self.torch.tensor([self.command.tolist()], device=self.base.device, dtype=self.torch.float32)

    def observation(self, reading=None):
        images = {}
        for name in spec.CAMERAS:
            images[name] = np.ascontiguousarray(
                numpy(self.base.scene[name].data.output["rgb"])[0, :, :, :3], dtype=np.uint8
            )
        if reading is None:
            reading = self.sampler.physics()
        state = {key: np.asarray(value, float) for key, value in self.sampler.public(reading).items()}
        return Observation(
            images=images,
            state=state,
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
            self.env.step(self.native_action())
        for _ in range(4):
            self.base.sim.render()
        self.t = 0.0
        if self.trace:
            self.trace.parent.mkdir(parents=True, exist_ok=True)
        return self.observation()

    def step(self, action):
        command = np.asarray(action.data, float)
        if command.shape != spec.INITIAL.shape or not np.isfinite(command).all():
            raise ValueError("Expected finite commands with the documented shape")
        self.command = np.clip(command, spec.LOW, spec.HIGH)
        self.env.step(self.native_action())
        self.t += 1 / spec.HZ
        reading = self.sampler.physics()
        score, success = self.proof.update(reading, self.command)
        info = {
            "score": score,
            "success": success,
            "invalid": self.proof.invalid,
            "proof_seconds": self.proof.elapsed,
            "physics": reading,
            "instruments": {
                key: np.asarray(value).tolist() for key, value in self.sampler.public(reading).items()
            },
        }
        if self.trace:
            with self.trace.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"t": self.t, "info": info}) + "\n")
        return StepResult(
            observation=self.observation(reading),
            terminated=success,
            termination_reason="success" if success else None,
            info=info,
        )

    def close(self):
        self.env = None


def embodiment(**kwargs):
    return Fixture(**kwargs)
