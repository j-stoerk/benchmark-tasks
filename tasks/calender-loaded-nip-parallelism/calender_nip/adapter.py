"""Inspect Robots embodiment with native gauge observations and total outcome scoring."""

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


class CalenderNip(EmbodimentBase):
    def __init__(self, trace_file=None, device="cuda:0", proof_scale=1.0, **kwargs):
        if kwargs:
            raise ValueError(f"Unsupported embodiment options: {sorted(kwargs)}")
        if float(proof_scale) not in (0.0, 1.0):
            raise ValueError("Proof scale is 1.0 normally or 0.0 for the no-force negative control")
        self.info = EmbodimentInfo(
            name="calender_nip",
            action_space=spec.action_space(),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, 360, 640) for name in ("overview_cam", "nip_cam")),
                state=StateSpec(
                    fields=(
                        StateField("gaps_um", (5,), "um"),
                        StateField("proof_forces_N", (2,), "N"),
                        StateField("rest_settings", (3,), "um,um,fraction"),
                    )
                ),
            ),
            control_hz=spec.HZ,
            is_simulated=True,
            capabilities=frozenset({"seedable", "resettable", "privileged_success"}),
            docs=spec.DOCS,
            environment_id="isaaclab-arena-calender-loaded-nip",
            environment_revision=spec.revision(),
            morphology="powered-laboratory-calender-adjustment",
        )
        self.device, self.proof_scale = device, float(proof_scale)
        self.trace = Path(trace_file) if trace_file else None
        self.env = None
        self.t = 0.0
        self.proof = spec.Proof()
        self.command = spec.INITIAL.copy()

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
        self.env = make_env(self.device, self.proof_scale)
        self.base = self.env.unwrapped
        self.robot = self.base.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.term = self.base.action_manager.get_term("bearings")

    def native_action(self, command):
        return self.torch.tensor([command.tolist()], device=self.base.device, dtype=self.torch.float32)

    def readout(self):
        q = numpy(self.robot.data.joint_pos)[0, self.ids]
        v = numpy(self.robot.data.joint_vel)[0, self.ids]
        return {
            "joint_positions": q.tolist(),
            "joint_velocities": v.tolist(),
            "gaps_um": np.round(spec.gaps(*q), 1).tolist(),
            "applied_proof_forces_N": numpy(self.term.applied_forces)[0].tolist(),
            "rest_settings": numpy(self.term.processed_actions)[0].tolist(),
        }

    def observation(self):
        images = {}
        for name in ("overview_cam", "nip_cam"):
            rgb = self.base.scene[name].data.output["rgb"]
            images[name] = np.ascontiguousarray(numpy(rgb)[0, :, :, :3], dtype=np.uint8)
        reading = self.readout()
        return Observation(
            images=images,
            state={
                "gaps_um": np.array(reading["gaps_um"]),
                "proof_forces_N": np.array(reading["applied_proof_forces_N"]),
                "rest_settings": np.array(reading["rest_settings"]),
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
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError("Expected three finite commands")
        if command[2] not in (0, 1):
            raise ValueError("Proof command must be 0 or 1")
        self.command = np.clip(command, spec.LOW, spec.HIGH)
        self.env.step(self.native_action(self.command))
        self.t += 1 / spec.HZ
        reading = self.readout()
        score, success = self.proof.update(reading, self.command[2] == 1)
        info = {
            "score": score,
            "success": success,
            "invalid": self.proof.invalid,
            "proof_seconds": self.proof.elapsed,
            "physics": reading,
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


def calender_nip(**kwargs):
    return CalenderNip(**kwargs)
