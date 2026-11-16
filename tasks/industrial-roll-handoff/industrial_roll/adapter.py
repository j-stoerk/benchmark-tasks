"""Inspect Robots adapter for the force-limited industrial handling machine."""

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


class IndustrialRoll(EmbodimentBase):
    def __init__(self, trace_file=None, device="cuda:0", lift_force=6000, **kwargs):
        if kwargs:
            raise ValueError(f"Unsupported embodiment options: {sorted(kwargs)}")
        self.info = EmbodimentInfo(
            name="industrial_roll",
            action_space=spec.action_space(),
            observation_space=ObservationSpace(
                cameras=tuple(CameraSpec(name, 360, 640) for name in ("overview_cam", "core_cam")),
                state=StateSpec(
                    fields=(StateField("machine_encoders", (6,), "m+normalized"), StateField("jaw_forces", (3,), "N"))
                ),
            ),
            control_hz=spec.HZ,
            is_simulated=True,
            capabilities=frozenset({"seedable", "resettable", "privileged_success"}),
            docs=spec.DOCS,
            environment_id="isaaclab-arena-industrial-roll-handoff",
            environment_revision=spec.revision(),
            morphology="powered-core-roll-transfer-machine",
        )
        self.device, self.lift_force = device, float(lift_force)
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
        self.env = make_env(self.device, self.lift_force)
        self.base = self.env.unwrapped
        self.receiver = self.base.scene["robot"]
        self.donor = self.base.scene["donor"]
        self.ids = self.receiver.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.donor_ids = self.donor.find_joints(["withdraw"])[0]

    def native_action(self, command):
        insert, y, z, clamp, donor, test = command
        values = [-insert, y, z - 1, *([clamp * 0.0078] * 3), -donor * 0.4, test]
        return self.torch.tensor([values], device=self.base.device, dtype=self.torch.float32)

    def readout(self):
        from yam_arena.scene import object_states

        roll = object_states(self.env, ["roll"])["roll"]
        q = numpy(self.receiver.data.joint_pos)[0, self.ids]
        d = float(numpy(self.donor.data.joint_pos)[0, self.donor_ids[0]])
        return {
            "roll_pos": roll.pos.tolist(),
            "roll_quat": roll.quat_xyzw.tolist(),
            "roll_lin_vel": roll.lin_vel.tolist(),
            "roll_ang_vel": roll.ang_vel.tolist(),
            "receiver_q": q.tolist(),
            "donor_q": d,
            "donor_contact": float(np.linalg.norm(numpy(self.base.scene["donor_contact"].data.net_forces_w)[0])),
            "jaw_forces": self.jaw_forces().tolist(),
            "axis_effort": numpy(self.receiver.data.applied_torque)[0, self.ids].tolist(),
            "external_pull_N": spec.PROOF_PULL if self.command[5] >= 0.5 else 0,
            "external_torque_Nm": spec.PROOF_TORQUE if self.command[5] >= 0.5 else 0,
        }

    def jaw_forces(self):
        return np.array([np.linalg.norm(numpy(self.base.scene[f"pad_{i}"].data.net_forces_w)[0]) for i in range(3)])

    def observation(self):
        images = {}
        for name in ("overview_cam", "core_cam"):
            rgb = self.base.scene[name].data.output["rgb"]
            images[name] = np.ascontiguousarray(numpy(rgb)[0, :, :, :3], dtype=np.uint8)
        q = numpy(self.receiver.data.joint_pos)[0, self.ids]
        d = float(numpy(self.donor.data.joint_pos)[0, self.donor_ids[0]])
        state = np.array([-q[0], q[1], 1 + q[2], np.mean(q[3:]) / 0.0078, -d / 0.4, self.command[5]])
        return Observation(
            images=images,
            state={"machine_encoders": state, "jaw_forces": self.jaw_forces()},
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
        command = np.asarray(action.data, dtype=float)
        if command.shape != (6,) or not np.isfinite(command).all():
            raise ValueError("Expected six finite machine commands")
        self.command = np.clip(command, spec.LOW, spec.HIGH)
        self.env.step(self.native_action(self.command))
        self.t += 1 / spec.HZ
        readout = self.readout()
        score, success = self.proof.update(readout, self.command[5] >= 0.5)
        info = {
            "score": score,
            "success": success,
            "dropped": self.proof.dropped,
            "proof_seconds": self.proof.elapsed,
            "physics": readout,
            "geometry": spec.geometry(readout),
        }
        if self.trace:
            with self.trace.open("a", encoding="utf-8") as f:
                f.write(json.dumps({"t": self.t, "info": info}) + "\n")
        return StepResult(
            observation=self.observation(),
            terminated=success,
            termination_reason="success" if success else None,
            info=info,
        )

    def close(self):
        self.env = None


def industrial_roll(**kwargs):
    return IndustrialRoll(**kwargs)
