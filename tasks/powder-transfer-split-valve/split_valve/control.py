"""Scripted commissioning demonstration using only exposed encoders/gauges."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, variant="normal"):
        self.info = PolicyInfo(name=spec.EMBODIMENT + "_demo", action_space=spec.action_space())
        self.variant = variant

    def reset(self, scene):
        self.phase, self.entered = "align", 0.0
        self.closed_since = None

    def act(self, observation):
        q = np.asarray(observation.state["encoders"])
        t = observation.state_time
        seating_z = spec.DOCK_Z - 0.0002
        goals = {
            "align": [0, 0, 1.10, 0, 0, 0],
            "dock": [0, 0, seating_z, 0, 0, 0],
            "lock": [0, 0, seating_z, 30, 0, 0],
            "proof": [0, 0, seating_z, 30, 0, 1],
            "open": [0, 0, seating_z, 30, 90, 0],
            "close": [0, 0, seating_z, 30, 0, 0],
            "unlock": [0, 0, seating_z, 0, 0, 0],
            "undock": [0, 0, 1.10, 0, 0, 0],
        }
        command = np.asarray(goals[self.phase], float)
        observed_goal = command[:3].copy()
        if self.phase not in ("align", "undock"):
            # The biased motor setpoint applies seating force. The native
            # casing/collar determines the actual interface height.
            observed_goal[2] = spec.DOCK_Z
        close = np.max(np.abs(q[:3] - observed_goal)) < 0.0002 and abs(q[3] - command[3]) < 1
        close &= abs(q[4] - command[4]) < 1 and abs(q[5] - command[4]) < 1
        advance = close and t - self.entered >= 0.2
        if self.phase in ("close", "unlock"):
            seated = np.max(np.abs(q[4:])) < 1 and np.min(observation.state["seal_normals"]) >= 0.5
            if seated and self.closed_since is None:
                self.closed_since = t
            if not seated:
                self.closed_since = None
            advance &= self.closed_since is not None and t - self.closed_since >= 1.0
        else:
            self.closed_since = None
        if self.phase == "proof":
            advance = t - self.entered >= 4.0
        if self.variant == "unlocked-proof" and self.phase in ("lock", "proof"):
            command[3] = 0
            command[5] = 1
            advance = False
        phases = list(goals)
        index = phases.index(self.phase)
        if advance and index < len(phases) - 1:
            self.phase = phases[index + 1]
            self.entered = t
        return ActionChunk(actions=[Action(data=command)], control_hz=spec.HZ)
