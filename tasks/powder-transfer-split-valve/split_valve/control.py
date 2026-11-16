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

    def act(self, observation):
        q = np.asarray(observation.state["encoders"])
        t = observation.state_time
        goals = {
            "align": [0, 0, 1.10, 0, 0, 0],
            "dock": [0, 0, 1.0, 0, 0, 0],
            "lock": [0, 0, 1.0, 30, 0, 0],
            "proof": [0, 0, 1.0, 30, 0, 1],
            "open": [0, 0, 1.0, 30, 90, 0],
            "close": [0, 0, 1.0, 30, 0, 0],
            "unlock": [0, 0, 1.0, 0, 0, 0],
            "undock": [0, 0, 1.10, 0, 0, 0],
        }
        command = np.asarray(goals[self.phase], float)
        close = np.max(np.abs(q[:3] - command[:3])) < 0.0002 and abs(q[3] - command[3]) < 1
        close &= abs(q[4] - command[4]) < 1 and abs(q[5] - command[4]) < 1
        advance = close and t - self.entered >= 0.2
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
