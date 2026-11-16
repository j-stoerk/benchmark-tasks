"""Scripted native retrieval attempt, separate from external LLM decisions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, variant="normal"):
        self.info = PolicyInfo(
            name=spec.EMBODIMENT + "_demo", action_space=spec.action_space(), checkpoint="native-retrieval-demo"
        )
        self.reset(None)

    def reset(self, scene):
        self.path = [
            ([0, 0, 0.90, 0, 0, 0, 0], 60),
            ([0, 0, 0.780, 0, 0, 0, 0], 40),
            ([0, 0, 0.766, 0, 0, 0, 0], 60),
            ([0, 0, 0.784, 0, 0, 0, 0], 80),
            *[([0, 0, float(z), 0, 0, 0, 0], 8) for z in np.arange(0.786, 0.932, 0.002)],
            ([0, 0, 0.94, 0, 0, 0, 0], 120),
        ]
        self.index = 0
        self.remaining = self.path[0][1]

    def act(self, observation):
        q = self.path[self.index][0]
        self.remaining -= 1
        if self.remaining <= 0 and self.index < len(self.path) - 1:
            self.index += 1
            self.remaining = self.path[self.index][1]
        return ActionChunk(actions=[Action(data=np.array(q, float))], control_hz=spec.HZ)
