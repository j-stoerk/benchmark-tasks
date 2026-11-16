"""Open scripted coating demonstration; genuine LLM policy is separate."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk

from . import spec


class Demo(PolicyBase):
    def __init__(self, variant="normal"):
        self.info = PolicyInfo(
            name=spec.EMBODIMENT + "_demo", action_space=spec.action_space(), checkpoint="native-contact-demo"
        )
        self.variant = variant
        self.reset(None)

    def reset(self, scene):
        self.index = 0
        # Foot datum setpoint is slightly below the glass; native contact and
        # a40N-limited servo determine the actual position and load.
        self.path = [
            ([0, -0.115, 0.88, 0, 0, 0, 0], 20),
            ([0, -0.115, 0.7534, 0, 0, 0, 0], 45),
            ([0, -0.102, 0.7534, 0, 0, 0, 0], 12),
            *[([0, float(y), 0.7534, 0, 0, 0, 0], 3) for y in np.arange(-0.098, 0.109, 0.004)],
            ([0, 0.108, 0.88, 0, 0, 0, 0], 60),
        ]
        self.remaining = self.path[0][1]

    def act(self, observation):
        q = self.path[min(self.index, len(self.path) - 1)][0]
        self.remaining -= 1
        if self.remaining <= 0 and self.index < len(self.path) - 1:
            self.index += 1
            self.remaining = self.path[self.index][1]
        return ActionChunk(actions=[Action(data=np.array(q, float))], control_hz=spec.HZ)
