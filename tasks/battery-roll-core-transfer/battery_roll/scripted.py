"""Physical waypoint demo with bounded joint-error integral compensation.

The native arms use compliant position actuators. Only observed joint positions
are used here; no object poses, simulator handles, attachments or state resets.
"""

from dataclasses import replace

import numpy as np
from inspect_robots_yam.packing import STATE_KEY
from yam_arena import ik
from yam_arena.scripted import YamScriptedPolicy


class RollScriptedPolicy(YamScriptedPolicy):
    def __init__(self, hold_s=6.0):
        super().__init__(hold_s=hold_s)
        self.info = replace(self.info, name="battery_roll_scripted")
        self._integral = np.zeros((2, 6))

    def reset(self, scene):
        super().reset(scene)
        self._integral[:] = 0

    def act(self, observation):
        chunk = super().act(observation)
        action = chunk.actions[0]
        nominal = np.asarray(action.data).reshape(2, 7)
        measured = np.asarray(observation.state[STATE_KEY]).reshape(2, 7)
        if not np.isfinite(measured).all():
            raise ValueError("The scripted feedback controller needs finite measured joints")
        self._integral = np.clip(self._integral + 0.8 * self._dt * (nominal[:, :6] - measured[:, :6]), -0.15, 0.15)
        corrected = nominal.copy()
        corrected[:, :6] = np.clip(nominal[:, :6] + self._integral, ik.LOWER, ik.UPPER)
        return replace(chunk, actions=[replace(action, data=corrected.ravel())])


def battery_roll_scripted(**kwargs):
    return RollScriptedPolicy(**kwargs)
