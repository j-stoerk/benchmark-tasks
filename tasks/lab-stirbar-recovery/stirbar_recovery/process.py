"""Short-range permanent-magnet approximation with finite balanced wrenches."""

import numpy as np

from .spec import matrix


def attraction(tool, bar, quat, enabled=True):
    """Return endpoint force/torque. No distance-based attachment is created."""
    tool = np.asarray(tool, float)
    bar = np.asarray(bar, float)
    axis = matrix(quat)[:, 0]
    offsets = [sign * 0.010 * axis for sign in (-1, 1)]
    offset = min(offsets, key=lambda off: np.linalg.norm(tool - bar - off))
    displacement = tool - bar - offset
    distance = float(np.linalg.norm(displacement))
    force = np.zeros(3)
    if enabled and 1e-6 < distance < 0.030:
        # 0.2N near contact (~8.5mm centre separation), inverse-fourth-power
        # falloff. This is a declared approximation requiring bench calibration
        # for a chosen commercial retriever, not manufacturer force data.
        magnitude = min(0.20, 0.20 * (0.0085 / max(distance, 0.0085)) ** 4)
        force = magnitude * displacement / distance
    return force, np.cross(offset, force), distance


class Process:
    def __init__(self, env, variant="normal"):
        self.env, self.variant = env, variant
        self.reset()

    def reset(self):
        self.force = np.zeros(3)
        self.distance = 0.0

    def substep(self, control, dt):
        from yam_arena.scene import object_states

        from .native import tool_pose, wrench

        p, R, _ = tool_pose(self.env)
        bar = object_states(self.env, ["stirbar"])["stirbar"]
        f, t, d = attraction(p, bar.pos, bar.quat_xyzw, self.variant == "normal")
        self.force = f
        self.distance = d
        wrench(self.env.scene["stirbar"], f.tolist(), t.tolist())
        robot = self.env.scene["robot"]
        reaction = -f
        # The magnetic force acts at the magnet head, not at the rod COM.
        torque = np.cross(-(R @ np.array([0.0, 0.0, 0.15])), reaction)
        wrench(robot, reaction.tolist(), torque.tolist(), robot.body_names.index("Tool"))

    def readout(self):
        return {"magnetic_force_N": self.force.tolist(), "magnetic_distance_m": self.distance}

    def instruments(self):
        from .native import array

        return [float(np.linalg.norm(array(self.env.scene["tool_contact"].data.net_normal_forces_w)[0]))]
