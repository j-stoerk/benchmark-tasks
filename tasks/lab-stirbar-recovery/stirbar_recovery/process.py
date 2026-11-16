"""Short-range permanent-magnet approximation with finite balanced wrenches."""

import numpy as np

from .spec import matrix


def attraction(tool, bar, quat, enabled=True, tool_quat=(0,0,0,1)):
    """Four softened magnetic-pole interactions with orientation torque.

    Each radial pair force is bounded. No nearest-pole switching, artificial
    capture joint, pose tracking or forced alignment is used.
    """
    tool = np.asarray(tool, float)
    bar = np.asarray(bar, float)
    axis = matrix(quat)[:, 0]
    tool_axis=matrix(tool_quat)[:,2]
    offsets=[sign*.008*axis for sign in (-1,1)]
    tool_offsets=[sign*.004*tool_axis for sign in (-1,1)]
    distance=min(float(np.linalg.norm(tool+t-bar-b)) for t in tool_offsets for b in offsets)
    force = np.zeros(3)
    torque=np.zeros(3)
    if enabled:
        for sign_b,offset in zip((-1,1),offsets):
            for sign_t,toffset in zip((-1,1),tool_offsets):
                delta=bar+offset-tool-toffset
                d=float(np.linalg.norm(delta))
                if d>=.030 or d<1e-9:
                    continue
                magnitude=min(.10,3.6e-6*d/(d*d+.002**2)**1.5)
                taper=1. if d<=.025 else .5*(1+np.cos(np.pi*(d-.025)/.005))
                pair=sign_b*sign_t*magnitude*taper*delta/d
                force+=pair
                torque+=np.cross(offset,pair)
    return force,torque,distance


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

        p, R, q = tool_pose(self.env)
        bar = object_states(self.env, ["stirbar"])["stirbar"]
        f, t, d = attraction(p, bar.pos, bar.quat_xyzw, self.variant == "normal",q)
        self.force = f
        self.distance = d
        wrench(self.env.scene["stirbar"], f.tolist(), t.tolist())
        robot = self.env.scene["robot"]
        reaction = -f
        # The magnetic force acts at the magnet head, not at the rod COM.
        tool_com=p+R@np.array([0.,0.,.15])
        torque=-t-np.cross(np.asarray(bar.pos)-tool_com,f)
        wrench(robot, reaction.tolist(), torque.tolist(), robot.body_names.index("Tool"))

    def readout(self):
        return {"magnetic_force_N": self.force.tolist(), "magnetic_distance_m": self.distance}

    def instruments(self):
        from .native import array

        return [float(np.linalg.norm(array(self.env.scene["tool_contact"].data.net_normal_forces_w)[0]))]
