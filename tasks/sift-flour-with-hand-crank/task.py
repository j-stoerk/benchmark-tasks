"""Hold an ordinary rotary sifter over a mixing bowl and work its hand crank."""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from sifter_parts import Prop, Solid, allow_demo_orientation_matrix, install, rings, set_powder_height, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
RETURN = (0.372, 0.157, TOP + 0.056)
BOWL = (0.338, -0.024, TOP)
WORK = (*BOWL[:2], TOP + 0.101)
PALE = (0.62, 0.67, 0.72)
BLUE = (0.07, 0.25, 0.47)
FLOUR = (0.94, 0.89, 0.73)
WOOD = (0.54, 0.29, 0.12)
RUBBER = (0.26, 0.24, 0.19)
RADIUS = 0.054
CRANK = 0.031
HANDLE = (0.0, 0.081, 0.018)
KNOB_Y = -0.082
AXLE_Z = -0.013
STOCK_G = 60.0
OUTLETS = np.array([(0.0, 0.0, -0.054)] + [
    (0.036 * math.cos(a), 0.036 * math.sin(a), -0.054)
    for a in np.linspace(0.0, math.tau, 6, endpoint=False)
])

barrel = [*rings(RADIUS, 0.002, 0.107, 0, PALE)]
barrel.extend([
    Solid((0, 0, -0.053), (RADIUS, 0.003), PALE, axis="Z"),
    Solid((0, 0.069, 0.018), (0.022, 0.047, 0.018), WOOD),
    Solid((0, 0.070, -0.013), (0.008, 0.033, 0.005), PALE),
])
FLOUR_INDEX = len(barrel)
barrel.append(Solid((0, 0, -0.032), (0.049, 0.035), FLOUR, axis="Z", collide=False))
bowl = [Solid((0, 0, 0.004), (0.095, 0.008), BLUE, axis="Z"), *rings(0.095, 0.004, 0.037, 0.020, BLUE)]
for side in (-1, 1):
    bowl.append(Solid((side * 0.048, 0, 0.0425), (0.008, 0.177, 0.008), WOOD))
    for end in (-1, 1):
        bowl.append(Solid((side * 0.048, end * 0.047, 0.054), (0.012, 0.012, 0.015), WOOD))
BOWL_INDEX = len(bowl)
bowl.append(Solid((0, 0, 0.008), (0.079, 0.00008), FLOUR, axis="Z", collide=False))

OBJECTS = [
    Prop("sifter", WORK, PALE, mass=0.29, size=(0.108, 0.16, 0.111), children=tuple(barrel)),
    Prop("rotor", (WORK[0], WORK[1], WORK[2] + AXLE_Z), PALE, mass=0.035, size=(0.09, 0.169, 0.075),
         hinge_to="sifter", hinge_friction=0.00015, children=(
        Solid((0, -0.007, 0), (0.003, 0.124), PALE, axis="Y"),
        Solid((0, -0.064, CRANK / 2), (0.008, 0.006, CRANK), PALE),
        Solid((0, KNOB_Y, CRANK), (0.011, 0.032), WOOD, axis="Y"),
        Solid((0, KNOB_Y - 0.004, CRANK), (0.0125, 0.020), RUBBER, axis="Y"),
        Solid((0, 0, -0.026), (0.047, 0.094, 0.003), PALE),
    )),
    Prop("bowl", BOWL, BLUE, size=(0.194, 0.194, 0.047), children=tuple(bowl)),
]


class Flow:
    def reset(self):
        self.remaining = STOCK_G
        self.caught = 0.0
        self.spill = 0.0
        self.old_angle = None
        self.support_orientation = None
        self.axle = None
        self.body_pos = None
        self.steps = 0

    def render(self):
        set_powder_height("sifter", FLOUR_INDEX, self.remaining / STOCK_G, 0.035, -0.050)
        set_powder_height("bowl", BOWL_INDEX, self.caught / STOCK_G, 0.012, 0.008)

    def step(self, objects):
        self.steps += 1
        sifter, rotor = objects["sifter"], objects["rotor"]
        support = quat_xyzw_to_matrix(sifter.quat_xyzw)
        angle_pose = support.T @ quat_xyzw_to_matrix(rotor.quat_xyzw)
        angle = math.atan2(angle_pose[0, 2], angle_pose[2, 2])
        change = math.remainder(angle - self.old_angle, math.tau) if self.old_angle is not None else 0.0
        self.old_angle = angle
        self.support_orientation = support
        self.axle = rotor.pos.copy()
        self.body_pos = sifter.pos.copy()
        outlets = sifter.pos + OUTLETS @ support.T
        within_rim = np.linalg.norm(outlets[:, :2] - BOWL[:2], axis=1) < 0.091
        fraction = float(np.mean(within_rim & (outlets[:, 2] >= TOP + 0.037)))
        dumping = support[2, 2] < math.cos(math.radians(59))
        through_mesh = 3.8 * max(0.0, abs(change) - 0.004)
        amount = min(self.remaining, 0.8 if dumping else through_mesh)
        self.remaining -= amount
        caught = 0.0 if dumping else amount * fraction
        self.caught += caught
        self.spill += amount - caught
        if amount > 0.0:
            self.render()
        if os.getenv("SIFTER_DIAGNOSTICS") and self.steps % 20 == 0:
            knob = rotor.pos + quat_xyzw_to_matrix(rotor.quat_xyzw) @ (0.0, KNOB_Y, CRANK)
            print("SIFTER", self.steps, "angle", round(angle, 2), "captured", round(self.caught, 1),
                  "spill", round(self.spill, 1), "over", round(fraction, 2), "pos", np.round(sifter.pos, 3).tolist(),
                  "knob", np.round(knob, 3).tolist(), "tips", np.round(list(tips.values()), 3).tolist(), flush=True)


flow = Flow()
flow.reset()


class CrankWaypoint:
    """Scripted gripper targets follow the measured crank as it turns under load."""

    def __init__(self, duration, finger, label, orientation, angle=None):
        self.duration, self.finger, self.label = duration, finger, label
        self.grip, self.angle = orientation, angle

    @property
    def yaw(self):
        angle = flow.old_angle if self.angle is None else self.angle
        turn = float(np.clip(0.40 * (angle or 0.0), -0.65, 0.65))
        return axis_angle_matrix(np.array((0.0, 1.0, 0.0)), turn) @ self.grip

    @property
    def pos(self):
        if flow.axle is None:
            return WORK[0], WORK[1] + KNOB_Y, WORK[2] + AXLE_Z + CRANK
        angle = flow.old_angle if self.angle is None else self.angle
        knob = flow.axle + flow.support_orientation @ (CRANK * math.sin(angle), KNOB_Y - 0.006, CRANK * math.cos(angle))
        return tuple(knob - (PAD_TIP_PAST_TCP - 0.004) * self.yaw[:, 2])


class BraceSifter:
    """Keep the loaded vessel centred as the crank transmits reaction torque."""

    finger, yaw, label = FINGER_CLOSED, -math.pi / 2, "keep the sifter level under load"

    def __init__(self, duration):
        self.duration = duration

    @property
    def pos(self):
        nominal = np.array(WORK) + HANDLE + (0, 0, 0.005)
        tip = tips.get("left")
        if flow.body_pos is None or tip is None:
            return tuple(nominal)
        target = np.array(WORK)
        correction = np.clip(2.4 * (target - flow.body_pos), -0.044, 0.044)
        desired = tip + correction + (0, 0, PAD_TIP_PAST_TCP)
        return tuple(nominal + np.clip(desired - nominal, -0.031, 0.031))


def reset():
    flow.reset()
    flow.render()


install(reset)


def success(objects):
    s = objects["sifter"]
    return (
        flow.caught >= 55 and flow.spill <= 3 and s.resting()
        and np.linalg.norm(s.pos[:2] - RETURN[:2]) < 0.035 and abs(s.pos[2] - RETURN[2]) < 0.012
        and all(np.linalg.norm(tip - s.pos) > 0.10 for tip in tips.values())
    )


def score(objects):
    flow.step(objects)
    if flow.spill > 3:
        return 0.0
    return 1.0 if success(objects) else 0.5 if flow.caught >= 55 else 0.25 if flow.caught >= 30 else 0.0


def demo():
    allow_demo_orientation_matrix()
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    lh, rh = -math.pi/2, math.pi/2
    grip0 = tuple(map(float, np.array(RETURN) + HANDLE + [0, 0, 0.005]))
    grip1 = tuple(map(float, np.array(WORK) + HANDLE + [0, 0, 0.005]))
    hover = TOP + 0.174
    carry_z = grip1[2] + 0.039
    lean = axis_angle_matrix(np.array([1, 0, 0]), math.radians(57)) @ grip_rotation(rh)
    above_knob = (WORK[0], WORK[1] + KNOB_Y - 0.030, hover)
    sweep = (0.74, 1.48, 0.75, 0.0, -0.75, -1.48, -0.75, 0.0)
    cycle = [CrankWaypoint(0.53, closed, "drive the crank through an alternating stroke", lean, angle)
             for angle in sweep]
    cycle.extend([
        CrankWaypoint(0.65, -0.017, "reopen around the actual knob", lean),
        CrankWaypoint(0.75, closed, "renew the crank grip", lean),
    ])
    turns = cycle * 7
    arms = {
        "left": [
            Waypoint(1.8, (*grip1[:2], hover), opened, "approach the sifter handle", lh),
            Waypoint(1.4, grip1, opened, "straddle its handle", lh),
            Waypoint(0.9, grip1, closed, "grasp the loaded sifter", lh),
            BraceSifter(4.7 + sum(w.duration for w in turns)),
            Waypoint(1.1, (*grip1[:2], carry_z), closed, "lift the empty sifter", lh),
            Waypoint(2.0, (*grip0[:2], carry_z), closed, "bring it back to the bench", lh),
            Waypoint(1.3, grip0, closed, "put the sifter down", lh),
            Waypoint(0.8, grip0, opened, "release it", lh),
            Waypoint(1.2, (*grip0[:2], hover), opened, "withdraw", lh),
        ],
        "right": [
            Waypoint(2.2, (0.32, -0.202, hover), opened, "wait outside the work area", rh),
            Waypoint(2.1, (0.32, -0.202, hover), opened, "wait until the sifter is supported", rh),
            Waypoint(1.4, above_knob, -0.017, "approach the crank", lean),
            CrankWaypoint(1.2, -0.017, "lower to the wooden knob", lean),
            CrankWaypoint(0.9, closed, "grasp the knob", lean),
            *turns,
            CrankWaypoint(0.8, opened, "let go of the crank", lean),
            Waypoint(1.4, (0.32, -0.202, hover), opened, "withdraw", rh),
        ],
    }
    return arms
