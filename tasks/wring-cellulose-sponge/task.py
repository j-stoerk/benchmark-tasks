"""Twist a wet sponge over a basin and leave it on the draining rack."""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from sponge_parts import Prop, Solid, enable_demo_wrist_roll, install, set_water_height, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
TRAY_X, BASIN_X = 0.448, 0.295
WORK_X = 0.303
RACK_Z = TOP + 0.032
WORK_Z = TOP + 0.094
ENDS = {"left": 0.053, "right": -0.053}
SPONGE = (0.87, 0.71, 0.19)
RACK = (0.24, 0.27, 0.30)
BASIN = (0.13, 0.28, 0.42)
WATER = (0.26, 0.53, 0.76)
BASIN_W, BASIN_L = 0.198, 0.284
INNER_W, INNER_L = 0.192, 0.278

basin_parts = [Solid((0, 0, 0.0025), (BASIN_W, BASIN_L, 0.005), BASIN)]
for sign in (-1, 1):
    basin_parts.extend([
        Solid((sign * 0.0975, 0, 0.024), (0.003, BASIN_L, 0.038), BASIN),
        Solid((0, sign * 0.1405, 0.024), (BASIN_W, 0.003, 0.038), BASIN),
    ])
WATER_INDEX = len(basin_parts)
basin_parts.append(Solid((0, 0, 0.006), (INNER_W, INNER_L, 0.00008), WATER, collide=False))

OBJECTS = [
    Prop("basin", (BASIN_X, 0, TOP), BASIN, size=(BASIN_W, BASIN_L, 0.044), children=tuple(basin_parts)),
    Prop("rack", (TRAY_X, 0, TOP), RACK, size=(0.10, 0.12, 0.023), children=tuple(
        Solid((0, y, 0.010), (0.103, 0.009, 0.020), RACK) for y in (-0.030, 0.030))),
    Prop("middle", (TRAY_X, 0, RACK_Z), SPONGE, size=(0.088, 0.048, 0.024), mass=0.037,
         children=(Solid((0, 0, 0), (0.088, 0.049, 0.024), SPONGE),)),
]
for side, y in ENDS.items():
    OBJECTS.append(Prop(side + "_end", (TRAY_X, y, RACK_Z), SPONGE,
                        size=(0.088, 0.047, 0.024), mass=0.031, hinge_to="middle",
                        children=(Solid((0, -math.copysign(0.002, y), 0), (0.088, 0.051, 0.024), SPONGE),)))


def rotation_about_y(reference, moving):
    relative = quat_xyzw_to_matrix(reference.quat_xyzw).T @ quat_xyzw_to_matrix(moving.quat_xyzw)
    return math.atan2(relative[0, 2], relative[2, 2])


class Wringing:
    def reset(self):
        self.wet_g = 90.0
        self.caught_g = 0.0
        self.spilled_g = 0.0
        self.peak_twist = 0.0
        self.steps = 0
        self.render()

    def render(self):
        height = self.caught_g / (1e6 * INNER_W * INNER_L)
        set_water_height("basin", WATER_INDEX, INNER_W, INNER_L, height, 0.005)

    def advance(self, objects):
        self.steps += 1
        centre = objects["middle"]
        sections = [objects[f"{side}_end"] for side in ENDS]
        a, b = (rotation_about_y(centre, section) for section in sections)
        counter_twist = abs(a - b) if a * b < 0 else 0.0
        if counter_twist < 0.16:
            self.peak_twist = 0.0
        increase = max(0.0, counter_twist - max(self.peak_twist, 0.30))
        self.peak_twist = max(self.peak_twist, counter_twist)
        if increase > 0:
            expelled = min(self.wet_g, 35.0 * increase * self.wet_g / 90.0)
            self.wet_g -= expelled
            outlets = [centre, *sections]
            inside = sum(abs(section.pos[0] - BASIN_X) < INNER_W / 2
                         and abs(section.pos[1]) < INNER_L / 2
                         and section.pos[2] > TOP + 0.006 for section in outlets)
            caught = expelled * inside / len(outlets)
            self.caught_g += caught
            self.spilled_g += expelled - caught
            self.render()
        if os.getenv("SPONGE_DIAGNOSTICS") and self.steps % 20 == 0:
            print("SPONGE", self.steps, "strain", round(counter_twist, 2), "water", round(self.caught_g, 1),
                  "spill", round(self.spilled_g, 1),
                  "middle", np.round(centre.pos, 3).tolist(), "tips", np.round(list(tips.values()), 3).tolist(), flush=True)


wringing = Wringing()
install(wringing.reset)


def success(objects):
    centre = objects["middle"]
    return (wringing.caught_g >= 40 and wringing.spilled_g <= 5 and centre.resting()
            and np.linalg.norm(centre.pos[:2] - (TRAY_X, 0)) < 0.023
            and abs(centre.pos[2] - RACK_Z) < 0.008
            and all(np.linalg.norm(tip - centre.pos) > 0.09 for tip in tips.values()))


def score(objects):
    wringing.advance(objects)
    if wringing.spilled_g > 5:
        return 0.0
    if success(objects):
        return 1.0
    if wringing.caught_g >= 40:
        return 0.5
    return 0.25 if wringing.caught_g >= 20 else 0.0


def demo():
    enable_demo_wrist_roll()
    hover = TOP + 0.111
    result = {}
    for side, y in ENDS.items():
        sign = math.copysign(1, y)
        down = grip_rotation(-sign * math.pi / 2)
        roll = axis_angle_matrix(np.array([0, 1, 0]), sign * math.radians(36)) @ down
        grasp = (TRAY_X, y, RACK_Z + 0.004)
        lifted = (TRAY_X, y, WORK_Z + 0.004)
        over = (WORK_X, y, WORK_Z + 0.004)
        wring = [
            Waypoint(2.3, over, FINGER_CLOSED, "counter-rotate the ends", roll),
            Waypoint(0.5, over, FINGER_CLOSED, "let water run into the basin", roll),
            Waypoint(2.2, over, FINGER_CLOSED, "straighten the sponge while keeping both grips", down),
            Waypoint(0.4, over, FINGER_CLOSED, "let the sponge relax", down),
        ]
        result[side] = [
            Waypoint(1.6, (TRAY_X, y, hover), FINGER_OPEN, "approach one end", down),
            Waypoint(1.5, grasp, FINGER_OPEN, "straddle the wet sponge", down),
            Waypoint(0.8, grasp, FINGER_CLOSED, "grip the sponge end", down),
            Waypoint(1.5, lifted, FINGER_CLOSED, "lift together", down),
            Waypoint(1.7, over, FINGER_CLOSED, "move over the basin", down),
            *(wring * 5),
            Waypoint(1.7, lifted, FINGER_CLOSED, "return to the rack", down),
            Waypoint(1.2, grasp, FINGER_CLOSED, "set the sponge down", down),
            Waypoint(0.7, grasp, FINGER_OPEN, "release both ends", down),
            Waypoint(1.3, (TRAY_X, y + sign * 0.083, hover), FINGER_OPEN, "withdraw", down),
        ]
    return result
