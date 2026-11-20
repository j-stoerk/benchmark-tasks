"""Push one inert tablet through foil and catch it in a cup."""

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
from blister_parts import Prop, Solid, allow_demo_orientation_matrix, install, rings, show_foil, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X = 0.360
START_Y, WORK_Y = 0.141, -0.015
START_Z, WORK_Z, TRAVEL_Z = TOP + 0.023, TOP + 0.046, TOP + 0.093
TARGET, GRIP = -0.061, 0.057
SUPPORT_Y = (-0.069, 0.032)
POCKETS = (("marked", TARGET), ("spare_one", -0.031), ("spare_two", -0.001), ("spare_three", 0.029))
CUP = (X, WORK_Y + TARGET, TOP)
SILVER = (0.76, 0.78, 0.79)
RED = (0.72, 0.065, 0.045)
WHITE = (0.91, 0.89, 0.82)
BLUE = (0.08, 0.34, 0.62)
MAT = (0.18, 0.20, 0.23)

card = [
    Solid((-0.009, 0, 0), (0.005, 0.144, 0.0012), SILVER),
    Solid((0.009, 0, 0), (0.005, 0.144, 0.0012), SILVER),
    Solid((0, -0.070, 0), (0.013, 0.004, 0.0012), SILVER),
    *(Solid((0, y, 0), (0.013, 0.017, 0.0012), SILVER) for y in (-0.046, -0.016, 0.014)),
    Solid((0, 0.054, 0), (0.013, 0.036, 0.0012), SILVER),
    Solid((0, GRIP, 0.0007), (0.020, 0.021, 0.0002), MAT, collide=False),
]
FOIL = {name: len(card) + i for i, (name, _) in enumerate(POCKETS)}
card.extend(Solid((0, y, -0.00065), (0.0064, 0.00018), SILVER, axis="Z", collide=False) for _, y in POCKETS)

OBJECTS = [
    Prop("pack", (X, START_Y, START_Z), SILVER, size=(0.023, 0.144, 0.0012), mass=0.010, children=tuple(card)),
    Prop("supports", (X, START_Y, TOP), MAT, size=(0.025, 0.144, 0.023), children=tuple(
        Solid((0, y, 0.011), (0.025, 0.006, 0.022), MAT) for y in SUPPORT_Y)),
    Prop("cup", CUP, BLUE, size=(0.062, 0.062, 0.034), children=(
        Solid((0, 0, 0.002), (0.031, 0.004), BLUE, axis="Z"),
        *rings(0.030, 0.002, 0.031, 0.018, BLUE),
    )),
]
for label, y in POCKETS:
    color = RED if label == "marked" else WHITE
    OBJECTS.extend([
        Prop(label + "_cap", (X, START_Y + y, START_Z + 0.007), color, size=(0.012, 0.012, 0.003),
             mass=0.006, attach_to="pack", crush_cap=True,
             children=(Solid((0, 0, 0), (0.0058, 0.0027), color, axis="Z"),)),
        Prop(label + "_tablet", (X, START_Y + y, START_Z + 0.002), WHITE, size=(0.0098, 0.0098, 0.0033),
             mass=0.0010, attach_to="pack", rupture_force=14.0,
             children=(Solid((0, 0, 0), (0.0049, 0.0033), WHITE, axis="Z"),)),
    ])


def reset():
    for index in FOIL.values():
        show_foil("pack", index, True)


install(reset)


def popped(pack, tablet, y):
    basis = quat_xyzw_to_matrix(pack.quat_xyzw)
    offset = basis.T @ (tablet.pos - pack.pos)
    return np.linalg.norm(offset - (0, y, 0.002)) > 0.007


def score(objects):
    pack, target = objects["pack"], objects["marked_tablet"]
    opened = {name for name, y in POCKETS if popped(pack, objects[name + "_tablet"], y)}
    for name in opened:
        show_foil("pack", FOIL[name], False)
    if os.getenv("BLISTER_DIAGNOSTICS"):
        cap = objects["marked_cap"]
        print("BLISTER", np.round(pack.pos, 4), np.round(target.pos, 4), "cap", np.round(cap.pos, 4),
              "open", sorted(opened), np.round(list(tips.values()), 4).tolist(), flush=True)
    caught = (opened == {"marked"} and target.resting() and np.linalg.norm(target.pos[:2] - CUP[:2]) < 0.025
              and TOP + 0.003 <= target.pos[2] < TOP + 0.026)
    if not caught:
        return 0.0
    basis = quat_xyzw_to_matrix(pack.quat_xyzw)
    supports = [basis.T @ (np.array([X, START_Y + y, START_Z]) - pack.pos) for y in SUPPORT_Y]
    returned = (pack.resting() and basis[2, 2] > 0.97
                and all(abs(x) < 0.019 and abs(y) < 0.071 and abs(z) < 0.003 for x, y, z in supports)
                and all(np.linalg.norm(tip - pack.pos) > 0.055 for tip in tips.values()))
    return 1.0 if returned else 0.5


def success(objects):
    return score(objects) == 1.0


def demo():
    allow_demo_orientation_matrix()
    l_yaw = axis_angle_matrix(np.array([0, 1, 0]), -math.radians(70)) @ grip_rotation(-math.pi / 2)
    r_yaw = axis_angle_matrix(np.array([1, 0, 0]), math.radians(22)) @ grip_rotation(0)
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    slight_open = -0.012
    hover = TOP + 0.137
    pick = (X - 0.008, START_Y + GRIP, START_Z + 0.004)
    beside = (X - 0.051, *pick[1:])
    support = (pick[0], WORK_Y + GRIP, WORK_Z)
    carried = (*support[:2], TRAVEL_Z)
    pocket = (X + 0.006, WORK_Y + TARGET - 0.004, WORK_Z + 0.023)
    place = (*pick[:2], START_Z + 0.008)
    press = (*pocket[:2], WORK_Z - 0.008)
    return {
        "left": [
            Waypoint(1.5, (*beside[:2], TOP + 0.072), slight_open, "approach alongside the label end", l_yaw),
            Waypoint(1.1, beside, slight_open, "lower to the edge of the card", l_yaw),
            Waypoint(1.0, pick, slight_open, "place jaws above and below the card", l_yaw),
            Waypoint(0.8, pick, closed, "pinch the top and bottom of the card", l_yaw),
            Waypoint(1.6, (*pick[:2], TRAVEL_Z), closed, "lift the card", l_yaw),
            Waypoint(1.7, carried, closed, "centre the marked pocket over the cup", l_yaw),
            Waypoint(1.2, support, closed, "lower the card to the cup rim", l_yaw),
            Waypoint(8.0, support, closed, "hold the card against the rim during the press", l_yaw),
            Waypoint(1.0, carried, closed, "lift the card clear of the rim", l_yaw),
            Waypoint(1.6, (*pick[:2], TRAVEL_Z), closed, "return to the supports", l_yaw),
            Waypoint(1.1, place, closed, "lower to the supports", l_yaw),
            Waypoint(0.7, place, slight_open, "release the card across both supports", l_yaw),
            Waypoint(0.8, (*beside[:2], place[2]), slight_open, "slide away from the card", l_yaw),
            Waypoint(0.8, (*beside[:2], TOP + 0.072), slight_open, "withdraw", l_yaw),
        ],
        "right": [
            Waypoint(1.7, (0.34, -0.20, hover), closed, "wait clear of the cup", r_yaw),
            Waypoint(7.5, (0.34, -0.20, hover), closed, "wait for the marked pocket", r_yaw),
            Waypoint(1.4, (*pocket[:2], hover), closed, "approach the red blister", r_yaw),
            Waypoint(1.4, pocket, closed, "lower over the centre of the cap", r_yaw),
            Waypoint(1.5, press, closed, "push the tablet through the foil", r_yaw),
            Waypoint(0.7, press, closed, "allow the tablet to drop into the cup", r_yaw),
            Waypoint(1.0, (*pocket[:2], hover), closed, "lift off the card", r_yaw),
            Waypoint(1.0, (0.34, -0.20, hover), opened, "withdraw", r_yaw),
        ],
    }
