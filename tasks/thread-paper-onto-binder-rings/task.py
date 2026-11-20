"""Thread a flexible letter sheet through three open binder rings."""

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
from binder_parts import Prop, Solid, allow_demo_orientation_matrix, install, set_ring_target, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SPINE_X, AXLE_Z = 0.399, TOP + 0.013
RING_R = 0.013
WIRE_WIDTH = 0.0028
HOLE_HALF = 0.00325
CONTACT_MARGIN = 0.00025
ROWS = (-0.108, 0.0, 0.108)
BOUNDS = (-0.1395, -0.054, 0.054, 0.1395)
HOLE_X = 0.0955
START_X, START_Z = 0.277, TOP + 0.0245
GRIP_X = 0.335
WHITE = (0.89, 0.88, 0.82)
RED = (0.39, 0.085, 0.075)
SILVER = (0.66, 0.68, 0.73)
BLACK = (0.13, 0.16, 0.19)


def wire(points):
    for p, q in zip(points, points[1:]):
        dx, _, dz = q - p
        yield Solid(tuple(((p + q) / 2).tolist()),
                    (float(np.linalg.norm(q - p)) + 0.001, WIRE_WIDTH, WIRE_WIDTH),
                    SILVER, pitch=-math.degrees(math.atan2(dz, dx)))


def fixed_ring(y):
    return np.array([(0.0, y, z) for z in (0, 0.008, 0.016, 0.026)])


def latch_ring(y):
    return np.array([(RING_R * math.sin(a), y, RING_R * (1 - math.cos(a)))
                     for a in np.linspace(0.14, math.pi, 11)])


base = [
    Solid((-0.119, 0, 0.0035), (0.233, 0.304, 0.007), RED),
    Solid((0.119, 0, 0.0035), (0.233, 0.304, 0.007), RED),
    Solid((0, 0, 0.0085), (0.023, 0.298, 0.004), BLACK),
]
ring_pieces = []
for y in ROWS:
    base.extend(wire(fixed_ring(y) + (0, 0, AXLE_Z - TOP)))
    ring_pieces.extend(wire(latch_ring(y)))
LEVER = np.array([-0.032, 0.153, 0.019])
ring_pieces.append(Solid(tuple(LEVER), (0.020, 0.014, 0.004), SILVER))

OBJECTS = [
    Prop("binder", (SPINE_X, 0, TOP), RED, size=(0.471, 0.304, 0.040), children=tuple(base)),
    Prop("rail", (SPINE_X, 0, AXLE_Z), SILVER, size=(0.036, 0.31, 0.032), mass=0.043,
         ring_rail=True, children=tuple(ring_pieces)),
    Prop("paper_support", (START_X, 0, TOP), BLACK, size=(0.16, 0.27, 0.024), children=tuple(
        Solid((0, y, 0.0155), (0.16, 0.012, 0.018), BLACK) for y in (-0.089, 0.089))),
]

HOLES = []
for index, hole_y in enumerate(ROWS):
    low, high = BOUNDS[index:index+2]
    centre = (low + high) / 2
    half = HOLE_HALF
    hy = hole_y - centre
    HOLES.append(np.array([HOLE_X, hy, 0]))
    xlo, xhi = HOLE_X - half, HOLE_X + half
    ylo, yhi = low - centre, high - centre
    panels = []
    for xa, xb, ya, yb in ((-0.108, xlo, ylo, yhi), (xhi, 0.108, ylo, yhi),
                           (xlo, xhi, ylo, hy - half), (xlo, xhi, hy + half, yhi)):
        panels.append(Solid(((xa + xb) / 2, (ya + yb) / 2, 0), (xb-xa, yb-ya, 0.00045), WHITE))
    OBJECTS.append(Prop(f"page_{index}", (START_X, centre, START_Z), WHITE,
                        size=(0.216, high-low, 0.00045), mass=0.003,
                        previous=f"page_{index-1}" if index else "", edge=(START_X, low, START_Z),
                        children=tuple(panels)))


closed_latch = False
seen_open = False
hole_positions = np.zeros((3, 3))
lever_position = np.zeros(3)


def reset():
    global closed_latch, seen_open
    closed_latch = seen_open = False
    set_ring_target(0.0)


install(reset)


def ring_through_hole(panel, local_hole, y):
    origin = np.array([SPINE_X, 0, AXLE_Z])
    basis = quat_xyzw_to_matrix(panel.quat_xyzw)
    wire_local = (origin + fixed_ring(y) - panel.pos) @ basis - local_hole
    for a, b in zip(wire_local, wire_local[1:]):
        if a[2] <= 0 < b[2] or b[2] <= 0 < a[2]:
            at_plane = a + (b - a) * (-a[2] / (b[2] - a[2]))
            # The square openings and wire cross-sections use box colliders.
            # Use the available clearance on both local axes including contact slop.
            clearance = HOLE_HALF - WIRE_WIDTH / 2 + CONTACT_MARGIN
            if np.max(np.abs(at_plane[:2])) <= clearance:
                return True
    return False


def score(objects):
    global closed_latch, seen_open
    rotation = quat_xyzw_to_matrix(objects["rail"].quat_xyzw)
    lever_position[:] = objects["rail"].pos + rotation @ LEVER
    angle = math.degrees(math.atan2(rotation[0, 2], rotation[2, 2]))
    seen_open |= angle > 36
    if seen_open and not closed_latch and angle < 28:
        set_ring_target(-42.0)
        closed_latch = True
    page = [objects[f"page_{i}"] for i in range(3)]
    for index, (panel, hole) in enumerate(zip(page, HOLES)):
        hole_positions[index] = panel.pos + quat_xyzw_to_matrix(panel.quat_xyzw) @ hole
    if any(np.linalg.norm(a.pos-b.pos) > 0.109 for a, b in zip(page, page[1:])):
        return 0.0
    threaded = sum(ring_through_hole(section, hole, y) for section, hole, y in zip(page, HOLES, ROWS))
    if os.getenv("BINDER_DIAGNOSTICS"):
        print("BINDER", round(angle, 2), "holes", np.round(hole_positions, 4).tolist(), "threaded", threaded,
              "tips", np.round(list(tips.values()), 4).tolist(), flush=True)
    if threaded < 2:
        return 0.25 if threaded else 0.0
    clear = all(min(np.linalg.norm(tip - state.pos) for state in page) > 0.085 for tip in tips.values())
    flat = all(quat_xyzw_to_matrix(p.quat_xyzw)[2, 2] > 0.9 for p in page)
    return 1.0 if threaded == 3 and angle < 3 and clear and flat and all(p.resting() for p in page) else 0.5


def success(objects):
    return score(objects) == 1.0


class AlignHole:
    """Correct the bound page's lateral pose while the grasp moves vertically."""

    def __init__(self, duration, side, index, height, finger, label, yaw):
        self.duration, self.side, self.index, self.height = duration, side, index, height
        self.finger, self.label, self.yaw = finger, label, yaw

    @property
    def pos(self):
        tip = tips.get(self.side)
        nominal_x = GRIP_X + SPINE_X - START_X - HOLE_X - (0.010 if self.side == "left" else 0.0)
        nominal_y = 0.129 if self.side == "left" else -0.139
        if tip is None:
            return nominal_x, nominal_y, self.height
        goal = np.array([SPINE_X + 0.0009, ROWS[self.index] - 0.0006])
        offsets = np.clip(1.8 * (goal - hole_positions[self.index, :2]), -0.010, 0.010)
        offset = PAD_TIP_PAST_TCP * self.yaw[:2, 2]
        x, y = tip[:2] - offset + offsets
        x = np.clip(x, nominal_x - 0.031, nominal_x + 0.031)
        y = np.clip(y, nominal_y - 0.022, nominal_y + 0.022)
        return float(x), float(y), self.height


class PressLever:
    duration, finger, label, yaw = 2.0, FINGER_CLOSED, "depress the one-touch lever", -math.pi / 2

    @property
    def pos(self):
        x, y, z = lever_position
        if not 0.35 < x < 0.40:
            return SPINE_X - 0.014, LEVER[1], AXLE_Z + 0.016
        return float(x), float(y), float(max(AXLE_Z + 0.010, z - 0.011))


def demo():
    allow_demo_orientation_matrix()
    opened = -0.011
    result = {}
    for side, sign in (("left", 1), ("right", -1)):
        yaw = -sign * math.pi/2
        tilt = axis_angle_matrix(np.array([1, 0, 0]), -sign * math.radians(70)) @ grip_rotation(0)
        y = 0.129 if side == "left" else -0.139
        grasp_x = GRIP_X - (0.010 if side == "left" else 0.0)
        beside_y = sign * 0.193
        index = 2 if side == "left" else 0
        thread_x = grasp_x + SPINE_X - START_X - HOLE_X
        hover = TOP + 0.100
        hold_z = AXLE_Z + 0.003
        result[side] = [
            Waypoint(1.8, (grasp_x, beside_y, hover), opened, "approach the page edge", tilt),
            Waypoint(1.1, (grasp_x, beside_y, START_Z + 0.002), opened, "lower outside the page", tilt),
            Waypoint(1.0, (grasp_x, y, START_Z + 0.002), opened, "slide pads above and below the edge", tilt),
            Waypoint(0.8, (grasp_x, y, START_Z + 0.002), FINGER_CLOSED, "hold both page edges", tilt),
            Waypoint(1.4, (grasp_x, y, hover), FINGER_CLOSED, "lift the intact page", tilt),
            Waypoint(1.8, (thread_x, y, hover), FINGER_CLOSED, "centre the three holes over the tips", tilt),
            AlignHole(1.8, side, index, hover, FINGER_CLOSED, "register the holes over the ring tips", tilt),
            AlignHole(2.7, side, index, hold_z, FINGER_CLOSED, "lower the page over the receiver tips", tilt),
            AlignHole(1.4, side, index, hold_z, FINGER_CLOSED, "settle all three holes around the receiver tips", tilt),
        ]
        if side == "left":
            result[side].extend([
                AlignHole(0.8, side, index, hold_z, opened, "release the left edge once threaded", tilt),
                Waypoint(1.2, (GRIP_X, beside_y, hover), FINGER_OPEN, "withdraw from the page", tilt),
                Waypoint(1.7, (SPINE_X - 0.012, LEVER[1], hover), FINGER_CLOSED, "approach the one-touch lever", yaw),
                PressLever(),
                Waypoint(1.2, (SPINE_X - 0.012, 0.195, hover), FINGER_OPEN, "withdraw from the binder", yaw),
            ])
        else:
            result[side].extend([
                AlignHole(7.1, side, index, hold_z, FINGER_CLOSED, "keep the page threaded during closure", tilt),
                AlignHole(0.8, side, index, hold_z, opened, "release the page", tilt),
                Waypoint(1.3, (GRIP_X, beside_y, hover), FINGER_OPEN, "withdraw from the binder", tilt),
            ])
    return result
