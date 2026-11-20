"""Seat a desktop tape reel and bring its leader to the tear bar."""

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
from tape_parts import Prop, Solid, allow_demo_orientation_matrix, install, tack, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X = 0.346
DISPENSER_Y = -0.060
AXLE = (X, DISPENSER_Y + 0.065, TOP + 0.046)
START = (X, 0.228, TOP + 0.035)
LAND = (X, DISPENSER_Y - 0.062, TOP + 0.034)
BLACK = (0.10, 0.12, 0.15)
STEEL = (0.58, 0.63, 0.67)
TAPE = (0.74, 0.82, 0.63)
TAB = (0.88, 0.72, 0.25)
SPOOL = (0.22, 0.28, 0.34)
N = 11
LENGTH = 0.0135
RADIUS = 0.0272

deck = [Solid((0, 0.009, 0.006), (0.068, 0.163, 0.012), BLACK),
        Solid((0, -0.062, 0.016), (0.051, 0.021, 0.016), BLACK),
        Solid((0, -0.060, 0.0295), (0.051, 0.012, 0.012), BLACK),
        Solid((0, -0.061, LAND[2] - TOP), (0.028, 0.014, 0.002), STEEL)]
for sign in (-1, 1):
    x = sign * 0.024
    deck.extend([
        Solid((x, 0.062, 0.025), (0.015, 0.037, 0.037), BLACK),
        Solid((x, 0.054, 0.049), (0.015, 0.005, 0.016), BLACK),
        Solid((x, 0.076, 0.049), (0.015, 0.005, 0.016), BLACK),
    ])

stand = []
for sign in (-1, 1):
    x = sign * 0.022
    stand.extend([
        Solid((x, 0, 0.0145), (0.006, 0.032, 0.029), BLACK),
        Solid((x, -0.0105, 0.031), (0.006, 0.005, 0.008), BLACK),
        Solid((x, 0.0105, 0.031), (0.006, 0.005, 0.008), BLACK),
    ])

OBJECTS = [
    Prop("dispenser", (X, DISPENSER_Y, TOP), BLACK, size=(0.068, 0.163, 0.056), children=tuple(deck)),
    Prop("refill_stand", (*START[:2], TOP), BLACK, size=(0.050, 0.032, 0.036), children=tuple(stand)),
    Prop("reel", START, SPOOL, size=(0.056, 0.05, 0.05), mass=0.068, children=(
        Solid((0, 0, 0), (0.025, 0.019), TAPE, axis="X"),
        Solid((0, 0, 0), (0.006, 0.056), SPOOL, axis="X"),
        Solid((-0.0115, 0, 0), (0.013, 0.004), SPOOL, axis="X"),
        Solid((0.0115, 0, 0), (0.013, 0.004), SPOOL, axis="X"),
    )),
]

arc = 2 * math.asin(LENGTH / (2 * RADIUS))
start_angle = -0.53 + N * arc
angles = [start_angle - i * arc for i in range(N + 1)]
points = [np.array(START) + (0, RADIUS * math.sin(a), RADIUS * math.cos(a)) for a in angles]
points[-1] += (0, 0.008 * math.sin(angles[-1]), 0.008 * math.cos(angles[-1]))
segment_lengths = []
for i in range(N):
    colour = TAB if i == N - 1 else TAPE
    vector = points[i + 1] - points[i]
    span = float(np.linalg.norm(vector))
    segment_lengths.append(span)
    OBJECTS.append(Prop(f"film_{i}", tuple(map(float, (points[i] + points[i + 1]) / 2)), colour,
                        size=(0.019, span - 0.00015, 0.0006), mass=0.00015,
                        angle_x=math.atan2(vector[2], vector[1]),
                        previous="reel" if i == 0 else f"film_{i-1}", edge=tuple(map(float, points[i])),
                        can_tack=i < N - 1, flip_width=True,
                        children=(Solid((0, 0, 0), (0.019, span - 0.00015, 0.0006), colour),)))

leader_pos = np.array([X, AXLE[1] - 0.020, AXLE[2] + 0.025])
leader_normal = np.array([0.0, 0.7, 0.7])
landing_segments = [f"film_{index}" for index in range(N - 1)]
landed = None
resting_tail = None
settled_steps = 0


def reset():
    global landed
    landed = None
    update_tail_stability(None)
    for name in landing_segments:
        tack(name)


install(reset)


def film_intact(objects):
    for i in range(N - 1):
        a, b = objects[f"film_{i}"], objects[f"film_{i+1}"]
        if np.linalg.norm(a.pos - b.pos) > (segment_lengths[i] + segment_lengths[i + 1]) * 0.56:
            return False
    return np.linalg.norm(objects["film_0"].pos - objects["reel"].pos) < RADIUS + 0.011


def adhesive_touches_cutter(section, length):
    basis = quat_xyzw_to_matrix(section.quat_xyzw)
    if basis[2, 2] < 0.85 or np.linalg.norm(section.lin_vel) > 0.16:
        return False
    samples = np.array([(0.0, t * length, -0.0003) for t in (-0.42, -0.21, 0, 0.21, 0.42)])
    face = section.pos + samples @ basis.T
    offset = np.abs(face - (np.array(LAND) + (0, 0, 0.001)))
    return bool(np.any(np.all(offset <= (0.013, 0.0068, 0.0015), axis=1)))


def update_tail_stability(tail):
    global resting_tail, settled_steps
    if tail is None:
        resting_tail, settled_steps = None, 0
    elif resting_tail is None or np.linalg.norm(tail - resting_tail) > 0.003:
        resting_tail, settled_steps = tail.copy(), 1
    else:
        settled_steps += 1


def score(objects):
    global landed
    reel = objects["reel"]
    lead = objects[f"film_{N-1}"]
    leader_pos[:] = lead.pos
    lead_basis = quat_xyzw_to_matrix(lead.quat_xyzw)
    leader_normal[:] = lead_basis[:, 2]
    basis = quat_xyzw_to_matrix(reel.quat_xyzw)
    seated = np.linalg.norm(reel.pos - AXLE) < 0.007 and abs(basis[0, 0]) > 0.97
    intact = film_intact(objects)
    if os.getenv("TAPE_DIAGNOSTICS"):
        print("TAPE", np.round(reel.pos, 3).tolist(), np.round(lead.pos, 3).tolist(),
              "normal", np.round(leader_normal, 2).tolist(), "seated", seated, "intact", intact,
              "tips", np.round(list(tips.values()), 3).tolist(), flush=True)
    if not seated or not intact:
        update_tail_stability(None)
        return 0.0
    tail = lead.pos + lead_basis @ (0, segment_lengths[-1] / 2, 0)
    across = abs(tail[0] - LAND[0]) < 0.009 and tail[1] < LAND[1] - 0.002 and abs(tail[2] - LAND[2]) < 0.009
    if not across:
        update_tail_stability(None)
        return 0.25
    if landed is None:
        for index, name in enumerate(landing_segments):
            section = objects[name]
            if adhesive_touches_cutter(section, segment_lengths[index]):
                tack(name, section)
                landed = name
                if os.getenv("TAPE_DIAGNOSTICS"):
                    print("LAND", name, "adhesive contact", np.round(section.pos, 4).tolist(), flush=True)
                break
    clear = all(np.linalg.norm(tip - lead.pos) > 0.065 for tip in tips.values())
    attached = landed is not None and adhesive_touches_cutter(objects[landed], LENGTH)
    update_tail_stability(tail if attached and reel.resting() and clear else None)
    if os.getenv("TAPE_DIAGNOSTICS") and clear:
        print("REST", landed, "attached", attached, "steady steps", settled_steps, flush=True)
    return 1.0 if settled_steps >= 20 else 0.5


def success(objects):
    return settled_steps >= 20


class LeaderWaypoint:
    """The demo approaches the actual end of the flexible tape."""

    def __init__(self, duration, finger, label, beside=0.0, above=0.0, side_bias=0.004):
        self.duration, self.finger, self.label = duration, finger, label
        self.beside, self.above, self.side_bias = beside, above, side_bias

    @property
    def yaw(self):
        tilt = axis_angle_matrix(np.array([0, 1, 0]), math.radians(-58))
        normal_roll = float(np.clip(math.atan2(leader_normal[2], leader_normal[1]) % math.pi,
                                    0.05, 0.24))
        roll = axis_angle_matrix(np.array([1, 0, 0]), normal_roll)
        return roll @ tilt @ grip_rotation(0)

    @property
    def pos(self):
        bias = (self.beside - 0.006, self.side_bias, self.above + 0.008)
        return tuple(leader_pos - (PAD_TIP_PAST_TCP - 0.004) * self.yaw[:, 2] + bias)


def demo():
    allow_demo_orientation_matrix()
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    left, right = -math.pi / 2, math.pi / 2
    h = TOP + 0.149
    pick = (*START[:2], START[2] + 0.014)
    above = (AXLE[0] + 0.020, AXLE[1], h)
    place = (*above[:2], AXLE[2] + 0.012)
    side_grip = axis_angle_matrix(np.array([0, 1, 0]), math.radians(-58)) @ grip_rotation(0)
    tab_pose = axis_angle_matrix(np.array([1, 0, 0]), math.radians(34)) @ side_grip
    flat_grip = axis_angle_matrix(np.array([1, 0, 0]), math.radians(78)) @ side_grip
    pull = (X - 0.012, LAND[1] - 0.013, TOP + 0.051)
    finish = (X - 0.012, LAND[1] - 0.016, LAND[2] + 0.0006)
    return {
        "left": [
            Waypoint(1.6, (*pick[:2], h), opened, "approach the reel", left),
            Waypoint(1.3, pick, opened, "straddle the reel", left),
            Waypoint(0.8, pick, closed, "grip both hub ends", left),
            Waypoint(1.3, (*pick[:2], h), closed, "lift the refill", left),
            Waypoint(1.7, above, closed, "align the journals over the cradle", left),
            Waypoint(1.7, place, closed, "lower into the two slots", left),
            Waypoint(0.7, place, opened, "release the reel in the cradle", left),
            Waypoint(1.1, (X, 0.177, h), opened, "withdraw", left),
        ],
        "right": [
            Waypoint(10.4, (0.33, -0.177, h), opened, "wait for the reel", right),
            LeaderWaypoint(1.4, -0.005, "approach above and outside the starter tab", beside=-0.041, above=0.072),
            LeaderWaypoint(1.0, -0.005, "lower beside the tape edge", beside=-0.027),
            LeaderWaypoint(1.4, -0.005, "slide the pads around the side of the tab"),
            LeaderWaypoint(0.7, -0.005, "bring the pad faces over the folded starter", beside=0.010, above=-0.004, side_bias=0.002),
            LeaderWaypoint(1.2, closed, "close on the film", beside=0.010, above=-0.004, side_bias=0.002),
            Waypoint(1.2, (X - 0.002, AXLE[1] + 0.024, TOP + 0.093), closed, "raise the freed tab above the roll", side_grip),
            Waypoint(1.4, (X - 0.003, AXLE[1] - 0.035, TOP + 0.093), closed, "draw the tape over the roll", side_grip),
            Waypoint(2.1, pull, closed, "pull and unwind the tape", tab_pose),
            Waypoint(1.6, finish, closed, "lay the adhesive face across the tear bar", flat_grip),
            Waypoint(0.9, finish, closed, "hold the tape against the tear bar", flat_grip),
            Waypoint(0.7, finish, opened, "release the leader", flat_grip),
            Waypoint(0.9, (finish[0] - 0.040, finish[1] - 0.010, finish[2] + 0.018), opened,
                     "move straight out from the attached tape", flat_grip),
            Waypoint(1.3, (0.33, -0.177, h), opened, "withdraw", right),
            Waypoint(3.0, (0.33, -0.177, h), opened, "allow the attached tape and hub to settle", right),
        ],
    }
