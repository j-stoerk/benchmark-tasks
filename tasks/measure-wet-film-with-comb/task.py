"""Measure a wet coating on a loose aluminium test coupon with a notch gauge.

The two feet at the ends of the steel comb should land on the coupon, with
the comb perpendicular to the wet film. Keep the comb still, lift it straight
up and park it in the second slot so the wet and dry teeth can be read.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.urdf import quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from props import Piece, Prop, arm_tips, color_tooth, install

SCENE = "station"
TOP = TABLE_TOP[SCENE]
WIDTH, DEPTH, PLATE = 0.120, 0.190, 0.006
COUPON = (0.405, 0.028, TOP + PLATE / 2)
FILM = 0.00075
FILM_Y = -0.035
PICK = (0.365, -0.255)
PARK = (0.365, -0.155)
RACK_BASE = 0.006
GAUGE_H = 0.043
FACES = (0.0, 0.00025, 0.0005, 0.001, 0.00125, 0.0)
TEETH = (-0.029, -0.017, -0.006, 0.006, 0.017, 0.029)
EXPECTED = tuple(gap < FILM for gap in FACES)
METAL = (0.70, 0.74, 0.78)
WET = (0.06, 0.42, 0.52)
RACK = (0.22, 0.22, 0.26)
HOLD_STEPS = 20


def rack(name, xy):
    x, y = xy
    return [
        Box(f"{name}_floor", (x, y, TOP + RACK_BASE / 2), RACK, size=(0.086, 0.040, RACK_BASE)),
        *(
            Box(f"{name}_{side}", (x, y + sign * 0.010, TOP + 0.010), RACK, size=(0.086, 0.010, 0.020))
            for side, sign in [("near", -1), ("far", 1)]
        ),
    ]


OBJECTS = [
    Prop("coupon", COUPON, METAL, mass=0.36, size=(WIDTH, DEPTH, PLATE), pieces=(
        Piece((0, 0, 0), (WIDTH, DEPTH, PLATE), METAL),
        Piece((0, FILM_Y - COUPON[1], PLATE / 2 + FILM / 2), (0.088, 0.054, FILM), WET, collision=False),
    )),
    *rack("pick", PICK),
    *rack("park", PARK),
    Prop("comb", (*PICK, TOP + RACK_BASE + GAUGE_H / 2), METAL, mass=0.062, size=(0.064, 0.003, GAUGE_H), pieces=(
        Piece((0, 0, 0.003), (0.064, 0.003, GAUGE_H - 0.006), METAL),
        *(
            Piece((x, 0, -GAUGE_H / 2 + (0.006 + gap) / 2), (0.005, 0.003, 0.006 - gap), METAL)
            for x, gap in zip(TEETH, FACES)
        ),
    )),
]


class Measurement:
    def reset(self):
        self.steps = 0
        self.contact_steps = 0
        self.wet = [False] * len(TEETH)
        self.measured = False
        self.scraped = False
        self.last = None
        self.impression = None

    def update(self, objects):
        self.steps += 1
        coupon, comb = objects["coupon"], objects["comb"]
        c_rot = quat_xyzw_to_matrix(coupon.quat_xyzw)
        g_rot = quat_xyzw_to_matrix(comb.quat_xyzw)
        points = np.array([(x, 0.0, -GAUGE_H / 2 + gap) for x, gap in zip(TEETH, FACES)])
        relative = (points @ g_rot.T + comb.pos - coupon.pos) @ c_rot
        heights = relative[:, 2] - PLATE / 2
        in_film = (abs(relative[:, 0]) < 0.044) & (abs(relative[:, 1] - (FILM_Y - COUPON[1])) < 0.027)
        wet_now = in_film & (heights <= FILM)
        moving_sideways = self.last is not None and np.max(np.linalg.norm(relative[:, :2] - self.last, axis=1)) > 0.0005
        self.last = relative[:, :2].copy()
        if np.any(wet_now):
            if self.impression is None:
                self.impression = relative[:, :2].copy()
            # An impression up to 3 mm long is readable after normal seating.
            # Use the contact trace displacement, not incidental approach speed.
            if np.max(np.linalg.norm(relative[:, :2] - self.impression, axis=1)) > 0.003:
                self.scraped = True
        else:
            self.impression = None
        level = np.dot(g_rot[:, 2], c_rot[:, 2]) >= math.cos(math.radians(2))
        feet = max(abs(heights[0]), abs(heights[-1])) <= 0.00045
        valid = level and feet and np.all(in_film) and not moving_sideways
        self.contact_steps = self.contact_steps + 1 if valid else 0
        for i, wetted in enumerate(wet_now):
            if wetted and not self.wet[i]:
                self.wet[i] = True
                color_tooth(i, WET)
        if self.contact_steps >= HOLD_STEPS and tuple(self.wet) == EXPECTED:
            self.measured = True
        if os.environ.get("WET_COMB_DIAGNOSTICS") and self.steps % 10 == 0:
            print("COMB", self.steps, "tip_mm", np.round(heights * 1000, 2).tolist(), "level", level,
                  "dwell", self.contact_steps, "wet", self.wet, "scrape", self.scraped,
                  "comb_xyz", np.round(comb.pos, 4).tolist(), flush=True)


READING = Measurement()
READING.reset()


def parked(objects):
    comb = objects["comb"]
    x, y, z = comb.pos
    rot = quat_xyzw_to_matrix(comb.quat_xyzw)
    return (
        abs(x - PARK[0]) <= 0.012 and abs(y - PARK[1]) <= 0.005
        and abs(z - (TOP + RACK_BASE + GAUGE_H / 2)) <= 0.002
        and rot[2, 2] >= math.cos(math.radians(8)) and comb.resting()
        and np.linalg.norm(objects["coupon"].pos[:2] - COUPON[:2]) <= 0.010
        and arm_tips and all(np.linalg.norm(comb.pos - tip) >= 0.05 for tip in arm_tips.values())
    )


def success(objects):
    return READING.measured and not READING.scraped and tuple(READING.wet) == EXPECTED and parked(objects)


def score(objects):
    READING.update(objects)
    if READING.scraped:
        return 0.0
    if success(objects):
        return 1.0
    return 0.5 if READING.measured and tuple(READING.wet) == EXPECTED else 0.0


install(READING.reset)


def demo():
    hover = TOP + 0.125
    grip_offset = GAUGE_H - 0.008
    rack_grip = TOP + RACK_BASE + grip_offset
    film_grip = TOP + PLATE + grip_offset
    left_xy = (COUPON[0], 0.104)
    park_xy = (PARK[0] + 0.006, PARK[1] + 0.003)
    O, C = FINGER_OPEN, FINGER_CLOSED
    return {
        "left": [
            Waypoint(2.0, (*left_xy, hover), C, "go over the uncoated coupon end"),
            Waypoint(1.2, (*left_xy, TOP + PLATE + PAD_TIP_PAST_TCP - 0.001), C, "steady the coupon"),
            Waypoint(16.0, (*left_xy, TOP + PLATE + PAD_TIP_PAST_TCP - 0.001), C, "hold during measurement"),
            Waypoint(1.0, (*left_xy, hover), O, "lift clear"),
        ],
        "right": [
            Waypoint(2.0, (*PICK, hover), O, "over the comb"),
            Waypoint(1.2, (*PICK, rack_grip), O, "descend around comb"),
            Waypoint(0.9, (*PICK, rack_grip), C, "grasp the comb"),
            Waypoint(1.3, (*PICK, hover), C, "lift from slot"),
            Waypoint(1.8, (COUPON[0], FILM_Y, hover), C, "carry above the wet sample"),
            Waypoint(1.6, (COUPON[0], FILM_Y, film_grip + 0.005), C, "align the feet above the surface"),
            Waypoint(1.5, (COUPON[0], FILM_Y, film_grip - 0.001), C, "touch down vertically"),
            Waypoint(1.6, (COUPON[0], FILM_Y, film_grip - 0.001), C, "allow teeth to wet"),
            Waypoint(1.5, (COUPON[0], FILM_Y, hover), C, "lift vertically out of the film"),
            Waypoint(1.6, (*park_xy, hover), C, "carry to reading slot"),
            Waypoint(1.4, (*park_xy, rack_grip + 0.010), C, "align above the slot"),
            Waypoint(1.3, (*park_xy, rack_grip - 0.001), C, "seat the comb"),
            Waypoint(0.6, (*park_xy, rack_grip - 0.001), C, "allow the feet to settle"),
            Waypoint(0.9, (*park_xy, rack_grip - 0.001), O, "release"),
            Waypoint(1.3, (*park_xy, hover), O, "withdraw"),
        ],
    }
