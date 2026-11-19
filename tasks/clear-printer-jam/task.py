"""Pull the exposed edge of an A4 sheet out of an open rear printer path.

The printer is unplugged and cool. The rear access door is already open.
Use both hands near the two edges and draw the sheet evenly out of the rollers.
Leave the whole sheet on the bench in front of the access door.
"""

import math
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(Path(__file__).parent))
from printer_parts import Panel, Roller, gripper_tips, install_printer_props

SCENE = "station"
TOP = TABLE_TOP[SCENE]
WIDTH = 0.210
SEGMENTS = 9
LENGTH = 0.297 / SEGMENTS
PAPER_Z = TOP + 0.071
FORWARD = 0.100
NIP_X = 0.472 + FORWARD
THICKNESS = 0.00010
JOINT_BREAK_N = 8.0
GREY = (0.35, 0.38, 0.42)
WHITE = (0.89, 0.89, 0.87)
EXIT_CORNER = np.array([0.365 + FORWARD, 0.0, PAPER_Z + LENGTH * (math.sin(math.radians(72)) + math.sin(math.radians(30)))])
angles = (72.0, 30.0, *([0.0] * 7))
panels = []
edge = EXIT_CORNER.copy()
for index, angle in enumerate(angles):
    radians = math.radians(angle)
    offset = LENGTH * np.array([math.cos(radians), 0.0, -math.sin(radians)])
    panels.append(Panel(
        f"page_{index}", tuple(map(float, edge + offset / 2)), WHITE, mass=0.080 * WIDTH * LENGTH,
        size=(LENGTH - 0.00008, WIDTH, THICKNESS), angle=radians,
        previous=panels[-1].name if panels else None, edge=tuple(edge), break_force=JOINT_BREAK_N,
    ))
    edge += offset

housing = [
    Box("printer_base", (0.595, 0, TOP + 0.013), GREY, size=(0.31, 0.386, 0.026)),
    Box("printer_lid", (0.633, 0, TOP + 0.153), GREY, size=(0.234, 0.386, 0.026)),
    Box("left_side", (0.595, 0.178, TOP + 0.078), GREY, size=(0.31, 0.030, 0.156)),
    Box("right_side", (0.595, -0.178, TOP + 0.078), GREY, size=(0.31, 0.030, 0.156)),
    Box("feed_guide", (0.603, 0, PAPER_Z - 0.006), (0.23, 0.25, 0.27), size=(0.229, 0.24, 0.010)),
    # Straight rear guide sits just under the jammed portion. The folded free edge hangs clear.
    Box("rear_lip", (0.417, 0, PAPER_Z - 0.006), GREY, size=(0.071, 0.24, 0.010)),
]
OBJECTS = [
    *(replace(box, pos=(box.pos[0] + FORWARD, *box.pos[1:])) for box in housing),
    Roller("lower_roller", (NIP_X, 0, PAPER_Z - 0.012), (0.13, 0.14, 0.15), size=(0.024, 0.225, 0.024)),
    Roller("upper_roller", (NIP_X, 0, PAPER_Z + 0.012), (0.13, 0.14, 0.15), mass=0.070,
           size=(0.024, 0.225, 0.024), spring=True),
    *panels,
]

install_printer_props()


def intact(states):
    from yam_arena.urdf import quat_xyzw_to_matrix

    for left, right in zip(panels, panels[1:]):
        a, b = states[left.name], states[right.name]
        a_direction = quat_xyzw_to_matrix(a.quat_xyzw)[:, 0]
        b_direction = quat_xyzw_to_matrix(b.quat_xyzw)[:, 0]
        seam_a = a.pos + a_direction * LENGTH / 2
        seam_b = b.pos - b_direction * LENGTH / 2
        if np.linalg.norm(seam_a - seam_b) > 0.004:
            return False
    return True


def full_sheet_out(states):
    return all(states[p.name].high[0] <= NIP_X - 0.013 for p in panels)


def success(states):
    page = [states[p.name] for p in panels]
    return (
        intact(states) and full_sheet_out(states)
        and all(TOP - 0.002 < part.low[2] and part.high[2] < PAPER_Z - 0.011 and part.resting() for part in page)
        and sum(part.low[2] < TOP + 0.006 for part in page) >= 3
        and all(np.min([np.linalg.norm(part.pos - tip) for part in page]) > 0.03 for tip in gripper_tips.values())
    )


step = 0


def score(states):
    global step
    step += 1
    connected = intact(states)
    if os.environ.get("PRINTER_DIAGNOSTICS") and step % 20 == 0:
        page = [states[p.name] for p in panels]
        print("PAGE", step, "intact", connected, "centers", np.round(np.asarray([part.pos for part in page], dtype=float), 3).tolist(),
              "tips", np.round(list(gripper_tips.values()), 3).tolist(), flush=True)
    if not connected:
        return 0.0
    if success(states):
        return 1.0
    if full_sheet_out(states):
        return 0.5
    return 0.0


def demo():
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    # Mirrored wrist yaw puts the jaws on the two faces of the upturned sheet.
    target_x = EXIT_CORNER[0] + 0.004
    grip_z = EXIT_CORNER[2] - 0.007
    return {
        side: [
            Waypoint(2.2, (target_x, y, grip_z + 0.038), opened, "approach the exposed edge", yaw),
            Waypoint(1.3, (target_x, y, grip_z), opened, "lower over the paper", yaw),
            Waypoint(1.0, (target_x, y, grip_z), closed, "pinch both sides", yaw),
            Waypoint(1.6, (target_x - 0.06, y, grip_z + 0.005), closed, "start sliding out of the rollers", yaw),
            Waypoint(1.8, (0.233 + FORWARD, y, grip_z + 0.005), closed, "support the curling page", yaw),
            Waypoint(2.5, (0.151 + FORWARD, y * 0.86, grip_z + 0.005), closed, "pull evenly clear", yaw),
            Waypoint(1.5, (0.135 + FORWARD, y * 0.86, TOP + 0.042), closed, "lower over the bench", yaw),
            Waypoint(0.8, (0.135 + FORWARD, y * 0.86, TOP + 0.042), opened, "release the page", yaw),
            Waypoint(1.6, (0.193 + FORWARD, y, TOP + 0.130), opened, "withdraw", yaw),
        ]
        for side, y, yaw in (("left", 0.081, -math.pi / 2), ("right", -0.081, math.pi / 2))
    }
