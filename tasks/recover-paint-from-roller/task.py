"""Recover excess wet paint by driving a full-size wire squeegee on a roller."""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena import demo as yam_demo
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from paint_parts import Prop, Solid, drag, install, paint_colour, paint_level, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
ROLLER = (0.475, -0.029, TOP + 0.029)
WORK = (0.334, -0.048, TOP + 0.137)
TOOL = (0.249, -0.257, TOP + 0.049)
TRAY = (0.310, -0.049, TOP)
LENGTH = 0.229
RADIUS = 0.0286
N_BANDS = 12
STOCK = 60.0
HANDLE = (0.0, 0.183, 0.017)
LOOP_HANDLE = (0.086, 0.0, 0.007)
STEEL = (0.67, 0.69, 0.72)
ROLL_GRIP = (0.13, 0.16, 0.19)
NAP = (0.86, 0.84, 0.76)
PAINT = (0.73, 0.34, 0.09)
BLUE = (0.10, 0.27, 0.42)

roller_parts = [
    Solid((0, 0, 0), (0.0195, LENGTH), NAP, axis="Y"),
    Solid((0, 0.132, 0), (0.004, 0.036), STEEL, axis="Y"),
    Solid((0, 0.157, 0.009), (0.010, 0.020, 0.018), STEEL),
    Solid((0, 0.192, 0.017), (0.026, 0.066, 0.023), ROLL_GRIP),
]
START_COATING = len(roller_parts)
bands = np.linspace(-LENGTH / 2, LENGTH / 2, N_BANDS + 1)
for a, b in zip(bands, bands[1:]):
    roller_parts.append(Solid((0, float((a + b) / 2), 0), (RADIUS, float(b - a)), PAINT,
                             axis="Y", collide=False))

loop_parts = []
radius = 0.0245
points = [np.array([radius * math.cos(a), 0, radius * math.sin(a)]) for a in np.linspace(0, math.tau, 27)]
for a, b in zip(points, points[1:]):
    dx, _, dz = b - a
    loop_parts.append(Solid(tuple(((a + b) / 2).tolist()), (float(np.linalg.norm(b-a)) + 0.001, 0.0037, 0.004),
                            STEEL, pitch=-math.degrees(math.atan2(dz, dx))))
loop_parts.extend([
    Solid((0.056, 0.0, 0.010), (0.067, 0.016, 0.007), STEEL),
    Solid((0.089, 0.0, 0.007), (0.066, 0.016, 0.013), STEEL),
])
tray_parts = [Solid((0, 0, 0.004), (0.168, 0.306, 0.008), BLUE)]
for sign in (-1, 1):
    tray_parts.extend([
        Solid((sign * 0.082, 0, 0.016), (0.004, 0.306, 0.032), BLUE),
        Solid((0, sign * 0.151, 0.016), (0.168, 0.004, 0.032), BLUE),
    ])
PUDDLE = len(tray_parts)
tray_parts.append(Solid((0, 0, 0.008), (0.067, 0.00008), PAINT, axis="Z", collide=False))

OBJECTS = [
    Prop("roller", ROLLER, NAP, mass=0.22, size=(0.057, 0.353, 0.057), children=tuple(roller_parts)),
    Prop("squeegee", TOOL, STEEL, mass=0.068, size=(0.155, 0.016, 0.056), children=tuple(loop_parts)),
    Prop("catch_tray", TRAY, BLUE, size=(0.168, 0.306, 0.032), children=tuple(tray_parts)),
    Prop("squeegee_rest", TOOL[:2] + (TOP,), BLUE, size=(0.18, 0.035, 0.049), children=(
        Solid((0.0, 0, 0.011), (0.049, 0.031, 0.022), BLUE),
        Solid((0.067, 0, 0.024), (0.040, 0.025, 0.048), BLUE),
    )),
    Prop("roller_rest", ROLLER[:2] + (TOP,), BLUE, size=(0.067, 0.328, 0.036), children=(
        Solid((0.0, -0.087, 0.005), (0.064, 0.031, 0.010), BLUE),
        Solid((0.0, 0.088, 0.005), (0.064, 0.031, 0.010), BLUE),
        Solid((-0.026, -0.087, 0.011), (0.010, 0.031, 0.022), BLUE),
        Solid((0.026, -0.087, 0.011), (0.010, 0.031, 0.022), BLUE),
        Solid((-0.026, 0.088, 0.011), (0.010, 0.031, 0.022), BLUE),
        Solid((0.026, 0.088, 0.011), (0.010, 0.031, 0.022), BLUE),
        Solid((0.0, 0.192, 0.0175), (0.041, 0.027, 0.035), BLUE),
    )),
]


class Paint:
    def __init__(self):
        self.cleared = set()
        self.caught = 0.0
        self.spilled = 0.0
        self.steps = 0
        self.roller, self.tool, self.basis = None, None, None

    def reset(self):
        self.__init__()
        drag(np.zeros(3))
        for i in range(N_BANDS):
            paint_colour("roller", START_COATING + i, PAINT)
        paint_level("catch_tray", PUDDLE, 0.0, 0.0036, 0.008)

    def advance(self, roller, tool):
        self.steps += 1
        basis = quat_xyzw_to_matrix(roller.quat_xyzw)
        self.roller, self.tool, self.basis = roller, tool, basis
        axis = basis[:, 1]
        tool_axis = quat_xyzw_to_matrix(tool.quat_xyzw)[:, 1]
        offset = basis.T @ (tool.pos - roller.pos)
        slip = float(np.dot(tool.lin_vel - roller.lin_vel, axis))
        alignment = abs(float(np.dot(tool_axis, axis)))
        engaged = (alignment > 0.95 and abs(offset[1]) < LENGTH/2 + 0.002
                   and np.linalg.norm(offset[[0, 2]]) < 0.006)
        resisting = float(np.clip(150.0 * slip, -4.5, 4.5)) if engaged else 0.0
        drag(resisting * axis)
        if os.getenv("PAINT_DIAGNOSTICS") and self.steps % 20 == 0:
            print("PAINT", self.steps, "relative", np.round(offset, 3).tolist(), "force", round(resisting, 2),
                  "alignment", round(alignment, 3), "collected", self.caught, "spill", self.spilled,
                  "roller", np.round(roller.pos, 3).tolist(), "tool", np.round(tool.pos, 3).tolist(),
                  "tips", np.round(list(tips.values()), 3).tolist(), flush=True)
        if not engaged or abs(slip) < 0.010:
            return
        i = int(np.clip(np.searchsorted(bands, offset[1]) - 1, 0, N_BANDS - 1))
        if i in self.cleared:
            return
        self.cleared.add(i)
        over = abs(tool.pos[0] - TRAY[0]) < 0.071 and abs(tool.pos[1] - TRAY[1]) < 0.141 and tool.pos[2] > TOP + 0.034
        if over:
            self.caught += STOCK / N_BANDS
        else:
            self.spilled += STOCK / N_BANDS
        paint_colour("roller", START_COATING + i, NAP)
        paint_level("catch_tray", PUDDLE, self.caught / STOCK, 0.0036, 0.008)


paint = Paint()
install(paint.reset)


def success(objects):
    roller, tool = (objects[name] for name in ("roller", "squeegee"))
    roller_handle = roller.pos + quat_xyzw_to_matrix(roller.quat_xyzw) @ HANDLE
    squeegee_handle = tool.pos + quat_xyzw_to_matrix(tool.quat_xyzw) @ LOOP_HANDLE
    hands_clear = all(min(np.linalg.norm(tip - handle) for handle in (roller_handle, squeegee_handle)) > 0.055
                      for tip in tips.values())
    return (paint.caught >= 45 and paint.spilled <= 5 and roller.resting() and tool.resting()
            and abs(roller.pos[0] - ROLLER[0]) < 0.040 and roller.pos[2] < TOP + 0.038
            and np.linalg.norm(tool.pos - TOOL) < 0.035
            and hands_clear)


def score(objects):
    paint.advance(objects["roller"], objects["squeegee"])
    if paint.spilled > 5:
        return 0.0
    if success(objects):
        return 1.0
    return 0.5 if paint.caught >= 45 else 0.25 if paint.caught >= 25 else 0.0


class LoopOnCover:
    """Guide the real loop along the tilted axis of the supported cover."""

    finger = FINGER_CLOSED

    def __init__(self, duration, along, above, label):
        self.duration, self.along, self.above, self.label = duration, along, above, label

    @property
    def yaw(self):
        down = grip_rotation(0)
        if paint.basis is None:
            return down
        axis = paint.basis[:, 1]
        turn = np.cross([0.0, 1.0, 0.0], axis)
        sine = np.linalg.norm(turn)
        if sine < 0.0001:
            return down
        angle = min(math.atan2(float(sine), float(axis[1])), math.radians(22))
        return axis_angle_matrix(turn / sine, angle) @ down

    @property
    def pos(self):
        nominal = np.array(WORK) + LOOP_HANDLE + (0, self.along, self.above + 0.005)
        tip = tips.get("right")
        if paint.roller is None or tip is None:
            return tuple(nominal)
        wanted = paint.roller.pos + paint.basis @ (0.0, self.along, self.above)
        grip_offset = tip - paint.tool.pos - PAD_TIP_PAST_TCP * self.yaw[:, 2]
        bounds = np.array([0.032, 0.043, 0.064])
        return tuple(nominal + np.clip(wanted + grip_offset - nominal, -bounds, bounds))


def demo():
    yam_demo.grip_rotation = lambda yaw: yaw if isinstance(yaw, np.ndarray) else grip_rotation(yaw)
    high = TOP + 0.171
    left, right = -math.pi / 2, 0.0
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    grip = tuple(np.array(ROLLER) + HANDLE + (0, 0, 0.006))
    work = tuple(np.array(WORK) + HANDLE + (0, 0, 0.006))
    tool_grip = tuple(np.array(TOOL) + LOOP_HANDLE + (0, 0, 0.005))
    approach = -LENGTH / 2 - 0.020
    far_end = LENGTH / 2 + 0.003
    return {
        "left": [
            Waypoint(1.6, (*grip[:2], high), opened, "approach the roller handle", left),
            Waypoint(1.3, grip, opened, "lower around the roller grip", left),
            Waypoint(0.8, grip, closed, "grasp the handle", left),
            Waypoint(1.5, (*grip[:2], work[2]), closed, "lift the full-size roller", left),
            Waypoint(1.8, work, closed, "hold it over the tray", left),
            Waypoint(13.0, work, closed, "brace the roller against the scraping force", left),
            Waypoint(1.6, (*grip[:2], work[2]), closed, "return the roller", left),
            Waypoint(1.2, grip, closed, "set down the roller", left),
            Waypoint(0.8, grip, opened, "release the frame", left),
            Waypoint(1.1, (*grip[:2], high), opened, "withdraw", left),
        ],
        "right": [
            Waypoint(1.7, (*tool_grip[:2], high), opened, "approach the squeegee handle", right),
            Waypoint(1.3, tool_grip, opened, "straddle the wire grip", right),
            Waypoint(0.8, tool_grip, closed, "grasp the squeegee", right),
            Waypoint(1.6, (*tool_grip[:2], high), closed, "lift the squeegee", right),
            LoopOnCover(1.8, approach, 0.054, "approach the free roller end"),
            LoopOnCover(1.6, approach, 0, "line up the loop with the cover"),
            LoopOnCover(5.4, far_end, 0, "draw the loop along the wet nap"),
            LoopOnCover(3.7, approach, 0, "slide the loop back over the free end"),
            LoopOnCover(1.4, approach, 0.054, "lift the loop clear"),
            Waypoint(1.6, (*tool_grip[:2], high), closed, "return the squeegee", right),
            Waypoint(1.2, tool_grip, closed, "lower to the rest", right),
            Waypoint(0.8, tool_grip, opened, "release the tool", right),
            Waypoint(1.2, (*tool_grip[:2], high), opened, "withdraw", right),
        ],
    }
