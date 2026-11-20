"""Wipe the front and back faces of both lenses with a folded cloth."""

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
from glasses_parts import Prop, Solid, allow_demo_orientation_matrix, cloth_reaction, install, show_mark, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
START = (0.360, -0.008, TOP + 0.049)
CLOTH = (0.351, -0.219, TOP + 0.046)
FRAME_GRIP = (-0.079, 0.062, 0.008)
CLOTH_GRIP = (0.0, 0.0, 0.052)
BLACK = (0.17, 0.13, 0.13)
LENS = (0.67, 0.82, 0.91)
BLUE = (0.16, 0.39, 0.72)
GREY = (0.44, 0.43, 0.40)
STAND = (0.28, 0.31, 0.35)
RIM = 0.0235
LENS_Y = (-0.031, 0.031)
WORLD_X = np.array((1.0, 0.0, 0.0))
WORLD_Z = np.array((0.0, 0.0, 1.0))
FRAME_HOLD = axis_angle_matrix(WORLD_X, math.radians(-36)) @ grip_rotation()
WIPE_LEAN = axis_angle_matrix(WORLD_X, math.radians(23)) @ grip_rotation(math.pi / 2)

frame = [Solid((0, 0, 0.006), (0.005, 0.016, 0.004), BLACK)]
marks = []
for y in LENS_Y:
    frame.append(Solid((0, y, 0), (RIM - 0.0012, 0.0028), LENS, axis="X", opacity=0.65))
    circumference = [np.array([0, y + RIM * math.cos(a), RIM * math.sin(a)])
                     for a in np.linspace(0, math.tau, 19)]
    for a, b in zip(circumference, circumference[1:]):
        _, dy, dz = b-a
        frame.append(Solid(tuple(((a+b)/2).tolist()), (0.004, float(np.linalg.norm(b-a)) + 0.001, 0.003), BLACK,
                           roll=math.degrees(math.atan2(dz, dy))))
    outer = y + math.copysign(RIM, y)
    temple_y = outer + math.copysign(0.006, y)
    frame.append(Solid((-0.056, temple_y, 0.008), (0.113, 0.006, 0.007), BLACK))
    frame.append(Solid((-0.119, temple_y, 0.003), (0.024, 0.007, 0.008), BLACK, pitch=-18))
    for sign in (-1, 1):
        for dy, dz in ((-0.008, -0.007), (0.008, 0.002), (0.0, 0.012)):
            p = (sign * 0.00163, y + dy, dz)
            marks.append((len(frame), np.array(p), sign))
            frame.append(Solid(p, (0.0023, 0.00014), GREY, axis="X", collide=False))

OBJECTS = [
    Prop("glasses", START, BLACK, size=(0.140, 0.127, 0.052), mass=0.023, children=tuple(frame)),
    Prop("cloth", CLOTH, BLUE, size=(0.012, 0.036, 0.084), mass=0.013, children=(
        Solid((0, 0, 0), (0.012, 0.036, 0.052), BLUE),
        Solid((0, 0, 0.040), (0.008, 0.023, 0.036), BLUE),
    )),
    Prop("glasses_rest", START[:2] + (TOP,), STAND, size=(0.156, 0.137, 0.054), children=(
        Solid((-0.018, -0.0605, 0.027), (0.021, 0.012, 0.054), STAND),
        Solid((-0.018, 0.0605, 0.027), (0.021, 0.012, 0.054), STAND),
        Solid((-0.111, 0, 0.0255), (0.023, 0.137, 0.051), STAND),
        Solid((-0.104, -0.0695, 0.030), (0.026, 0.007, 0.060), STAND),
        Solid((-0.104, 0.0695, 0.030), (0.026, 0.007, 0.060), STAND),
    )),
    Prop("cloth_rest", CLOTH[:2] + (TOP,), STAND, size=(0.030, 0.050, 0.024), children=(
        Solid((0, 0, 0.010), (0.03, 0.05, 0.020), STAND),
        Solid((-0.007, 0, 0.024), (0.006, 0.05, 0.008), STAND),
    )),
]

travel = np.zeros(len(marks))
working_poses = {}


def reset():
    travel.fill(0.0)
    working_poses.clear()
    for index, _, _ in marks:
        show_mark(index, True)
    cloth_reaction(np.zeros(3))


install(reset)


def score(objects):
    glasses, cloth = objects["glasses"], objects["cloth"]
    lens_basis = quat_xyzw_to_matrix(glasses.quat_xyzw)
    cloth_basis = quat_xyzw_to_matrix(cloth.quat_xyzw)
    working_poses.update(frame=glasses, cloth=cloth, basis=lens_basis)
    transverse = lens_basis.T @ (cloth.lin_vel - glasses.lin_vel)
    side = float(np.dot(cloth.pos - glasses.pos, lens_basis[:, 0]))
    aligned = abs(np.dot(cloth_basis[:, 0], lens_basis[:, 0])) > 0.82
    touching = False
    if aligned:
        for i, (index, mark, direction) in enumerate(marks):
            if direction * side <= 0:
                continue
            local = cloth_basis.T @ (glasses.pos + lens_basis @ mark - cloth.pos)
            face_gap = abs(abs(local[0]) - 0.006)
            if face_gap <= 0.0025 and abs(local[1]) < 0.016 and abs(local[2]) < 0.024:
                touching = True
                if travel[i] < 0.010:
                    travel[i] += np.linalg.norm(transverse[1:]) / 20.0
                    if travel[i] >= 0.010:
                        show_mark(index, False)
    cloth_reaction(-math.copysign(1.1, side) * lens_basis[:, 0] if touching else np.zeros(3))
    cleaned = int(np.count_nonzero(travel >= 0.010))
    if os.getenv("EYEWEAR_DIAGNOSTICS"):
        print("EYEWEAR", cleaned, "signed gap", round(side, 4), "touching", touching,
              "mark travel mm", np.round(travel * 1000, 1).tolist(),
              "frame", np.round(glasses.pos, 4).tolist(), "cloth", np.round(cloth.pos, 4).tolist(),
              "tips", np.round(list(tips.values()), 4).tolist(), flush=True)
    release = (glasses.resting() and cloth.resting() and np.linalg.norm(glasses.pos[:2] - START[:2]) < 0.026
               and np.linalg.norm(cloth.pos[:2] - CLOTH[:2]) < 0.032
               and all(min(np.linalg.norm(tip - glasses.pos), np.linalg.norm(tip - cloth.pos)) > 0.058
                       for tip in tips.values()))
    if cleaned == len(marks) and release:
        return 1.0
    return 0.5 if cleaned >= 8 else 0.25 if cleaned >= 4 else 0.0


class WipeStroke:
    finger = FINGER_CLOSED

    def __init__(self, duration, x, y, z, label):
        self.duration, self.x, self.y, self.z, self.label = duration, x, y, z, label

    @property
    def yaw(self):
        basis = working_poses.get("basis")
        if basis is None:
            return WIPE_LEAN
        turning = np.clip(math.atan2(basis[1, 0], basis[0, 0]), -0.45, 0.45)
        return axis_angle_matrix(WORLD_Z, float(turning)) @ WIPE_LEAN

    @property
    def pos(self):
        local = np.array([self.x, self.y, self.z])
        nominal = np.array(START) + local + CLOTH_GRIP
        if not working_poses or "right" not in tips:
            return tuple(nominal)
        frame, cloth, basis = (working_poses[key] for key in ("frame", "cloth", "basis"))
        cloth_target = frame.pos + basis @ local
        grip_offset = tips["right"] - cloth.pos - PAD_TIP_PAST_TCP * self.yaw[:, 2]
        bounds = np.array([0.040, 0.045, 0.065])
        return tuple(nominal + np.clip(cloth_target + grip_offset - nominal, -bounds, bounds))


class ClothGrip:
    yaw = math.pi / 2

    def __init__(self, duration, finger, label):
        self.duration, self.finger, self.label = duration, finger, label

    @property
    def pos(self):
        nominal = np.array(CLOTH) + CLOTH_GRIP
        cloth = working_poses.get("cloth")
        if cloth is None:
            return tuple(nominal)
        basis = quat_xyzw_to_matrix(cloth.quat_xyzw)
        centre = cloth.pos + basis @ (0, 0, 0.046)
        desired = centre + (0, 0, PAD_TIP_PAST_TCP - 0.006)
        return tuple(nominal + np.clip(desired - nominal, -0.015, 0.015))


class FrameBrace:
    finger, yaw, label = FINGER_CLOSED, FRAME_HOLD, "steady the temple away from the cloth"

    def __init__(self, duration):
        self.duration = duration

    @property
    def pos(self):
        nominal = np.array(START) + FRAME_GRIP
        frame, tip = working_poses.get("frame"), tips.get("left")
        if frame is None or tip is None:
            return tuple(nominal)
        correction = np.clip(1.75 * (np.array(START) - frame.pos), -0.020, 0.020)
        target = tip + correction - PAD_TIP_PAST_TCP * FRAME_HOLD[:, 2]
        return tuple(nominal + np.clip(target - nominal, -0.029, 0.029))


def demo():
    allow_demo_orientation_matrix()
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    lh, rh = FRAME_HOLD, math.pi / 2
    frame_open = -0.010
    high = TOP + 0.156
    grip = tuple(np.array(START) + FRAME_GRIP - (0, 0, PAD_TIP_PAST_TCP) - PAD_TIP_PAST_TCP * lh[:, 2])
    wipe = tuple(np.array(CLOTH) + CLOTH_GRIP)
    # The cloth can turn in the grasp as it is wiped. Approach its rest from
    # the inside so the hanging end centres on the support before release.
    setdown = (CLOTH[0] + 0.003, CLOTH[1] + 0.039, TOP + 0.093)
    strokes = []
    for side in (1, -1):
        for y in LENS_Y:
            x = side * 0.006
            strokes.extend([
                WipeStroke(1.0, x + side * 0.030, y, 0.053, "move over the selected lens face"),
                WipeStroke(0.8, x + side * 0.030, y, 0.019, "lower without touching the frame"),
                WipeStroke(0.8, x, y - 0.013, 0.008, "touch the lens with the folded cloth"),
                WipeStroke(1.2, x, y + 0.014, 0.011, "sweep across the upper marks"),
                WipeStroke(1.2, x, y - 0.014, -0.004, "wipe back across the lens"),
                WipeStroke(1.2, x, y + 0.014, -0.003, "sweep across the lower marks"),
                WipeStroke(0.8, x + side * 0.030, y, 0.042, "lift the cloth clear"),
            ])
        strokes.append(Waypoint(1.2, (0.361, -0.122, high), closed, "move the cloth clear of the frame", rh))
    return {
        "left": [
            Waypoint(1.6, (*grip[:2], high), frame_open, "approach the frame arm", lh),
            Waypoint(1.2, grip, frame_open, "straddle the temple", lh),
            Waypoint(0.8, grip, closed, "hold the frame by the temple", lh),
            FrameBrace(4.5 + sum(w.duration for w in strokes)),
            Waypoint(0.7, grip, opened, "let go of the frame", lh),
            Waypoint(1.1, (*grip[:2], high), opened, "withdraw", lh),
        ],
        "right": [
            Waypoint(1.6, (*wipe[:2], high), -0.010, "approach the cloth", rh),
            ClothGrip(1.3, -0.010, "straddle the top fold"),
            ClothGrip(1.0, closed, "grip the cloth fold"),
            Waypoint(1.4, (*wipe[:2], high), closed, "lift the cloth", rh),
            Waypoint(2.3, (0.36, -0.13, high), closed, "wait for the frame", rh),
            *strokes,
            Waypoint(1.5, (*setdown[:2], high), closed, "return the cloth", rh),
            Waypoint(1.4, setdown, closed, "set the cloth on its rest", rh),
            Waypoint(0.4, setdown, closed, "let the cloth settle on the rest", rh),
            Waypoint(0.9, setdown, opened, "release the cloth", rh),
            Waypoint(1.1, (*setdown[:2], high), opened, "withdraw", rh),
        ],
    }
