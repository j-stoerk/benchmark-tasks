"""Transfer an inert electrode-roll handling mock by a spindle through its hollow
core. A grey 120 mm diameter, 50 mm wide roll starts upright on the table between
two low chocks, axis along y. A blue 240 mm spindle lies parallel to it in front
of the right arm. Insert the spindle, lift it at its ends and set it on the two
blue bearing rests. The roll must hang from its core between the rests. Both
TCPs must stay 30 mm outside the roll's oriented bounding box throughout the
trial. There is no wound-film, tension, dust, coating or electrochemical model.
"""

import itertools
import math

import numpy as np
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box
from yam_arena.urdf import quat_xyzw_to_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
OUTER_R, INNER_R, ROLL_WIDTH = 0.060, 0.025, 0.050
SHAFT_R, SHAFT_LENGTH = 0.009, 0.240
HANDLE_SIZE = (0.024, 0.030, 0.024)
HANDLE_YS = (-0.100, 0.100)
SHAFT_HALF_HEIGHT = HANDLE_SIZE[2] / 2
TCP_SHAFT_OFFSET = PAD_TIP_PAST_TCP - 0.006
SEGMENTS = 24
ROLL_XY = (0.200, 0.090)
SHAFT_XY = (0.160, -0.160)
TARGET = (0.320, 0.0)
SHAFT_Z = TOP + 0.125
REST_Z = SHAFT_Z - SHAFT_R
SAG = INNER_R * math.cos(math.pi / SEGMENTS) - SHAFT_R
REST_YS = (-0.065, 0.065)
SIZES = {"roll": (OUTER_R * 2, ROLL_WIDTH, OUTER_R * 2), "spindle": (HANDLE_SIZE[0], SHAFT_LENGTH, HANDLE_SIZE[2])}
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = (
    "Insert the blue spindle through the electrode roll's hollow core, lift the spindle by its ends, "
    "and set it across both blue bearing rests so the roll hangs centered midway between them. "
    "Keep both gripper TCPs "
    "at least 30 mm outside the roll's bounding box throughout; do not handle the roll directly. "
    "Release the spindle and move both grippers at least 30 mm clear of both movable objects."
)
NOVELTY_REVIEW = "provisional_2026_11_08_through_core_tool_support_and_suspended_roll"


def rests():
    result = []
    for i, y in enumerate(REST_YS):
        height = REST_Z - TOP
        result.append(
            Box(
                f"rest_{i}", pos=(TARGET[0], y, TOP + height / 2), size=(0.060, 0.018, height), color=(0.12, 0.32, 0.75)
            )
        )
        for side, dx in enumerate((-0.015, 0.015)):
            result.append(
                Box(
                    f"cheek_{i}_{side}",
                    pos=(TARGET[0] + dx, y, REST_Z + 0.0125),
                    size=(0.006, 0.018, 0.025),
                    color=(0.15, 0.40, 0.85),
                )
            )
    return result


OBJECTS = [
    Asset("roll", asset="battery_training_roll", xy=ROLL_XY, extent=0.120, mass=0.150, yaw=0.0),
    Asset("spindle", asset="battery_training_spindle", xy=SHAFT_XY, extent=SHAFT_LENGTH, mass=0.085, yaw=0.0),
    *rests(),
    *[
        Box(
            f"chock_{i}",
            pos=(ROLL_XY[0] + dx, ROLL_XY[1], TOP + 0.0125),
            size=(0.010, 0.040, 0.025),
            color=(0.30, 0.30, 0.32),
        )
        for i, dx in enumerate((-0.056, 0.056))
    ],
]


def rotation(obj):
    q = np.asarray(obj.quat_xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        return None
    if not np.isfinite(obj.pos).all():
        return None
    return quat_xyzw_to_matrix(q / np.linalg.norm(q))


def threaded(objects):
    """The finite spindle crosses both bore faces and fits the actual polygonal aperture."""
    roll, shaft = objects["roll"], objects["spindle"]
    rr, rs = rotation(roll), rotation(shaft)
    if rr is None or rs is None:
        return False
    local_axis = rr.T @ rs[:, 1]
    if abs(local_axis[1]) < math.cos(math.radians(3)):
        return False
    offset = (np.asarray(shaft.pos) - roll.pos) @ rr
    angles = (np.arange(SEGMENTS) + 0.5) * 2 * math.pi / SEGMENTS
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    limit = INNER_R * math.cos(math.pi / SEGMENTS) - SHAFT_R / abs(local_axis[1]) + 0.0008
    for y in (-ROLL_WIDTH / 2, ROLL_WIDTH / 2):
        along = (y - offset[1]) / local_axis[1]
        if abs(along) > SHAFT_LENGTH / 2 - 0.025:
            return False
        center = offset + along * local_axis
        if np.any(normals @ center[[0, 2]] > limit):
            return False
    # Check each grip collar's exact convex intersection with the bore slab.
    signs = np.array(list(itertools.product((-1, 1), repeat=3)))
    edges = [(i, j) for i in range(8) for j in range(i + 1, 8) if np.count_nonzero(signs[i] != signs[j]) == 1]
    transform = rr.T @ rs
    for handle_y in HANDLE_YS:
        vertices = signs * (np.asarray(HANDLE_SIZE) / 2) + [0, handle_y, 0]
        vertices = vertices @ transform.T + offset
        points = [p for p in vertices if abs(p[1]) <= ROLL_WIDTH / 2]
        for a, b in edges:
            for boundary in (-ROLL_WIDTH / 2, ROLL_WIDTH / 2):
                delta = vertices[b] - vertices[a]
                if abs(delta[1]) > 1e-12:
                    t = (boundary - vertices[a, 1]) / delta[1]
                    if 0 <= t <= 1:
                        points.append(vertices[a] + t * delta)
        if points and np.any(
            np.asarray(points)[:, [0, 2]] @ normals.T > INNER_R * math.cos(math.pi / SEGMENTS) + 0.0008
        ):
            return False
    return True


def on_rests(objects):
    shaft = objects["spindle"]
    r = rotation(shaft)
    if r is None or abs(r[1, 1]) < math.cos(math.radians(2)):
        return False
    axis = r[:, 1]
    for y in REST_YS:
        along = (y - shaft.pos[1]) / axis[1]
        point = np.asarray(shaft.pos) + along * axis
        if (
            abs(along) > SHAFT_LENGTH / 2 - 0.005
            or abs(point[0] - TARGET[0]) > 0.002
            or abs(point[2] - SHAFT_R - REST_Z) > 0.0015
        ):
            return False
    return abs(shaft.pos[1] - TARGET[1]) <= 0.005


def roll_at_target(objects):
    roll = objects["roll"]
    r = rotation(roll)
    return (
        r is not None
        and abs(r[1, 1]) >= math.cos(math.radians(3))
        and abs(roll.pos[0] - TARGET[0]) <= 0.003
        and abs(roll.pos[1] - TARGET[1]) <= 0.006
    )


def suspended(objects):
    roll, shaft = objects["roll"], objects["spindle"]
    return (
        threaded(objects)
        and on_rests(objects)
        and roll_at_target(objects)
        and abs(shaft.pos[2] - roll.pos[2] - SAG) <= 0.002
        and roll.pos[2] - OUTER_R >= TOP + 0.040
    )


def invalid(objects):
    for name, obj in objects.items():
        if name not in SIZES:
            continue
        r = rotation(obj)
        if r is None or obj.pos[2] - np.abs(r[2]) @ (np.asarray(SIZES[name]) / 2) < TOP - 0.005:
            return True
    return False


def handling_violation(objects, tools):
    """Conservative TCP exclusion zone, checked at every 20 Hz control sample."""
    points = np.asarray(tools, dtype=float)
    roll = objects["roll"]
    r = rotation(roll)
    if r is None or points.shape != (2, 3) or not np.isfinite(points).all():
        return True
    local = (points - roll.pos) @ r
    outside = np.maximum(np.abs(local) - np.asarray(SIZES["roll"]) / 2, 0)
    return bool(np.any(np.linalg.norm(outside, axis=1) < 0.030))


def score(objects):
    """Geometry only; battery_roll adds measured release and a sticky handling violation."""
    if invalid(objects) or not all(objects[n].resting(0.005, 0.1) for n in SIZES) or not threaded(objects):
        return 0.0
    if suspended(objects):
        return 1.0
    return 0.5 if roll_at_target(objects) else 0.25


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    return float(np.linalg.norm(np.asarray(objects["roll"].pos[:2]) - TARGET))


def demo():
    """Candidate insertion, synchronized two-end lift, bearing placement and release."""
    # Equivalent opposing wrist orientations keep each arm within its roll limit.
    yaw, closed = math.pi / 2, -0.0095
    travel = TOP + 0.165
    inserted_z = TOP + OUTER_R + TCP_SHAFT_OFFSET
    rx, ry = ROLL_XY
    right_grip = (rx, ry - 0.100, inserted_z)
    left_grip = (rx, ry + 0.100, inserted_z)
    right = [
        Waypoint(1.5, (SHAFT_XY[0], SHAFT_XY[1] - 0.100, travel), FINGER_OPEN, "above spindle end", yaw),
        Waypoint(
            1.5,
            (SHAFT_XY[0], SHAFT_XY[1] - 0.100, TOP + SHAFT_HALF_HEIGHT + TCP_SHAFT_OFFSET),
            FINGER_OPEN,
            "descend to spindle",
            yaw,
        ),
        Waypoint(
            0.8,
            (SHAFT_XY[0], SHAFT_XY[1] - 0.100, TOP + SHAFT_HALF_HEIGHT + TCP_SHAFT_OFFSET),
            closed,
            "grasp spindle end",
            yaw,
        ),
        Waypoint(1.5, (SHAFT_XY[0], SHAFT_XY[1] - 0.100, travel), closed, "lift spindle", yaw),
        Waypoint(1.5, (rx, ry - 0.320, travel), closed, "align with bore", yaw),
        Waypoint(1.5, (rx, ry - 0.320, inserted_z), closed, "lower to bore axis", yaw),
        Waypoint(1.0, (rx, ry - 0.320, inserted_z), closed, "settle before insertion", yaw),
        Waypoint(3.0, right_grip, closed, "thread spindle through core", yaw),
    ]
    insert_time = sum(w.duration for w in right)
    left = [
        Waypoint(insert_time, (0.140, 0.240, travel), FINGER_OPEN, "wait outside protected roll"),
        Waypoint(1.5, (rx, ry + 0.100, travel), FINGER_OPEN, "above other spindle end", -math.pi / 2),
        Waypoint(1.2, left_grip, FINGER_OPEN, "descend to other end", -math.pi / 2),
        Waypoint(0.8, left_grip, closed, "grasp other end", -math.pi / 2),
    ]
    right.append(Waypoint(3.5, right_grip, closed, "wait for second grip", yaw))
    for side, plan, sign in (("left", left, 1), ("right", right, -1)):
        yaw = -math.pi / 2 if side == "left" else math.pi / 2
        plan.extend(
            [
                Waypoint(2.0, (rx, ry + sign * 0.100, travel), closed, "lift through core", yaw),
                Waypoint(2.5, (rx, sign * 0.100, travel), closed, "center between bearing rests", yaw),
                Waypoint(2.5, (TARGET[0], sign * 0.100, travel), closed, "approach above bearing cheeks", yaw),
                Waypoint(1.5, (TARGET[0], sign * 0.100, SHAFT_Z + TCP_SHAFT_OFFSET), closed, "lower onto rests", yaw),
                Waypoint(1.0, (TARGET[0], sign * 0.100, SHAFT_Z + TCP_SHAFT_OFFSET), FINGER_OPEN, "release", yaw),
                Waypoint(1.0, (TARGET[0], sign * 0.100, travel), FINGER_OPEN, "retreat", yaw),
                Waypoint(1.5, (0.140, sign * 0.240, travel), FINGER_OPEN, f"park {side}", yaw),
            ]
        )
    return {
        side: [Waypoint(w.duration * 1.5, w.pos, w.finger, w.label, w.yaw) for w in plan]
        for side, plan in (("left", left), ("right", right))
    }
