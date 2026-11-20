"""Gather a bread bag neck and engage its hand-applied retaining clip."""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from bag_parts import Prop, Solid, install, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
LOAF = (0.367, 0.139, TOP + 0.048)
NECK = (LOAF[0], 0.015, TOP + 0.071)
CLIP = (0.335, -0.195, TOP + 0.043)
WORK_Y = -0.066
WORK_Z = TOP + 0.068
DARK = (0.32, 0.21, 0.12)
FILM = (0.67, 0.76, 0.79)
BREAD = (0.56, 0.31, 0.15)
CLIP_RED = (0.70, 0.10, 0.08)
STAND = (0.25, 0.28, 0.30)
N_FOLDS = 5
FOLD_WIDTH = 0.009
FOLD_HALF_LENGTH = 0.051
CLIP_WINDOW_HALF_HEIGHT = 0.0093 - 0.0028 / 2
SUCCESS_HOLD_S = 2.0
SUCCESS_DRIFT = 0.0015
SUCCESS_ANGLE_COS = math.cos(math.radians(5) / 2)
NECK_CREASE = np.array((NECK[0] - 0.019, NECK[1] - FOLD_HALF_LENGTH, NECK[2]))

OBJECTS = [
    Prop("loaf", LOAF, BREAD, mass=0.26, size=(0.127, 0.244, 0.096), children=(
        Solid((0, 0, 0), (0.120, 0.240, 0.088), BREAD),
        Solid((0, 0, 0.045), (0.104, 0.237, 0.009), DARK),
        Solid((0, 0, 0), (0.122, 0.242, 0.096), FILM, collide=False, opacity=0.19),
        Solid((0, -0.110, 0.031), (0.088, 0.035, 0.007), FILM, collide=False, opacity=0.47),
    )),
    Prop("clip", CLIP, CLIP_RED, mass=0.0008, size=(0.0349, 0.0012, 0.0214), children=(
        Solid((0.00945, 0, 0), (0.016, 0.0012, 0.0214), CLIP_RED),
        Solid((-0.005, 0, -0.0093), (0.014, 0.0012, 0.0028), CLIP_RED),
        Solid((-0.005, 0, 0.0093), (0.014, 0.0012, 0.0028), CLIP_RED),
    )),
    Prop("clip_rest", CLIP[:2] + (TOP,), STAND, size=(0.044, 0.025, 0.035), children=(
        Solid((0, 0, 0.016), (0.044, 0.025, 0.032), STAND),
        Solid((0, -0.003, 0.0335), (0.043, 0.004, 0.003), STAND),
        Solid((0, 0.003, 0.0335), (0.043, 0.004, 0.003), STAND),
    )),
    Prop("bag_rest", (NECK[0] - 0.010, 0.007, TOP), STAND, size=(0.046, 0.016, 0.061), children=(
        Solid((0, 0, 0.0305), (0.046, 0.016, 0.061), STAND),
    )),
    Prop("loaf_guides", LOAF[:2] + (TOP,), STAND, size=(0.147, 0.221, 0.016), children=(
        Solid((-0.069, 0.031, 0.008), (0.009, 0.190, 0.016), STAND),
        Solid((0.069, 0.031, 0.008), (0.009, 0.190, 0.016), STAND),
        Solid((0, 0.129, 0.008), (0.147, 0.009, 0.016), STAND),
    )),
]

for sign in (-1, 1):
    name = "clip_upper_tooth" if sign > 0 else "clip_lower_tooth"
    tooth = tuple((np.array(CLIP) + (-0.0138, 0, sign * 0.0065)).tolist())
    pivot = tuple((np.array(CLIP) + (-0.0070, 0, sign * 0.0093)).tolist())
    OBJECTS.append(Prop(name, tooth, CLIP_RED, mass=0.0001,
                        size=(0.0073, 0.0012, 0.0084), previous="clip", edge=pivot,
                        limits=tuple(sorted((0.0, 13.0 * sign))), hinge_stiffness=0.00038,
                        children=(Solid((0, 0, sign * 0.0012), (0.0073, 0.0012, 0.0060), CLIP_RED),
                                  Solid((0.0012, 0, -sign * 0.0028), (0.0048, 0.0012, 0.0032),
                                        CLIP_RED, pitch=22.0 * sign))))

edge = NECK_CREASE.copy()
for index in range(N_FOLDS):
    pitch = math.copysign(math.radians(65), 1 - 2 * (index % 2))
    span = FOLD_WIDTH * np.array([math.cos(pitch), 0, -math.sin(pitch)])
    center = edge + span / 2
    OBJECTS.append(Prop(f"film_{index}", tuple(center.tolist()), FILM,
                        size=(FOLD_WIDTH, 2 * FOLD_HALF_LENGTH, 0.00055), mass=0.00028,
                        angle_y=pitch, previous=f"film_{index-1}" if index else "loaf",
                        edge=tuple(edge.tolist()), children=(
                            Solid((0, 0, 0), (FOLD_WIDTH, 2 * FOLD_HALF_LENGTH, 0.00055), FILM, opacity=0.65),
                        )))
    edge = edge + span

clip_pose = None
film_centre = None
stable_frames = 0
stable_anchor = None


def reset():
    global clip_pose, film_centre, stable_frames, stable_anchor
    clip_pose = None
    film_centre = None
    stable_frames = 0
    stable_anchor = None
    tips.clear()


install(reset)


def film_section(clip, film):
    clip_basis = quat_xyzw_to_matrix(clip.quat_xyzw)
    fold_basis = quat_xyzw_to_matrix(film.quat_xyzw)
    start = clip_basis.T @ (film.pos - clip.pos)
    direction = clip_basis.T @ fold_basis[:, 1]
    if abs(direction[1]) < 0.85:
        return None
    at_clip = -start[1] / direction[1]
    if abs(at_clip) >= FOLD_HALF_LENGTH - 0.008:
        return None
    centre = start + direction * at_clip
    lateral = clip_basis.T @ fold_basis[:, 0] * FOLD_WIDTH / 2
    return np.array([centre + lateral, centre - lateral])


def film_in_clip(clip, film):
    limits = film_section(clip, film)
    if limits is None:
        return False
    return np.all((-0.0118 < limits[:, 0]) & (limits[:, 0] < 0.0015) & (abs(limits[:, 2]) < CLIP_WINDOW_HALF_HEIGHT))


def closure(objects):
    clip, loaf = objects["clip"], objects["loaf"]
    folds = [objects[f"film_{i}"] for i in range(N_FOLDS)]
    loaf_basis = quat_xyzw_to_matrix(loaf.quat_xyzw)
    anchor = loaf.pos + loaf_basis @ (NECK_CREASE - LOAF)
    half_spans = [quat_xyzw_to_matrix(fold.quat_xyzw)[:, 0] * (FOLD_WIDTH / 2) for fold in folds]
    leading = [fold.pos - span for fold, span in zip(folds, half_spans)]
    trailing = [fold.pos + span for fold, span in zip(folds, half_spans)]
    if any(np.linalg.norm(a - b) > 0.005 for a, b in zip([anchor, *trailing[:-1]], leading)):
        return 0, False
    in_clip = sum(film_in_clip(clip, fold) for fold in folds)
    rear = loaf_basis @ (0, -0.120, 0) + loaf.pos
    on_bench = abs(loaf.pos[2] - LOAF[2]) < 0.006 and loaf_basis[2, 2] > math.cos(math.radians(10))
    intact = rear[1] - clip.pos[1] > 0.024 and on_bench
    release = len(tips) == 2 and all(np.linalg.norm(tip - clip.pos) > 0.060 for tip in tips.values())
    secured = in_clip == N_FOLDS and intact and release
    return in_clip, secured


def score(objects):
    global clip_pose, film_centre, stable_frames, stable_anchor
    clip_pose = objects["clip"]
    loaf = objects["loaf"]
    folds = [objects[f"film_{i}"] for i in range(N_FOLDS)]
    film_centre = np.mean([fold.pos for fold in folds], axis=0)
    in_clip, secured = closure(objects)
    if not secured:
        stable_frames, stable_anchor = 0, None
    elif stable_anchor is not None and (
        np.linalg.norm(clip_pose.pos - stable_anchor[0]) <= SUCCESS_DRIFT
        and abs(np.dot(clip_pose.quat_xyzw, stable_anchor[1])) >= SUCCESS_ANGLE_COS
        and np.linalg.norm(loaf.pos - stable_anchor[2]) <= SUCCESS_DRIFT
    ):
        stable_frames += 1
    else:
        stable_frames = 1
        stable_anchor = (clip_pose.pos.copy(), clip_pose.quat_xyzw.copy(), loaf.pos.copy())
    if os.getenv("BAG_DIAGNOSTICS"):
        sections = [film_section(clip_pose, fold) for fold in folds]
        print("BAG", in_clip, "secured", secured, "steady frames", stable_frames,
              "clip", np.round(objects["clip"].pos, 4).tolist(),
              "normal", np.round(quat_xyzw_to_matrix(clip_pose.quat_xyzw)[:, 1], 3).tolist(),
              "opening mm", [np.round(section[:, (0, 2)] * 1000, 1).tolist() if section is not None else None
                             for section in sections],
              "film", np.round([fold.pos for fold in folds], 4).tolist(),
              "tips", np.round(list(tips.values()), 4).tolist(), flush=True)
    return 1.0 if success(objects) else 0.5 if in_clip == N_FOLDS else 0.25 if in_clip >= 2 else 0.0


def success(objects):
    return stable_frames >= round(SUCCESS_HOLD_S * 20)


class ClipGrip:
    yaw = 0.0

    def __init__(self, duration, finger, label):
        self.duration, self.finger, self.label = duration, finger, label

    @property
    def pos(self):
        nominal = np.array(CLIP) + (0.010, 0, 0.014)
        if clip_pose is None:
            return tuple(nominal)
        basis = quat_xyzw_to_matrix(clip_pose.quat_xyzw)
        wanted = clip_pose.pos + basis @ (0.010, 0, 0.001)
        target = wanted + (0, 0, PAD_TIP_PAST_TCP)
        return tuple(nominal + np.clip(target - nominal, -0.015, 0.015))


class ClipSeat:
    def __init__(self, duration, finger, label, nominal, orientation):
        self.duration, self.finger, self.label = duration, finger, label
        self.nominal, self.yaw = np.array(nominal), orientation

    @property
    def pos(self):
        tip = tips.get("right")
        if clip_pose is None or film_centre is None or tip is None:
            return tuple(self.nominal)
        clip_target = np.array((film_centre[0] + 0.002, WORK_Y, film_centre[2] - 0.001))
        target = clip_target + tip - clip_pose.pos - PAD_TIP_PAST_TCP * self.yaw[:, 2]
        correction = np.clip(target - self.nominal, (-0.016, -0.008, -0.010), (0.016, 0.008, 0.010))
        return tuple(self.nominal + correction)


def demo():
    def waypoint_rotation(orientation):
        return orientation if isinstance(orientation, np.ndarray) else grip_rotation(orientation)

    yam_demo.grip_rotation = waypoint_rotation
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    world_x = np.array((1.0, 0.0, 0.0))
    left = axis_angle_matrix(world_x, math.radians(-16)) @ grip_rotation(-math.pi / 2)
    right = axis_angle_matrix(world_x, math.radians(26)) @ grip_rotation()

    def tcp_at(tip, rotation):
        return tuple(np.asarray(tip) - PAD_TIP_PAST_TCP * rotation[:, 2])

    h = TOP + 0.148
    neck = (NECK[0] - 0.009, 0.002, NECK[2] - 0.015)
    hold = tcp_at(neck, left)
    feed = (hold[0] + 0.009, hold[1], hold[2])
    pick = (CLIP[0] + 0.010, CLIP[1], CLIP[2] + 0.014)
    carry = tcp_at((neck[0] + 0.043, -0.125, WORK_Z + 0.031), right)
    enter = tcp_at((neck[0] + 0.043, WORK_Y, WORK_Z), right)
    seat = tcp_at((neck[0] + 0.002, WORK_Y, WORK_Z - 0.003), right)
    held_position = None

    class HoldSeatedClip:
        yaw = right

        def __init__(self, duration, finger, label):
            self.duration, self.finger, self.label = duration, finger, label

        @property
        def pos(self):
            nonlocal held_position
            if held_position is None:
                tip = tips.get("right")
                held_position = seat if tip is None else tcp_at(tip, right)
            return held_position

    return {
        "left": [
            Waypoint(1.7, (*hold[:2], h), -0.018, "approach the bag neck", left),
            Waypoint(1.2, hold, -0.018, "straddle the loose folds", left),
            Waypoint(0.9, hold, closed, "gather the film", left),
            Waypoint(1.3, hold, closed, "keep the gathered neck level", left),
            Waypoint(3.1, hold, closed, "support the neck until the clip arrives", left),
            Waypoint(1.8, feed, closed, "feed the gathered neck into the clip", left),
            Waypoint(1.0, feed, closed, "hold the film until the clip is seated", left),
            Waypoint(0.8, feed, -0.010, "release the gathered film", left),
            Waypoint(0.9, (feed[0], feed[1], h), -0.010, "lift vertically clear of the film", left),
            Waypoint(0.8, (hold[0], hold[1] + 0.090, h), opened, "withdraw from the closure", left),
            Waypoint(3.0, (hold[0], hold[1] + 0.090, h), opened, "check that the film stays clipped", left),
        ],
        "right": [
            Waypoint(1.7, (*pick[:2], h), -0.005, "approach the upright clip", 0.0),
            ClipGrip(1.4, -0.006, "straddle the back of the clip"),
            ClipGrip(1.0, closed, "take the clip by its back"),
            Waypoint(1.5, (*pick[:2], h), closed, "lift clear of the slotted rest", 0.0),
            Waypoint(1.5, carry, closed, "bring the clip beside the neck", right),
            Waypoint(1.3, enter, closed, "align the slot with the gathered film", right),
            ClipSeat(2.5, closed, "push the neck through the slot", seat, right),
            HoldSeatedClip(2.2, closed, "support the seated clip while the film expands"),
            HoldSeatedClip(1.0, opened, "let go of the clip"),
            Waypoint(1.1, (seat[0] + 0.040, -0.127, h), opened, "withdraw from the closure", right),
            Waypoint(3.0, (seat[0] + 0.040, -0.127, h), opened, "check that the clip stays on", right),
        ],
    }
