"""Remove a dashboard phone mount with its rubber cup and dashboard pad intact."""

import math
import os
import sys
from pathlib import Path

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena import demo as yam_demo
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

sys.path.insert(0, str(Path(__file__).parent))
from suction_parts import Body, Piece, install_parts, clear_seal, update_lever_drive, tips

SCENE = "station"
TOP = TABLE_TOP[SCENE]
X = 0.360
CUP_BOTTOM = TOP + 0.008
CUP = (X, 0, CUP_BOTTOM + 0.007)
HANDLE_Y = 0.106
HANDLE_HEIGHT = CUP_BOTTOM + 0.070
LEVER = (X - 0.009, -0.034, CUP_BOTTOM + 0.038)
TAB = (X + 0.031, -0.019, CUP_BOTTOM + 0.004)
PARK_HANDLE = (0.382, 0.245)
RUBBER = (0.14, 0.15, 0.17)
PLASTIC = (0.18, 0.22, 0.27)
GRIP = (0.25, 0.28, 0.37)
LEVER_COLOR = (0.43, 0.47, 0.54)

OBJECTS = [
    Cylinder("dash_pad", (X, 0, TOP + 0.004), (0.29, 0.29, 0.31), radius=0.048, height=0.008),
    Body("mount", CUP, PLASTIC, mass=0.22, size=(0.074, 0.18, 0.08), pieces=(
        Piece((0, 0, -0.003), (0.074, 0.074, 0.008), RUBBER, cylinder=True),
        Piece((0, 0, 0.010), (0.049, 0.049, 0.022), PLASTIC, cylinder=True),
        Piece((0, 0.018, 0.042), (0.032, 0.037, 0.052), PLASTIC),
        Piece((0, 0.079, 0.058), (0.026, 0.125, 0.020), GRIP),
        Piece((0, 0.145, 0.057), (0.061, 0.031, 0.033), PLASTIC),
    ), fixed_seal=True),
    Body("lever", LEVER, LEVER_COLOR, mass=0.011, size=(0.026, 0.044, 0.007), pieces=(
        Piece((0, -0.023, 0), (0.026, 0.044, 0.007), LEVER_COLOR),
    ), joint_limits=(-74, 1), stiffness=0.0023),
    Body("tab", TAB, RUBBER, mass=0.003, size=(0.016, 0.034, 0.004), pieces=(
        Piece((0, -0.012, 0), (0.016, 0.034, 0.004), RUBBER),
    ), initial_tilt=-27, joint_limits=(-35, 55), stiffness=0.0045),
]

class Suction:
    def reset(self):
        self.vented = False
        self.lever_open = False
        self.forced = False
        self.steps = 0
        self.tab_pivot = np.array(TAB)
        self.mount_basis = np.eye(3)
        self.tab_angle = -27.0

    def update(self, objects):
        self.steps += 1
        if not self.vented and np.linalg.norm(objects["mount"].pos - CUP) > 0.008:
            self.forced = True
        mount = quat_xyzw_to_matrix(objects["mount"].quat_xyzw)
        lever = mount.T @ quat_xyzw_to_matrix(objects["lever"].quat_xyzw)
        tab = mount.T @ quat_xyzw_to_matrix(objects["tab"].quat_xyzw)
        lever_angle = math.degrees(math.atan2(lever[2, 1], lever[1, 1]))
        tab_angle = math.degrees(math.atan2(tab[2, 1], tab[1, 1]))
        self.tab_pivot, self.mount_basis, self.tab_angle = objects["tab"].pos, mount, tab_angle
        open_now = lever_angle < -38
        if open_now != self.lever_open:
            self.lever_open = open_now
            update_lever_drive(-68.0 if open_now else 0.0)
        if self.lever_open and tab_angle < -49 and not self.vented:
            clear_seal()
            self.vented = True
        if os.environ.get("SUCTION_DIAGNOSTICS") and self.steps % 10 == 0:
            print("MOUNT", self.steps, round(lever_angle, 2), round(tab_angle, 2),
                  "vented", self.vented, "forced", self.forced, "tips",
                  np.round(list(tips.values()), 3).tolist(), flush=True)


process = Suction()
process.reset()
install_parts(process.reset)

# These prop motions require the scripted demo's wrist to follow a physical
# hinge. Keep the native IK planner and the native joint action interface.
stock_grip_rotation = yam_demo.grip_rotation


def grip_rotation(yaw):
    return yaw if isinstance(yaw, np.ndarray) else stock_grip_rotation(yaw)


yam_demo.grip_rotation = grip_rotation


def success(objects):
    mount = objects["mount"]
    return (
        process.vented and not process.forced and mount.resting()
        and np.linalg.norm(mount.pos[:2] - (X, 0)) >= 0.115
        and mount.low[2] < TOP + 0.008
        and all(np.linalg.norm(tip - mount.pos) >= 0.12 for tip in tips.values())
    )


def score(objects):
    process.update(objects)
    if process.forced:
        return 0.0
    return 1.0 if success(objects) else 0.5 if process.vented else 0.25 if process.lever_open else 0.0


def outward_peel_pose(tilt, clearance_degrees):
    pose = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), tilt) @ stock_grip_rotation(0)
    closing, approach = pose[:, 0], pose[:, 2]
    turn = -math.copysign(math.radians(clearance_degrees), np.cross(closing, approach)[0])
    return axis_angle_matrix(closing, turn) @ pose


class PeelTab:
    """Keep the grip on the tab arc while bringing the edge up to vent."""

    def __init__(self, duration, finger, label, advance=11.0):
        self.duration, self.finger, self.label, self.advance = duration, finger, label, advance

    @property
    def angle(self):
        return max(-57.0, process.tab_angle - self.advance)

    @property
    def clearance(self):
        return float(np.clip((-process.tab_angle - 40.0) / 8.0, 0.0, 1.0))

    @property
    def yaw(self):
        tilt = math.radians(90 + self.angle)
        return process.mount_basis @ outward_peel_pose(tilt, 14.0 * self.clearance)

    @property
    def pos(self):
        swing = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(self.angle))
        wanted = process.tab_pivot + process.mount_basis @ swing @ (0.002 * self.clearance, -0.023, 0)
        observed = tips.get("right", wanted)
        motion = np.clip(wanted - observed, -0.010, 0.010)
        return tuple(observed + 1.6 * motion - (PAD_TIP_PAST_TCP - 0.004) * self.yaw[:, 2])


def demo():
    opened, closed = FINGER_OPEN, FINGER_CLOSED
    hover = TOP + 0.145
    handle = (X, HANDLE_Y, HANDLE_HEIGHT + 0.005)
    left_yaw = -math.pi / 2
    right_yaw = math.pi / 2
    down = stock_grip_rotation(right_yaw)
    lean = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(18)) @ down
    peel_grip = outward_peel_pose(math.radians(63), 0.0)
    middle_peel_grip = outward_peel_pose(math.radians(52), 0.0)

    def pivot_grasp(pivot, offset, angle, duration, finger, label):
        rot = axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(angle))
        xyz = tuple(np.asarray(pivot) + rot @ np.asarray(offset))
        return Waypoint(duration, xyz, finger, label, lean)

    lever_offset = (0.004, -0.035, 0.006)
    tab_offset = (0.004, -0.020, 0.006)
    tip = pivot_grasp(LEVER, lever_offset, 0, 1.6, opened, "straddle the lever tip")
    tab = pivot_grasp(TAB, tab_offset, -27, 1.6, opened, "approach the raised tab")
    tab_opening = -0.009
    tab_end = (X + 0.031, -0.052, TOP + 0.026)
    outside_tab = (tab_end[0], -0.082, tab_end[2])
    partially_peeled = (tab_end[0], -0.050, TOP + 0.035)
    return {
        "left": [
            Waypoint(2.0, (*handle[:2], hover), opened, "approach the mount arm", left_yaw),
            Waypoint(1.3, handle, opened, "lower around the mount arm", left_yaw),
            Waypoint(0.8, handle, closed, "take the mount's weight", left_yaw),
            Waypoint(22.4, handle, closed, "support while opening and venting", left_yaw),
            Waypoint(1.6, (*handle[:2], hover), closed, "lift the vented mount", left_yaw),
            Waypoint(1.5, (*PARK_HANDLE, hover), closed, "bring to the clear bench", left_yaw),
            Waypoint(1.3, (*PARK_HANDLE, handle[2] - 0.006), closed, "put it down", left_yaw),
            Waypoint(0.8, (*PARK_HANDLE, handle[2] - 0.006), opened, "release", left_yaw),
            Waypoint(1.1, (PARK_HANDLE[0], PARK_HANDLE[1] + 0.07, hover), opened, "withdraw", left_yaw),
        ],
        "right": [
            Waypoint(2.5, (*tip.pos[:2], hover), opened, "approach the release lever", right_yaw),
            tip,
            pivot_grasp(LEVER, lever_offset, 0, 1.0, closed, "grasp lever"),
            pivot_grasp(LEVER, lever_offset, -21, 1.0, closed, "start the lever arc"),
            pivot_grasp(LEVER, lever_offset, -43, 1.2, closed, "open lever past detent"),
            pivot_grasp(LEVER, lever_offset, -47, 0.8, closed, "let lever pass the detent"),
            pivot_grasp(LEVER, lever_offset, -47, 0.7, opened, "release lever"),
            Waypoint(1.5, (*tab.pos[:2], hover), opened, "move to cup pull tab", right_yaw),
            tab,
            pivot_grasp(TAB, tab_offset, -27, 0.8, closed, "pinch the tab"),
            pivot_grasp(TAB, tab_offset, -44, 1.0, closed, "start peeling"),
            pivot_grasp(TAB, tab_offset, -56, 1.0, closed, "lift the rim to let air in"),
            Waypoint(1.2, (tab.pos[0], tab.pos[1] - 0.050, hover), opened, "draw the lever clear", right_yaw),
            Waypoint(1.2, (*outside_tab[:2], hover), tab_opening, "approach from outside the peel edge", peel_grip),
            Waypoint(1.1, outside_tab, tab_opening, "lower alongside the flexible edge", peel_grip),
            Waypoint(1.3, tab_end, tab_opening, "slide one pad beneath the raised tab", peel_grip),
            Waypoint(0.8, tab_end, closed, "pinch the protruding tab", peel_grip),
            Waypoint(1.1, partially_peeled, closed, "start peeling along the tab arc", middle_peel_grip),
            PeelTab(2.9, closed, "follow the tab arc until the cup vents"),
            PeelTab(0.7, tab_opening, "release the peeled edge", advance=0.0),
            Waypoint(1.1, (X + 0.04, -0.14, hover), opened, "withdraw from the mount", right_yaw),
        ],
    }
