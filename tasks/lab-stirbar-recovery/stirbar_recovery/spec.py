"""A free magnetic stirring bar must pass through the actual flask neck."""

import hashlib
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

SLUG = "lab-stirbar-recovery"
EMBODIMENT = "stirbar_recovery"
MORPHOLOGY = "single-six-axis-cartesian-gimbal-tool-robot"
HZ = 20
HEIGHT, WIDTH = 720, 1280
CAMERAS = ("overview_cam", "detail_cam")
FIELDS = (("encoders", (6,), "m,rad"), ("process_readouts", (1,), "N"))
JOINTS = ["slide_x", "slide_y", "slide_z", "rotate_z", "rotate_y", "rotate_x"]
INITIAL = np.array([0.0, 0.0, 1.07, 0.0, 0.0, 0.0, 0.0])
LOW = np.array([-0.30, -0.30, 0.74, -np.pi, -1.7, -1.7, 0.0])
HIGH = np.array([0.30, 0.30, 1.15, np.pi, 1.7, 1.7, 1.0])
VARIANTS = ("normal", "no-grip", "no-material")
TOOL_MASS = 0.05
TOOL_COM = (0.0, 0.0, 0.15)
MOUNT_Z = 0.310
FREE_OBJECTS = [("stirbar", (0.0045, 0.0, 0.7555), (0.0, 0.0, 0.0, 1.0))]
RIM_Z = 0.855
BAR_LENGTH = 0.025
BAR_RADIUS = 0.0035
CALL_LIMIT = 30
CAMERA_VIEWS = [
    ("overview_cam", (0.36, -0.45, 1.06), (0.0, 0.0, 0.86), 30),
    ("detail_cam", (0.15, -0.25, 1.03), (0.0, 0.0, 0.85), 45),
]
INSTRUCTION = (
    "Recover the white magnetic stirring bar from the empty flask using the "
    "magnetic retriever. Bring the entire bar at least25mm above the flask rim "
    "and keep it on the retriever there for two seconds."
)
DOCS = """Single six-axis Cartesian-gimbal robot with a300mm magnetic retriever
mounted as its end tool. This trial begins with the tool ready; tool pickup is
not tested. The magnet is permanent, not an on/off switch. All tool and bar
contacts and bar motion are native dynamics. No grasp joint or part pose write.
Action [x_m,y_m,z_m,yaw_rad,pitch_rad,roll_rad,unused_0]. XYZ is the centre
of the10mm diameter rounded magnet head; the8mm rod extends along local+Z.
Native speed120mm/s,40N slide force limits,0.8rad/s and2Nm rotary limits.
Bench topZ0.750m; flask bottom inner surfaceZ0.752m; mouth centreXY=0,
rimZ0.855m, neck inside diameter22mm. The100mL-class borosilicate flask has
a56mm inside-diameter lower body, a tapered shoulder and a22mm inside neck.
A free25x7mm cylindrical PTFE-encapsulated magnetic stirbar,3g, lies near
the centre of the floor, approximately alongX. You must lower the magnet to
capture it and lift/reorient it through the neck. Pulling a transverse bar
against the neck does not count as extraction. Let gravity and the finite
magnetic force reorient it, or adjust the tool pose. The actual field model
uses two poles in each magnet, including orientation torque and balanced robot reaction.
It is a bounded short-range approximation, not a measured OEM field curve.
Only native RGB, joint encoders and tool contactN are policy observations.
Success is judged from video/ruler: the entire bar>=25mm above the rim,
magnetically retained, for2s. Lower partial tiers credit a15mm magnetic lift
or reaching the mouth. No prescribed trajectory or forced magnetic capture.
At most30 LLM calls and2400control actions;400actions maximum per call.
"""


def action_space():
    return Box(shape=(7,), low=LOW, high=HIGH)


def revision():
    h = hashlib.sha256()
    for p in sorted(Path(__file__).parent.glob("*")):
        if p.suffix in (".py", ".usda"):
            h.update(p.name.encode() + p.read_bytes())
    return h.hexdigest()


def matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def geometry(reading):
    bar = reading["parts"]["stirbar"]
    p = np.array(bar["pos"])
    u = matrix(bar["quat"])[:, 0]
    extent = 0.5 * BAR_LENGTH * abs(u[2]) + BAR_RADIUS * np.sqrt(max(0.0, 1 - u[2] * u[2]))
    ends = [p + sign * 0.010 * u for sign in (-1, 1)]
    near = min(float(np.linalg.norm(np.asarray(reading["tool_pos"]) - end)) for end in ends)
    return {
        "bottom_z": float(p[2] - extent),
        "centre_z": float(p[2]),
        "retained": near <= 0.013,
        "speed": float(np.linalg.norm(bar["lin_vel"])),
    }


class Proof:
    def __init__(self):
        self.elapsed = 0.0
        self.invalid = False

    def update(self, reading, command):
        g = geometry(reading)
        if not np.isfinite([g["bottom_z"], g["speed"]]).all() or g["centre_z"] < 0.70:
            self.invalid = True
        valid = g["retained"] and g["bottom_z"] >= RIM_Z + 0.025 and not self.invalid
        self.elapsed = self.elapsed + 1 / HZ if valid else 0.0
        score = (
            1.0
            if self.elapsed >= 2.0
            else (
                0.5
                if g["retained"] and g["centre_z"] >= RIM_Z
                else (0.25 if g["retained"] and g["bottom_z"] >= 0.767 else 0.0)
            )
        )
        return (0.0, False) if self.invalid else (score, score == 1.0)
