"""SI-unit design, control contract and measured outcome rules. No simulator imports."""

import hashlib
import math
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

MASS = 200.0
RADIUS, BORE, WIDTH = 0.300, 0.0381, 0.370
SEGMENTS = 48
HZ = 20
EXPAND_TARGET = 0.0089
PROOF_TORQUE, PROOF_PULL, PROOF_SECONDS = 25.0, 300.0, 2.0
TARGET_Y, TARGET_Z = 0.350, 1.080
LOW = np.array([0, -0.05, 0.97, 0, 0, 0], float)
HIGH = np.array([0.24, 0.40, 1.15, 1, 1, 1], float)
INITIAL = np.array([0, 0.012, 0.985, 0, 0, 0], float)
JOINT_NAMES = ["slide_x", "slide_y", "slide_z", "jaw_0", "jaw_1", "jaw_2"]
INSTRUCTION = (
    "Transfer the full 200 kg electrode roll from the donor shaft to the powered receiving chuck. "
    "Align and insert the receiver, expand its core grip, and use the force gauges to transfer support "
    "before withdrawing the donor. Lift and move the supported "
    "roll to y=350 mm, axis height 1080 mm. Stop the carriage and pass the two-second retention test "
    "under 25 N m torque and 300 N axial pull. Do not drop the roll or rely on the donor during the test."
)
DOCS = """Powered industrial transfer machine, not a handheld spindle or YAM arm.
World X is the horizontal core axis; Z is up. The roll starts centred at (0,0,1) m.
The receiver enters from +X. Receiver nose X = 0.260 - insertion.
Commands: [insertion_m, lateral_y_m, axis_height_m, expand, donor_withdraw, proof].
Insertion 0..0.24 m; Y -0.05..0.40 m; Z 0.97..1.15 m; remaining commands 0..1.
Expand=1 applies +4 kN to each of three geared radial jaw drives; expand=0 retracts
with -4 kN per drive. Intermediate commands interpolate the drive force. The
9 mm mechanical stroke and 4 mm/s joint velocity limits remain physical.
Donor_withdraw=1 retracts it 0.40 m.
Proof=1 applies the actual 25 Nm +X torque and 300 N +X pull to the roll.
Axes slew at X=40, Y=60, Z=25 mm/s; donor=60 mm/s; jaws=4 mm/s.
Do not withdraw the donor until aligned insertion and expansion are complete.
Images: overview shows the full support path. The axial core close-up is aimed
at the final Y=350 mm, Z=1080 mm position; initially it looks beside the roll.
State machine_encoders is measured insertion, Y,
axis Z, mean normalized jaw extension, donor withdrawal fraction, last proof command.
No roll pose, scoring flags or success oracle is given to the policy.
Three ideal pad-force gauges report actual contact-force magnitudes in N;
all three must exceed 500 N before the donor is withdrawn.
donor_support is the actual donor normal-force resultant in N. Use this physical
gauge to preload the receiver and unload the donor before axial withdrawal.
"""


def action_space():
    return Box(shape=(6,), low=LOW, high=HIGH)


def revision():
    digest = hashlib.sha256()
    for p in sorted(Path(__file__).parent.iterdir()):
        if p.suffix in {".py", ".usda"}:
            digest.update(p.name.encode() + b"\0" + p.read_bytes() + b"\0")
    return digest.hexdigest()


def quat_matrix(q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError("Invalid XYZW orientation")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def geometry(readout):
    p = np.asarray(readout["roll_pos"], float)
    q = np.asarray(readout["receiver_q"], float)
    finite = (
        p.shape == (3,)
        and q.shape == (6,)
        and all(
            np.isfinite(np.asarray(readout[key], float)).all()
            for key in (
                "roll_pos",
                "roll_quat",
                "roll_lin_vel",
                "roll_ang_vel",
                "receiver_q",
                "donor_q",
                "donor_contact",
                "jaw_forces",
            )
        )
    )
    if not finite:
        return {k: False for k in ("valid", "inserted", "clamped", "donor_clear", "at_target", "stable")}
    try:
        axis = quat_matrix(readout["roll_quat"])[:, 0]
    except ValueError:
        return {k: False for k in ("valid", "inserted", "clamped", "donor_clear", "at_target", "stable")}
    nose, end = 0.260 + q[0], 0.660 + q[0]
    overlap = min(end, p[0] + WIDTH / 2) - max(nose, p[0] - WIDTH / 2)
    aligned = abs(axis[0]) >= math.cos(math.radians(2)) and np.linalg.norm(p[1:] - [q[1], 1 + q[2]]) <= 0.004
    inserted = overlap >= 0.140 and aligned
    return {
        "valid": bool(p[2] >= 0.700 and abs(p[0]) < 0.25 and abs(p[1]) < 0.6),
        "inserted": bool(inserted),
        "clamped": bool(inserted and np.min(q[3:]) >= 0.0055 and np.min(readout["jaw_forces"]) >= 500),
        "donor_clear": bool(readout["donor_q"] <= -0.350 and readout["donor_contact"] < 10),
        "at_target": bool(abs(p[1] - TARGET_Y) <= 0.005 and abs(p[2] - TARGET_Z) <= 0.005 and abs(p[0]) <= 0.005),
        "stable": bool(
            np.linalg.norm(readout["roll_lin_vel"]) < 0.010 and np.linalg.norm(readout["roll_ang_vel"]) < 0.15
        ),
    }


class Proof:
    """A real loaded retention interval; it never sets an object's state or creates a joint."""

    def __init__(self):
        self.elapsed = 0.0
        self.dropped = False
        self.baseline = None
        self.orientation = None
        self.passed = False
        self.failed = False

    def update(self, readout, requested, dt=1 / HZ):
        g = geometry(readout)
        self.dropped |= not g["valid"]
        if self.dropped:
            return 0.0, False
        ready = all(g[k] for k in ("clamped", "donor_clear", "at_target"))
        if requested:
            if self.baseline is None:
                self.baseline = np.asarray(readout["roll_pos"]).copy()
                self.orientation = quat_matrix(readout["roll_quat"])
            delta = np.linalg.norm(np.asarray(readout["roll_pos"]) - self.baseline)
            rel = self.orientation.T @ quat_matrix(readout["roll_quat"])
            angle = math.acos(float(np.clip((np.trace(rel) - 1) / 2, -1, 1)))
            self.failed |= not ready or delta > 0.003 or angle > math.radians(3)
            if not self.failed:
                self.elapsed += dt
                self.passed |= self.elapsed >= PROOF_SECONDS - 1e-9
        else:
            self.elapsed, self.baseline, self.orientation, self.failed = 0.0, None, None, False
            self.passed = False
        if self.passed and ready and requested and not self.failed:
            return 1.0, True
        if g["clamped"] and g["donor_clear"] and readout["roll_pos"][2] >= 1.045 and g["stable"]:
            return 0.5, False
        if g["inserted"] and not g["donor_clear"] and readout["donor_contact"] >= 1000 and g["stable"]:
            return 0.25, False
        return 0.0, False
