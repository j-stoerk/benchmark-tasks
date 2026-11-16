"""Full-size tooling recipe, manipulation contract and measured retention rubric."""

import hashlib
import math
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

HZ = 20
BASE_Z = 0.8
ARBOR_R = 0.0324
BORE = 0.0325
BLADE_R = 0.050
SPACER_R = 0.040
BLADE_H = 0.002
SEGMENTS = 96
POSITIONS = np.array([0.050, 0.125, 0.205, 0.280])
TOLERANCE = 0.00005
JOINT_NAMES = ["slide_x", "slide_y", "slide_z", "jaw_0", "jaw_1", "jaw_2"]
INITIAL = np.array([-0.15, -0.4, 1.35, 0, 0, 0], float)
LOW = np.array([-0.18, -0.45, 0.78, 0, 0, 0], float)
HIGH = np.array([0.65, 0.45, 1.40, 1, 1, 1], float)
HEIGHTS = [0.050, BLADE_H, 0.073, BLADE_H, 0.078, BLADE_H, 0.073, BLADE_H, 0.048]
# Initial magazines intentionally do not encode assembly order.
SLOTS = [4, 0, 6, 2, 8, 1, 3, 5, 7]
PARTS = []
for index, (height, slot) in enumerate(zip(HEIGHTS, SLOTS, strict=True)):
    blade = index % 2 == 1
    radius = BLADE_R if blade else SPACER_R
    density = 14500 if blade else 7850
    PARTS.append(
        {
            "name": f"part_{index}",
            "height": height,
            "radius": radius,
            "blade": blade,
            "mass": math.pi * (radius**2 - BORE**2) * height * density,
            "source": [0.24 + (slot % 3) * 0.16, -0.25 + (slot // 3) * 0.25, BASE_Z + height / 2],
            "target_z": BASE_Z + sum(HEIGHTS[:index]) + height / 2,
        }
    )
TOP = BASE_Z + sum(HEIGHTS)
DOCS = """Powered three-axis open-centre, three-jaw assembly head; no YAM arms.
Action [tool_x_m,tool_y_m,tool_z_m,grip,clamp,proof]. The first three are jaw-centre
setpoints, X -0.18..0.65, Y -0.45..0.45, Z 0.78..1.40 m; axis slew 0.40 m/s.
Grip 0 opens all three jaws, grip 1 closes them with 50 N per drive and 20 mm/s
joint-speed limit. The 30 mm tall pads grip OUTSIDE each ring, centred at tool Z.
The palm has a clear central bore; do not try to grasp inside a blade bore.
Clamp 1 moves the retracted platen from Y=-0.45 to Y=0 at 0.10 m/s, then applies
500 N DOWNWARD. Clamp 0 raises it before parking at Y=-0.45. The native PLC
waits for actual alignment before lowering; allow about 8 seconds. Clamp only after all nine parts have been stacked
and the head has moved to Z>=1.30 m. Proof 1 applies 25 N upward and 0.5 Nm Z
torque to EACH of the four free blades (100 N / 2 Nm total). It never supports
their weight. Use the four actual blade-height gauges and clamp normal-force
gauge to verify the final condition. The native shaft and all free ring contacts
carry the stack; no grasp attachment or prescribed part trajectory is used.
The arbor centre is X=Y=0, shoulder top Z=0.800 m. Blade cutting faces are their
LOWER faces at 50,125,205,280 mm from this shoulder; the final stack is 330 mm.
Assemble spacer 50 mm, lower knife 2 mm, spacer 73 mm, knife, spacer 78 mm,
knife, spacer 73 mm, knife, spacer 48 mm. Interchangeable 73 mm spacers
and identical blades may exchange identity. All magazines have their support
top at Z=0.800 m. Source XYZ centres and physical sizes are listed in instruction.
Lift a held part to Z=1.35 m before crossing the table. Place its bottom about
1 mm above the current stack, open the jaws fully, and retract before moving.
Observations: native RGB, actual tool XYZ/mean normalized jaw extension and last
clamp/proof commands, three pad normal forces in N, pressure-plate contact N,
and four blade cutting-face heights in mm (-1 when that blade is off the arbor).
No part pose, scorer status or successful-grasp oracle is supplied. The height
gauges correspond to practical axial metrology, not gripper setpoints.
At most 30 LLM calls and 2400 control actions at 20 Hz. A response may include
one action or a sequence of action/step pairs, with at most 400 total steps per
call; use observations between uncertain grasps. Success needs all components
seated and released, four cutting positions +/-0.05 mm, real clamp contact, and
a three-second loaded retention interval with under 0.05 mm translation and
under 1 degree rotation of each blade. Cutting/burr quality is not tested.
"""
INSTRUCTION = (
    "Assemble and retain the pilot electrode-slitter cassette on the vertical service arbor. "
    "Build the 330 mm stack so its four blade cutting faces are at 50, 125, 205 and 280 mm above the shoulder. "
    "Apply the 500 N axial clamp, release the handling head and pass the three-second 100 N / 2 Nm retention "
    "test. Leave clamp and proof on. Do not drop tooling off the table. Initial magazine parts: "
    + "; ".join(
        f"{p['name']} {'blade' if p['blade'] else 'spacer'} h={p['height'] * 1000:g} mm at "
        f"({p['source'][0]:.2f},{p['source'][1]:.2f},{p['source'][2]:.5f}) m"
        for p in PARTS
    )
    + "."
)


def action_space():
    return Box(shape=(6,), low=LOW, high=HIGH)


def revision():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.iterdir()):
        if path.suffix in {".py", ".usda"}:
            digest.update(path.name.encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()


def rotation(q):
    q = np.asarray(q, float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError("Invalid quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def geometry(readout):
    try:
        items = readout["parts"]
        poses = [np.asarray(items[p["name"]]["pos"], float) for p in PARTS]
        rotations = [rotation(items[p["name"]]["quat"]) for p in PARTS]
        velocity = [np.asarray(items[p["name"]]["lin_vel"], float) for p in PARTS]
        angular = [np.asarray(items[p["name"]]["ang_vel"], float) for p in PARTS]
        q = np.asarray(readout["tool"], float)
        force = float(readout["clamp_contact_N"])
        clamp_z = float(readout["clamp_z"])
        clamp_y = float(readout["clamp_y"])
        finite = all(a.shape == (3,) and np.isfinite(a).all() for a in poses + velocity + angular)
        valid = finite and q.shape == (6,) and np.isfinite(q).all() and math.isfinite(force + clamp_z + clamp_y)
        valid = valid and all(p[2] > 0.70 and abs(p[0]) < 0.9 and abs(p[1]) < 0.65 for p in poses)
        if not valid:
            return {"valid": False}
        centered = all(np.linalg.norm(p[:2]) <= 0.00015 for p in poses)
        upright = all(r[2, 2] >= math.cos(math.radians(0.2)) for r in rotations)
        intervals = sorted(
            (p[2] - part["height"] / 2, p[2] + part["height"] / 2) for p, part in zip(poses, PARTS, strict=True)
        )
        seated = abs(intervals[0][0] - BASE_Z) <= TOLERANCE
        seated &= all(abs(a[1] - b[0]) <= TOLERANCE for a, b in zip(intervals[:-1], intervals[1:], strict=True))
        cutting_faces = sorted(
            p[2] - BLADE_H / 2 - BASE_Z for p, part in zip(poses, PARTS, strict=True) if part["blade"]
        )
        recipe = np.max(np.abs(np.asarray(cutting_faces) - POSITIONS)) <= TOLERANCE
        stable = all(np.linalg.norm(v) < 0.0015 for v in velocity) and all(np.linalg.norm(v) < 0.05 for v in angular)
        released = np.linalg.norm(q[:2]) >= 0.18 or q[2] - 0.015 > max(TOP, clamp_z + 0.01) + 0.03
        clamped = force >= 300 and abs(clamp_z - 0.01 - TOP) <= 0.0001 and abs(clamp_y) <= 0.00015
        return {
            "valid": True,
            "assembled": bool(centered and upright and seated and recipe),
            "stable": bool(stable),
            "released": bool(released),
            "clamped": bool(clamped),
        }
    except (KeyError, TypeError, ValueError, IndexError):
        return {"valid": False}


class Proof:
    def __init__(self):
        self.invalid = False
        self.elapsed = 0.0
        self.baseline = None
        self.failed = False

    def update(self, readout, requested):
        g = geometry(readout)
        self.invalid |= not g["valid"]
        if self.invalid:
            return 0.0, False
        ready = g["assembled"] and g["stable"] and g["released"]
        score = 0.5 if ready and g["clamped"] else (0.25 if ready else 0.0)
        actual = np.asarray(readout.get("applied_proof", []), float)
        loaded = actual.shape == (2,) and np.isfinite(actual).all() and np.allclose(actual, [100, 2], atol=0.001)
        if not requested:
            self.failed = False
        if not (requested and loaded and ready and g["clamped"]):
            self.elapsed, self.baseline = 0.0, None
            return score, False
        blades = [readout["parts"][p["name"]] for p in PARTS if p["blade"]]
        current = [(np.asarray(p["pos"]), rotation(p["quat"])) for p in blades]
        if self.baseline is None:
            self.baseline = current
        retained = True
        for (pos, rot), (initial, orient) in zip(current, self.baseline, strict=True):
            angle = math.acos(float(np.clip((np.trace(orient.T @ rot) - 1) / 2, -1, 1)))
            retained &= np.linalg.norm(pos - initial) <= TOLERANCE and angle <= math.radians(1)
        self.failed |= not retained
        if retained and not self.failed:
            self.elapsed += 1 / HZ
        else:
            self.elapsed = 0.0
        success = self.elapsed >= 3.0 - 1e-9
        return (1.0 if success else score), success
