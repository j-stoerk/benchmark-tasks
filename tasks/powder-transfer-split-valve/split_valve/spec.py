"""Mechanical docking, load proof and actual coupled-disc cycle contract."""

import hashlib
import math
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

SLUG = "powder-transfer-split-valve"
EMBODIMENT = "powder_valve"
MORPHOLOGY = "powered-split-valve-docking-and-proof-machine"
HZ, WIDTH, HEIGHT = 20, 1280, 720
CAMERAS = ("overview_cam", "valve_cam")
CALL_LIMIT, RUN_LIMIT = 30, 1600
VARIANTS = ("normal", "unlocked-proof")
FIELDS = (
    ("encoders", (6,), "m,m,m,deg,deg,deg"),
    ("seal_normals", (16,), "N"),
    ("retaining_contact", (1,), "N"),
    ("net_proof", (1,), "N"),
)
JOINTS = ["slide_x", "slide_y", "slide_z", "collar", "active", "passive"]
PAYLOAD = 8.0
SEAT_ANGLE_OFFSET = math.pi / 8
DOCK_Z = 1.0
INITIAL = np.array([0.008, -0.006, 1.10, 0, 0, 0], float)
LOW = np.array([-0.025, -0.025, 0.98, 0, 0, 0], float)
HIGH = np.array([0.025, 0.025, 1.12, 32, 90, 1], float)
INSTRUCTION = (
    "Dock and mechanically prove the DN100 powder-transfer split-valve fixture. Align the supported 8 kg "
    "upper assembly at X=Y=0, interface height Z=1.000 m, with both discs closed. Turn the retaining collar "
    "to 30 degrees, pass the two-second 100 N net separation proof, then turn proof off. Open the active "
    "disc to 90 degrees so actual face contact opens the passive disc too. Close both discs, return the "
    "collar to zero and raise the upper half to Z=1.100 m. Leave both detached discs seated closed. Do not "
    "open an undocked or unlocked half. This is a mechanical readiness test; it does not certify containment."
)
DOCS = """Action [upper_x_m,upper_y_m,upper_interface_z_m,collar_degrees,active_disc_degrees,proof].
XYZ ranges: +/-25 mm laterally, Z=0.98..1.12 m. Collar 0..32 degrees; active
disc 0..90 degrees; proof 0/1. XY slew 20 mm/s, Z 25 mm/s, collar 20 deg/s,
disc 30 deg/s. The upper payload including disc and seal carriers is 8 kg.
The lower half is fixed at Z=1.000 m. Dock with XYZ=(0,0,1.000), discs closed,
and collar zero so the three upper lugs pass the actual collar slots. Rotate
the collar to 30 only after seating. Its native ledges then resist separation.
The native lever pins/collar block mechanically inhibit opening while unlocked
and inhibit unlocking while open. An attempt is not penalized unless the actual
mechanism opens an undocked/unlocked vessel or loses its specified support.
Proof 1 disables Z positioning and applies upward effort equal to payload
weight plus 100 N; the native guide remains free along Z. Only real collar-lug
contact can retain it. It is not held by a position drive during the proof.
Allow four seconds of proof to include settling and the two-second loaded hold.
Stationarity uses one-second encoder ranges <=0.1 mm and <=0.5 degree; loaded
baseline changes must remain within those bounds. Stop proof before operating the discs. The active disc is motorized. The upper
passive disc has only a return spring: native contact between the complementary
half-thickness discs must transmit rotation. It is not a follower joint.
Eight spring-backed seal carriers per half provide measured native seating
contact. Native 10 kN/m radial springs (32 Ns/m) and 1 kN/m axial flexures
(9 Ns/m, +/-8 mm) approximate elastomer seating and lip deflection. Each
25 g seat unit is a 5 g guide plus a 20 g shoe; total upper payload is 8 kg.
Shoe boundaries lie on the rotor Y axis so opposite sides of the lip can
deflect in opposite axial directions. Continuous elastomer strain is not modeled.
After opening BOTH discs to at least 88 degrees, close them to within 2 degrees,
then unlock and raise the upper half 100 mm. The final detached halves each need
all eight seal contact gauges above 0.5 N. Leak/particle transport is excluded.
The six encoder readings are X,Y,Z,collar,active,passive, with angles in degrees.
Sixteen radial disc-contact gauges, downward collar contact on the upper half, and the net
proof effort accompany two RGB views. No score flags are policy inputs.
At most 30 LLM calls and 1600 control actions at 20 Hz; each call may select a
sequence of waypoints with 1..400 total actions. Use encoder/force feedback.
"""


def radial_force(vector, index):
    force = np.asarray(vector, float)
    if force.shape != (3,) or not np.isfinite(force).all():
        raise ValueError("Expected a finite native contact-force vector")
    angle = (index % 8) * math.pi / 4 + SEAT_ANGLE_OFFSET
    return max(0.0, float(force @ np.array([math.cos(angle), math.sin(angle), 0])))


def action_space():
    return Box(shape=(6,), low=LOW, high=HIGH)


def revision():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.iterdir()):
        if path.suffix in {".py", ".usda"}:
            digest.update(path.name.encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()


class Proof:
    def __init__(self):
        self.invalid = False
        self.elapsed = 0.0
        self.retained = False
        self.opened = False
        self.closed_after_open = False
        self.history = []
        self.baseline = None

    def update(self, reading, command):
        try:
            q = np.asarray(reading["encoders"], float)
            v = np.asarray(reading["velocities"], float)
            seals = np.asarray(reading["seal_normals"], float)
            contact = float(reading["retaining_contact_N"])
            net = float(reading["net_proof_N"])
            finite = all(np.isfinite(a).all() for a in (q, v, seals)) and math.isfinite(contact + net)
            valid = finite and q.shape == (6,) and v.shape == (6,) and seals.shape == (16,)
            valid = valid and abs(q[0]) < 0.04 and abs(q[1]) < 0.04 and 0.95 < q[2] < 1.16
        except (KeyError, TypeError, ValueError, IndexError):
            valid = False
        if not valid:
            self.invalid = True
        if self.invalid:
            self.elapsed = 0.0
            return 0.0, False
        docked = np.linalg.norm(q[:2]) <= 0.0005 and abs(q[2] - DOCK_Z) <= 0.0005
        locked = 27 <= q[3] <= 32
        discs_closed = np.max(np.abs(q[4:])) <= 2
        seals_seated = np.min(seals) >= 0.5
        # Actual exposure is a failure; an interlock successfully blocking an
        # attempted motion is not itself a failure.
        # The 20-degree slots clear the 6.6-degree lug footprint only within
        # about 6.7 degrees of zero. At 10 degrees there is positive ledge
        # overlap; the pin interlock blocks unlocking before this boundary.
        if not discs_closed and not (docked and q[3] >= 10):
            self.invalid = True
            return 0.0, False
        self.history.append(q.copy())
        self.history = self.history[-(HZ + 1) :]
        window = np.asarray(self.history)
        still = (
            len(self.history) == HZ + 1
            and np.max(np.ptp(window[:, :3], axis=0)) <= 0.0001
            and np.max(np.ptp(window[:, 3:], axis=0)) <= 0.5
        )
        requested = command[5] >= 0.5
        loaded = abs(net - 100) <= 1 and contact >= 80
        if requested and self.retained and not (docked and locked and loaded and still):
            self.retained = self.opened = self.closed_after_open = False
        if requested and docked and locked and loaded and still:
            if self.baseline is None:
                self.baseline = q.copy()
            retained_pose = (
                np.max(np.abs(q[:3] - self.baseline[:3])) <= 0.0001
                and np.max(np.abs(q[3:] - self.baseline[3:])) <= 0.5
            )
            if not retained_pose:
                self.elapsed = 0.0
                self.baseline = q.copy()
                self.retained = self.opened = self.closed_after_open = False
            self.elapsed += 1 / HZ
            self.retained |= self.elapsed >= 2 - 1e-9
        else:
            self.elapsed = 0.0
            self.baseline = None
        if (
            self.retained
            and not requested
            and abs(net) < 1
            and docked
            and locked
            and np.min(q[4:]) >= 88
            and abs(q[4] - q[5]) <= 2
        ):
            self.opened = True
        if self.opened and docked and locked and discs_closed and seals_seated:
            self.closed_after_open = True
        complete = (
            self.retained
            and self.closed_after_open
            and not requested
            and abs(net) < 1
            and q[2] >= 1.095
            and abs(q[3]) <= 2
            and discs_closed
            and seals_seated
            and still
        )
        if complete:
            return 1.0, True
        if docked and locked and self.retained:
            return 0.5, False
        if docked and discs_closed and seals_seated and still:
            return 0.25, False
        return 0.0, False
