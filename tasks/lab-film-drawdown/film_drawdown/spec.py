"""Physical film drawdown, with outcome scoring from a visible coating map."""

import hashlib
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

SLUG = "lab-film-drawdown"
EMBODIMENT = "film_drawdown"
MORPHOLOGY = "single-six-axis-cartesian-gimbal-tool-robot"
HZ = 20
HEIGHT, WIDTH = 720, 1280
CAMERAS = ("overview_cam", "detail_cam")
FIELDS = (("encoders", (6,), "m,rad"), ("process_readouts", (4,), "N,mL,percent"))
JOINTS = ["slide_x", "slide_y", "slide_z", "rotate_z", "rotate_y", "rotate_x"]
INITIAL = np.array([0.0, -0.14, 0.88, 0.0, 0.0, 0.0, 0.0])
LOW = np.array([-0.30, -0.30, 0.74, -np.pi, -1.7, -1.7, 0.0])
HIGH = np.array([0.30, 0.30, 1.15, np.pi, 1.7, 1.7, 1.0])
VARIANTS = ("normal", "no-grip", "no-material")
TOOL_MASS = 0.7
TOOL_COM = (0.0, 0.0, 0.018)
MOUNT_Z = 0.045
SURFACE_Z = 0.754
FREE_OBJECTS = []
CALL_LIMIT = 30
CAMERA_VIEWS = [
    ("overview_cam", (0.48, -0.60, 1.12), (0.0, 0.0, 0.80), 24),
    ("detail_cam", (0.16, -0.30, 1.12), (0.0, 0.0, 0.754), 38),
]
INSTRUCTION = (
    "Use the fixed-gap film applicator to draw the blue coating over the entire "
    "80 by 200 mm rectangle on the glass plate. Leave no dry stripe wider than 5 mm. "
    "Finish with the applicator lifted clear of the coated rectangle."
)
DOCS = """Single six-axis Cartesian-gimbal tool robot, with a real 100x30x30mm,
0.7kg fixed-gap applicator mounted as its end tool. This task evaluates the
drawdown stroke, not picking up the applicator or adjusting its gap.
Actions [x_m,y_m,z_m,yaw_rad,pitch_rad,roll_rad,unused_0]. XYZ refers to the
midpoint between the two feet at their bottom faces. Z/Y/X rotations share this
point. Limits +/-0.30m XY, Z0.74..1.15m, yaw +/-pi, pitch/roll +/-1.7rad.
Native force-limited Cartesian servo:120mm/s,40N per slide;0.8rad/s,2Nm per gimbal.
The glass top is Z0.754m. Its coating rectangle is X-0.040..0.040m,
Y-0.100..0.100m. The initial3mL paint bead is across X at Y-0.105m.
The applicator is100mm wide along its localX and travels along local+Y,
with feet at localX+/-45mm. Its working gap is0.10mm. Keep both feet on
the glass and pull the loaded bead through the marked area; an airborne
or tilted bar will not deposit a valid continuous film. RGB shows the actual
discretized coating, including missed regions. A finite contact/viscous response
stands in for paint; the bar and substrate contacts are native PhysX.
The coating model is a benchmark approximation, not a material-quality or
electrode-thickness certification. Native force is used to trigger physical
paint response; real scoring uses only the visible coated area and tool clearance.
Readouts are tool normal contactN, remaining beadmL, coated percentage and
largest dry-stripe widthmm. Success:>=95percent painted, no dry stripe>5mm,
then lift the tool at least30mm above the glass for one second. At most30LLM
calls/2400native actions. Each call may contain at most400actions. The LLM is
code-aware and receives native camera images and actual encoder/readout values.
"""


def action_space():
    return Box(shape=(7,), low=LOW, high=HIGH)


def revision():
    h = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob("*")):
        if path.suffix in (".py", ".usda"):
            h.update(path.name.encode() + path.read_bytes())
    return h.hexdigest()


class Proof:
    def __init__(self):
        self.elapsed = 0.0
        self.invalid = False

    def update(self, reading, command):
        state = reading["process"]
        coverage = state["coverage"]
        clear = reading["tool_pos"][2] >= SURFACE_Z + 0.030
        qualifying = coverage >= 0.95 and state["largest_dry_stripe_mm"] <= 5 and clear
        self.elapsed = self.elapsed + 1 / HZ if qualifying else 0.0
        score = 1.0 if self.elapsed >= 1.0 else (0.5 if coverage >= 0.60 else (0.25 if coverage >= 0.25 else 0.0))
        return score, score == 1.0
