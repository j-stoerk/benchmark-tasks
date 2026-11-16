"""Measurable fixture specification and history-dependent loaded-gap scoring."""

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HZ = 20
RADIUS = 0.1
FACE = 0.3
ROLLER_MASS = math.pi * RADIUS**2 * FACE * 7850
BEARING_HALF_SPAN = 0.175
SPRING = 20_000_000.0  # N/m PER bearing: declared benchmark spring, not OEM data
SPRING_DAMPING = 30_000.0  # Ns/m per bearing
PROOF_FORCES = np.array([8000.0, 12000.0])  # N, upward separating proof forces
STATIONS = np.linspace(-0.14, 0.14, 5)
GAP = 200.0  # micrometres
TOLERANCE = 10.0
PROOF_SECONDS = 3.0
JOINT_NAMES = ["elevation", "inclination"]
INITIAL = np.array([650.0, 350.0, 0.0])
LOW = np.array([-1200.0, -1200.0, 0.0])
HIGH = np.array([2000.0, 2000.0, 1.0])
INSTRUCTION = (
    "Commission the laboratory calender's loaded nip. Using the left and right bearing adjustments and the five "
    "gap gauges, set all five gaps to 200 +/-10 micrometres. Turn on the separating proof load (8 kN left, "
    "12 kN right), compensate the resulting gap and tilt changes, and hold all five gaps in range under the full "
    "load for 3 uninterrupted seconds. Leave the proof load on. Avoid roll contact."
)
DOCS = (
    "Action [left_setting_um, right_setting_um, proof_on]. Settings are the unloaded spring-rest gap at Y=-175 "
    "and +175 mm, each -1200..2000 um; they move at at most 100 um/s. Increasing a setting opens that end. "
    "proof_on is 0 or 1; the physical force ramps over 5 seconds. The two 20 MN/m bearing springs support a "
    "74 kg rigid steel roll on native elevation/inclination joints. Gravity and the applied forces change the "
    "actual gap: settings are not gap readings. Five measured gaps (um) at Y=-140,-70,0,70,140 mm, two measured "
    "proof forces (N), and rate-limited spring-rest setpoints (um plus proof fraction) accompany two RGB views. "
    "Use the "
    "gauges for feedback. Resolution 0.1 um; ideal sensor noise/bias zero. Native fixed frame, rotation-locked "
    "200 mm diameter, 300 mm face rolls. A service spreader applies upward forces at the bearings. No foil "
    "densification, thermal crown, roll bending or OEM machine calibration is claimed. One chunk repeats a "
    "single action for 1..200 control steps at 20 Hz; at most 30 LLM calls and 1200 actions. Full score requires "
    "the full proof load and all five gaps in range continuously for 3 seconds with low roll velocity. "
    "Any gap <=0 um or a nonfinite/detached/out-of-range mechanism invalidates the trial."
)


def revision():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob("*")):
        if path.suffix in (".py", ".usda"):
            digest.update(path.name.encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def action_space():
    from inspect_robots.spaces import Box

    return Box(shape=(3,), low=LOW, high=HIGH)


def joint_targets(settings):
    left, right = np.asarray(settings, float)[:2] * 1e-6
    return np.array([(left + right) / 2, math.atan((right - left) / (2 * BEARING_HALF_SPAN))])


def gaps(elevation, inclination):
    """Vertical surface separation of the two true cylinders at x=0.

    The native upper axis lies at z=0.5+elevation, rotated about +X.
    Include the cylinder's secant-radius correction, not just its axis distance.
    """
    return 1e6 * (elevation + STATIONS * math.tan(inclination) - RADIUS * (1 / math.cos(inclination) - 1))


def equilibrium(settings, forces=PROOF_FORCES, gravity=True):
    """Offline prediction for bench comparison, NEVER used to move native bodies."""
    target = joint_targets(settings)
    # One 1 kg translation carriage plus the tilt link carrying the steel roll.
    fz = np.sum(forces) - ((ROLLER_MASS + 1.0) * 9.81 if gravity else 0)
    torque = BEARING_HALF_SPAN * (forces[1] - forces[0])
    return target + [fz / (2 * SPRING), torque / (2 * SPRING * BEARING_HALF_SPAN**2)]


@dataclass
class Proof:
    elapsed: float = 0.0
    invalid: bool = False

    def update(self, reading, requested):
        try:
            q = np.asarray(reading["joint_positions"], float)
            v = np.asarray(reading["joint_velocities"], float)
            force = np.asarray(reading["applied_proof_forces_N"], float)
            g = np.asarray(reading["gaps_um"], float)
            finite = all(np.isfinite(x).all() for x in (q, v, force, g))
            valid = (
                finite
                and q.shape == (2,)
                and v.shape == (2,)
                and force.shape == (2,)
                and g.shape == (5,)
                and abs(q[0]) < 0.0029
                and abs(q[1]) < 0.009
                and np.min(g) > 0
                and np.min(force) >= -0.1
                and np.max(force) <= 12_100
            )
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.invalid = True
        if self.invalid:
            self.elapsed = 0.0
            return 0.0, False
        still = abs(v[0]) < 5e-6 and abs(v[1]) < 3e-5
        good = np.abs(g - GAP) <= TOLERANCE
        loaded = np.all(np.abs(force - PROOF_FORCES) <= 0.01 * PROOF_FORCES)
        unloaded = np.max(np.abs(force)) < 10
        score = 0.0
        if still and unloaded and good[2]:
            score = 0.25
            if good.all():
                score = 0.5
        if requested and loaded and good.all() and still:
            self.elapsed += 1 / HZ
        else:
            self.elapsed = 0.0
        success = self.elapsed >= PROOF_SECONDS - 1e-9
        return (1.0 if success else score), success
