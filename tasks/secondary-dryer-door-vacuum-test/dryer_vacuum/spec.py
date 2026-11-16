"""Proposed dry vacuum-readiness contract; native implementation is incomplete."""

import hashlib
from pathlib import Path

import numpy as np
from inspect_robots.spaces import Box

SLUG = "secondary-dryer-door-vacuum-test"
HZ, PHYSICS_HZ, CALL_LIMIT, RUN_LIMIT = 20, 480, 30, 2400
CAMERAS = ("overview_cam", "door_cam")
JOINTS = ["door", "hinge_float", "swing_0", "draw_0", "swing_1", "draw_1", "pump_valve", "vent_valve"]
INITIAL = np.array([-60, 0, 0, 0, 0, 0, 90, 0, 0], float)
LOW = np.array([-100, 0, 0, 0, 0, 0, 0, 0, 0], float)
HIGH = np.array([0, 90, 10, 90, 10, 90, 90, 1, 1], float)
LINEAR_ENCODERS = [1, 3, 5]
ANGULAR_ENCODERS = [0, 2, 4, 6, 7]
INSTRUCTION = (
    "Close, mechanically retain and vacuum-test the full-size125 L electrode-dryer commissioning fixture. "
    "Close the32 kg hinged door with the vent open. Swing both clamps to90 degrees and draw their "
    "screws until all16 seal segments are compressed0.6..1.0 mm, with at most0.2 mm variation. "
    "Release the door motor. Close the vent, open the pump valve and switch the pump on until chamber "
    "pressure is at most20 kPa absolute. Then close the pump valve and switch the pump off. Pass a "
    "continuous10 s isolated hold with pressure rise at most0.5 kPa. Leave door and clamps closed, both "
    "valves closed, pump off and door motor disconnected. Do not unlatch or open the door under more than1 kPa differential "
    "pressure. This is a dry mechanical-readiness test, not a drying-quality or OEM leak-certification test."
)
DOCS = """Custom full-size cold commissioning machine; no arms or grasp attachments.
Action [door_deg,swing0_deg,draw0_mm,swing1_deg,draw1_mm,pump_valve_deg,vent_valve_deg,pump_on,door_motor_on].
Door-100..0 degrees; both clamp swings0..90; screw draws0..10 mm; valves0..90
(0 shut,90 open); final two controls are switches0/1. Start door-60, clamps
parked and retracted, pump off/shut, vent90 and door motor disconnected.
The steel door is560x560x13 mm,32.003 kg, hinged on its left edge. The cavity
opening is500x500 mm, volume125 L. Pressure acts over this opening, not over
the larger outside door. At20 kPa absolute, ambient101.3 kPa creates20,325 N
inward force and about5,691 Nm at the hinge. Native frame/stop contacts must
carry that load; a displayed pressure alone cannot establish success.
Door motor is60 Nm and should be used only with vent open; target slew20 deg/s.
Swing clamps before screw draw. About2 mm draw closes the pad clearance;
about2.8 mm total draw seats the door against0.8 mm seal-compression stops.
A4 mm screw target provides force bias; do not expect its encoder to reach4 mm
against the door. Use actual compression/contact readouts. Screw force cap1 kN
each, slew1 mm/s; swing and valve slew45 deg/s. No door position drive remains
when its motor switch is off. The floating hinge allows small normal seating.
Sixteen spring-backed shoes approximate one continuous face seal. Each spring
is125 kN/m, not an OEM material law; native actual gap controls gas leakage.
Check all16 compression readings0.6..1.0 mm, their range<=0.2 mm, both actual
clamp swings>=88 degrees, clamp draws2..5 mm and actual contact before pumping.
Turn off the door motor before the pressure test. Close vent, open pump and
switch it on. Nominal constant chamber-side pump speed16.5 m3/hour gives about
45 s to20 kPa; use the measured gauge. Isolate and switch the pump off before
the10 s hold. Both valves must be within0.5 degree of shut. The hold measures
actual gas mass/volume evolution and restarts if isolation or seating is lost.
Encoders are [door_deg,hinge_float_mm,swing0_deg,draw0_mm,swing1_deg,draw1_mm,pump_deg,vent_deg].
Other public readouts are absolute pressure,16 seal compressions/contacts,
two clamp contacts, frame reaction and the two actual switch states, plus RGB.
No score, hidden success flag or prescribed pressure is supplied. A1 s encoder
window and the full hold baseline must remain within0.1 mm and0.2 degree.
At most30 LLM calls and2400 actions at20 Hz; each response may select at most
400 explicit actions. Pump, seal and leakage parameters are reduced benchmark
choices requiring calibration. No heat, solvent, electrode moisture or plate
elasticity is modeled. Full-score native feasibility is not yet demonstrated.
"""


def action_space():
    return Box(shape=(9,), low=LOW, high=HIGH)


def revision():
    digest = hashlib.sha256()
    for path in sorted(Path(__file__).parent.iterdir()):
        if path.suffix in (".py", ".usda"):
            digest.update(path.name.encode() + b"\0" + path.read_bytes() + b"\0")
    return digest.hexdigest()


class Proof:
    def __init__(self):
        self.invalid = False
        self.history = []
        self.elapsed = 0.0
        self.baseline = None

    def update(self, reading):
        try:
            q = np.asarray(reading["encoders"], float)
            compression = np.asarray(reading["seal_compression_mm"], float)
            seal_forces = np.asarray(reading["seal_forces_N"], float)
            clamps = np.asarray(reading["clamp_forces_N"], float)
            pressure = float(reading["pressure_pa"])
            applied = float(reading["applied_pressure_force_N"])
            support = float(reading["frame_support_N"])
            motor_on = float(reading["motor_on"])
            motor_torque = float(reading["door_motor_torque_Nm"])
            pump_on = float(reading["pump_on"])
            valid = (
                q.shape == (8,)
                and compression.shape == seal_forces.shape == (16,)
                and clamps.shape == (2,)
                and all(
                    np.isfinite(value).all()
                    for value in (
                        q,
                        compression,
                        seal_forces,
                        clamps,
                        pressure,
                        applied,
                        support,
                        motor_on,
                        motor_torque,
                        pump_on,
                    )
                )
            )
            valid &= 0 <= pressure <= 106300 and -105 <= q[0] <= 2 and abs(q[1]) <= 2.5
        except (KeyError, TypeError, ValueError, IndexError):
            valid = False
        self.invalid |= not valid
        if self.invalid:
            return 0.0, False
        differential = 101300 - pressure
        latched = np.min(q[[2, 4]]) >= 85 and np.min(q[[3, 5]]) >= 2
        if abs(differential) > 1000 and (abs(q[0]) > 1 or not latched):
            self.invalid = True
            return 0.0, False
        self.history.append(q.copy())
        self.history = self.history[-(HZ + 1) :]
        still = len(self.history) == HZ + 1
        if still:
            ranges = np.ptp(self.history, axis=0)
            still = np.max(ranges[LINEAR_ENCODERS]) <= 0.1 and np.max(ranges[ANGULAR_ENCODERS]) <= 0.2
        seated = (
            abs(q[0]) <= 0.2
            and np.min(compression) >= 0.6
            and np.max(compression) <= 1
            and np.ptp(compression) <= 0.2
            and np.min(seal_forces) >= 50
        )
        retained = (
            np.min(q[[2, 4]]) >= 88
            and np.min(q[[3, 5]]) >= 2
            and np.max(q[[3, 5]]) <= 5
            and np.min(clamps) >= 50
        )
        released = motor_on < 0.5 and abs(motor_torque) <= 0.1
        expected = max(0.0, differential) * 0.25
        loaded = abs(applied - expected) <= 1 and support >= 0.8 * expected
        ready = bool(seated and retained and released and still)
        if differential > 1000 and not loaded:
            ready = False
        score = 0.5 if ready and pressure <= 20000 and loaded else 0.25 if ready else 0.0
        isolated = pump_on < 0.5 and np.max(np.abs(q[[6, 7]])) <= 0.5
        if not (ready and isolated and loaded):
            self.elapsed, self.baseline = 0.0, None
            return score, False
        if self.baseline is None:
            if pressure > 20000:
                return score, False
            self.baseline = (pressure, q.copy())
        initial_pressure, initial_q = self.baseline
        unchanged = (
            np.max(np.abs((q - initial_q)[LINEAR_ENCODERS])) <= 0.1
            and np.max(np.abs((q - initial_q)[ANGULAR_ENCODERS])) <= 0.2
        )
        if pressure - initial_pressure > 500 or not unchanged:
            self.elapsed, self.baseline = 0.0, None
            return score, False
        self.elapsed += 1 / HZ
        success = self.elapsed >= 10 - 1e-9
        return (1.0 if success else score), success
