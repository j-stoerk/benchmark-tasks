"""Counterexamples to pressure-only, motor-supported and leaking test claims."""

import unittest
from copy import deepcopy

from dryer_vacuum import spec


def ready(pressure=20000):
    force = max(0, 101300 - pressure) * 0.25
    return dict(
        encoders=[0, 0.8, 90, 2.8, 90, 2.8, 0, 0],
        seal_compression_mm=[0.8] * 16,
        seal_forces_N=[100] * 16,
        clamp_forces_N=[1000] * 2,
        pressure_pa=pressure,
        applied_pressure_force_N=force,
        frame_support_N=force,
        motor_on=0,
        door_motor_torque_Nm=0,
        pump_on=0,
    )


def evolve(proof, reading, frames=240):
    for _ in range(frames):
        result = proof.update(reading)
    return result


class Contract(unittest.TestCase):
    def test_complete_isolated_loaded_test(self):
        self.assertEqual(evolve(spec.Proof(), ready()), (1, True))

    def test_air_pressure_seating_only_gets_quarter_credit(self):
        self.assertEqual(evolve(spec.Proof(), ready(101300)), (0.25, False))

    def test_displayed_vacuum_without_native_load_path_gets_no_score(self):
        for key in ("applied_pressure_force_N", "frame_support_N"):
            row = ready()
            row[key] = 0
            self.assertEqual(evolve(spec.Proof(), row), (0, False))

    def test_door_motor_cannot_substitute_for_clamps_and_seal(self):
        row = ready()
        row["motor_on"] = 1
        self.assertEqual(evolve(spec.Proof(), row), (0, False))

    def test_running_pump_cannot_hide_a_leak(self):
        row = ready()
        row["pump_on"] = 1
        self.assertEqual(evolve(spec.Proof(), row), (0.5, False))

    def test_open_vent_cannot_pass_an_isolated_hold(self):
        row = ready()
        row["encoders"][7] = 10
        self.assertEqual(evolve(spec.Proof(), row), (0.5, False))

    def test_opening_or_unlatching_under_vacuum_latches_zero(self):
        for index, value in ((0, -2), (2, 80), (3, 1)):
            proof = spec.Proof()
            row = ready()
            row["encoders"][index] = value
            self.assertEqual(proof.update(row), (0, False))
            self.assertEqual(evolve(proof, ready()), (0, False))

    def test_one_missing_or_undercompressed_segment_defeats_seating(self):
        for key, value in (("seal_compression_mm", 0.1), ("seal_forces_N", 0)):
            row = ready()
            row[key][9] = value
            self.assertEqual(evolve(spec.Proof(), row), (0, False))

    def test_500_pa_limit_cannot_be_reset_to_an_above_target_baseline(self):
        proof = spec.Proof()
        evolve(proof, ready(), 60)
        self.assertEqual(evolve(proof, ready(20600)), (0.25, False))

    def test_hold_requires_ten_continuous_seconds(self):
        proof = spec.Proof()
        row = ready()
        evolve(proof, row, 190)
        interrupted = deepcopy(row)
        interrupted["pump_on"] = 1
        proof.update(interrupted)
        self.assertFalse(evolve(proof, row, 100)[1])
        self.assertTrue(evolve(proof, row, 120)[1])

    def test_nonfinite_sensor_cannot_be_used_as_a_task_score(self):
        row = ready()
        row["frame_support_N"] = float("nan")
        proof = spec.Proof()
        self.assertEqual(proof.update(row), (0, False))
        self.assertTrue(proof.invalid)


if __name__ == "__main__":
    unittest.main()
