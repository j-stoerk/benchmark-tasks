"""Adversarial checks of the service contract, independent of native motion."""

import unittest
from copy import deepcopy

import numpy as np
from split_valve import spec


def measurement(z=1.0, collar=30, active=0, passive=0, force=100, net=100):
    return {
        "encoders": [0, 0, z, collar, active, passive],
        "velocities": [0] * 6,
        "seal_normals": [2] * 16,
        "retaining_contact_N": force,
        "net_proof_N": net,
    }


def load_proof():
    proof = spec.Proof()
    for _ in range(80):
        proof.update(measurement(), [0, 0, 1, 30, 0, 1])
    return proof


def cycle(proof):
    opening = measurement(active=90, passive=90)
    opening["net_proof_N"] = 0
    proof.update(opening, [0, 0, 1, 30, 90, 0])
    proof.update(measurement(net=0), [0, 0, 1, 30, 0, 0])
    return settle_detached(proof)


def settle_detached(proof, reading=None):
    if reading is None:
        reading = measurement(z=1.1, collar=0, net=0)
    for _ in range(21):
        result = proof.update(reading, [0, 0, 1.1, 0, 0, 0])
    return result


class Contract(unittest.TestCase):
    def test_axial_contact_cannot_masquerade_as_radial_seating(self):
        for index in range(16):
            self.assertEqual(spec.radial_force([0, 0, 2000], index), 0)

    def test_radial_contact_is_measured_in_each_shoes_direction(self):
        for index in range(16):
            angle = index % 8 * np.pi / 4 + spec.SEAT_ANGLE_OFFSET
            self.assertAlmostEqual(spec.radial_force([2 * np.cos(angle), 2 * np.sin(angle), 7], index), 2)

    def test_full_measured_cycle(self):
        self.assertEqual(cycle(load_proof()), (1.0, True))

    def test_initial_detached_state_has_no_score(self):
        self.assertEqual(spec.Proof().update(measurement(z=1.1, collar=0), spec.INITIAL), (0.0, False))

    def test_stationary_seated_docking_is_partial(self):
        proof = spec.Proof()
        for _ in range(21):
            result = proof.update(measurement(collar=0), [0, 0, 1, 0, 0, 0])
        self.assertEqual(result, (0.25, False))

    def test_no_history_from_final_pose(self):
        self.assertEqual(cycle(spec.Proof()), (0.0, False))

    def test_commanded_proof_without_actual_effort_does_not_count(self):
        proof = spec.Proof()
        reading = measurement()
        reading["net_proof_N"] = 0
        for _ in range(100):
            proof.update(reading, [0, 0, 1, 30, 0, 1])
        self.assertFalse(proof.retained)

    def test_no_hidden_position_hold_as_retention(self):
        proof = spec.Proof()
        for _ in range(100):
            proof.update(measurement(force=0), [0, 0, 1, 30, 0, 1])
        self.assertFalse(proof.retained)

    def test_unlocked_retention_not_credited(self):
        proof = spec.Proof()
        for _ in range(100):
            proof.update(measurement(collar=0), [0, 0, 1, 0, 0, 1])
        self.assertFalse(proof.retained)

    def test_hold_must_be_continuous(self):
        proof = spec.Proof()
        for _ in range(21):
            proof.update(measurement(force=0), [0, 0, 1, 30, 0, 0])
        for _ in range(39):
            proof.update(measurement(), [0, 0, 1, 30, 0, 1])
        proof.update(measurement(force=0), [0, 0, 1, 30, 0, 1])
        proof.update(measurement(), [0, 0, 1, 30, 0, 1])
        self.assertFalse(proof.retained)

    def test_motion_under_load_cannot_count(self):
        proof = spec.Proof()
        reading = measurement()
        for index in range(80):
            reading["encoders"][2] = 1 + (0.0002 if index % 2 else -0.0002)
            proof.update(reading, [0, 0, 1, 30, 0, 1])
        self.assertFalse(proof.retained)

    def test_active_rotation_without_passive_cannot_complete(self):
        proof = load_proof()
        proof.update(measurement(active=90, passive=0, net=0), [0, 0, 1, 30, 90, 0])
        proof.update(measurement(net=0), [0, 0, 1, 30, 0, 0])
        self.assertEqual(settle_detached(proof), (0.0, False))

    def test_a_damaged_coupling_erases_partial_credit_and_latches_zero(self):
        proof = load_proof()
        result = proof.update(measurement(active=8, passive=5, net=0), [0, 0, 1, 30, 90, 0])
        self.assertEqual(result, (0.0, False))
        self.assertTrue(proof.invalid)
        self.assertEqual(cycle(proof), (0.0, False))

    def test_each_half_must_reseat_before_separation(self):
        proof = load_proof()
        proof.update(measurement(active=90, passive=90, net=0), [0, 0, 1, 30, 90, 0])
        reading = measurement(net=0)
        reading["seal_normals"][15] = 0
        proof.update(reading, [0, 0, 1, 30, 0, 0])
        self.assertEqual(settle_detached(proof), (0.0, False))

    def test_unlocked_exposure_latches_failure(self):
        proof = load_proof()
        self.assertEqual(proof.update(measurement(collar=0, active=5), [0, 0, 1, 0, 90, 0]), (0.0, False))
        self.assertEqual(cycle(proof), (0.0, False))

    def test_blocked_opening_attempt_is_not_exposure(self):
        proof = spec.Proof()
        for _ in range(21):
            result = proof.update(measurement(collar=0), [0, 0, 1, 0, 90, 0])
        self.assertEqual(result, (0.25, False))
        self.assertFalse(proof.invalid)

    def test_small_bounded_encoder_jitter_preserves_a_mechanical_hold(self):
        proof = spec.Proof()
        for index in range(80):
            reading = measurement()
            reading["encoders"][3] += 0.0001 * np.sin(index)
            reading["velocities"][3] = 3 * np.cos(index)
            proof.update(reading, [0, 0, 1, 30, 0, 1])
        self.assertTrue(proof.retained)

    def test_a_loaded_retention_failure_requires_a_fresh_proof(self):
        proof = load_proof()
        proof.update(measurement(force=0), [0, 0, 1, 30, 0, 1])
        self.assertFalse(proof.retained)
        self.assertEqual(cycle(proof), (0.0, False))

    def test_missing_disc_contact_prevents_final_score(self):
        proof = load_proof()
        cycle(proof)
        for index in range(16):
            reading = measurement(z=1.1, collar=0, net=0)
            reading["seal_normals"][index] = 0
            self.assertEqual(proof.update(reading, [0, 0, 1.1, 0, 0, 0]), (0.0, False))

    def test_actual_proof_effort_must_be_removed_before_final_success(self):
        proof = load_proof()
        cycle(proof)
        self.assertEqual(settle_detached(proof, measurement(z=1.1, collar=0, net=100)), (0.0, False))

    def test_nonfinite_or_malformed_instrument_fails_closed(self):
        for field, value in [
            ("encoders", [0] * 5),
            ("seal_normals", [2] * 15),
            ("retaining_contact_N", float("nan")),
        ]:
            reading = deepcopy(measurement())
            reading[field] = value
            self.assertEqual(spec.Proof().update(reading, [0, 0, 1, 30, 0, 1]), (0.0, False))

    def test_work_envelope_escape_latches_zero(self):
        proof = load_proof()
        self.assertEqual(proof.update(measurement(z=1.17), [0, 0, 1, 30, 0, 1]), (0.0, False))
        self.assertEqual(cycle(proof), (0.0, False))

    def test_control_dimensions_and_payload(self):
        self.assertEqual(spec.INITIAL.shape, (6,))
        self.assertTrue(np.all(spec.INITIAL >= spec.LOW))
        self.assertTrue(np.all(spec.INITIAL <= spec.HIGH))
        self.assertAlmostEqual(7.500 + 0.300 + 8 * 0.025, spec.PAYLOAD)


if __name__ == "__main__":
    unittest.main()
