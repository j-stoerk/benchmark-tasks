"""Independent analytical limits, conservation and adversarial seal cases."""

import math
import unittest
from dataclasses import replace

from dryer_vacuum.gas import Chamber, Parameters, aperture, nozzle_per_area


def evolve(chamber, seconds, dt=0.1, **changes):
    inputs = dict(volume_m3=0.125, gaps_m=[0] * 16, pump_angle_deg=0, vent_angle_deg=0, pump_on=False)
    inputs.update(changes)
    for _ in range(round(seconds / dt)):
        reading = chamber.advance(dt, **inputs)
    return reading


class Gas(unittest.TestCase):
    def test_pump_down_matches_constant_speed_analytical_limit(self):
        p = replace(Parameters(), residual_channel_m=0)
        chamber = Chamber(p)
        evolve(chamber, 20, dt=0.05, pump_angle_deg=90, pump_on=True)
        analytical = p.ambient_pa * math.exp(-p.pump_m3_s * 20 / p.chamber_m3)
        self.assertLess(abs(chamber.pressure / analytical - 1), 0.001)

    def test_isolated_gas_obeys_pv_conservation_on_door_motion(self):
        chamber = Chamber(replace(Parameters(), residual_channel_m=0), initial_pa=20000)
        old_mass = chamber.mass
        evolve(chamber, 1, volume_m3=0.130)
        self.assertAlmostEqual(chamber.mass, old_mass, places=10)
        self.assertAlmostEqual(chamber.pressure, 20000 * 0.125 / 0.130, places=5)

    def test_native_pressure_load_must_be_twenty_kilonewtons(self):
        chamber = Chamber(initial_pa=20000)
        self.assertAlmostEqual(chamber.inward_door_force, 20325)

    def test_uniformly_compressed_seal_passes_only_the_proposed_gross_hold(self):
        chamber = Chamber(initial_pa=20000)
        evolve(chamber, 10, gaps_m=[-0.0003] * 16)
        self.assertLess(chamber.pressure - 20000, 500)
        self.assertGreater(chamber.pressure - 20000, 0)

    def test_one_hidden_gap_fails_the_pressure_hold(self):
        chamber = Chamber(initial_pa=20000)
        evolve(chamber, 10, gaps_m=[0.0001] + [-0.0003] * 15)
        self.assertGreater(chamber.pressure - 20000, 500)

    def test_open_door_cannot_be_evacuated_with_the_nominal_pump(self):
        chamber = Chamber()
        evolve(chamber, 5, gaps_m=[0.02] * 16, pump_angle_deg=90, pump_on=True)
        self.assertGreater(chamber.pressure, 100000)

    def test_running_pump_does_not_act_through_closed_isolation_valve(self):
        chamber = Chamber(replace(Parameters(), residual_channel_m=0))
        evolve(chamber, 5, pump_on=True)
        self.assertAlmostEqual(chamber.pressure, 101300, places=5)

    def test_open_vent_admits_air_and_releases_door_load(self):
        chamber = Chamber(initial_pa=20000)
        evolve(chamber, 30, vent_angle_deg=90)
        self.assertGreater(chamber.pressure, 100000)
        self.assertLess(chamber.inward_door_force, 325)

    def test_one_step_conserves_mass_even_with_large_openings(self):
        chamber = Chamber(initial_pa=20000)
        reading = evolve(
            chamber, 1, dt=1, gaps_m=[0.1] * 16, vent_angle_deg=90, pump_angle_deg=90, pump_on=True
        )
        self.assertLess(abs(reading["mass_balance_residual_kg"]), 1e-9)
        self.assertGreater(chamber.mass, 0)
        self.assertLessEqual(chamber.pressure, 101300)

    def test_nozzle_has_finite_sonic_limit_and_no_equal_pressure_flow(self):
        p = Parameters()
        self.assertEqual(nozzle_per_area(101300, 101300, p), 0)
        self.assertAlmostEqual(nozzle_per_area(101300, 0, p), nozzle_per_area(101300, 10000, p))

    def test_pressure_above_ambient_leaks_out_and_reverses_force(self):
        chamber = Chamber(initial_pa=120000)
        reading = evolve(chamber, 1, vent_angle_deg=90)
        self.assertLess(chamber.pressure, 120000)
        self.assertLess(reading["vent_inflow_kg_s"], 0)
        self.assertLess(chamber.inward_door_force, 0)

    def test_valve_and_geometry_errors_fail_before_state_mutation(self):
        chamber = Chamber()
        mass = chamber.mass
        with self.assertRaises(ValueError):
            evolve(chamber, 1, gaps_m=[0] * 15)
        with self.assertRaises(ValueError):
            aperture(float("nan"))
        with self.assertRaises(ValueError):
            aperture(91)
        self.assertEqual(chamber.mass, mass)

    def test_time_step_refinement_reduces_pump_integration_error(self):
        errors = []
        p = replace(Parameters(), residual_channel_m=0)
        exact = 101300 * math.exp(-p.pump_m3_s * 5 / p.chamber_m3)
        for dt in (0.1, 0.05, 0.025):
            chamber = Chamber(p)
            evolve(chamber, 5, dt=dt, pump_angle_deg=90, pump_on=True)
            errors.append(abs(chamber.pressure - exact))
        self.assertLess(errors[1], errors[0] * 0.51)
        self.assertLess(errors[2], errors[1] * 0.51)


if __name__ == "__main__":
    unittest.main()
