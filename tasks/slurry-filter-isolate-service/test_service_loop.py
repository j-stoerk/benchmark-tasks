"""Mass, pressure-work and service failure controls for the partial circuit."""

import math
import unittest

from filter_service.hydraulics import ATM, COVER_AREA
from filter_service.service_loop import RT, ServiceLoop, air_flow


def evolve(loop, angles, seconds, gaps=None, hz=240):
    for _ in range(round(seconds * hz)):
        loop.update(angles, gaps or [-0.0002] * 16, 1 / hz)
    return loop


class TwoPhaseCircuit(unittest.TestCase):
    def test_closed_node_conserves_both_masses(self):
        loop = ServiceLoop()
        initial = (loop.water, loop.air_mass, loop.gauge)
        evolve(loop, [0, 0, 0, 0], 1)
        for observed, expected in zip((loop.water, loop.air_mass, loop.gauge), initial, strict=True):
            self.assertAlmostEqual(observed, expected)

    def test_actual_cover_volume_performs_pressure_work(self):
        loop = ServiceLoop()
        water, air = loop.water, loop.air_mass
        loop.update([0, 0, 0, 0], [0] * 16, 1 / 240, capacity_m3=loop.capacity + 10e-6)
        self.assertEqual(loop.water, water)
        self.assertEqual(loop.air_mass, air)
        self.assertAlmostEqual(loop.absolute, air * RT / 60e-6)
        self.assertAlmostEqual(loop.cover_force, loop.gauge * COVER_AREA)

    def test_top_vent_relieves_pressure_but_does_not_empty_water(self):
        loop = ServiceLoop()
        water = loop.water
        evolve(loop, [0, 0, 90, 0], 1)
        self.assertLess(abs(loop.gauge), 1)
        self.assertLessEqual(loop.water, water)
        self.assertGreater(loop.water, water - 50e-6)

    def test_bottom_drain_without_air_admission_stalls_under_vacuum(self):
        loop = evolve(ServiceLoop(gauge=0), [0, 0, 0, 90], 8)
        self.assertGreater(loop.water, 0.0013)
        self.assertLess(loop.gauge, -1000)
        self.assertLess(loop.cover_force, 0)

    def test_vent_and_drain_remove_liquid_without_negative_mass(self):
        loop = evolve(ServiceLoop(), [0, 0, 90, 90], 45)
        self.assertLess(loop.water, 1e-7)
        self.assertGreater(loop.air_mass, 0)
        self.assertLess(abs(loop.gauge), 1)

    def test_empty_node_refills_through_supply(self):
        loop = ServiceLoop()
        loop.water = 0
        loop.air_mass = ATM * loop.capacity / RT
        evolve(loop, [90, 0, 90, 0], 10)
        self.assertGreater(loop.water, 1e-4)
        self.assertGreater(loop.air_mass, 0)

    def test_isolating_supply_does_not_isolate_return_pressure(self):
        loop = evolve(ServiceLoop(gauge=0), [0, 90, 0, 0], 3)
        self.assertGreater(loop.gauge, 27000)

    def test_unseated_cover_cannot_hold_pressure(self):
        loop = evolve(ServiceLoop(), [0, 0, 0, 0], 2, [0.0001] * 16)
        self.assertLess(loop.gauge, 1000)

    def test_integrated_fluxes_conserve_water_and_air(self):
        loop = ServiceLoop()
        initial_water, initial_air = loop.water, loop.air_mass
        water_flux = air_flux = 0.0
        dt = 1 / 240
        for _ in range(240):
            reading = loop.update([20, 0, 60, 90], [0] * 16, dt)
            water_flux += dt * (
                reading["supply_m3_s"]
                + reading["return_m3_s"]
                - reading["drain_m3_s"]
                - reading["bleed_water_m3_s"]
            )
            air_flux += dt * reading["air_out_kg_s"]
        self.assertAlmostEqual(loop.water - initial_water, water_flux, places=12)
        self.assertAlmostEqual(loop.air_mass - initial_air, -air_flux, places=12)

    def test_choked_air_flow_saturates_and_reverses(self):
        choked = air_flow(300000, 100000, 1e-6)
        self.assertAlmostEqual(choked, air_flow(300000, 50000, 1e-6))
        self.assertAlmostEqual(choked, -air_flow(100000, 300000, 1e-6))

    def test_drainage_step_convergence(self):
        values = [evolve(ServiceLoop(), [0, 0, 90, 90], 1, hz=hz).water for hz in (240, 480, 960)]
        self.assertLess(abs(values[1] - values[2]), 1e-7)
        self.assertLess(abs(values[1] - values[2]), abs(values[0] - values[1]))

    def test_invalid_measurements_fail(self):
        for angles, gaps, capacity in [
            ([0, 0, 0], [0], 0.0015),
            ([0, 0, 0, 91], [0], 0.0015),
            ([0, 0, 0, 0], [math.nan], 0.0015),
            ([0, 0, 0, 0], [0], -1),
        ]:
            with self.assertRaises(ValueError):
                ServiceLoop().update(angles, gaps, 1 / 240, capacity_m3=capacity)


if __name__ == "__main__":
    unittest.main()
