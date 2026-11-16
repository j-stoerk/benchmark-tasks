"""Pressure/load conservation and failure-control checks before native coupling."""

import math
import unittest

from filter_service.hydraulics import WaterLoop, orifice, seal_flow


def evolve(loop, angles, gaps, seconds=3, hz=480):
    for _ in range(round(seconds * hz)):
        loop.update(angles, gaps, 1 / hz)
    return loop.gauge


class Circuit(unittest.TestCase):
    def test_real_cover_load(self):
        self.assertAlmostEqual(WaterLoop().cover_force, 50000 * math.pi * 0.1**2 / 4)

    def test_isolation_alone_does_not_relieve_pressure(self):
        self.assertAlmostEqual(evolve(WaterLoop(), [0, 0, 0], [0] * 16, 10), 50000)

    def test_vent_physically_relaxes_trapped_pressure(self):
        self.assertLess(evolve(WaterLoop(), [0, 0, 90], [0] * 16, 5), 1000)

    def test_connected_return_repressurizes_pocket(self):
        self.assertGreater(evolve(WaterLoop(gauge=0), [0, 90, 0], [0] * 16), 29000)

    def test_supply_restores_test_pressure(self):
        self.assertGreater(evolve(WaterLoop(gauge=0), [90, 0, 0], [0] * 16), 49000)

    def test_hovering_cover_leaks(self):
        self.assertLess(evolve(WaterLoop(), [0, 0, 0], [0.0001] * 16), 1000)

    def test_one_local_unseated_segment_loses_hold(self):
        self.assertLess(evolve(WaterLoop(), [0, 0, 0], [0.00005] + [0] * 15, 10), 49000)

    def test_smooth_compressed_faces_have_no_open_channel(self):
        self.assertEqual(seal_flow(50000, [-0.0002] * 16), 0)

    def test_flow_reverses_with_pressure(self):
        self.assertAlmostEqual(orifice(30000, 0, 1e-6), -orifice(0, 30000, 1e-6))

    def test_step_convergence_before_native_coupling(self):
        values = [evolve(WaterLoop(), [0, 0, 90], [0] * 16, 0.5, hz) for hz in (240, 480, 960)]
        self.assertLess(abs(values[1] - values[2]), 100)
        self.assertLess(abs(values[1] - values[2]), abs(values[0] - values[1]))

    def test_bad_measurement_fails(self):
        for angle, gaps, dt in [
            ([0, 0, float("nan")], [0] * 16, 0.002),
            ([0, 0, 0], [float("inf")], 0.002),
            ([0, 0, 0], [0] * 16, 0.02),
        ]:
            with self.assertRaises(ValueError):
                WaterLoop().update(angle, gaps, dt)

    def test_cover_motion_changes_pressure_in_a_closed_node(self):
        loop = WaterLoop()
        initial = loop.gas_volume
        loop.update([0, 0, 0], [0] * 16, 1 / 480, volume_change_m3=initial * 0.1)
        self.assertAlmostEqual(loop.gauge, (101325 + 50000) / 1.1 - 101325)

    def test_mechanical_compression_can_exceed_source_pressure(self):
        loop = WaterLoop()
        loop.update([0, 0, 0], [0] * 16, 1 / 480, volume_change_m3=-loop.gas_volume * 0.1)
        self.assertGreater(loop.gauge, 50000)


if __name__ == "__main__":
    unittest.main()
