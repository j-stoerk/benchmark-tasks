"""Check the authored solids' swept clearance and interlock engagement offline."""

import math
import unittest
from pathlib import Path

import numpy as np
from pxr import Usd, UsdPhysics


def rotation(axis, angle):
    c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    if axis == "Y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def box(stage, path, axis, angle):
    prim = stage.GetPrimAtPath("/Machine/" + path)
    matrix = rotation(axis, angle)
    centre = matrix @ np.asarray(prim.GetAttribute("xformOp:translate").Get(), float)
    half = np.asarray(prim.GetAttribute("xformOp:scale").Get(), float) / 2
    return centre, matrix, half


def intersects(first, second):
    a, ra, ha = first
    b, rb, hb = second
    axes = [*ra.T, *rb.T, *[np.cross(x, y) for x in ra.T for y in rb.T]]
    for vector in axes:
        if np.linalg.norm(vector) < 1e-10:
            continue
        vector = vector / np.linalg.norm(vector)
        radius_a = sum(abs(vector @ ra[:, i]) * ha[i] for i in range(3))
        radius_b = sum(abs(vector @ rb[:, i]) * hb[i] for i in range(3))
        if abs(vector @ (b - a)) > radius_a + radius_b:
            return False
    return True


class Geometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stage = Usd.Stage.Open(str(Path(__file__).parent / "split_valve/machine.usda"))

    def test_detached_upper_payload_mass_is_representative(self):
        names = [
            "Upper",
            "Passive",
            *[f"Seal_upper_{i}" for i in range(8)],
            *[f"Guide_upper_{i}" for i in range(8)],
            *[f"Tangent_upper_{i}" for i in range(8)],
        ]
        total = sum(
            UsdPhysics.MassAPI(self.stage.GetPrimAtPath("/Machine/" + name)).GetMassAttr().Get()
            for name in names
        )
        self.assertAlmostEqual(total, 8, places=5)

    def test_each_solid_disc_has_one_closed_convex_collision_mesh(self):
        for body in ("Active", "Passive"):
            disc = self.stage.GetPrimAtPath("/Machine/" + body + "/disc")
            self.assertEqual(disc.GetTypeName(), "Mesh")
            self.assertTrue(disc.HasAPI(UsdPhysics.CollisionAPI))
            self.assertLessEqual(len(disc.GetAttribute("points").Get()), 64)
            self.assertEqual(list(disc.GetChildren()), [])
            self.assertEqual(len(disc.GetAttribute("faceVertexCounts").Get()), 34)

    def test_drive_dogs_allow_coupled_sweep_and_vertical_disengagement(self):
        for angle in range(0, 91, 2):
            lower = box(self.stage, "Active/drive_dog", "Y", angle)
            upper = box(self.stage, "Passive/drive_dog", "Y", angle)
            # Ignore shared boundary contact, checking penetration of solids.
            lower = (lower[0], lower[1], lower[2] - 1e-7)
            upper = (upper[0] + [0, 0, 0.0001], upper[1], upper[2] - 1e-7)
            self.assertFalse(intersects(lower, upper), angle)
            separated = (upper[0] + [0, 0, 0.1], upper[1], upper[2])
            self.assertFalse(intersects(lower, separated), angle)

    def test_drive_dogs_physically_block_disc_desynchronization_both_directions(self):
        for active, passive in ((2, 0), (0, 2), (55, 5), (5, 55), (90, 88), (88, 90)):
            lower = box(self.stage, "Active/drive_dog", "Y", active)
            upper = box(self.stage, "Passive/drive_dog", "Y", passive)
            upper = (upper[0] + [0, 0, 0.0001], upper[1], upper[2])
            self.assertTrue(intersects(lower, upper), (active, passive))

    def test_drive_dogs_remain_outside_the_collar_and_casing(self):
        for body in ("Active", "Passive"):
            for angle in range(0, 91, 2):
                centre, _, half = box(self.stage, body + "/drive_dog", "Y", angle)
                self.assertLess(centre[1] + half[1], -0.120)

    def test_seal_shoes_clear_the_housings_over_the_whole_radial_stroke(self):
        for housing in ("base/housing", "Upper/housing"):
            clearances = []
            for prim in self.stage.Traverse():
                if str(prim.GetPath()).startswith("/Machine/" + housing + "/sector_"):
                    points = np.asarray(prim.GetAttribute("points").Get())
                    clearances.append(np.linalg.norm((points[0, :2] + points[3, :2]) / 2))
            self.assertTrue(clearances)
            outer_radius = 0.052
            half_angle = math.pi / 8
            maximum_corner_radius = math.hypot(
                outer_radius * math.cos(half_angle) + 0.003, outer_radius * math.sin(half_angle) + 0.002
            )
            self.assertGreater(min(clearances), maximum_corner_radius + 0.0001)

    def test_no_rigid_lip_segment_bridges_the_rotors_bending_axis(self):
        for half in ("lower", "upper"):
            for index in range(8):
                prim = self.stage.GetPrimAtPath(f"/Machine/Seal_{half}_{index}")
                centre = np.asarray(prim.GetAttribute("xformOp:translate").Get())
                angle = prim.GetAttribute("xformOp:rotateZ").Get()
                matrix = rotation("Z", angle)
                x_values = []
                for child in self.stage.Traverse():
                    if str(child.GetPath()).startswith(str(prim.GetPath()) + "/shoe/sector_"):
                        x_values.extend(
                            (np.asarray(child.GetAttribute("points").Get()) @ matrix.T + centre)[:, 0]
                        )
                self.assertTrue(x_values)
                self.assertTrue(min(x_values) >= -1e-8 or max(x_values) <= 1e-8)

    def test_pins_clear_the_entire_continuous_rim_and_bridge(self):
        radii = []
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).startswith(("/Machine/Collar/rim/", "/Machine/Collar/bridge/")):
                points = prim.GetAttribute("points").Get()
                if points:
                    radii.extend(np.linalg.norm(np.asarray(points)[:, :2], axis=1))
        self.assertTrue(radii)
        for pin in ("closed_pin", "open_pin"):
            centre, _, half = box(self.stage, "Active/" + pin, "Y", 0)
            self.assertGreater(centre[1] - half[1] - max(radii), 0.010)

    def test_unlocked_block_physically_prevents_opening(self):
        block = box(self.stage, "Collar/interlock", "Z", 0)
        self.assertFalse(intersects(box(self.stage, "Active/closed_pin", "Y", 0), block))
        self.assertTrue(intersects(box(self.stage, "Active/closed_pin", "Y", 5), block))

    def test_locked_collar_allows_the_full_disc_sweep(self):
        obstacles = ["interlock", "interlock_support", "interlock_standoff"]
        for angle in range(0, 91, 2):
            for pin in ("closed_pin", "open_pin"):
                for obstacle in obstacles:
                    self.assertFalse(
                        intersects(
                            box(self.stage, "Active/" + pin, "Y", angle),
                            box(self.stage, "Collar/" + obstacle, "Z", 30),
                        ),
                        (angle, pin, obstacle),
                    )

    def test_open_pin_blocks_before_the_retaining_overlap_is_lost(self):
        pin = box(self.stage, "Active/open_pin", "Y", 90)
        self.assertTrue(intersects(pin, box(self.stage, "Collar/interlock", "Z", 10)))
        self.assertFalse(intersects(pin, box(self.stage, "Collar/interlock", "Z", 30)))


if __name__ == "__main__":
    unittest.main()
