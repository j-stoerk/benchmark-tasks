"""Physical envelope, pressure area and real clamp-path geometry checks."""

import math
import unittest
from pathlib import Path

import numpy as np
from pxr import Usd, UsdPhysics


def rotation(axis, degrees):
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    if axis == "Z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def intersects(first, second):
    a, ra, ha = first
    b, rb, hb = second
    for vector in [*ra.T, *rb.T, *[np.cross(x, y) for x in ra.T for y in rb.T]]:
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
        cls.stage = Usd.Stage.Open(str(Path(__file__).parent / "dryer_vacuum/machine.usda"))

    def primitive(self, path):
        return self.stage.GetPrimAtPath("/Machine/" + path)

    def box(self, path):
        prim = self.primitive(path)
        return (
            np.array(prim.GetAttribute("xformOp:translate").Get()),
            np.array(prim.GetAttribute("xformOp:scale").Get()) / 2,
        )

    def door(self, angle):
        pivot = np.array(self.primitive("Hinge").GetAttribute("xformOp:translate").Get())
        anchor = np.array(self.primitive("door").GetAttribute("physics:localPos1").Get())
        local, half = self.box("Door/plate")
        matrix = rotation("Z", angle)
        return pivot + matrix @ (local - anchor), matrix, half

    def clamp(self, index, part, angle):
        pivot = np.array(self.primitive(f"Draw_{index}").GetAttribute("xformOp:translate").Get())
        local, half = self.box(f"Clamp_{index}/" + part)
        matrix = rotation("Y", angle)
        return pivot + matrix @ local, matrix, half

    def test_pressure_area_is_the_actual_half_metre_opening(self):
        left, lh = self.box("base/left")
        right, rh = self.box("base/right")
        bottom, bh = self.box("base/bottom")
        top, th = self.box("base/top")
        width = right[0] - rh[0] - left[0] - lh[0]
        height = top[2] - th[2] - bottom[2] - bh[2]
        self.assertAlmostEqual(width * height, 0.25, places=7)
        self.assertAlmostEqual(width * height * (101300 - 20000), 20325, places=2)

    def test_door_mass_and_full_size_are_preserved(self):
        door = self.primitive("Door")
        self.assertAlmostEqual(UsdPhysics.MassAPI(door).GetMassAttr().Get(), 25)
        _, half = self.box("Door/plate")
        self.assertTrue(np.allclose(2 * half, [0.560, 0.013, 0.560]))
        self.assertTrue(self.primitive("anchor"))

    def test_hard_stops_limit_a_four_mm_seal_to_twenty_percent_squeeze(self):
        centre, _, half = self.door(0)
        door_inside = centre[1] + half[1]
        fronts = []
        for index in range(4):
            point, size = self.box(f"base/stop_{index}")
            fronts.append(point[1] - size[1])
        self.assertTrue(np.allclose(fronts, [-0.0092] * 4))
        self.assertAlmostEqual((fronts[0] - door_inside) / 0.004, 0.20, places=5)

    def test_parked_clamps_clear_the_entire_door_sweep(self):
        for angle in range(-100, 1, 2):
            for index in range(2):
                for part in ("arm", "pad"):
                    self.assertFalse(
                        intersects(self.door(angle), self.clamp(index, part, 0)), (angle, index, part)
                    )

    def test_clamps_swing_clear_then_draw_to_the_actual_closed_door(self):
        door = self.door(0)
        for angle in range(0, 91, 2):
            for index in range(2):
                for part in ("arm", "pad"):
                    self.assertFalse(intersects(door, self.clamp(index, part, angle)))
        for index in range(2):
            centre, _, half = self.clamp(index, "pad", 90)
            outer = door[0][1] - door[2][1]
            gap = outer - centre[1] - half[1]
            self.assertAlmostEqual(gap, 0.002, places=6)
            self.assertAlmostEqual(gap + 0.0008, 0.0028, places=6)
            upper = self.primitive(f"draw_{index}").GetAttribute("physics:upperLimit").Get()
            self.assertGreater(upper, gap + 0.0008)

    def test_continuous_seal_segments_keep_actual_door_contacts(self):
        for index in range(16):
            shoe = self.primitive(f"Seal_{index}")
            targets = shoe.GetRelationship("physics:filteredPairs").GetTargets()
            self.assertEqual(len(targets), 15)
            self.assertTrue(all(str(path).startswith("/Machine/Seal_") for path in targets))
            self.assertTrue(self.primitive(f"Seal_{index}/shoe").HasAPI(UsdPhysics.CollisionAPI))
            joint = self.primitive(f"seal_{index}")
            self.assertEqual(joint.GetAttribute("physics:axis").Get(), "Y")


if __name__ == "__main__":
    unittest.main()
