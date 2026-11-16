"""Finite paint response to measured native bar motion and actual contact.

No pose writes or commanded-path coverage. The surface response is deliberately
limited to visible coverage, with no film-thickness/material-quality claim.
"""

import numpy as np


class Paint:
    def __init__(self):
        self.grid = np.zeros((40, 16), dtype=bool)
        self.reservoir_ml = 3.0
        self.loaded = False
        self.previous = None
        self.force = 0.0

    def update(self, p, R, normal_N, dt, enabled=True):
        from .spec import SURFACE_Z

        p = np.asarray(p, float)
        R = np.asarray(R, float)
        changed = []
        previous = self.previous
        self.previous = p.copy()
        self.force = float(normal_N)
        if previous is None or not enabled:
            return changed
        velocity = (p - previous) / dt
        feet = [p + R @ np.array([x, 0.0, 0.0]) for x in (-0.045, 0.045)]
        seated = (
            normal_N >= 0.2
            and all(abs(foot[2] - SURFACE_Z) < 0.00015 for foot in feet)
            and np.dot(R[:, 2], [0, 0, 1]) > 0.9998
        )
        if not seated:
            return changed
        # The bead is collected only by contact across its actual footprint.
        if abs(p[1] + 0.105) < 0.008 and abs(p[0]) < 0.005:
            self.loaded = True
        if not self.loaded or self.reservoir_ml <= 0 or velocity[1] <= 0 or np.linalg.norm(velocity[:2]) > 0.15:
            return changed
        # Native swept motion crosses each5mm raster cell; the finite wet bead
        # can reach that cell only within the real80mm working aperture.
        for row in range(40):
            y = -0.100 + (row + 0.5) * 0.005
            if not min(previous[1], p[1]) - 0.0025 <= y <= max(previous[1], p[1]) + 0.0025:
                continue
            for col in range(16):
                if self.grid[row, col]:
                    continue
                x = -0.040 + (col + 0.5) * 0.005
                local = R.T @ (np.array([x, y, p[2]]) - p)
                if abs(local[0]) <= 0.040 and abs(local[1]) <= 0.005 and self.reservoir_ml >= 0.0025:
                    self.grid[row, col] = True
                    self.reservoir_ml -= 0.0025  # 25mm2 x0.1mm =2.5microlitres
                    changed.append((row, col))
        return changed

    def readout(self):
        # Longest contiguous missing interval in any row, directly checkable
        # against a5mm transparent grid on the real test card.
        largest = 0
        for row in self.grid:
            run = 0
            for wet in row:
                run = 0 if wet else run + 1
                largest = max(largest, run)
        return {
            "coverage": float(self.grid.mean()),
            "largest_dry_stripe_mm": largest * 5,
            "reservoir_ml": max(0.0, self.reservoir_ml),
            "contact_N": self.force,
        }


class Process:
    def __init__(self, env, variant="normal"):
        self.env = env
        self.variant = variant
        self.reset()

    def reset(self):
        self.paint = Paint()
        self.attributes = None

    def substep(self, control, dt):
        from .native import array, tool_pose, wrench

        p, R, _ = tool_pose(self.env)
        normal = array(self.env.scene["tool_contact"].data.net_normal_forces_w)[0].reshape(-1, 3).sum(0)[2]
        previous = self.paint.previous
        changed = self.paint.update(p, R, max(0.0, normal), dt, self.variant != "no-material")
        if self.attributes is None:
            import omni.usd
            from pxr import UsdGeom

            stage = omni.usd.get_context().get_stage()
            self.attributes = {}
            for prim in stage.Traverse():
                if prim.GetName().startswith("paint_cell_"):
                    _, _, row, col = prim.GetName().split("_")
                    attribute = UsdGeom.Gprim(prim).GetDisplayColorAttr()
                    self.attributes[(int(row), int(col))] = attribute
                    from pxr import Gf, Vt

                    attribute.Set(Vt.Vec3fArray([Gf.Vec3f(0.95, 0.95, 0.94)]))
        if changed:
            from pxr import Gf, Vt

            for index in changed:
                self.attributes[index].Set(Vt.Vec3fArray([Gf.Vec3f(0.08, 0.34, 0.70)]))
        force = np.zeros(3)
        if previous is not None and self.paint.loaded and normal > 0.2:
            v = (p - previous) / dt
            force[:2] = -np.clip(20 * v[:2], -3.0, 3.0)
        robot = self.env.scene["robot"]
        i = robot.body_names.index("Tool")
        # Viscous reaction acts at the working surface, not at a fictitious
        # commanded point. Native contact supplies the vertical load.
        torque = np.cross(-(R @ np.array([0, 0, 0.018])), force)
        wrench(robot, force.tolist(), torque.tolist(), i)

    def readout(self):
        return self.paint.readout()

    def instruments(self):
        p = self.readout()
        return [p["contact_N"], p["reservoir_ml"], 100 * p["coverage"], p["largest_dry_stripe_mm"]]
