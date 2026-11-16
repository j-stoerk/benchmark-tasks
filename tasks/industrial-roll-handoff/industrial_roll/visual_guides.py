"""Non-colliding covers show the native prismatic guide/drive support path."""

import math
from pathlib import Path


def add_visible_guides():
    from pxr import Gf, Usd, UsdGeom, Vt

    folder = Path(__file__).parent
    for filename in ("receiver.usda", "donor.usda"):
        stage = Usd.Stage.Open(str(folder / filename))

        def style(geom, color):
            geom.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
            # Deliberately no collision/mass/rigid-body API: only the existing
            # native joints and existing shaft/pad contact carry any load.

        def box(parent, name, size, pos, color=(0.12, 0.22, 0.34)):
            g = UsdGeom.Cube.Define(stage, f"/Machine/{parent}/visual_{name}")
            g.CreateSizeAttr(1)
            g.AddTranslateOp().Set(Gf.Vec3d(*pos))
            g.AddScaleOp().Set(Gf.Vec3f(*size))
            style(g, color)

        def rod(parent, name, height, radius, pos):
            g = UsdGeom.Cylinder.Define(stage, f"/Machine/{parent}/visual_{name}")
            g.CreateAxisAttr("Z")
            g.CreateHeightAttr(height)
            g.CreateRadiusAttr(radius)
            g.AddTranslateOp().Set(Gf.Vec3d(*pos))
            style(g, (0.55, 0.58, 0.62))

        if filename.startswith("receiver"):
            for i, side in enumerate((-1, 1)):
                box("base", f"x_guide_{i}", (0.40, 0.04, 0.04), (-0.12, side * 0.37, 0.34))
                rod("Y", f"z_guide_{i}", 0.50, 0.014, (0.12, side * 0.07, -0.03))
            box("Y", "guide_foundation", (0.18, 0.22, 0.06), (0.06, 0, -0.28))
            box("Head", "bearing_carriage", (0.08, 0.20, 0.10), (0.10, 0, 0), (0.18, 0.29, 0.42))
        else:
            box("base", "bearing_crossmember", (0.16, 0.8, 0.06), (0, 0, 0.26))
            box("base", "bearing_pedestal", (0.12, 0.12, 0.08), (0, 0, 0.29))
            # A true hollow visual bearing ring, so the moving shaft is visible
            # through it. Twenty-four side panels leave the bore unobstructed.
            for i in range(24):
                angle = (i + 0.5) * 2 * math.pi / 24
                radius = (0.039 + 0.070) / 2
                p = f"/Machine/base/visual_bearing_segment_{i}"
                g = UsdGeom.Cube.Define(stage, p)
                g.CreateSizeAttr(1)
                g.AddTranslateOp().Set(Gf.Vec3d(0, radius * math.cos(angle), 0.4 + radius * math.sin(angle)))
                g.AddRotateXOp().Set(math.degrees(angle))
                g.AddScaleOp().Set(Gf.Vec3f(0.10, 0.031, 2 * radius * math.sin(math.pi / 24)))
                style(g, (0.12, 0.22, 0.34))
        stage.GetRootLayer().Export(str(folder / filename))
