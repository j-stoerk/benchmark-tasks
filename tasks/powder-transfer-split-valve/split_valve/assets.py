"""Original DN100 split-valve mechanical fixture, authored in SI units."""

import math
from pathlib import Path

PACKAGE = Path(__file__).parent
SECTORS = 96


def generate():
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Machine").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)

    def body(name, pos, mass, angle=0):
        x = UsdGeom.Xform.Define(stage, "/Machine/" + name)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if angle:
            x.AddRotateZOp().Set(math.degrees(angle))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        UsdPhysics.MassAPI.Apply(x.GetPrim()).CreateMassAttr(mass)

    def style(shape, color, collision=True):
        shape.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        if collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())

    def box(path, size, pos, color=(0.18, 0.28, 0.38), collision=True):
        shape = UsdGeom.Cube.Define(stage, "/Machine/" + path)
        shape.CreateSizeAttr(1)
        shape.AddTranslateOp().Set(Gf.Vec3d(*pos))
        shape.AddScaleOp().Set(Gf.Vec3f(*size))
        style(shape, color, collision)

    def wedge(path, inner, outer, a, b, height, z, color, origin=(0, 0)):
        cross = [
            (inner * math.cos(a), inner * math.sin(a)),
            (outer * math.cos(a), outer * math.sin(a)),
            (outer * math.cos(b), outer * math.sin(b)),
            (inner * math.cos(b), inner * math.sin(b)),
        ]
        # A disc sector uses a triangular prism, avoiding degenerate inner faces.
        if inner == 0:
            cross = [cross[0], cross[1], cross[2]]
            counts = [3, 3, 4, 4, 4]
            indices = [0, 2, 1, 3, 4, 5, 0, 1, 4, 3, 1, 2, 5, 4, 2, 0, 3, 5]
        else:
            counts = [4] * 6
            indices = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
        points = [
            (x - origin[0], y - origin[1], zz + z) for zz in (-height / 2, height / 2) for x, y in cross
        ]
        mesh = UsdGeom.Mesh.Define(stage, "/Machine/" + path)
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        style(mesh, color)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    def arc(parent, inner, outer, start, stop, height, z, color, origin=(0, 0), count=None):
        count = count or max(1, round((stop - start) / (2 * math.pi) * SECTORS))
        for i in range(count):
            a = start + (stop - start) * i / count
            b = start + (stop - start) * (i + 1) / count
            wedge(f"{parent}/sector_{i:03d}", inner, outer, a, b, height, z, color, origin)

    def joint(name, parent, child, axis, pos, bounds, revolute=False, angle=0):
        kind = UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint
        j = kind.Define(stage, "/Machine/" + name)
        j.CreateBody0Rel().SetTargets(["/Machine/" + parent])
        j.CreateBody1Rel().SetTargets(["/Machine/" + child])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2))))
        j.CreateLocalRot1Attr(Gf.Quatf(1))
        j.CreateLowerLimitAttr(bounds[0])
        j.CreateUpperLimitAttr(bounds[1])
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if revolute else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0)
        drive.CreateDampingAttr(0)
        drive.CreateMaxForceAttr(1000)
        return j

    steel = (0.55, 0.59, 0.63)
    body("base", (0, 0, 0), 100)
    for i, x in enumerate((-0.25, 0.25)):
        box(f"base/post_{i}", (0.06, 0.06, 1.65), (x, 0.15, 0.825))
        box(f"base/foot_{i}", (0.20, 0.24, 0.04), (x, 0.15, 0.02))
    box("base/crossbeam", (0.60, 0.10, 0.08), (0, 0.15, 1.65))
    box("base/lower_support", (0.06, 0.30, 0.90), (0, 0.22, 0.45))
    box("base/lower_bracket", (0.22, 0.32, 0.04), (0, 0.10, 0.90))
    arc("base/housing", 0.0515, 0.080, 0, 2 * math.pi, 0.016, 0.992, steel)
    arc("base/neck", 0.052, 0.060, 0, 2 * math.pi, 0.08, 0.94, steel)
    body("X", (0, 0, 0), 2)
    body("Y", (0, 0, 0), 2)
    box("X/rail", (0.30, 0.06, 0.04), (0, 0.15, 1.60))
    box("Y/carriage", (0.08, 0.12, 0.06), (0, 0.15, 1.54))
    box("Y/guide", (0.05, 0.05, 0.4), (0, 0.15, 1.32))
    body("Upper", (0, 0, 0), 7.675)
    arc("Upper/housing", 0.0515, 0.060, 0, 2 * math.pi, 0.016, 0.008, steel)
    arc("Upper/neck", 0.052, 0.060, 0, 2 * math.pi, 0.10, 0.066, (0.38, 0.48, 0.56))
    arc("Upper/vessel", 0.085, 0.090, 0, 2 * math.pi, 0.20, 0.216, steel)
    box("Upper/arm", (0.08, 0.20, 0.04), (0, 0.10, 0.30))
    for i in range(3):
        angle = i * 2 * math.pi / 3
        lug = UsdGeom.Xform.Define(stage, f"/Machine/Upper/lug_frame_{i}")
        lug.AddRotateZOp().Set(math.degrees(angle))
        box(f"Upper/lug_frame_{i}/lug", (0.024, 0.008, 0.006), (0.070, 0, 0.013), (0.85, 0.53, 0.10))
    joint("slide_x", "base", "X", "X", (0, 0, 0), (-0.04, 0.04))
    joint("slide_y", "X", "Y", "Y", (0, 0, 0), (-0.04, 0.04))
    joint("slide_z", "Y", "Upper", "Z", (0, 0, 0), (0.80, 1.25))

    body("Collar", (0, 0, 1.0), 1.0)
    arc("Collar/rim", 0.100, 0.110, 0, 2 * math.pi, 0.020, 0.009, steel)
    arc("Collar/bridge", 0.083, 0.105, 0, 2 * math.pi, 0.010, 0.0271, steel)
    for i in range(3):
        start = math.radians(i * 120 + 10)
        stop = math.radians(i * 120 + 110)
        arc(f"Collar/ledge_{i}", 0.065, 0.085, start, stop, 0.006, 0.0191, (0.22, 0.38, 0.56))
    box("Collar/interlock", (0.04, 0.018, 0.006), (0.04, 0.085, -0.0031), (0.85, 0.53, 0.10))
    box("Collar/lever", (0.16, 0.016, 0.012), (0.18, 0, 0.018), (0.85, 0.53, 0.10))
    joint("collar", "base", "Collar", "Z", (0, 0, 1.0), (0, 32), revolute=True)
    body("Active", (0, 0, 1.0), 0.125)
    body("Passive", (0, 0, 0), 0.125)
    arc("Active/disc", 0, 0.050, 0, 2 * math.pi, 0.002, -0.001, (0.22, 0.52, 0.74))
    arc("Passive/disc", 0, 0.050, 0, 2 * math.pi, 0.002, 0.001, (0.84, 0.48, 0.16))
    # The pins belong to the active rotor and physically meet the collar block.
    box("Active/closed_pin", (0.004, 0.004, 0.004), (0.04, 0.085, 0.002), (0.85, 0.2, 0.12))
    box("Active/open_pin", (0.004, 0.004, 0.004), (0, 0.085, 0.04), (0.85, 0.2, 0.12))
    box("Active/shaft", (0.008, 0.18, 0.008), (0, 0, 0), steel, collision=False)
    box("Active/pin_carrier", (0.044, 0.006, 0.044), (0.02, 0.085, 0.02), steel, collision=False)
    box("Passive/shaft", (0.008, 0.12, 0.008), (0, 0, 0), steel, collision=False)
    joint("active", "base", "Active", "Y", (0, 0, 1.0), (0, 90), revolute=True)
    joint("passive", "Upper", "Passive", "Y", (0, 0, 0), (0, 90), revolute=True)
    for half, parent, z in (("lower", "base", 0.999), ("upper", "Upper", 0.001)):
        for i in range(8):
            angle = i * math.pi / 4
            name = f"Seal_{half}_{i}"
            pos = (0.0509 * math.cos(angle), 0.0509 * math.sin(angle), z)
            body(name, pos, 0.025, angle)
            arc(
                name + "/shoe",
                0.0498,
                0.052,
                -math.pi / 8,
                math.pi / 8,
                0.002,
                0,
                (0.18, 0.20, 0.22),
                (0.0509, 0),
                12,
            )
            joint(f"seal_{half}_{i}", parent, name, "X", pos, (-0.001, 0.003), angle=angle)
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Machine/anchor")
    fixed.CreateBody1Rel().SetTargets(["/Machine/base"])
    stage.GetRootLayer().Export(str(PACKAGE / "machine.usda"))


if __name__ == "__main__":
    generate()
