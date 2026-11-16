"""Original SI-unit geometry; finite-compliance native calender articulation."""

from pathlib import Path

from . import spec

PACKAGE = Path(__file__).parent


def generate():
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Calender")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    def body(name, pos, mass):
        b = UsdGeom.Xform.Define(stage, "/Calender/" + name)
        b.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(mass)
        return b.GetPrim().GetPath()

    def style(shape, color, collision=True):
        UsdGeom.Gprim(shape.GetPrim()).CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        if collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())

    def box(name, size, pos, color=(0.16, 0.29, 0.44)):
        b = UsdGeom.Cube.Define(stage, "/Calender/" + name)
        b.CreateSizeAttr(1)
        b.AddTranslateOp().Set(Gf.Vec3d(*pos))
        b.AddScaleOp().Set(Gf.Vec3f(*size))
        style(b, color)

    def cylinder(name, pos, color=(0.62, 0.65, 0.68)):
        c = UsdGeom.Cylinder.Define(stage, "/Calender/" + name)
        c.CreateAxisAttr("Y")
        c.CreateRadiusAttr(spec.RADIUS)
        c.CreateHeightAttr(spec.FACE)
        c.AddTranslateOp().Set(Gf.Vec3d(*pos))
        style(c, color)

    base = body("base", (0, 0, 0), 250 + spec.ROLLER_MASS)
    box("base/foot", (0.6, 0.65, 0.08), (0, 0, 0.04))
    for i, side in enumerate((-1, 1)):
        box(f"base/post_{i}", (0.16, 0.07, 0.7), (0, side * 0.235, 0.43))
        box(f"base/bearing_{i}", (0.14, 0.06, 0.12), (0, side * 0.175, 0.30))
        box(f"Upper/bearing_{i}", (0.14, 0.04, 0.10), (0, side * 0.175, 0), (0.21, 0.45, 0.67))
    box("base/crosshead", (0.18, 0.53, 0.08), (0, 0, 0.78))
    cylinder("base/lower_roll", (0, 0, 0.3))
    slider = body("Slider", (0, 0, 0.5), 1.0)
    # Tilt link includes the upper roll and its idealized bearing housings in its stated mass.
    upper = body("Upper", (0, 0, 0.5), spec.ROLLER_MASS)
    cylinder("Upper/roll", (0, 0, 0))
    box("Upper/witness", (0.008, 0.30, 0.008), (0.101, 0, 0), (0.85, 0.20, 0.10))
    # Show the native spring-drive support path. These telescoping housings are
    # visual geometry, so they neither alter inertia nor add a parallel load path.
    for i, side in enumerate((-1, 1)):
        for parent, part, z, length, radius, color in [
            ("base", "spring_housing", 0.65, 0.18, 0.025, (0.18, 0.30, 0.45)),
            ("Upper", "piston_rod", 0.12, 0.14, 0.0125, (0.55, 0.58, 0.62)),
        ]:
            shape = UsdGeom.Cylinder.Define(stage, f"/Calender/{parent}/{part}_{i}")
            shape.CreateAxisAttr("Z")
            shape.CreateHeightAttr(length)
            shape.CreateRadiusAttr(radius)
            shape.AddTranslateOp().Set(Gf.Vec3d(0, side * spec.BEARING_HALF_SPAN, z))
            style(shape, color, collision=False)
    for i, y in enumerate(spec.STATIONS):
        # 0.9 mm fixed paired-sensor head in the widening entry wedge at x=25 mm.
        box(f"base/probe_{i}", (0.004, 0.012, 0.0009), (0.025, float(y), 0.4), (0.95, 0.63, 0.08))
        box(f"base/probe_arm_{i}", (0.16, 0.006, 0.001), (0.107, float(y), 0.4))
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Calender/anchor")
    fixed.CreateBody1Rel().SetTargets([base])
    for name, klass, parent, child, axis, position, bounds, stiffness, damping in [
        (
            "elevation",
            UsdPhysics.PrismaticJoint,
            base,
            slider,
            "Z",
            (0, 0, 0.5),
            (-0.002, 0.003),
            2 * spec.SPRING,
            2 * spec.SPRING_DAMPING,
        ),
        (
            "inclination",
            UsdPhysics.RevoluteJoint,
            slider,
            upper,
            "X",
            (0, 0, 0),
            (-0.57, 0.57),
            2 * spec.SPRING * spec.BEARING_HALF_SPAN**2,
            2 * spec.SPRING_DAMPING * spec.BEARING_HALF_SPAN**2,
        ),
    ]:
        j = klass.Define(stage, "/Calender/" + name)
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*position))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLowerLimitAttr(bounds[0])
        j.CreateUpperLimitAttr(bounds[1])
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear" if name == "elevation" else "angular")
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateStiffnessAttr(stiffness)
        drive.CreateDampingAttr(damping)
        drive.CreateMaxForceAttr(100_000.0)
    stage.GetRootLayer().Export(str(PACKAGE / "calender.usda"))


if __name__ == "__main__":
    generate()
