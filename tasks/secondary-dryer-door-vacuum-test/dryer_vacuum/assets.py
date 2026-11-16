"""Original full-size vacuum-readiness fixture geometry; runtime coupling pending."""

import math
from pathlib import Path

PACKAGE = Path(__file__).parent


def generate():
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Machine").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    steel = (0.42, 0.46, 0.50)

    def body(name, position, mass, inertia=None):
        transform = UsdGeom.Xform.Define(stage, "/Machine/" + name)
        transform.AddTranslateOp().Set(Gf.Vec3d(*position))
        UsdPhysics.RigidBodyAPI.Apply(transform.GetPrim())
        properties = UsdPhysics.MassAPI.Apply(transform.GetPrim())
        properties.CreateMassAttr(mass)
        properties.CreateCenterOfMassAttr(Gf.Vec3f(0))
        if inertia is not None:
            properties.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        return transform.GetPrim()

    def box(path, dimensions, centre, color=steel, collision=True):
        shape = UsdGeom.Cube.Define(stage, "/Machine/" + path)
        shape.CreateSizeAttr(1)
        shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
        shape.AddScaleOp().Set(Gf.Vec3f(*dimensions))
        shape.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        if collision:
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())

    def joint(name, parent, child, axis, anchor, limits, rotary=False, child_anchor=(0, 0, 0)):
        schema = UsdPhysics.RevoluteJoint if rotary else UsdPhysics.PrismaticJoint
        value = schema.Define(stage, "/Machine/" + name)
        value.CreateBody0Rel().SetTargets(["/Machine/" + parent])
        value.CreateBody1Rel().SetTargets(["/Machine/" + child])
        value.CreateAxisAttr(axis)
        value.CreateLocalPos0Attr(Gf.Vec3f(*anchor))
        value.CreateLocalPos1Attr(Gf.Vec3f(*child_anchor))
        value.CreateLocalRot0Attr(Gf.Quatf(1))
        value.CreateLocalRot1Attr(Gf.Quatf(1))
        value.CreateLowerLimitAttr(limits[0])
        value.CreateUpperLimitAttr(limits[1])
        drive = UsdPhysics.DriveAPI.Apply(value.GetPrim(), "angular" if rotary else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0)
        drive.CreateDampingAttr(0)
        drive.CreateMaxForceAttr(1000)

    body("base", (0, 0, 0), 100)
    # A 500x500x500 mm open-front cavity, separate from the door's outer size.
    box("base/left", (0.020, 0.500, 0.540), (-0.260, 0.250, 1.0))
    box("base/right", (0.020, 0.500, 0.540), (0.260, 0.250, 1.0))
    box("base/top", (0.540, 0.500, 0.020), (0, 0.250, 1.260))
    box("base/bottom", (0.540, 0.500, 0.020), (0, 0.250, 0.740))
    box("base/back", (0.540, 0.020, 0.540), (0, 0.510, 1.0))
    for side, x in enumerate((-0.230, 0.230)):
        for depth, y in enumerate((0.050, 0.450)):
            box(f"base/leg_{side}_{depth}", (0.050, 0.050, 0.720), (x, y, 0.360))
            box(f"base/foot_{side}_{depth}", (0.120, 0.120, 0.020), (x, y, 0.010))
    for index, (x, z) in enumerate(
        ((-0.2725, 0.7275), (0.2725, 0.7275), (-0.2725, 1.2725), (0.2725, 1.2725))
    ):
        box(f"base/stop_{index}", (0.025, 0.020, 0.025), (x, 0.0008, z), (0.75, 0.58, 0.15))

    pivot = (-0.280, -0.0165, 1.0)
    body("Hinge", pivot, 2, (0.001, 0.001, 0.001))
    box("Hinge/bearing_cover", (0.050, 0.050, 0.160), (0, 0, 0), collision=False)
    joint("hinge_float", "base", "Hinge", "Y", pivot, (-0.002, 0.002))
    door_dimensions = (0.560, 0.013, 0.560)
    door_mass = math.prod(door_dimensions) * 7850
    inertia = [
        door_mass * (door_dimensions[(i + 1) % 3] ** 2 + door_dimensions[(i + 2) % 3] ** 2) / 12
        for i in range(3)
    ]
    body("Door", (0, -0.0165, 1.0), door_mass, inertia)
    box("Door/plate", door_dimensions, (0, 0, 0), (0.25, 0.42, 0.60))
    box("Door/handle", (0.025, 0.040, 0.180), (0.220, -0.040, 0), collision=False)
    joint("door", "Hinge", "Door", "Z", (0, 0, 0), (-100, 0), rotary=True, child_anchor=(-0.280, 0, 0))

    for index in range(16):
        side, along = divmod(index, 4)
        offset = -0.1875 + 0.125 * along
        if side in (0, 2):
            x, z = offset, 0.750 if side == 0 else 1.250
            dimensions = (0.125, 0.004, 0.008)
        else:
            x, z = 0.250 if side == 1 else -0.250, 1.0 + offset
            dimensions = (0.008, 0.004, 0.125)
        name = f"Seal_{index}"
        position = (x, -0.008, z)
        inertia = [
            0.05 * (dimensions[(i + 1) % 3] ** 2 + dimensions[(i + 2) % 3] ** 2) / 12 for i in range(3)
        ]
        prim = body(name, position, 0.050, inertia)
        box(name + "/shoe", dimensions, (0, 0, 0), (0.10, 0.11, 0.12))
        joint(f"seal_{index}", "base", name, "Y", position, (-0.0005, 0.0012))
        # These are numerical segments of one continuous seal. Internal rigid
        # seams must not invent hard contacts between neighbouring rubber nodes.
        filters = UsdPhysics.FilteredPairsAPI.Apply(prim)
        filters.CreateFilteredPairsRel().SetTargets(
            ["/Machine/Seal_" + str(other) for other in range(16) if other != index]
        )

    for index, z in enumerate((0.820, 1.180)):
        box(f"base/clamp_bracket_{index}", (0.100, 0.120, 0.060), (0.305, 0.070, z))
        position = (0.335, -0.050, z)
        body(f"Draw_{index}", position, 2, (0.001, 0.001, 0.001))
        box(f"Draw_{index}/guide_cover", (0.060, 0.120, 0.060), (0, 0.020, 0), collision=False)
        joint(f"draw_{index}", "base", f"Draw_{index}", "Y", position, (0, 0.010))
        body(f"Clamp_{index}", position, 0.500)
        box(f"Clamp_{index}/arm", (0.020, 0.020, 0.140), (0, 0, -0.060), (0.70, 0.53, 0.12))
        box(f"Clamp_{index}/pad", (0.026, 0.020, 0.030), (0, 0.015, -0.120), steel)
        joint(f"swing_{index}", f"Draw_{index}", f"Clamp_{index}", "Y", (0, 0, 0), (0, 90), rotary=True)

    box("base/valve_manifold", (0.100, 0.100, 0.300), (-0.335, 0.200, 1.120))
    box("base/manifold_bracket", (0.100, 0.150, 0.040), (-0.300, 0.200, 0.950))
    for name, position, color in (
        ("PumpValve", (-0.350, 0.100, 1.220), (0.20, 0.45, 0.75)),
        ("VentValve", (-0.350, 0.100, 1.020), (0.75, 0.35, 0.18)),
    ):
        body(name, position, 0.100)
        box(
            "base/" + name + "_stem",
            (0.012, 0.100, 0.012),
            (position[0], 0.150, position[2]),
            collision=False,
        )
        box(name + "/lever", (0.015, 0.080, 0.015), (0, -0.040, 0), color)
        joint(
            "pump_valve" if name == "PumpValve" else "vent_valve",
            "base",
            name,
            "X",
            position,
            (0, 90),
            rotary=True,
        )

    anchor = UsdPhysics.FixedJoint.Define(stage, "/Machine/anchor")
    anchor.CreateBody1Rel().SetTargets(["/Machine/base"])
    stage.GetRootLayer().Export(str(PACKAGE / "machine.usda"))


if __name__ == "__main__":
    generate()
