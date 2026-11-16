"""Original tooling geometry, open-centre chuck, gantry and force-driven platen."""

import math
from pathlib import Path

from . import spec

PACKAGE = Path(__file__).parent


def generate():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    def stage_for(rootname):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, "/" + rootname)
        stage.SetDefaultPrim(root.GetPrim())
        return stage, root.GetPrim()

    def body(stage, path, position, mass, angle=0):
        b = UsdGeom.Xform.Define(stage, path)
        b.AddTranslateOp().Set(Gf.Vec3d(*position))
        if angle:
            b.AddRotateZOp().Set(math.degrees(angle))
        UsdPhysics.RigidBodyAPI.Apply(b.GetPrim())
        UsdPhysics.MassAPI.Apply(b.GetPrim()).CreateMassAttr(mass)

    def style(geom, color, collision=True):
        geom.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
        if collision:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())

    def box(stage, path, size, pos, color=(0.16, 0.25, 0.35), collision=True):
        b = UsdGeom.Cube.Define(stage, path)
        b.CreateSizeAttr(1)
        b.AddTranslateOp().Set(Gf.Vec3d(*pos))
        b.AddScaleOp().Set(Gf.Vec3f(*size))
        style(b, color, collision)

    def cylinder(stage, path, radius, height, pos, color=(0.45, 0.48, 0.52), collision=True):
        c = UsdGeom.Cylinder.Define(stage, path)
        c.CreateAxisAttr("Z")
        c.CreateRadiusAttr(radius)
        c.CreateHeightAttr(height)
        c.AddTranslateOp().Set(Gf.Vec3d(*pos))
        style(c, color, collision)

    def ring(stage, parent, radius, bore, height, z=0, color=(0.48, 0.51, 0.56)):
        for index in range(spec.SEGMENTS):
            a, b = index * 2 * math.pi / spec.SEGMENTS, (index + 1) * 2 * math.pi / spec.SEGMENTS
            cross = [
                (bore * math.cos(a), bore * math.sin(a)),
                (radius * math.cos(a), radius * math.sin(a)),
                (radius * math.cos(b), radius * math.sin(b)),
                (bore * math.cos(b), bore * math.sin(b)),
            ]
            points = [(x, y, zz + z) for zz in (-height / 2, height / 2) for x, y in cross]
            mesh = UsdGeom.Mesh.Define(stage, f"{parent}/sector_{index:03d}")
            mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
            mesh.CreateFaceVertexCountsAttr([4] * 6)
            mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
            mesh.CreateSubdivisionSchemeAttr("none")
            style(mesh, (0.78, 0.19, 0.08) if index == 0 else color)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")

    def joint(stage, name, parent, child, axis, pos, bounds, angle=0, force=False):
        j = UsdPhysics.PrismaticJoint.Define(stage, "/Machine/" + name)
        j.CreateBody0Rel().SetTargets(["/Machine/" + parent])
        j.CreateBody1Rel().SetTargets(["/Machine/" + child])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2))))
        j.CreateLocalRot1Attr(Gf.Quatf(1))
        j.CreateLowerLimitAttr(bounds[0])
        j.CreateUpperLimitAttr(bounds[1])
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0 if force else 3_000_000.0)
        drive.CreateDampingAttr(30.0 if force else 15_000.0)
        drive.CreateMaxForceAttr(1000.0)
        drive.CreateTargetPositionAttr(0.0)

    for part in spec.PARTS:
        stage, root = stage_for("Part")
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(part["mass"])
        ring(
            stage,
            "/Part",
            part["radius"],
            spec.BORE,
            part["height"],
            color=(0.32, 0.36, 0.40) if part["blade"] else (0.48, 0.52, 0.56),
        )
        stage.GetRootLayer().Export(str(PACKAGE / (part["name"] + ".usda")))

    stage, root = stage_for("Machine")
    UsdPhysics.ArticulationRootAPI.Apply(root)
    body(stage, "/Machine/base", (0, 0, 0), 150)
    for i, x in enumerate((-0.28, 0.80)):
        for j, y in enumerate((-0.60, 0.60)):
            box(stage, f"/Machine/base/post_{i}_{j}", (0.06, 0.06, 1.6), (x, y, 0.8))
            box(stage, f"/Machine/base/foot_{i}_{j}", (0.16, 0.16, 0.04), (x, y, 0.02))
    for i, side in enumerate((-1, 1)):
        box(stage, f"/Machine/base/rail_{i}", (1.14, 0.06, 0.08), (0.26, side * 0.60, 1.5))
        box(stage, f"/Machine/base/roof_rail_{i}", (1.14, 0.06, 0.06), (0.26, side * 0.60, 1.63))
    body(stage, "/Machine/X", (0, 0, 1.5), 5)
    body(stage, "/Machine/Y", (0, 0, 1.5), 5)
    body(stage, "/Machine/Head", (0, 0, 0), 2)
    box(stage, "/Machine/X/crossbar", (0.08, 1.22, 0.06), (0, 0, 0))
    box(stage, "/Machine/Y/carriage", (0.10, 0.12, 0.10), (0, 0, 0.05))
    cylinder(stage, "/Machine/Y/z_housing", 0.025, 0.6, (-0.12, 0, -0.3), collision=False)
    cylinder(stage, "/Machine/Head/z_rod", 0.012, 0.2, (-0.12, 0, 0.18), collision=False)
    box(stage, "/Machine/Head/bracket", (0.10, 0.06, 0.04), (-0.10, 0, 0.08))
    ring(stage, "/Machine/Head/Palm", 0.075, 0.035, 0.020, z=0.060, color=(0.16, 0.28, 0.42))
    joint(stage, "slide_x", "base", "X", "X", (0, 0, 1.5), (-0.2, 0.7))
    joint(stage, "slide_y", "X", "Y", "Y", (0, 0, 0), (-0.5, 0.5))
    joint(stage, "slide_z", "Y", "Head", "Z", (0, 0, -1.5), (0.75, 1.45))
    for i in range(3):
        angle = i * 2 * math.pi / 3
        pos = (0.060 * math.cos(angle), 0.060 * math.sin(angle), 0)
        body(stage, f"/Machine/Jaw{i}", pos, 0.1, angle)
        box(stage, f"/Machine/Jaw{i}/pad", (0.004, 0.008, 0.030), (0, 0, 0), (0.84, 0.48, 0.08))
        box(stage, f"/Machine/Jaw{i}/finger", (0.004, 0.008, 0.050), (0, 0, 0.035))
        joint(stage, f"jaw_{i}", "Head", f"Jaw{i}", "X", pos, (-0.023, 0), angle, force=True)
        if i:
            prim = stage.GetPrimAtPath(f"/Machine/jaw_{i}")
            prim.AddAppliedSchema("PhysxMimicJointAPI:rotX")
            prefix = "physxMimicJoint:rotX:"
            prim.CreateRelationship(prefix + "referenceJoint").SetTargets(["/Machine/jaw_0"])
            prim.CreateAttribute(prefix + "gearing", Sdf.ValueTypeNames.Float).Set(-1.0)
            prim.CreateAttribute(prefix + "offset", Sdf.ValueTypeNames.Float).Set(0.0)
    body(stage, "/Machine/PressY", (0, 0, 1.20), 3.0)
    box(stage, "/Machine/PressY/carriage", (0.16, 0.24, 0.04), (0, 0, 0.40))
    body(stage, "/Machine/Platen", (0, 0, 1.20), 1.0)
    ring(stage, "/Machine/Platen/ContactRing", 0.050, spec.BORE, 0.020, color=(0.20, 0.32, 0.46))
    for i, side in enumerate((-1, 1)):
        box(stage, f"/Machine/Platen/ear_{i}", (0.12, 0.06, 0.02), (0, side * 0.07, 0))
        for j, x in enumerate((-0.04, 0.04)):
            cylinder(stage, f"/Machine/Platen/rod_{i}_{j}", 0.005, 0.4, (x, side * 0.09, 0.2), collision=False)
    box(stage, "/Machine/base/press_crosshead", (0.16, 1.22, 0.06), (0, 0, 1.63))
    joint(stage, "press_y", "base", "PressY", "Y", (0, 0, 1.20), (-0.46, 0.01))
    joint(stage, "press", "PressY", "Platen", "Z", (0, 0, 0), (-0.09, 0.01), force=True)
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Machine/anchor")
    fixed.CreateBody1Rel().SetTargets(["/Machine/base"])
    stage.GetRootLayer().Export(str(PACKAGE / "machine.usda"))


if __name__ == "__main__":
    generate()
