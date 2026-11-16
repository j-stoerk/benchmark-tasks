"""Original full-size USD geometry and physically articulated handling equipment."""

import math
from pathlib import Path

from . import spec

PACKAGE = Path(__file__).parent


def generate():
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    def stage_for(name, rootname="Machine"):
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        root = UsdGeom.Xform.Define(stage, "/" + rootname)
        stage.SetDefaultPrim(root.GetPrim())
        return stage, root

    def body(stage, name, pos, mass, angle=0):
        x = UsdGeom.Xform.Define(stage, name)
        x.AddTranslateOp().Set(Gf.Vec3d(*pos))
        if angle:
            x.AddRotateXOp().Set(math.degrees(angle))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(mass)
        return x

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.CreateAttribute(
            "physxCollision:contactOffset", __import__("pxr.Sdf", fromlist=["Sdf"]).ValueTypeNames.Float
        ).Set(0.0002)
        UsdGeom.Gprim(prim).CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))

    def cube(stage, path, size, pos=(0, 0, 0), color=(0.18, 0.3, 0.5)):
        g = UsdGeom.Cube.Define(stage, path)
        g.CreateSizeAttr(1)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        g.AddScaleOp().Set(Gf.Vec3f(*size))
        collide(g.GetPrim(), color)

    def cyl(stage, path, length, radius, pos=(0, 0, 0), color=(0.16, 0.4, 0.8)):
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateAxisAttr("X")
        g.CreateHeightAttr(length)
        g.CreateRadiusAttr(radius)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos))
        collide(g.GetPrim(), color)

    def joint(stage, path, a, b, axis, pos0=(0, 0, 0), angle=0, bounds=(-1, 1)):
        j = UsdPhysics.PrismaticJoint.Define(stage, path)
        j.CreateBody0Rel().SetTargets([a])
        j.CreateBody1Rel().SetTargets([b])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0))
        j.CreateLocalRot0Attr(Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(math.sin(angle / 2), 0, 0)))
        j.CreateLocalRot1Attr(Gf.Quatf(1))
        j.CreateLowerLimitAttr(bounds[0])
        j.CreateUpperLimitAttr(bounds[1])
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        jaw = path.rsplit("/", 1)[-1].startswith("jaw_")
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateStiffnessAttr(5_000_000.0 if jaw else 1_500_000.0)
        drive.CreateDampingAttr(5000.0 if jaw else 30000.0)
        drive.CreateMaxForceAttr(4000.0 if jaw else 6000.0)

    for filename in ("receiver.usda", "donor.usda"):
        stage, root = stage_for(filename)
        UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
        body(stage, "/Machine/base", (0, 0, 0), 150)
        for index, side in enumerate((-1, 1)):
            cube(stage, f"/Machine/base/post_{index}", (0.08, 0.06, 1.2), (0, side * 0.37, 0))
        cube(stage, "/Machine/base/foot", (0.32, 0.8, 0.08), (0, 0, -0.56))
        cube(stage, "/Machine/base/crossbeam", (0.08, 0.8, 0.08), (0, 0, 0.56))
        fix = UsdPhysics.FixedJoint.Define(stage, "/Machine/root_joint")
        fix.CreateBody1Rel().SetTargets(["/Machine/base"])
        if filename.startswith("receiver"):
            for name, mass in (("X", 20), ("Y", 20), ("Head", 35)):
                body(stage, "/Machine/" + name, (0, 0, 0.4), mass)
            cube(stage, "/Machine/X/rail", (0.1, 0.8, 0.08))
            cube(stage, "/Machine/Y/mast", (0.08, 0.1, 0.5), (0, 0, -0.25))
            cyl(stage, "/Machine/Head/shaft", 0.4, 0.028, (-0.2, 0, 0))
            cyl(stage, "/Machine/Head/drive_housing", 0.08, 0.08, (0.04, 0, 0))
            joint(stage, "/Machine/slide_x", "/Machine/base", "/Machine/X", "X", (0, 0, 0.4), bounds=(-0.26, 0))
            joint(stage, "/Machine/slide_y", "/Machine/X", "/Machine/Y", "Y", bounds=(-0.06, 0.42))
            joint(stage, "/Machine/slide_z", "/Machine/Y", "/Machine/Head", "Z", bounds=(-0.04, 0.16))
            for i in range(3):
                angle = i * 2 * math.pi / 3
                radial = (0, 0.025 * math.cos(angle), 0.025 * math.sin(angle))
                pos = (-0.315, radial[1], 0.4 + radial[2])
                name = f"/Machine/Jaw{i}"
                body(stage, name, pos, 0.5, angle)
                cube(stage, name + "/lug", (0.15, 0.012, 0.016), color=(0.9, 0.55, 0.08))
                joint(
                    stage,
                    f"/Machine/jaw_{i}",
                    "/Machine/Head",
                    name,
                    "Y",
                    (-0.315, radial[1], radial[2]),
                    angle,
                    (0, 0.009),
                )
                if i:
                    follower = stage.GetPrimAtPath(f"/Machine/jaw_{i}")
                    # PhysX 6.1 probes rotX for a one-DOF prismatic mimic,
                    # irrespective of the physical prismatic axis. This is an
                    # internal ideal scroll/gear coupling, never a roll attach.
                    follower.AddAppliedSchema("PhysxMimicJointAPI:rotX")
                    prefix = "physxMimicJoint:rotX:"
                    follower.CreateRelationship(prefix + "referenceJoint").SetTargets(["/Machine/jaw_0"])
                    follower.CreateAttribute(prefix + "gearing", Sdf.ValueTypeNames.Float).Set(-1.0)
                    follower.CreateAttribute(prefix + "offset", Sdf.ValueTypeNames.Float).Set(0.0)
        else:
            body(stage, "/Machine/Shaft", (0, 0, 0.4), 45)
            cyl(stage, "/Machine/Shaft/shaft", 0.56, 0.0379, (0.28, 0, 0), (0.45, 0.48, 0.52))
            joint(stage, "/Machine/withdraw", "/Machine/base", "/Machine/Shaft", "X", (0, 0, 0.4), bounds=(-0.42, 0))
        stage.GetRootLayer().Export(str(PACKAGE / filename))

    stage, root = stage_for("roll.usda", "Roll")
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(spec.MASS)
    for i in range(spec.SEGMENTS):
        a, b = i * 2 * math.pi / spec.SEGMENTS, (i + 1) * 2 * math.pi / spec.SEGMENTS
        cross = [
            (spec.BORE * math.cos(a), spec.BORE * math.sin(a)),
            (spec.RADIUS * math.cos(a), spec.RADIUS * math.sin(a)),
            (spec.RADIUS * math.cos(b), spec.RADIUS * math.sin(b)),
            (spec.BORE * math.cos(b), spec.BORE * math.sin(b)),
        ]
        points = [(x, y, z) for x in (-spec.WIDTH / 2, spec.WIDTH / 2) for y, z in cross]
        mesh = UsdGeom.Mesh.Define(stage, f"/Roll/sector_{i:02d}")
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
        mesh.CreateSubdivisionSchemeAttr("none")
        collide(mesh.GetPrim(), (0.85, 0.15, 0.10) if i == 0 else (0.22, 0.25, 0.27))
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    stage.GetRootLayer().Export(str(PACKAGE / "roll.usda"))


if __name__ == "__main__":
    generate()
