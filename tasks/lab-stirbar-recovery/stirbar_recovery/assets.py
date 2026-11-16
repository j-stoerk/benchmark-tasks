"""Original metre-scale USD geometry and a six-axis force-limited tool robot."""

import math
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from . import spec

PACKAGE = Path(__file__).parent


def new_stage(name):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/" + name)
    stage.SetDefaultPrim(root.GetPrim())
    return stage


def style(geom, color, collision=True):
    geom.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    if collision:
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())


def box(stage, path, size, pos, color=(0.3, 0.4, 0.5), collision=True):
    geom = UsdGeom.Cube.Define(stage, path)
    geom.CreateSizeAttr(1.0)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    style(geom, color, collision)
    return geom


def cylinder(stage, path, radius, height, pos, color=(0.8, 0.8, 0.8), axis="Z", collision=True):
    geom = UsdGeom.Cylinder.Define(stage, path)
    geom.CreateAxisAttr(axis)
    geom.CreateRadiusAttr(radius)
    geom.CreateHeightAttr(height)
    geom.AddTranslateOp().Set(Gf.Vec3d(*pos))
    style(geom, color, collision)
    return geom


def ring(stage, path, ro, ri, height, pos, color=(0.7, 0.8, 0.85), segments=48):
    # Convex wedges preserve the real opening; no convex hull across a bore.
    for i in range(segments):
        a, b = i * 2 * math.pi / segments, (i + 1) * 2 * math.pi / segments
        points = [
            (r * math.cos(t) + pos[0], r * math.sin(t) + pos[1], z + pos[2])
            for z in (-height / 2, height / 2)
            for r, t in ((ri, a), (ro, a), (ro, b), (ri, b))
        ]
        m = UsdGeom.Mesh.Define(stage, f"{path}/sector_{i:03d}")
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        m.CreateFaceVertexCountsAttr([4] * 6)
        m.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
        m.CreateSubdivisionSchemeAttr("none")
        style(m, color)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")


def rigid(stage, path, pos, mass, inertia=None):
    body = UsdGeom.Xform.Define(stage, path)
    body.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    api = UsdPhysics.MassAPI.Apply(body.GetPrim())
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(0))
    if inertia is not None:
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
    body.GetPrim().AddAppliedSchema("PhysxRigidBodyAPI")
    body.GetPrim().CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)


def joint(stage, name, parent, child, axis, translation, bounds, rotary=False):
    cls = UsdPhysics.RevoluteJoint if rotary else UsdPhysics.PrismaticJoint
    j = cls.Define(stage, "/Robot/" + name)
    j.CreateBody0Rel().SetTargets(["/Robot/" + parent])
    j.CreateBody1Rel().SetTargets(["/Robot/" + child])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*translation))
    j.CreateLocalPos1Attr(Gf.Vec3f(0))
    j.CreateLowerLimitAttr(bounds[0])
    j.CreateUpperLimitAttr(bounds[1])
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if rotary else "linear")
    d.CreateTypeAttr("force")
    d.CreateMaxForceAttr(2.0 if rotary else 40.0)
    d.CreateStiffnessAttr(40.0 if rotary else 20000.0)
    d.CreateDampingAttr(3.0 if rotary else 250.0)


def generate_robot(tool_builder):
    stage = new_stage("Robot")
    root = stage.GetDefaultPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    rigid(stage, "/Robot/Base", (0, 0, 0), 10.0)
    # Physical supports are kept behind the bench, away from the close-up.
    for x in (-0.40, 0.40):
        box(
            stage, f"/Robot/Base/post_{str(x).replace('-', 'm').replace('.', '_')}", (0.025, 0.025, 1.2), (x, 0.38, 0.6)
        )
    box(stage, "/Robot/Base/rail", (0.85, 0.025, 0.025), (0, 0.38, 1.2))
    for name, mass in [("X", 0.10), ("Y", 0.10), ("Z", 0.10), ("Yaw", 0.05), ("Pitch", 0.05), ("Tool", spec.TOOL_MASS)]:
        rigid(stage, "/Robot/" + name, (0, 0, 0), mass)
    UsdPhysics.MassAPI(stage.GetPrimAtPath("/Robot/Tool")).CreateCenterOfMassAttr(Gf.Vec3f(*spec.TOOL_COM))
    # X/Y links carry visible telescoping hardware; Tool's own shape determines
    # contact. All three rotational joints share one real gimbal centre.
    box(stage, "/Robot/X/x_carriage", (0.04, 0.04, 0.03), (0, 0.38, 1.2), collision=False)
    box(stage, "/Robot/Y/y_rail", (0.025, 0.76, 0.025), (0, 0, 1.2), collision=False)
    cylinder(stage, "/Robot/Z/vertical_support", 0.008, 0.15, (0, 0, 0.10), collision=False)
    cylinder(stage, "/Robot/Tool/tool_mount", 0.012, 0.025, (0, 0, spec.MOUNT_Z), (0.12, 0.25, 0.45))
    joint(stage, "slide_x", "Base", "X", "X", (0, 0, 0), (-0.35, 0.35))
    joint(stage, "slide_y", "X", "Y", "Y", (0, 0, 0), (-0.35, 0.35))
    joint(stage, "slide_z", "Y", "Z", "Z", (0, 0, 0), (0.74, 1.2))
    joint(stage, "rotate_z", "Z", "Yaw", "Z", (0, 0, 0), (-180, 180), True)
    joint(stage, "rotate_y", "Yaw", "Pitch", "Y", (0, 0, 0), (-100, 100), True)
    joint(stage, "rotate_x", "Pitch", "Tool", "X", (0, 0, 0), (-100, 100), True)
    tool_builder(stage, "/Robot/Tool")
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Robot/anchor")
    fixed.CreateBody1Rel().SetTargets(["/Robot/Base"])
    stage.GetRootLayer().Export(str(PACKAGE / "robot.usda"))
