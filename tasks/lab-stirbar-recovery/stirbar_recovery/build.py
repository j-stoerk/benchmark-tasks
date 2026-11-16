"""Original hollow flask, real-size bar and300mm retrieval tool geometry."""

import math

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

from .assets import (
    PACKAGE,
    cylinder,
    generate_robot,
    new_stage,
    rigid,
    ring,
    style,
)


def cone_wall(stage, parent, lower_ro, lower_ri, upper_ro, upper_ri, z0, z1, segments=48):
    material=UsdShade.Material.Define(stage,'/Fixture/ClearGlass')
    shader=UsdShade.Shader.Define(stage,'/Fixture/ClearGlass/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.65,.80,.90))
    shader.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.12)
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.15)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    for i in range(segments):
        a, b = i * 2 * math.pi / segments, (i + 1) * 2 * math.pi / segments
        points = []
        for z, ro, ri in ((z0, lower_ro, lower_ri), (z1, upper_ro, upper_ri)):
            points.extend([(r * math.cos(t), r * math.sin(t), z) for r, t in ((ri, a), (ro, a), (ro, b), (ri, b))])
        m = UsdGeom.Mesh.Define(stage, f"{parent}/sector_{i:03d}")
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        m.CreateFaceVertexCountsAttr([4] * 6)
        m.CreateFaceVertexIndicesAttr([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7])
        m.CreateSubdivisionSchemeAttr("none")
        style(m, (0.63, 0.79, 0.88))
        m.CreateDisplayOpacityAttr(Vt.FloatArray([0.25]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")


def generate():
    def tool(stage, path):
        cylinder(stage, path + "/magnet_head", 0.005, 0.010, (0, 0, 0), (0.90, 0.91, 0.91))
        cylinder(stage, path + "/ptfe_rod", 0.004, 0.295, (0, 0, 0.1525), (0.94, 0.94, 0.92))
        cylinder(stage, path + "/head_witness", 0.00505, 0.001, (0, 0, 0.0045), (0.85, 0.16, 0.06), collision=False)

    generate_robot(tool)
    stage = new_stage("Fixture")
    cylinder(stage, "/Fixture/flask_bottom", 0.030, 0.002, (0, 0, 0.751), (0.65, 0.80, 0.87))
    cone_wall(stage, "/Fixture/lower_wall", 0.030, 0.028, 0.030, 0.028, 0.752, 0.800)
    cone_wall(stage, "/Fixture/shoulder", 0.030, 0.028, 0.013, 0.011, 0.800, 0.833)
    cone_wall(stage, "/Fixture/neck", 0.013, 0.011, 0.013, 0.011, 0.833, 0.855)
    ring(stage, "/Fixture/rim", 0.014, 0.011, 0.003, (0, 0, 0.855), (0.55, 0.75, 0.85))
    stage.GetRootLayer().Export(str(PACKAGE / "fixture.usda"))
    stage = new_stage("Part")
    mass = 0.003
    transverse = mass * (3 * 0.0035**2 + 0.025**2) / 12
    axial = mass * 0.0035**2 / 2
    rigid(stage, "/Part", (0, 0, 0), mass, (axial, transverse, transverse))
    cylinder(stage, "/Part/bar", 0.0035, 0.025, (0, 0, 0), (0.96, 0.96, 0.93), axis="X")
    cylinder(stage, "/Part/mark", 0.00355, 0.002, (0, 0, 0), (0.07, 0.35, 0.70), axis="X", collision=False)
    stage.GetRootLayer().Export(str(PACKAGE / "stirbar.usda"))


if __name__ == "__main__":
    generate()
