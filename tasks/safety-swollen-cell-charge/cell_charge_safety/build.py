"""Original inert swollen-pouch prop, labelled mock charger and real pushbuttons."""

from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

from .assets import (
    PACKAGE,
    box,
    cylinder,
    generate_robot,
    joint,
    new_stage,
    rigid,
    style,
)


def label(stage, path, word, pos, size):
    file = PACKAGE / (word.lower().replace(" ", "_") + ".png")
    image = Image.new("RGB", (512, 128), "#eceeef")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=65)
    draw.text((256, 64), word, font=font, fill="#17202b", anchor="mm")
    image.save(file)
    mesh = UsdGeom.Mesh.Define(stage, path)
    x, y, z = pos
    w, h = size
    mesh.CreatePointsAttr(
        [
            Gf.Vec3f(x - w / 2, y - h / 2, z),
            Gf.Vec3f(x + w / 2, y - h / 2, z),
            Gf.Vec3f(x + w / 2, y + h / 2, z),
            Gf.Vec3f(x - w / 2, y + h / 2, z),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr("none")
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    uv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
    mat = UsdShade.Material.Define(stage, path + "_material")
    shader = UsdShade.Shader.Define(stage, path + "_material/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, path + "_material/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(file.name))
    reader = UsdShade.Shader.Define(stage, path + "_material/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def wire(stage, path, points, color):
    for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
        delta = Gf.Vec3d(*(b[j] - a[j] for j in range(3)))
        geom = UsdGeom.Cylinder.Define(stage, f"{path}/segment_{i}")
        geom.CreateRadiusAttr(0.0014)
        geom.CreateHeightAttr(delta.GetLength())
        geom.CreateAxisAttr("Z")
        geom.AddTranslateOp().Set(Gf.Vec3d(*[(a[j] + b[j]) / 2 for j in range(3)]))
        rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), delta.GetNormalized()).GetQuat()
        geom.AddOrientOp().Set(Gf.Quatf(rotation.GetReal(), Gf.Vec3f(rotation.GetImaginary())))
        style(geom, color, False)


def generate():
    def tool(stage, path):
        cylinder(stage, path + "/probe", 0.005, 0.03, (0, 0, 0.015), (0.15, 0.25, 0.40))

    generate_robot(tool)
    stage = Usd.Stage.Open(str(PACKAGE / "robot.usda"))
    for name, x, color in [("Start", 0.175, (0.08, 0.7, 0.12)), ("Stop", 0.105, (0.8, 0.08, 0.04))]:
        rigid(stage, "/Robot/" + name, (x, 0.04, 0.7835), 0.002)
        cylinder(stage, "/Robot/" + name + "/face", 0.008, 0.003, (0, 0, 0), color)
        joint(stage, name.lower() + "_button", "Base", name, "Z", (x, 0.04, 0.7835), (-0.003, 0.0))
    stage.GetRootLayer().Export(str(PACKAGE / "robot.usda"))
    stage = new_stage("Fixture")
    box(stage, "/Fixture/charger_base", (0.15, 0.10, 0.020), (0.14, 0.04, 0.760), (0.15, 0.22, 0.30))
    # Real clearance holes let the two buttons travel their full3mm; a solid
    # collider under the button would prevent the unsafe-control check.
    for name, size, pos in [
        ("front", (0.15, 0.038, 0.004), (0.14, 0.009, 0.778)),
        ("back", (0.15, 0.042, 0.004), (0.14, 0.074, 0.778)),
        ("left", (0.026, 0.024, 0.004), (0.078, 0.041, 0.778)),
        ("middle", (0.046, 0.024, 0.004), (0.140, 0.041, 0.778)),
        ("right", (0.026, 0.024, 0.004), (0.202, 0.041, 0.778)),
    ]:
        box(stage, "/Fixture/lid_" + name, size, pos, (0.15, 0.22, 0.30))
    for name, size, pos in [
        ("front", (0.15, 0.002, 0.010), (0.14, -0.009, 0.775)),
        ("back", (0.15, 0.002, 0.010), (0.14, 0.089, 0.775)),
        ("left", (0.002, 0.10, 0.010), (0.066, 0.04, 0.775)),
        ("right", (0.002, 0.10, 0.010), (0.214, 0.04, 0.775)),
    ]:
        box(stage, "/Fixture/wall_" + name, size, pos, (0.15, 0.22, 0.30))
    label(stage, "/Fixture/start_label", "START", (0.175, 0.014, 0.7801), (0.036, 0.010))
    label(stage, "/Fixture/stop_label", "STOP", (0.105, 0.014, 0.7801), (0.032, 0.010))
    label(stage, "/Fixture/charger_label", "Li-ion 3.7 V", (0.14, 0.075, 0.7801), (0.10, 0.018))
    cylinder(stage, "/Fixture/charge_led", 0.003, 0.0005, (0.14, 0.042, 0.7803), (0.06, 0.08, 0.07), collision=False)
    box(stage, "/Fixture/robot_stop_panel", (0.07, 0.030, 0.015), (0.14, -0.075, 0.7575), (0.16, 0.22, 0.31))
    label(stage, "/Fixture/robot_stop_label", "STOP ACK", (0.14, -0.080, 0.7651), (0.055, 0.012))
    cylinder(
        stage, "/Fixture/robot_stop_led", 0.003, 0.0005, (0.14, -0.065, 0.7653), (0.06, 0.08, 0.07), collision=False
    )
    box(stage, "/Fixture/pouch_seam", (0.096, 0.066, 0.002), (-0.105, 0.02, 0.751), (0.64, 0.65, 0.68))
    dome = UsdGeom.Sphere.Define(stage, "/Fixture/bulged_foil")
    dome.CreateRadiusAttr(1.0)
    dome.AddTranslateOp().Set(Gf.Vec3d(-0.105, 0.02, 0.765))
    dome.AddScaleOp().Set(Gf.Vec3f(0.045, 0.030, 0.014))
    style(dome, (0.76, 0.77, 0.80))
    # Exposed paired tabs and connected cable make the object a recognisable
    # foil-pouch battery, rather than an abstract labelled block.
    for i, x in enumerate((-0.120, -0.090)):
        box(stage, f"/Fixture/tab_{i}", (0.010, 0.020, 0.0003), (x, 0.061, 0.752), (0.72, 0.67, 0.51), False)
    wire(
        stage,
        "/Fixture/positive",
        [(-0.120, 0.070, 0.753), (-0.11, 0.10, 0.754), (0, 0.11, 0.754), (0.07, 0.09, 0.760)],
        (0.75, 0.05, 0.035),
    )
    wire(
        stage,
        "/Fixture/negative",
        [(-0.090, 0.070, 0.753), (-0.08, 0.085, 0.754), (0.01, 0.085, 0.754), (0.07, 0.075, 0.760)],
        (0.035, 0.04, 0.045),
    )
    box(stage, "/Fixture/connector", (0.015, 0.012, 0.008), (0.073, 0.0825, 0.760), (0.65, 0.07, 0.04), False)
    stage.GetRootLayer().Export(str(PACKAGE / "fixture.usda"))
    # Printable original prop forms. Foil over the rounded form gives a
    # repeatable visual swelling cue without using a damaged real battery.
    (PACKAGE / "props.scad").write_text(
        """// Millimetres. Original reusable inert props; no live cell.
// Select part="pouch" or part="case". Render(F6), then export STL.
part="pouch";
if(part=="pouch") translate([0,0,14]) scale([45,30,14]) sphere(r=1,$fn=64);
if(part=="case") difference(){
  cube([150,100,30],center=true);
  translate([0,0,-2]) cube([146,96,30],center=true);
  for(x=[-35,35]) translate([x,0,10]) cylinder(r=9,h=15,$fn=48);
}
""",
        encoding="utf-8",
    )


if __name__ == "__main__":
    generate()
