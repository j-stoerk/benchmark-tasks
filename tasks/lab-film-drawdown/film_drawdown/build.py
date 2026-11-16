"""Generate original native full-size applicator and ordinary glass substrate."""

from .assets import PACKAGE, box, generate_robot, new_stage


def generate():
    def tool(stage, path):
        box(stage, path + "/body", (0.100, 0.030, 0.025), (0, 0, 0.0175), (0.70, 0.72, 0.75))
        for i, x in enumerate((-0.045, 0.045)):
            box(stage, path + f"/foot_{i}", (0.010, 0.030, 0.005), (x, 0, 0.0025), (0.76, 0.78, 0.80))
        # Actual rigid blade lip100micrometres above the foot datum.
        box(stage, path + "/blade", (0.080, 0.003, 0.0049), (0, 0, 0.00255), (0.68, 0.70, 0.72))

    generate_robot(tool)
    stage = new_stage("Fixture")
    box(stage, "/Fixture/glass", (0.120, 0.300, 0.004), (0, 0, 0.752), (0.87, 0.90, 0.91))
    for x in (-0.058, 0.058):
        box(
            stage,
            "/Fixture/clamp_" + str(x).replace("-", "m").replace(".", "_"),
            (0.008, 0.020, 0.006),
            (x, 0, 0.755),
            (0.25, 0.30, 0.35),
        )
    for row in range(40):
        for col in range(16):
            box(
                stage,
                f"/Fixture/paint_cell_{row:02d}_{col:02d}",
                (0.00498, 0.00498, 0.00002),
                (-0.040 + (col + 0.5) * 0.005, -0.100 + (row + 0.5) * 0.005, 0.75402),
                (0.95, 0.95, 0.94),
                False,
            )
    box(stage, "/Fixture/initial_bead", (0.080, 0.008, 0.001), (0, -0.105, 0.7545), (0.08, 0.34, 0.70), False)
    # Four permanent corner marks show the intended80x200mm region in video.
    for i, (x, y) in enumerate([(-0.04, -0.10), (0.04, -0.10), (-0.04, 0.10), (0.04, 0.10)]):
        box(stage, f"/Fixture/mark_{i}", (0.002, 0.002, 0.00005), (x, y, 0.75404), (0.9, 0.1, 0.1), False)
    stage.GetRootLayer().Export(str(PACKAGE / "fixture.usda"))


if __name__ == "__main__":
    generate()
