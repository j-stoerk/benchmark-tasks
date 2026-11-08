"""Generate redistributable USD primitives with a genuinely open roll core."""

import math
from pathlib import Path

from yam_arena.task_file import load

PACKAGE = Path(__file__).parent


def roll_segments():
    m = load(PACKAGE.parent / "task.py").module
    segments = []
    for i in range(m.SEGMENTS):
        a, b = i * 2 * math.pi / m.SEGMENTS, (i + 1) * 2 * math.pi / m.SEGMENTS
        cross = [
            (m.INNER_R * math.cos(a), m.INNER_R * math.sin(a)),
            (m.OUTER_R * math.cos(a), m.OUTER_R * math.sin(a)),
            (m.OUTER_R * math.cos(b), m.OUTER_R * math.sin(b)),
            (m.INNER_R * math.cos(b), m.INNER_R * math.sin(b)),
        ]
        segments.append([(x, y, z) for y in (-m.ROLL_WIDTH / 2, m.ROLL_WIDTH / 2) for x, z in cross])
    return segments


def header():
    return (
        '#usda 1.0\n(\n    defaultPrim = "Body"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n'
        'def Xform "Body" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])\n{\n'
        "    bool physics:rigidBodyEnabled = true\n    float physics:mass = 0.15\n"
    )


def asset_texts():
    m = load(PACKAGE.parent / "task.py").module
    roll = header()
    faces = [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
    for i, vertices in enumerate(roll_segments()):
        points = ", ".join("(" + ", ".join(f"{v:.10f}" for v in p) + ")" for p in vertices)
        roll += (
            f'    def Mesh "Sector_{i:02d}" (prepend apiSchemas = '
            '["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI"])\n    {\n'
            f"        point3f[] points = [{points}]\n"
            "        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]\n"
            f"        int[] faceVertexIndices = {faces}\n"
            '        uniform token subdivisionScheme = "none"\n'
            '        uniform token physics:approximation = "convexHull"\n'
            "        bool physics:collisionEnabled = true\n"
            "        float physxCollision:contactOffset = 0.0005\n"
            "        float physxCollision:restOffset = 0\n"
            "        color3f[] primvars:displayColor = [(0.24, 0.26, 0.28)]\n    }\n"
        )
    roll += "}\n"
    spindle = (
        header().replace("= 0.15", "= 0.085")
        + '    def Cylinder "Shaft" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])\n'
        '    {\n        uniform token axis = "Y"\n'
        f"        double height = {m.SHAFT_LENGTH}\n        double radius = {m.SHAFT_R}\n"
        "        bool physics:collisionEnabled = true\n"
        "        float physxCollision:contactOffset = 0.0005\n"
        "        float physxCollision:restOffset = 0\n"
        "        color3f[] primvars:displayColor = [(0.05, 0.25, 0.85)]\n    }\n"
    )
    for i, y in enumerate(m.HANDLE_YS):
        spindle += (
            f'    def Cube "Grip_{i}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])\n'
            "    {\n        double size = 1\n"
            f"        double3 xformOp:translate = (0, {y}, 0)\n"
            f"        double3 xformOp:scale = {m.HANDLE_SIZE}\n"
            '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            "        bool physics:collisionEnabled = true\n"
            "        float physxCollision:contactOffset = 0.0005\n"
            "        float physxCollision:restOffset = 0\n"
            "        color3f[] primvars:displayColor = [(0.05, 0.25, 0.85)]\n    }\n"
        )
    spindle += "}\n"
    return {"roll.usda": roll, "spindle.usda": spindle}


def write_assets():
    for filename, content in asset_texts().items():
        (PACKAGE / filename).write_text(content, encoding="utf-8")


def register_assets():
    """Register USD paths through Arena's public registry after Kit starts."""
    from isaaclab_arena.assets.registries import AssetRegistry

    registry = AssetRegistry()
    for name, filename in (("battery_training_roll", "roll.usda"), ("battery_training_spindle", "spindle.usda")):
        path = str(PACKAGE / filename)
        if not Path(path).is_file():
            raise FileNotFoundError(f"Missing bundled asset: {filename}")
        if registry.is_registered(name):
            if registry.get_asset_by_name(name).usd_path != path:
                raise ValueError(f"Another asset already uses registry key {name}")
        else:
            asset = type(name, (), {"usd_path": path, "name": name, "registry_name": name, "tags": ["object"]})
            registry.register(asset, name)


if __name__ == "__main__":
    write_assets()
