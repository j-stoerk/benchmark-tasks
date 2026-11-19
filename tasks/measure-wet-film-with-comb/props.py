"""Add coupon and toothed-comb props to the stock YAM task scene."""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState

_tooth_paths = []
_props = {}
arm_tips = {}


@dataclass(frozen=True)
class Piece:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]
    collision: bool = True


@dataclass(frozen=True)
class Prop(Box):
    pieces: tuple[Piece, ...] = ()


def spawn(prim_path, config, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = _props[config.prop_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    if orientation is not None:
        w, x, y, z = orientation
        root.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
    rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid.CreateSolverPositionIterationCountAttr(24)
    rigid.CreateMaxDepenetrationVelocityAttr(0.5)

    material_path = prim_path + "/Friction"
    friction = sim.RigidBodyMaterialCfg(static_friction=0.72, dynamic_friction=0.55)
    friction.func(material_path, friction)
    if spec.name == "comb":
        _tooth_paths.clear()
    for i, piece in enumerate(spec.pieces):
        path = f"{prim_path}/piece_{i}"
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*piece.center))
        cube.AddScaleOp().Set(Gf.Vec3f(*piece.size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb(piece.color))])
        if piece.collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim()).CreateContactOffsetAttr(0.00025)
            bind_physics_material(path, material_path, stage=stage)
        if spec.name == "comb" and i > 0:
            _tooth_paths.append(path)
    return prim


def color_tooth(index, color):
    if index >= len(_tooth_paths):
        return
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom
    from yam_arena.station import linear_rgb

    tooth = UsdGeom.Gprim(get_current_stage().GetPrimAtPath(_tooth_paths[index]))
    tooth.GetDisplayColorAttr().Set([Gf.Vec3f(*linear_rgb(color))])


def _patch_scene(scene):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PropSpawner(SpawnerCfg):
        func = clone(spawn)
        prop_name: str = ""

    original_object = scene.task_object
    original_states = scene.object_states
    props = set()

    def make_object(spec):
        if not isinstance(spec, Prop):
            return original_object(spec)
        props.add(spec.name)
        _props[spec.name] = spec
        return Object(
            name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=PropSpawner(prop_name=spec.name),
            initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=(0, 0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
        )

    def array(value):
        return getattr(value, "torch", value).detach().cpu().numpy().astype(np.float64)

    def object_states(env, names):
        from yam_arena.runtime import pad_tips_world

        states = original_states(env, [name for name in names if name not in props])
        base = env.unwrapped
        origin = array(base.scene.env_origins[0])
        arm_tips.update({side: pad_tips_world(base.scene[f"{side}_robot"]) - origin for side in ("left", "right")})
        for name in props.intersection(names):
            state = base.scene[name].data
            pose = array(state.root_link_pose_w)[0]
            velocity = array(state.root_com_vel_w)[0]
            pos = pose[:3] - origin
            bbox = base.arena_world.get_aabb_w(name)
            states[name] = ObjectState(pos, pose[3:7], velocity[:3], velocity[3:],
                                       array(bbox.min_point[0]) - origin, array(bbox.max_point[0]) - origin)
        return states

    scene.task_object = make_object
    scene.object_states = object_states


class _SceneExtension(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        original = spec.loader.exec_module

        def exec_module(module):
            original(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def install(reset_reading):
    """Extend props and clear task data on reset without altering either arm."""
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, _SceneExtension())

    original_reset = YamArenaEmbodiment.reset

    def reset(self, *args, **kwargs):
        if self.task is not None and self.task.path.parent == Path(__file__).parent:
            reset_reading()
            original_reset(self, *args, **kwargs)
            for i in range(len(_tooth_paths)):
                color_tooth(i, (0.70, 0.74, 0.78))
            return self._observation()
        return original_reset(self, *args, **kwargs)

    YamArenaEmbodiment.reset = reset
