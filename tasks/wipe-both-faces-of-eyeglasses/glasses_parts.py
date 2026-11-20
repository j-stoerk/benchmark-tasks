"""Native glasses, folded cloth and removable markings."""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState


@dataclass(frozen=True)
class Solid:
    pos: tuple
    size: tuple
    color: tuple
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    axis: str = ""
    collide: bool = True
    opacity: float = 1.0


@dataclass(frozen=True)
class Prop(Box):
    children: tuple = ()


props = {}
tips = {}
visual_paths = {}
assets = {}


def spawn(path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    stage = get_current_stage()
    prop = props[cfg.prop_name]
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(translation or prop.pos)))
    root = xform.GetPrim()
    if prop.free:
        UsdPhysics.RigidBodyAPI.Apply(root)
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr(prop.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
        rb.CreateSolverPositionIterationCountAttr(24)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
    material = sim.RigidBodyMaterialCfg(static_friction=0.8, dynamic_friction=0.65)
    material.func(path + "/rubber", material)
    for i, item in enumerate(prop.children):
        child = f"{path}/shape_{i}"
        if item.axis:
            geom = UsdGeom.Cylinder.Define(stage, child)
            geom.CreateRadiusAttr(item.size[0])
            geom.CreateHeightAttr(item.size[1])
            geom.CreateAxisAttr(item.axis)
            geom.AddTranslateOp().Set(Gf.Vec3d(*item.pos))
        else:
            geom = UsdGeom.Cube.Define(stage, child)
            geom.CreateSizeAttr(1)
            geom.AddTranslateOp().Set(Gf.Vec3d(*item.pos))
            if item.yaw:
                geom.AddRotateZOp().Set(math.degrees(item.yaw))
            if item.pitch:
                geom.AddRotateYOp().Set(item.pitch)
            if item.roll:
                geom.AddRotateXOp().Set(item.roll)
            geom.AddScaleOp().Set(Gf.Vec3f(*item.size))
        geom.CreateDisplayColorAttr([Gf.Vec3f(*item.color)])
        if item.opacity != 1:
            geom.CreateDisplayOpacityAttr([item.opacity])
        if item.collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim()).CreateContactOffsetAttr(0.0006)
            bind_physics_material(child, path + "/rubber", stage=stage)
        else:
            visual_paths[prop.name, i] = child
    return root


def show_mark(index, present):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    path = visual_paths.get(("glasses", index))
    if path:
        mark = UsdGeom.Imageable(get_current_stage().GetPrimAtPath(path))
        mark.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if present else UsdGeom.Tokens.invisible)


def cloth_reaction(axis_force):
    import torch

    for name, sign in (("cloth", -1.0), ("glasses", 1.0)):
        if name in assets:
            body = assets[name]
            force = torch.as_tensor(sign * axis_force, dtype=torch.float32, device=body.device).reshape(1, 1, 3)
            kwargs = dict(forces=force, torques=torch.zeros_like(force), is_global=True)
            if hasattr(body, "permanent_wrench_composer"):
                body.permanent_wrench_composer.set_forces_and_torques_index(**kwargs)
            else:
                body.set_external_force_and_torque(**kwargs)


def patch_scene(scene):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.runtime import pad_tips_world

    @configclass
    class PropSpawner(SpawnerCfg):
        func = clone(spawn)
        prop_name: str = ""

    base_object, base_states = scene.task_object, scene.object_states

    def create(prop):
        if not isinstance(prop, Prop):
            return base_object(prop)
        props[prop.name] = prop
        return Object(name=prop.name, object_type=ObjectType.RIGID if prop.free else ObjectType.BASE,
                      spawner_cfg=PropSpawner(prop_name=prop.name),
                      initial_pose=Pose(position_xyz=prop.pos, rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))

    def numpy(value):
        return getattr(value, "torch", value).detach().cpu().numpy().astype(float)

    def states(env, names):
        rigid = {name for name in names if name in props and props[name].free}
        out = base_states(env, [name for name in names if name not in rigid])
        base = env.unwrapped
        origin = numpy(base.scene.env_origins[0])
        tips.update({side: pad_tips_world(base.scene[f"{side}_robot"]) - origin for side in ("left", "right")})
        for name in rigid:
            assets[name] = base.scene[name]
            state = assets[name].data
            pose = numpy(state.root_link_pose_w)[0]
            v = numpy(state.root_com_vel_w)[0]
            bounds = base.arena_world.get_aabb_w(name)
            out[name] = ObjectState(pose[:3]-origin, pose[3:7], v[:3], v[3:],
                                    numpy(bounds.min_point[0])-origin, numpy(bounds.max_point[0])-origin)
        return out

    scene.task_object, scene.object_states = create, states


class Loader(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        original = spec.loader.exec_module

        def load(module):
            original(module)
            patch_scene(module)

        spec.loader.exec_module = load
        return spec


def install(reset_process):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    if "yam_arena.scene" in sys.modules:
        patch_scene(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, Loader())
    original = YamArenaEmbodiment.reset

    def reset(self, *args, **kwargs):
        obs = original(self, *args, **kwargs)
        if self.task and self.task.path.parent == Path(__file__).parent:
            reset_process()
        return obs

    YamArenaEmbodiment.reset = reset


def allow_demo_orientation_matrix():
    from yam_arena import demo

    native = demo.grip_rotation

    def grip(yaw):
        return yaw if isinstance(yaw, np.ndarray) else native(yaw)

    demo.grip_rotation = grip
