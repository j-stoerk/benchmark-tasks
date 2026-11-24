"""Native compound props and bendable film for the bread-bag closure."""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix


@dataclass(frozen=True)
class Solid:
    pos: tuple
    size: tuple
    color: tuple
    pitch: float = 0.0
    collide: bool = True
    opacity: float = 1.0


@dataclass(frozen=True)
class Prop(Box):
    children: tuple = ()
    angle_y: float = 0.0
    previous: str = ""
    edge: tuple = ()
    limits: tuple[float, float] = (-75.0, 75.0)
    hinge_stiffness: float = 0.000035


props = {}
tips = {}
def initial_quat(prop):
    return 0.0, math.sin(prop.angle_y / 2), 0.0, math.cos(prop.angle_y / 2)


def spawn(path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    stage = get_current_stage()
    prop = props[cfg.prop_name]
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(translation or prop.pos)))
    x, y, z, w = initial_quat(prop)
    xform.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
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
        geom = UsdGeom.Cube.Define(stage, child)
        geom.CreateSizeAttr(1)
        geom.AddTranslateOp().Set(Gf.Vec3d(*item.pos))
        if item.pitch:
            geom.AddRotateYOp().Set(item.pitch)
        geom.AddScaleOp().Set(Gf.Vec3f(*item.size))
        geom.CreateDisplayColorAttr([Gf.Vec3f(*item.color)])
        if item.opacity < 1.0:
            geom.CreateDisplayOpacityAttr([item.opacity])
        if item.collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim()).CreateContactOffsetAttr(0.00025)
            bind_physics_material(child, path + "/rubber", stage=stage)
    if prop.previous:
        parent = props[prop.previous]
        hinge = UsdPhysics.RevoluteJoint.Define(stage, path + "/bend")
        hinge.CreateBody0Rel().SetTargets([path.rsplit("/", 1)[0] + "/" + parent.name])
        hinge.CreateBody1Rel().SetTargets([path])
        for part, pos, rot in ((parent, hinge.CreateLocalPos0Attr(), hinge.CreateLocalRot0Attr()),
                               (prop, hinge.CreateLocalPos1Attr(), hinge.CreateLocalRot1Attr())):
            q = initial_quat(part)
            local = quat_xyzw_to_matrix(q).T @ (np.asarray(prop.edge) - part.pos)
            pos.Set(Gf.Vec3f(*local.tolist()))
            rot.Set(Gf.Quatf(q[3], -q[0], -q[1], -q[2]))
        hinge.CreateAxisAttr("Y")
        hinge.CreateExcludeFromArticulationAttr(True)
        hinge.CreateCollisionEnabledAttr(False)
        hinge.CreateLowerLimitAttr(prop.limits[0])
        hinge.CreateUpperLimitAttr(prop.limits[1])
        hinge.CreateBreakForceAttr(10.0)
        drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateDampingAttr(0.000012)
        drive.CreateStiffnessAttr(prop.hinge_stiffness)
        drive.CreateTargetPositionAttr(0)
        drive.CreateMaxForceAttr(0.022)
    return root


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
                      initial_pose=Pose(position_xyz=prop.pos, rotation_xyzw=initial_quat(prop)))

    def numpy(value):
        return getattr(value, "torch", value).detach().cpu().numpy().astype(float)

    def states(env, names):
        rigid = {name for name in names if name in props and props[name].free}
        out = base_states(env, [name for name in names if name not in rigid])
        base = env.unwrapped
        origin = numpy(base.scene.env_origins[0])
        tips.update({side: pad_tips_world(base.scene[f"{side}_robot"]) - origin for side in ("left", "right")})
        for name in rigid:
            state = base.scene[name].data
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


def install(reset_hook):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    if "yam_arena.scene" in sys.modules:
        patch_scene(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, Loader())
    original = YamArenaEmbodiment.reset

    def reset(self, *args, **kwargs):
        result = original(self, *args, **kwargs)
        if self.task and self.task.path.parent == Path(__file__).parent:
            reset_hook()
        return result

    YamArenaEmbodiment.reset = reset
