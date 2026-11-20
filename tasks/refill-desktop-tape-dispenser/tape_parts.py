"""Native cradle, roller and hinged tape film for the desktop dispenser."""

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
    yaw: float = 0.0
    axis: str = ""
    collide: bool = True


@dataclass(frozen=True)
class Prop(Box):
    children: tuple = ()
    angle_x: float = 0.0
    previous: str = ""
    edge: tuple = ()
    can_tack: bool = False
    flip_width: bool = False


props = {}
tips = {}
tack_joints = {}

def initial_quat(prop):
    sine, cosine = math.sin(prop.angle_x / 2), math.cos(prop.angle_x / 2)
    if prop.flip_width:
        return 0.0, cosine, sine, 0.0
    return sine, 0.0, 0.0, cosine


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
            geom.AddScaleOp().Set(Gf.Vec3f(*item.size))
        geom.CreateDisplayColorAttr([Gf.Vec3f(*item.color)])
        if item.collide:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim()).CreateContactOffsetAttr(0.0006)
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
        hinge.CreateAxisAttr("X")
        hinge.CreateExcludeFromArticulationAttr(True)
        hinge.CreateCollisionEnabledAttr(False)
        hinge.CreateLowerLimitAttr(-95)
        hinge.CreateUpperLimitAttr(95)
        hinge.CreateBreakForceAttr(16.0)
        drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateDampingAttr(0.000015)
        drive.CreateStiffnessAttr(0.0015)
        drive.CreateTargetPositionAttr(0)
        drive.CreateMaxForceAttr(0.035)
    if prop.can_tack:
        tack = UsdPhysics.FixedJoint.Define(stage, path + "/adhesive_contact")
        tack.CreateBody1Rel().SetTargets([path])
        tack.CreateExcludeFromArticulationAttr(True)
        tack.CreateJointEnabledAttr(False)
        tack.CreateBreakForceAttr(3.0)
        tack_joints[prop.name] = tack.GetPath()
    return root


def tack(name, state=None):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    if name not in tack_joints:
        return
    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(tack_joints[name]))
    if state is not None:
        x, y, z, w = state.quat_xyzw.tolist()
        joint.CreateLocalPos0Attr(Gf.Vec3f(*state.pos.tolist()))
        joint.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
    joint.GetJointEnabledAttr().Set(state is not None)


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


def allow_demo_orientation_matrix():
    from yam_arena import demo

    native = demo.grip_rotation

    def orientation(yaw):
        return yaw if isinstance(yaw, np.ndarray) else native(yaw)

    demo.grip_rotation = orientation
