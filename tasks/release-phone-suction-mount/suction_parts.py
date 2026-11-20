"""Prop joints and suction release for the stock bimanual YAM task file."""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

specs = {}
joint_paths = {}
tips = {}


@dataclass(frozen=True)
class Piece:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]
    cylinder: bool = False


@dataclass(frozen=True)
class Body(Box):
    pieces: tuple[Piece, ...] = ()
    fixed_seal: bool = False
    joint_limits: tuple[float, float] | None = None
    initial_tilt: float = 0.0
    stiffness: float = 0.0


def _quat_x(degrees):
    half = math.radians(degrees) / 2
    return math.sin(half), 0.0, 0.0, math.cos(half)


def _rotation(yaw, tilt=0.0):
    sx, _, _, cx = _quat_x(tilt)
    sz, cz = math.sin(yaw / 2), math.cos(yaw / 2)
    return cz * sx, sz * sx, sz * cx, cz * cx


def _spawn(path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    spec = specs[cfg.body_name]
    stage = get_current_stage()
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(translation or spec.pos)))
    x, y, z, w = _rotation(spec.yaw, spec.initial_tilt)
    xform.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    root = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(spec.mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
    rb.CreateSolverPositionIterationCountAttr(24)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    if spec.name == "tab":
        friction = sim.RigidBodyMaterialCfg(static_friction=1.05, dynamic_friction=0.90)
    else:
        friction = sim.RigidBodyMaterialCfg(static_friction=0.75, dynamic_friction=0.60)
    material = path + "/Material"
    friction.func(material, friction)
    for i, piece in enumerate(spec.pieces):
        part = f"{path}/piece_{i}"
        if piece.cylinder:
            geom = UsdGeom.Cylinder.Define(stage, part)
            geom.CreateAxisAttr("Z")
            geom.CreateRadiusAttr(piece.size[0] / 2)
            geom.CreateHeightAttr(piece.size[2])
            geom.AddTranslateOp().Set(Gf.Vec3d(*piece.center))
        else:
            geom = UsdGeom.Cube.Define(stage, part)
            geom.CreateSizeAttr(1)
            geom.AddTranslateOp().Set(Gf.Vec3d(*piece.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*piece.size))
        geom.CreateDisplayColorAttr([Gf.Vec3f(*piece.color)])
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim()).CreateContactOffsetAttr(0.0007)
        bind_physics_material(part, material, stage=stage)

    if spec.fixed_seal:
        seal_path = path + "/suction_seal"
        joint = UsdPhysics.FixedJoint.Define(stage, seal_path)
        joint.CreateBody1Rel().SetTargets([path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*spec.pos))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr(Gf.Quatf(w, x, y, z))
        joint.CreateBreakForceAttr(85.0)
        joint.CreateExcludeFromArticulationAttr(True)
        joint_paths["seal"] = seal_path
    if spec.joint_limits is not None:
        pivot_path = path + "/pivot"
        pivot = UsdPhysics.RevoluteJoint.Define(stage, pivot_path)
        pivot.CreateBody0Rel().SetTargets([path.rsplit('/', 1)[0] + "/mount"])
        pivot.CreateBody1Rel().SetTargets([path])
        pivot.CreateAxisAttr("X")
        mount = specs["mount"]
        relative = np.array(spec.pos) - np.array(mount.pos)
        relative = quat_xyzw_to_matrix(_rotation(mount.yaw)).T @ relative
        pivot.CreateLocalPos0Attr(Gf.Vec3f(*relative.tolist()))
        pivot.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        tx, ty, tz, tw = _quat_x(spec.initial_tilt)
        pivot.CreateLocalRot1Attr(Gf.Quatf(tw, -tx, -ty, -tz))
        pivot.CreateLowerLimitAttr(float(spec.joint_limits[0]))
        pivot.CreateUpperLimitAttr(float(spec.joint_limits[1]))
        pivot.CreateCollisionEnabledAttr(False)
        pivot.CreateExcludeFromArticulationAttr(True)
        drive = UsdPhysics.DriveAPI.Apply(pivot.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(spec.stiffness)
        drive.CreateDampingAttr(0.00012)
        drive.CreateMaxForceAttr(0.24)
        drive.CreateTargetPositionAttr(0)
        joint_paths[spec.name] = pivot_path
    return root


def _prim(name):
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage().GetPrimAtPath(joint_paths[name])


def clear_seal():
    from pxr import UsdPhysics

    UsdPhysics.FixedJoint(_prim("seal")).GetJointEnabledAttr().Set(False)


def update_lever_drive(target):
    from pxr import UsdPhysics

    UsdPhysics.DriveAPI(_prim("lever"), "angular").GetTargetPositionAttr().Set(target)


def _patch_scene(scene):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.runtime import pad_tips_world

    @configclass
    class BodySpawner(SpawnerCfg):
        func = clone(_spawn)
        body_name: str = ""

    native_object, native_states = scene.task_object, scene.object_states

    def build(spec):
        if not isinstance(spec, Body):
            return native_object(spec)
        specs[spec.name] = spec
        return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=BodySpawner(body_name=spec.name),
                      initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=_rotation(spec.yaw, spec.initial_tilt)))

    def array(value):
        return getattr(value, "torch", value).detach().cpu().numpy().astype(np.float64)

    def states(env, names):
        state = native_states(env, [name for name in names if name not in specs])
        base = env.unwrapped
        origin = array(base.scene.env_origins[0])
        tips.update({arm: pad_tips_world(base.scene[f"{arm}_robot"]) - origin for arm in ("left", "right")})
        for name in specs.keys() & set(names):
            data = base.scene[name].data
            pose = array(data.root_link_pose_w)[0]
            v = array(data.root_com_vel_w)[0]
            bounds = base.arena_world.get_aabb_w(name)
            state[name] = ObjectState(pose[:3] - origin, pose[3:7], v[:3], v[3:],
                                      array(bounds.min_point[0]) - origin, array(bounds.max_point[0]) - origin)
        return state

    scene.task_object, scene.object_states = build, states


class _SceneLoader(importlib.abc.MetaPathFinder):
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


def install_parts(reset_process):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, _SceneLoader())
    native_reset = YamArenaEmbodiment.reset

    def reset(self, *args, **kwargs):
        if self.task is not None and self.task.path.parent == Path(__file__).parent:
            reset_process()
            if self._env is not None and "seal" in joint_paths:
                from pxr import UsdPhysics

                UsdPhysics.FixedJoint(_prim("seal")).GetJointEnabledAttr().Set(True)
                update_lever_drive(0.0)
        return native_reset(self, *args, **kwargs)

    YamArenaEmbodiment.reset = reset
