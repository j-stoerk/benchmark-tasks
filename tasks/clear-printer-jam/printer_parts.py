"""Native bend joints for the paper and a spring-loaded printer roller."""

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass

import numpy as np
from yam_arena.task_file import Box, ObjectState
from yam_arena.urdf import quat_xyzw_to_matrix

parts = {}
gripper_tips = {}


@dataclass(frozen=True)
class Panel(Box):
    angle: float = 0.0
    previous: str | None = None
    edge: tuple = ()
    break_force: float = 8.0


@dataclass(frozen=True)
class Roller(Box):
    spring: bool = False


def _quat_y(angle):
    return (0, math.sin(angle / 2), 0, math.cos(angle / 2))


def _spawn(path, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import bind_physics_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    stage = get_current_stage()
    spec = parts[cfg.part_name]
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*(translation or spec.pos)))
    angle = spec.angle if isinstance(spec, Panel) else 0.0
    x, y, z, w = _quat_y(angle)
    xform.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    prim = xform.GetPrim()
    if spec.free:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateLinearDampingAttr(0.16)
        rb.CreateAngularDampingAttr(0.18)

    shape_path = path + "/surface"
    is_page = isinstance(spec, Panel)
    if is_page:
        surface = UsdGeom.Cube.Define(stage, shape_path)
        surface.CreateSizeAttr(1)
        surface.AddScaleOp().Set(Gf.Vec3f(*spec.size))
    else:
        surface = UsdGeom.Cylinder.Define(stage, shape_path)
        surface.CreateAxisAttr("Y")
        surface.CreateRadiusAttr(spec.size[0] / 2)
        surface.CreateHeightAttr(spec.size[1])
    surface.CreateDisplayColorAttr([Gf.Vec3f(*spec.color)])
    UsdPhysics.CollisionAPI.Apply(surface.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(surface.GetPrim()).CreateContactOffsetAttr(0.00022)
    material = sim.RigidBodyMaterialCfg(static_friction=0.55 if is_page else 0.76, dynamic_friction=0.47 if is_page else 0.64)
    material.func(path + "/surface_friction", material)
    bind_physics_material(shape_path, path + "/surface_friction", stage=stage)

    if is_page and spec.previous:
        previous = parts[spec.previous]
        hinge = UsdPhysics.RevoluteJoint.Define(stage, path + "/bend")
        hinge.CreateBody0Rel().SetTargets([f"{path.rsplit('/', 1)[0]}/{spec.previous}"])
        hinge.CreateBody1Rel().SetTargets([path])
        for part, position, rotation in ((previous, hinge.CreateLocalPos0Attr(), hinge.CreateLocalRot0Attr()),
                                         (spec, hinge.CreateLocalPos1Attr(), hinge.CreateLocalRot1Attr())):
            q = _quat_y(part.angle)
            local = quat_xyzw_to_matrix(q).T @ (np.array(spec.edge) - np.array(part.pos))
            position.Set(Gf.Vec3f(*local.tolist()))
            rotation.Set(Gf.Quatf(q[3], -q[0], -q[1], -q[2]))
        hinge.CreateAxisAttr("Y")
        hinge.CreateLowerLimitAttr(-145.0)
        hinge.CreateUpperLimitAttr(145.0)
        hinge.CreateCollisionEnabledAttr(False)
        hinge.CreateExcludeFromArticulationAttr(True)
        hinge.CreateBreakForceAttr(spec.break_force)
        drive = UsdPhysics.DriveAPI.Apply(hinge.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.00014)
        drive.CreateDampingAttr(0.000006)
        drive.CreateTargetPositionAttr(0.0)
    if isinstance(spec, Roller) and spec.spring:
        slide = UsdPhysics.PrismaticJoint.Define(stage, path + "/preload")
        slide.CreateBody1Rel().SetTargets([path])
        slide.CreateLocalPos0Attr(Gf.Vec3f(*spec.pos))
        slide.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        slide.CreateAxisAttr("Z")
        slide.CreateLowerLimitAttr(-0.001)
        slide.CreateUpperLimitAttr(0.005)
        slide.CreateExcludeFromArticulationAttr(True)
        drive = UsdPhysics.DriveAPI.Apply(slide.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(300.0)
        drive.CreateDampingAttr(1.5)
        drive.CreateTargetPositionAttr(-0.003)
        drive.CreateMaxForceAttr(3.0)
    return prim


def _patch(scene):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.runtime import pad_tips_world

    @configclass
    class PartSpawner(SpawnerCfg):
        func = clone(_spawn)
        part_name: str = ""

    native_object, native_states = scene.task_object, scene.object_states

    def build(spec):
        if not isinstance(spec, (Panel, Roller)):
            return native_object(spec)
        parts[spec.name] = spec
        rotation = _quat_y(spec.angle) if isinstance(spec, Panel) else (0, 0, 0, 1)
        return Object(name=spec.name, object_type=ObjectType.RIGID if spec.free else ObjectType.BASE,
                      spawner_cfg=PartSpawner(part_name=spec.name),
                      initial_pose=Pose(position_xyz=spec.pos, rotation_xyzw=rotation))

    def array(value):
        return getattr(value, "torch", value).detach().cpu().numpy()

    def states(env, names):
        scene_state = native_states(env, [name for name in names if not isinstance(parts.get(name), Panel)])
        base = env.unwrapped
        origin = array(base.scene.env_origins[0])
        gripper_tips.update({arm: pad_tips_world(base.scene[f"{arm}_robot"]) - origin for arm in ("left", "right")})
        for name in names:
            if isinstance(parts.get(name), Panel):
                data = base.scene[name].data
                pose = array(data.root_link_pose_w)[0]
                v = array(data.root_com_vel_w)[0]
                bounds = base.arena_world.get_aabb_w(name)
                scene_state[name] = ObjectState(pose[:3] - origin, pose[3:7], v[:3], v[3:],
                                                array(bounds.min_point[0]) - origin, array(bounds.max_point[0]) - origin)
        return scene_state

    scene.task_object, scene.object_states = build, states


class _LoadScene(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(fullname)
        process_module = spec.loader.exec_module

        def exec_module(module):
            process_module(module)
            _patch(module)

        spec.loader.exec_module = exec_module
        return spec


def install_printer_props():
    if "yam_arena.scene" in sys.modules:
        _patch(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, _LoadScene())
