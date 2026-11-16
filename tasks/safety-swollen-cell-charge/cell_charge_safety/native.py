"""Single Cartesian-gimbal tool robot in Arena; tasks supply the physical process.

The robot has six force-limited native joints. Task objects never follow commanded
poses. Material-response models may apply finite, reaction-balanced wrenches.
"""

from pathlib import Path

import isaaclab.sim as sim
import numpy as np
import torch
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import mdp
from isaaclab.managers import (
    ActionTerm,
    ActionTermCfg,
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    SceneEntityCfg,
)
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab_arena.assets.object_library import DirectionalLight, DomeLight
from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
from isaaclab_arena.environments.isaaclab_arena_environment import (
    IsaacLabArenaEnvironment,
)
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.cameras import ArenaCameraCfg
from isaaclab_arena.utils.configclass import make_configclass
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import (
    PhysxArticulationRootPropertiesCfg,
    PhysxCollisionPropertiesCfg,
    PhysxRigidBodyPropertiesCfg,
)
from yam_arena.scene import Object, ObjectType, _box, env_cfg_callback, object_states

from . import spec
from .process import Process

PACKAGE = Path(__file__).parent


def tensor(value):
    return getattr(value, "torch", value)


def array(value):
    return tensor(value).detach().cpu().numpy().astype(float)


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def tool_pose(env, body_name="Tool"):
    robot = env.scene["robot"]
    data = robot.data
    i = robot.body_names.index(body_name)
    p = array(data.body_link_pos_w if hasattr(data, "body_link_pos_w") else data.body_pos_w)[0, i]
    q = array(data.body_link_quat_w if hasattr(data, "body_link_quat_w") else data.body_quat_w)[0, i]
    return p, quaternion_matrix(q), q


def wrench(asset, force, torque, body_id=None):
    kwargs = dict(
        forces=torch.tensor([[force]], device=asset.device, dtype=torch.float32),
        torques=torch.tensor([[torque]], device=asset.device, dtype=torch.float32),
        is_global=True,
    )
    if body_id is not None:
        kwargs["body_ids"] = [body_id]
    if hasattr(asset, "permanent_wrench_composer"):
        asset.permanent_wrench_composer.set_forces_and_torques_index(**kwargs)
    else:
        asset.set_external_force_and_torque(**kwargs)


class ToolAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINTS, preserve_order=True)[0]
        self._raw = torch.tensor([spec.INITIAL.tolist()], device=self.device)
        self._processed = self._raw.clone()
        self.speed = torch.tensor([0.12, 0.12, 0.12, 0.8, 0.8, 0.8, 4.0], device=self.device)
        self.process = Process(env, cfg.variant)
        self.abort_hold = None

    @property
    def action_dim(self):
        return 7

    @property
    def raw_actions(self):
        return self._raw

    @property
    def processed_actions(self):
        return self._processed

    def process_actions(self, actions):
        self._raw[:] = actions
        if getattr(spec, "ABORT_ENABLED", False):
            if self.abort_hold is None and float(actions[0, 6]) >= 0.5:
                self.abort_hold = tensor(self.robot.data.joint_pos)[:, self.ids].clone()
            if self.abort_hold is not None:
                self._processed[:, :6] = self.abort_hold
                self._processed[:, 6] = 1.0
                return
        change = self.speed * self._env.step_dt
        self._processed += torch.clamp(actions - self._processed, -change, change)

    def apply_actions(self):
        # Native Cartesian slides plus Z/Y/X gimbal; a material model does not
        # move any object to these setpoints.
        q = self._processed
        if hasattr(self.robot, "set_joint_position_target_index"):
            self.robot.set_joint_position_target_index(target=q[:, :6], joint_ids=self.ids)
        else:
            self.robot.set_joint_position_target(q[:, :6], joint_ids=self.ids)
        self.process.substep(float(q[0, 6]), self._env.physics_dt)

    def reset(self, env_ids=None):
        self._raw[:] = torch.tensor(spec.INITIAL.tolist(), device=self.device)
        self._processed[:] = self._raw
        self.process.reset()
        self.abort_hold = None


@configclass
class ToolActionCfg(ActionTermCfg):
    class_type: type = ToolAction
    asset_name: str = "robot"
    variant: str = "normal"


def encoders(env):
    return tensor(env.scene["robot"].data.joint_pos)


@configclass
class PolicyObs(ObservationGroupCfg):
    joints: ObservationTermCfg = ObservationTermCfg(func=encoders)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = False


@configclass
class ObsCfg:
    policy: PolicyObs = PolicyObs()


def camera_quat(eye, target):
    from yam_arena.ik import matrix_to_quat_xyzw

    x = np.asarray(target) - eye
    x /= np.linalg.norm(x)
    y = np.cross([0, 0, 1], x)
    y /= np.linalg.norm(y)
    return tuple(matrix_to_quat_xyzw(np.column_stack((x, y, np.cross(x, y)))))


class ToolRobot(EmbodimentBase):
    name = "single_cartesian_gimbal_tool_robot"

    def __init__(self, variant="normal"):
        super().__init__(enable_cameras=True)
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Robot",
            spawn=sim.UsdFileCfg(
                usd_path=str(PACKAGE / "robot.usda"),
                ensure_drives_exist=True,
                activate_contact_sensors=True,
                rigid_props=PhysxRigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=0.1),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, solver_position_iteration_count=32, solver_velocity_iteration_count=2
                ),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.00003, rest_offset=0),
                physics_material=sim.RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.35),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos=dict(zip(spec.JOINTS, spec.INITIAL[:6].tolist())), joint_vel={".*": 0.0}
            ),
            actuators={
                "slides": ImplicitActuatorCfg(
                    joint_names_expr=["slide_.*"],
                    joint_effort_limit=40.0,
                    joint_velocity_limit=0.12,
                    stiffness=20000.0,
                    damping=250.0,
                ),
                "gimbal": ImplicitActuatorCfg(
                    joint_names_expr=["rotate_.*"],
                    joint_effort_limit=2.0,
                    joint_velocity_limit=0.8,
                    stiffness=40.0,
                    damping=3.0,
                ),
            },
        )
        for name, params in getattr(spec, "PASSIVE_ACTUATORS", {}).items():
            robot.actuators[name] = ImplicitActuatorCfg(**params)
        sensors = [
            (
                "tool_contact",
                ContactSensorCfg,
                ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/Tool", history_length=24),
            )
        ]
        for name, body_name in getattr(spec, "EXTRA_CONTACTS", []):
            sensors.append(
                (
                    name,
                    ContactSensorCfg,
                    ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/" + body_name, history_length=24),
                )
            )
        self.scene_config = make_configclass("LabToolScene", [("robot", ArticulationCfg, robot), *sensors])()
        self.action_config = make_configclass(
            "LabToolActions", [("machine", ToolActionCfg, ToolActionCfg(variant=variant))]
        )()
        self.observation_config = ObsCfg()
        self.event_config = make_configclass(
            "LabToolEvents",
            [
                (
                    "restore_joints",
                    EventTermCfg,
                    EventTermCfg(
                        func=mdp.reset_joints_by_offset,
                        mode="reset",
                        params={
                            "asset_cfg": SceneEntityCfg("robot"),
                            "position_range": (0.0, 0.0),
                            "velocity_range": (0.0, 0.0),
                        },
                    ),
                )
            ],
        )()
        cameras = []
        for name, eye, target, lens in spec.CAMERA_VIEWS:
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                height=720,
                width=1280,
                data_types=["rgb"],
                spawn=sim.PinholeCameraCfg(focal_length=lens, clipping_range=(0.001, 10)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=camera_quat(eye, target), convention="world"),
            )
            cameras.append((name, CameraCfg, cfg))
        self.camera_config = make_configclass("LabToolCameras", cameras, bases=(ArenaCameraCfg,))()


def make_env(device="cpu", variant="normal"):
    objects = [
        _box("floor", (3, 3, 0.1), (0, 0, -0.05), (0.25, 0.28, 0.31)),
        _box("bench", (0.9, 0.85, 0.05), (0, 0, 0.725), (0.25, 0.29, 0.33)),
        Object(
            name="fixture",
            object_type=ObjectType.BASE,
            usd_path=str(PACKAGE / "fixture.usda"),
            initial_pose=Pose(position_xyz=(0, 0, 0)),
        ),
        DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=900.0, color=(1.0, 1.0, 1.0))),
        DirectionalLight(
            initial_pose=Pose(position_xyz=(0, 0, 3), rotation_xyzw=(0, -0.382683, 0, 0.92388)),
            spawner_cfg=sim.DistantLightCfg(intensity=600.0, color=(1.0, 1.0, 1.0)),
        ),
    ]
    for name, pos, quat in spec.FREE_OBJECTS:
        objects.append(
            Object(
                name=name,
                object_type=ObjectType.RIGID,
                usd_path=str(PACKAGE / (name + ".usda")),
                initial_pose=Pose(position_xyz=pos, rotation_xyzw=quat),
                spawn_cfg_addon={
                    "physics_material": sim.RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.25),
                    "collision_props": PhysxCollisionPropertiesCfg(contact_offset=0.00003, rest_offset=0),
                    "rigid_props": PhysxRigidBodyPropertiesCfg(
                        disable_gravity=False,
                        max_depenetration_velocity=0.1,
                        solver_position_iteration_count=32,
                        solver_velocity_iteration_count=2,
                    ),
                },
            )
        )

    def configure(cfg):
        env_cfg_callback(spec.HZ)(cfg)
        cfg.sim.dt = 1 / 480
        cfg.decimation = 24
        cfg.sim.render_interval = 24
        cfg.sim.physics.solver_type = 1
        cfg.sim.physics.enable_ccd = True
        return cfg

    environment = IsaacLabArenaEnvironment(
        name=spec.SLUG, scene=Scene(objects), embodiment=ToolRobot(variant), env_cfg_callback=configure
    )
    return ArenaEnvBuilder(
        environment, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    ).make_registered()


class Sampler:
    def __init__(self, env):
        self.env = env
        self.robot = env.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINTS, preserve_order=True)[0]
        self.term = env.action_manager.get_term("machine")

    def physics(self):
        pos, rotation, quat = tool_pose(self.env)
        states = object_states(self.env, [obj[0] for obj in spec.FREE_OBJECTS]) if spec.FREE_OBJECTS else {}
        return {
            "encoders": array(self.robot.data.joint_pos)[0, self.ids].tolist(),
            "joint_velocities": array(self.robot.data.joint_vel)[0, self.ids].tolist(),
            "tool_pos": pos.tolist(),
            "tool_quat_xyzw": quat.tolist(),
            "tool_normal_contact_N": array(self.env.scene["tool_contact"].data.net_normal_forces_w)[0]
            .reshape(-1, 3)
            .sum(0)
            .tolist(),
            "parts": {
                name: {
                    "pos": value.pos.tolist(),
                    "quat": value.quat_xyzw.tolist(),
                    "lin_vel": value.lin_vel.tolist(),
                    "ang_vel": value.ang_vel.tolist(),
                }
                for name, value in states.items()
            },
            "process": self.term.process.readout(),
            "physics_device": str(self.env.device),
            "physics_dt_s": float(self.env.physics_dt),
        }

    def public(self, reading):
        return {"encoders": reading["encoders"], "process_readouts": self.term.process.instruments()}
