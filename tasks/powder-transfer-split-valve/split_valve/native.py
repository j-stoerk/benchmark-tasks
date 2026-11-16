"""Native articulation, contact instruments and bounded machine actuators."""

import math

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
from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.cameras import ArenaCameraCfg
from isaaclab_arena.utils.configclass import make_configclass
from isaaclab_arena.utils.pose import Pose
from isaaclab_physx.sim.schemas import (
    PhysxArticulationRootPropertiesCfg,
    PhysxCollisionPropertiesCfg,
    PhysxRigidBodyPropertiesCfg,
)
from yam_arena.scene import _box, env_cfg_callback

from . import spec
from .assets import PACKAGE


def tensor(value):
    return getattr(value, "torch", value)


def array(value):
    return tensor(value).detach().cpu().numpy().astype(float)


class ValveAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINTS, preserve_order=True)[0]
        self._raw = torch.tensor([spec.INITIAL.tolist()], device=self.device)
        self._processed = self._raw.clone()
        self.speed = torch.tensor([0.020, 0.020, 0.025, 20, 30, 20], device=self.device)
        self.net_proof = torch.zeros(env.num_envs, device=self.device)

    @property
    def action_dim(self):
        return 6

    @property
    def raw_actions(self):
        return self._raw

    @property
    def processed_actions(self):
        return self._processed

    def process_actions(self, actions):
        self._raw[:] = actions
        limit = self.speed * self._env.step_dt
        self._processed += torch.clamp(actions - self._processed, -limit, limit)

    def apply_actions(self):
        q = tensor(self.robot.data.joint_pos)[:, self.ids]
        v = tensor(self.robot.data.joint_vel)[:, self.ids]
        target = self._processed
        position = torch.cat((target[:, :2], target[:, 3:5] * math.pi / 180), dim=1)
        ids = [self.ids[i] for i in (0, 1, 3, 4)]
        requested = target[:, 5] >= 0.5
        normal = 100000 * (target[:, 2] - q[:, 2]) - 1000 * v[:, 2] + spec.PAYLOAD * 9.81
        # During proof the vertical motor is strictly force-controlled. Native
        # bayonet contacts, rather than the guide drive, must oppose this load.
        effort = torch.where(requested, spec.PAYLOAD * 9.81 + 100, torch.clamp(normal, -1000, 1000))[:, None]
        self.net_proof[:] = torch.where(requested, 100, 0)
        if hasattr(self.robot, "set_joint_position_target_index"):
            self.robot.set_joint_position_target_index(target=position, joint_ids=ids)
            self.robot.set_joint_effort_target_index(target=effort, joint_ids=[self.ids[2]])
        else:
            self.robot.set_joint_position_target(position, joint_ids=ids)
            self.robot.set_joint_effort_target(effort, joint_ids=[self.ids[2]])

    def reset(self, env_ids=None):
        self._raw[:] = torch.tensor(spec.INITIAL.tolist(), device=self.device)
        self._processed[:] = self._raw
        self.net_proof[:] = 0


@configclass
class ValveActionCfg(ActionTermCfg):
    class_type: type = ValveAction
    asset_name: str = "robot"


def encoders(env):
    return tensor(env.scene["robot"].data.joint_pos)


@configclass
class PolicyObs(ObservationGroupCfg):
    machine: ObservationTermCfg = ObservationTermCfg(func=encoders)

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
    z = np.cross(x, y)
    return tuple(matrix_to_quat_xyzw(np.column_stack((x, y, z))))


class ValveMachine(EmbodimentBase):
    name = "split_valve_docking_machine"

    def __init__(self):
        super().__init__(enable_cameras=True)
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Machine",
            spawn=sim.UsdFileCfg(
                usd_path=str(PACKAGE / "machine.usda"),
                ensure_drives_exist=True,
                activate_contact_sensors=True,
                rigid_props=PhysxRigidBodyPropertiesCfg(
                    disable_gravity=False, max_depenetration_velocity=0.1
                ),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=64,
                    solver_velocity_iteration_count=16,
                ),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.00001, rest_offset=0),
                physics_material=sim.RigidBodyMaterialCfg(static_friction=0.30, dynamic_friction=0.20),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos={
                    "slide_x": 0.008,
                    "slide_y": -0.006,
                    "slide_z": 1.10,
                    "collar": 0,
                    "active": 0,
                    "passive": 0,
                    "seal_.*": 0,
                },
                joint_vel={".*": 0},
            ),
            actuators={
                "lateral": ImplicitActuatorCfg(
                    joint_names_expr=["slide_x", "slide_y"],
                    joint_effort_limit=1000,
                    joint_velocity_limit=0.020,
                    stiffness=1_500_000,
                    damping=10000,
                ),
                "vertical": ImplicitActuatorCfg(
                    joint_names_expr=["slide_z"],
                    joint_effort_limit=1000,
                    joint_velocity_limit=0.025,
                    stiffness=0,
                    damping=0,
                ),
                "collar": ImplicitActuatorCfg(
                    joint_names_expr=["collar"],
                    joint_effort_limit=5,
                    joint_velocity_limit=math.radians(20),
                    stiffness=120,
                    damping=2,
                ),
                "active": ImplicitActuatorCfg(
                    joint_names_expr=["active"],
                    joint_effort_limit=5,
                    joint_velocity_limit=math.radians(30),
                    stiffness=120,
                    damping=2,
                ),
                "passive_return": ImplicitActuatorCfg(
                    joint_names_expr=["passive"],
                    joint_effort_limit=0.8,
                    joint_velocity_limit=math.radians(60),
                    stiffness=0.5,
                    damping=0.1,
                ),
                "seal_springs": ImplicitActuatorCfg(
                    joint_names_expr=["seal_.*"],
                    joint_effort_limit=30,
                    joint_velocity_limit=0.1,
                    stiffness=10000,
                    damping=5,
                ),
            },
        )
        sensors = [
            (
                "retaining_contact",
                ContactSensorCfg,
                ContactSensorCfg(
                    prim_path="{ENV_REGEX_NS}/Machine/Upper",
                    filter_prim_paths_expr=["{ENV_REGEX_NS}/Machine/Collar"],
                    history_length=1,
                ),
            )
        ]
        for half in ("lower", "upper"):
            for i in range(8):
                name = f"seal_{half}_{i}"
                disc = "Active" if half == "lower" else "Passive"
                sensor = ContactSensorCfg(
                    prim_path=f"{{ENV_REGEX_NS}}/Machine/Seal_{half}_{i}",
                    filter_prim_paths_expr=["{ENV_REGEX_NS}/Machine/" + disc],
                    history_length=1,
                )
                sensors.append((name, ContactSensorCfg, sensor))
        self.scene_config = make_configclass("ValveScene", [("robot", ArticulationCfg, robot), *sensors])()
        self.action_config = make_configclass(
            "ValveActions", [("machine", ValveActionCfg, ValveActionCfg())]
        )()
        self.observation_config = ObsCfg()
        self.event_config = make_configclass(
            "ValveEvents",
            [
                (
                    "restore_joints",
                    EventTermCfg,
                    EventTermCfg(
                        func=mdp.reset_joints_by_offset,
                        mode="reset",
                        params={
                            "asset_cfg": SceneEntityCfg("robot"),
                            "position_range": (0, 0),
                            "velocity_range": (0, 0),
                        },
                    ),
                )
            ],
        )()
        cameras = []
        for name, eye, target in [
            ("overview_cam", (0.8, -1.0, 1.55), (0, 0, 0.95)),
            ("valve_cam", (0.26, -0.32, 1.17), (0, 0, 1.015)),
        ]:
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                height=720,
                width=1280,
                data_types=["rgb"],
                spawn=sim.PinholeCameraCfg(focal_length=20, clipping_range=(0.005, 10)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=camera_quat(eye, target), convention="world"),
            )
            cameras.append((name, CameraCfg, cfg))
        self.camera_config = make_configclass("ValveCameras", cameras, bases=(ArenaCameraCfg,))()


def make_env(device="cuda:0", variant="normal"):
    if variant not in spec.VARIANTS:
        raise ValueError("Unknown control variant")
    objects = [
        _box("floor", (2, 2, 0.10), (0, 0, -0.05), (0.22, 0.24, 0.26)),
        DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=800, color=(1.0, 1.0, 1.0))),
        DirectionalLight(
            initial_pose=Pose(position_xyz=(0, 0, 3), rotation_xyzw=(0, -0.382683, 0, 0.92388)),
            spawner_cfg=sim.DistantLightCfg(intensity=500, color=(1.0, 1.0, 1.0)),
        ),
    ]

    def configure(cfg):
        env_cfg_callback(spec.HZ)(cfg)
        cfg.sim.dt = 1 / 240
        cfg.decimation = 12
        cfg.sim.render_interval = 12
        return cfg

    environment = IsaacLabArenaEnvironment(
        name="split_valve_docking",
        scene=Scene(objects),
        embodiment=ValveMachine(),
        env_cfg_callback=configure,
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
        self.seal_ids = self.robot.find_joints(["seal_lower_.*", "seal_upper_.*"], preserve_order=True)[0]

    def normal(self, name):
        return float(np.linalg.norm(array(self.env.scene[name].data.force_matrix_w)[0]))

    def physics(self):
        q = array(self.robot.data.joint_pos)[0, self.ids]
        v = array(self.robot.data.joint_vel)[0, self.ids]
        q[3:] *= 180 / math.pi
        v[3:] *= 180 / math.pi
        return {
            "encoders": q.tolist(),
            "velocities": v.tolist(),
            "seal_deflections_m": array(self.robot.data.joint_pos)[0, self.seal_ids].tolist(),
            "seal_normals": [
                self.normal(f"seal_{half}_{i}") for half in ("lower", "upper") for i in range(8)
            ],
            "retaining_contact_N": self.normal("retaining_contact"),
            "net_proof_N": float(array(self.term.net_proof)[0]),
        }

    def public(self, reading):
        return {
            "encoders": reading["encoders"],
            "seal_normals": reading["seal_normals"],
            "retaining_contact": [reading["retaining_contact_N"]],
            "net_proof": [reading["net_proof_N"]],
        }
