"""Actual Arena/PhysX roll dynamics, native spring drives and separating forces."""

import isaaclab.sim as sim
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
from isaaclab.sensors import CameraCfg
from isaaclab.utils import configclass
from isaaclab_arena.embodiments.embodiment_base import EmbodimentBase
from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
from isaaclab_arena.environments.isaaclab_arena_environment import (
    IsaacLabArenaEnvironment,
)
from isaaclab_arena.scene.scene import Scene
from isaaclab_arena.utils.cameras import ArenaCameraCfg
from isaaclab_arena.utils.configclass import make_configclass
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


class BearingAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.body_ids = self.robot.find_bodies(["Upper"])[0]
        self._raw = torch.tensor([spec.INITIAL.tolist()], device=self.device)
        self._processed = self._raw.clone()
        self.speed = torch.tensor([100.0, 100.0, 0.2], device=self.device)
        self.force = torch.zeros((env.num_envs, 1, 3), device=self.device)
        self.torque = self.force.clone()
        self.applied_forces = torch.zeros((env.num_envs, 2), device=self.device)

    @property
    def action_dim(self):
        return 3

    @property
    def raw_actions(self):
        return self._raw

    @property
    def processed_actions(self):
        return self._processed

    def process_actions(self, actions):
        self._raw[:] = actions
        change = self.speed * self._env.step_dt
        self._processed += torch.clamp(actions - self._processed, -change, change)

    def apply_actions(self):
        target = torch.stack(
            (
                (self._processed[:, 0] + self._processed[:, 1]) * 0.5e-6,
                torch.atan((self._processed[:, 1] - self._processed[:, 0]) * 1e-6 / (2 * spec.BEARING_HALF_SPAN)),
            ),
            dim=1,
        )
        if hasattr(self.robot, "set_joint_position_target_index"):
            self.robot.set_joint_position_target_index(target=target, joint_ids=self.ids)
        else:
            self.robot.set_joint_position_target(target, joint_ids=self.ids)
        load = self._processed[:, 2:3] * self.cfg.proof_scale
        self.applied_forces[:] = load * torch.tensor(spec.PROOF_FORCES.tolist(), device=self.device)
        self.force[:] = 0
        self.torque[:] = 0
        self.force[:, 0, 2] = self.applied_forces.sum(dim=1)
        self.torque[:, 0, 0] = spec.BEARING_HALF_SPAN * (self.applied_forces[:, 1] - self.applied_forces[:, 0])
        if hasattr(self.robot, "permanent_wrench_composer"):
            self.robot.permanent_wrench_composer.set_forces_and_torques_index(
                forces=self.force,
                torques=self.torque,
                body_ids=self.body_ids,
                is_global=True,
            )
        else:
            self.robot.set_external_force_and_torque(
                self.force,
                self.torque,
                body_ids=self.body_ids,
                is_global=True,
            )

    def reset(self, env_ids=None):
        self._raw[:] = torch.tensor(spec.INITIAL.tolist(), device=self.device)
        self._processed[:] = self._raw
        self.applied_forces[:] = 0


@configclass
class BearingActionCfg(ActionTermCfg):
    class_type: type = BearingAction
    asset_name: str = "robot"
    proof_scale: float = 1.0


def encoders(env):
    return tensor(env.scene["robot"].data.joint_pos)


@configclass
class PolicyObs(ObservationGroupCfg):
    mechanism: ObservationTermCfg = ObservationTermCfg(func=encoders)

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = False


@configclass
class ObsCfg:
    policy: PolicyObs = PolicyObs()


def camera_quat(eye, target):
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw

    x = np.asarray(target) - eye
    x /= np.linalg.norm(x)
    y = np.cross([0, 0, 1], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return tuple(matrix_to_quat_xyzw(np.column_stack((x, y, z))))


class CalenderMachine(EmbodimentBase):
    name = "laboratory_calender_commissioning_machine"

    def __init__(self, proof_scale=1.0):
        super().__init__(enable_cameras=True)
        initial = spec.joint_targets(spec.INITIAL)
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Calender",
            spawn=sim.UsdFileCfg(
                usd_path=str(PACKAGE / "calender.usda"),
                ensure_drives_exist=True,
                rigid_props=PhysxRigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=0.02),
                articulation_props=PhysxArticulationRootPropertiesCfg(
                    enabled_self_collisions=True,
                    solver_position_iteration_count=64,
                    solver_velocity_iteration_count=16,
                ),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.000002, rest_offset=0),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos=dict(zip(spec.JOINT_NAMES, initial.tolist(), strict=True)),
                joint_vel={".*": 0.0},
            ),
            actuators={
                "vertical_springs": ImplicitActuatorCfg(
                    joint_names_expr=["elevation"],
                    joint_effort_limit=100_000.0,
                    joint_velocity_limit=0.02,
                    stiffness=2 * spec.SPRING,
                    damping=2 * spec.SPRING_DAMPING,
                ),
                "differential_springs": ImplicitActuatorCfg(
                    joint_names_expr=["inclination"],
                    joint_effort_limit=20_000.0,
                    joint_velocity_limit=0.05,
                    stiffness=2 * spec.SPRING * spec.BEARING_HALF_SPAN**2,
                    damping=2 * spec.SPRING_DAMPING * spec.BEARING_HALF_SPAN**2,
                ),
            },
        )
        self.scene_config = make_configclass("CalenderScene", [("robot", ArticulationCfg, robot)])()
        self.action_config = make_configclass(
            "CalenderActions",
            [("bearings", BearingActionCfg, BearingActionCfg(proof_scale=proof_scale))],
        )()
        self.observation_config = ObsCfg()
        self.event_config = make_configclass(
            "CalenderEvents",
            [
                (
                    "restore_bearings",
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
            ("overview_cam", (1.10, -0.95, 1.0), (0, 0, 0.40)),
            ("nip_cam", (0.45, 0.55, 0.55), (0, 0, 0.4)),
        ]:
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                height=360,
                width=640,
                data_types=["rgb"],
                spawn=sim.PinholeCameraCfg(focal_length=20, clipping_range=(0.005, 10)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=camera_quat(eye, target), convention="world"),
            )
            cameras.append((name, CameraCfg, cfg))
        self.camera_config = make_configclass("CalenderCameras", cameras, bases=(ArenaCameraCfg,))()


def make_env(device="cuda:0", proof_scale=1.0):
    from isaaclab_arena.assets.object_library import DomeLight

    scene = Scene(
        [
            _box("floor", (3, 3, 0.1), (0, 0, -0.05), (0.2, 0.22, 0.24)),
        DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=800, color=(1.0, 1.0, 1.0))),
        ]
    )
    environment = IsaacLabArenaEnvironment(
        name="calender_loaded_nip_parallelism",
        scene=scene,
        embodiment=CalenderMachine(proof_scale),
        env_cfg_callback=env_cfg_callback(spec.HZ),
    )
    return ArenaEnvBuilder(
        environment,
        ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device),
    ).make_registered()
