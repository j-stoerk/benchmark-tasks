"""Actual Isaac Lab-Arena machine. Import only after AppLauncher starts Kit."""

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
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.utils import configclass
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
from yam_arena.scene import Object, ObjectType, _box, env_cfg_callback

from . import spec
from .assets import PACKAGE


def tensor(value):
    return getattr(value, "torch", value)


class MachineAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.receiver = env.scene["robot"]
        self.donor = env.scene["donor"]
        self.roll = env.scene["roll"]
        self.ids = self.receiver.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.donor_ids = self.donor.find_joints(["withdraw"])[0]
        self._raw = torch.zeros((env.num_envs, 8), device=self.device)
        self._processed = self._raw.clone()
        self._processed[:, :6] = tensor(self.receiver.data.default_joint_pos)[:, self.ids]
        self.speed = torch.tensor([0.04, 0.06, 0.025, 0.004, 0.004, 0.004, 0.06, 100], device=self.device)
        self.force = torch.zeros((env.num_envs, 1, 3), device=self.device)
        self.torque = self.force.clone()

    @property
    def action_dim(self):
        return 8

    @property
    def raw_actions(self):
        return self._raw

    @property
    def processed_actions(self):
        return self._processed

    def process_actions(self, actions):
        self._raw[:] = actions
        step = self.speed * self._env.step_dt
        self._processed += torch.clamp(actions - self._processed, -step, step)

    def apply_actions(self):
        for machine, target, ids in (
            (self.receiver, self._processed[:, :6], self.ids),
            (self.donor, self._processed[:, 6:7], self.donor_ids),
        ):
            if hasattr(machine, "set_joint_position_target_index"):
                machine.set_joint_position_target_index(target=target, joint_ids=ids)
            else:
                machine.set_joint_position_target(target, joint_ids=ids)
        effort = (self._processed[:, 3:6] / spec.EXPAND_TARGET * 2 - 1) * 4000.0
        if hasattr(self.receiver, "set_joint_effort_target_index"):
            self.receiver.set_joint_effort_target_index(target=effort, joint_ids=self.ids[3:])
        else:
            self.receiver.set_joint_effort_target(effort, joint_ids=self.ids[3:])
        active = (self._processed[:, 7] >= 0.5).float()
        self.force[:] = 0
        self.torque[:] = 0
        self.force[:, 0, 0] = active * spec.PROOF_PULL
        self.torque[:, 0, 0] = active * spec.PROOF_TORQUE
        if hasattr(self.roll, "permanent_wrench_composer"):
            self.roll.permanent_wrench_composer.set_forces_and_torques_index(
                forces=self.force, torques=self.torque, is_global=True
            )
        else:
            self.roll.set_external_force_and_torque(self.force, self.torque, is_global=True)

    def reset(self, env_ids=None):
        self._raw[:] = 0
        self._processed[:] = 0
        self._processed[:, :6] = tensor(self.receiver.data.default_joint_pos)[:, self.ids]


@configclass
class MachineActionCfg(ActionTermCfg):
    class_type: type = MachineAction
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


def articulation(filename, path, position, initial, lift_force=6000):
    receiver = filename.startswith("receiver")
    actuators = {
        "axes": ImplicitActuatorCfg(
            joint_names_expr=["slide_[xyz]"] if receiver else ["withdraw"],
            joint_effort_limit={"slide_x": 10000, "slide_y": 10000, "slide_z": lift_force} if receiver else 6000,
            joint_velocity_limit=0.08,
            stiffness=1_500_000.0,
            damping=30000.0,
        )
    }
    if receiver:
        actuators["collet"] = ImplicitActuatorCfg(
            joint_names_expr=["jaw_.*"],
            joint_effort_limit=4000.0,
            joint_velocity_limit=0.004,
            stiffness=0.0,
            damping=800.0,
        )
    return ArticulationCfg(
        prim_path=path,
        spawn=sim.UsdFileCfg(
            usd_path=str(PACKAGE / filename),
            ensure_drives_exist=True,
            activate_contact_sensors=True,
            rigid_props=PhysxRigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=0.2),
            articulation_props=PhysxArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=16
            ),
            collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.0002, rest_offset=0),
            physics_material=sim.RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.30),
        ),
        init_state=ArticulationCfg.InitialStateCfg(pos=position, joint_pos=initial, joint_vel={".*": 0.0}),
        actuators=actuators,
    )


def camera_quat(eye, target):
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw

    x = np.asarray(target) - eye
    x = x / np.linalg.norm(x)
    y = np.cross([0, 0, 1], x)
    y = y / np.linalg.norm(y)
    z = np.cross(x, y)
    return tuple(matrix_to_quat_xyzw(np.column_stack((x, y, z))))


class TransferMachine(EmbodimentBase):
    name = "industrial_roll_transfer_machine"

    def __init__(self, lift_force=6000):
        super().__init__(enable_cameras=True)
        self.scene_config = make_configclass(
            "MachineScene",
            [
                (
                    "robot",
                    ArticulationCfg,
                    articulation(
                        "receiver.usda",
                        "{ENV_REGEX_NS}/Receiver",
                        (0.66, 0, 0.6),
                        {"slide_x": 0, "slide_y": 0.012, "slide_z": -0.015, "jaw_.*": 0},
                        lift_force,
                    ),
                ),
                (
                    "donor",
                    ArticulationCfg,
                    articulation("donor.usda", "{ENV_REGEX_NS}/Donor", (-0.6, 0, 0.6), {"withdraw": 0}),
                ),
                (
                    "donor_contact",
                    ContactSensorCfg,
                    ContactSensorCfg(
                        prim_path="{ENV_REGEX_NS}/Donor/Shaft",
                        history_length=1,
                    ),
                ),
            ]
            + [
                (
                    f"pad_{i}",
                    ContactSensorCfg,
                    ContactSensorCfg(
                        prim_path=f"{{ENV_REGEX_NS}}/Receiver/Jaw{i}",
                        history_length=1,
                    ),
                )
                for i in range(3)
            ],
        )()
        self.action_config = make_configclass("MachineActions", [("machine", MachineActionCfg, MachineActionCfg())])()
        self.observation_config = ObsCfg()
        self.event_config = make_configclass(
            "MachineEvents",
            [
                (
                    "restore_" + name,
                    EventTermCfg,
                    EventTermCfg(
                        func=mdp.reset_joints_by_offset,
                        mode="reset",
                        params={
                            "asset_cfg": SceneEntityCfg(name),
                            "position_range": (0.0, 0.0),
                            "velocity_range": (0.0, 0.0),
                        },
                    ),
                )
                for name in ("robot", "donor")
            ],
        )()
        cams = []
        for name, eye, target in [
            ("overview_cam", (1.45, -1.65, 1.9), (0, 0.12, 0.85)),
            ("core_cam", (0.95, -0.42, 1.35), (0.05, 0, 1.0)),
        ]:
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                height=360,
                width=640,
                data_types=["rgb"],
                spawn=sim.PinholeCameraCfg(focal_length=18, clipping_range=(0.02, 20)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=camera_quat(eye, target), convention="world"),
            )
            cams.append((name, CameraCfg, cfg))
        self.camera_config = make_configclass("MachineCameras", cams, bases=(ArenaCameraCfg,))()


def make_env(device="cuda:0", lift_force=6000):
    material = sim.RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.30)
    roll = Object(
        name="roll",
        usd_path=str(PACKAGE / "roll.usda"),
        object_type=ObjectType.RIGID,
        initial_pose=Pose(position_xyz=(0, 0, 1)),
        spawn_cfg_addon={
            "physics_material": material,
            "collision_props": PhysxCollisionPropertiesCfg(contact_offset=0.0002, rest_offset=0),
            "rigid_props": PhysxRigidBodyPropertiesCfg(
                disable_gravity=False, solver_position_iteration_count=64, solver_velocity_iteration_count=16
            ),
        },
    )
    from isaaclab_arena.assets.object_library import DomeLight

    scene = Scene(
        [
            _box("floor", (4, 4, 0.1), (0, 0, -0.05), (0.20, 0.22, 0.24)),
            _box("destination", (0.42, 0.02, 0.004), (0, spec.TARGET_Y, 0.003), (0.08, 0.75, 0.25)),
            DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=3000, color=(1.0, 1.0, 1.0))),
            roll,
        ]
    )
    environment = IsaacLabArenaEnvironment(
        name="industrial_roll_handoff",
        scene=scene,
        embodiment=TransferMachine(lift_force),
        env_cfg_callback=env_cfg_callback(spec.HZ),
    )
    return ArenaEnvBuilder(
        environment, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    ).make_registered()
