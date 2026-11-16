"""Arena/PhysX assembly: all tooling is free, with contact-only chuck handling."""

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
    PhysxSDFMeshPropertiesCfg,
)
from yam_arena.scene import Object, ObjectType, _box, env_cfg_callback

from . import spec
from .assets import PACKAGE


def tensor(value):
    return getattr(value, "torch", value)


class AssemblyAction(ActionTerm):
    def __init__(self, cfg, env):
        super().__init__(cfg, env)
        self.robot = env.scene["robot"]
        self.ids = self.robot.find_joints(spec.JOINT_NAMES, preserve_order=True)[0]
        self.press_ids = self.robot.find_joints(["press"])[0]
        self.press_y_ids = self.robot.find_joints(["press_y"])[0]
        self._raw = torch.tensor([spec.INITIAL.tolist()], device=self.device)
        self._processed = self._raw.clone()
        self.speed = torch.tensor([0.40, 0.40, 0.40, 20, 20, 20], device=self.device)
        self.force = torch.zeros((env.num_envs, 1, 3), device=self.device)
        self.torque = self.force.clone()
        self.applied_proof = torch.zeros((env.num_envs, 2), device=self.device)
        self.lateral_target = torch.full((env.num_envs, 1), -0.45, device=self.device)

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
        change = self.speed * self._env.step_dt
        self._processed += torch.clamp(actions - self._processed, -change, change)

    def apply_actions(self):
        q = self._processed
        xyz = q[:, :3]
        jaws = (1 - 2 * q[:, 3:4]).repeat(1, 3) * self.cfg.grip_force
        press_y = tensor(self.robot.data.joint_pos)[:, self.press_y_ids]
        press_z = tensor(self.robot.data.joint_pos)[:, self.press_ids]
        requested = q[:, 4:5] >= 0.5
        aligned = torch.abs(press_y) < 0.00002
        press = torch.where(requested & aligned, -500.0, 1000.0)
        # PLC sequencing uses actual joint feedback: retract before parking,
        # and align the crosshead before applying its downward force.
        park = torch.full_like(press_y, -0.45)
        raised = press_z > 0.0495
        desired_lateral = torch.where(requested, torch.zeros_like(press_y), park)
        self.lateral_target[:] = torch.where(raised, desired_lateral, self.lateral_target)
        lateral = self.lateral_target
        if hasattr(self.robot, "set_joint_position_target_index"):
            self.robot.set_joint_position_target_index(target=xyz, joint_ids=self.ids[:3])
            self.robot.set_joint_effort_target_index(target=jaws, joint_ids=self.ids[3:])
            self.robot.set_joint_effort_target_index(target=press, joint_ids=self.press_ids)
            self.robot.set_joint_position_target_index(target=lateral, joint_ids=self.press_y_ids)
        else:
            self.robot.set_joint_position_target(xyz, joint_ids=self.ids[:3])
            self.robot.set_joint_effort_target(jaws, joint_ids=self.ids[3:])
            self.robot.set_joint_effort_target(press, joint_ids=self.press_ids)
            self.robot.set_joint_position_target(lateral, joint_ids=self.press_y_ids)
        active = (q[:, 5] >= 0.5).float()
        self.applied_proof[:, 0] = 100 * active
        self.applied_proof[:, 1] = 2 * active
        self.force[:] = 0
        self.torque[:] = 0
        self.force[:, 0, 2] = 25 * active
        self.torque[:, 0, 2] = 0.5 * active
        for part in spec.PARTS:
            if not part["blade"]:
                continue
            asset = self._env.scene[part["name"]]
            if hasattr(asset, "permanent_wrench_composer"):
                asset.permanent_wrench_composer.set_forces_and_torques_index(
                    forces=self.force,
                    torques=self.torque,
                    is_global=True,
                )
            else:
                asset.set_external_force_and_torque(self.force, self.torque, is_global=True)

    def reset(self, env_ids=None):
        self._raw[:] = torch.tensor(spec.INITIAL.tolist(), device=self.device)
        self._processed[:] = self._raw
        self.applied_proof[:] = 0
        self.lateral_target[:] = -0.45


@configclass
class AssemblyActionCfg(ActionTermCfg):
    class_type: type = AssemblyAction
    asset_name: str = "robot"
    grip_force: float = 50.0


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
    import numpy as np
    from yam_arena.ik import matrix_to_quat_xyzw

    x = np.asarray(target) - eye
    x /= np.linalg.norm(x)
    y = np.cross([0, 0, 1], x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    return tuple(matrix_to_quat_xyzw(np.column_stack((x, y, z))))


class AssemblyMachine(EmbodimentBase):
    name = "slitter_cassette_assembly_machine"

    def __init__(self, grip_force=50):
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
                    solver_velocity_iteration_count=1,
                ),
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.00002, rest_offset=0),
                physics_material=sim.RigidBodyMaterialCfg(static_friction=0.65, dynamic_friction=0.5),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos={
                    "slide_x": -0.15,
                    "slide_y": -0.4,
                    "slide_z": 1.35,
                    "jaw_.*": 0,
                    "press": 0.05,
                    "press_y": -0.45,
                },
                joint_vel={".*": 0.0},
            ),
            actuators={
                "axes": ImplicitActuatorCfg(
                    joint_names_expr=["slide_.*"],
                    joint_effort_limit=1000.0,
                    joint_velocity_limit=0.40,
                    stiffness=3_000_000.0,
                    damping=15_000.0,
                ),
                "jaws": ImplicitActuatorCfg(
                    joint_names_expr=["jaw_.*"],
                    joint_effort_limit=50.0,
                    joint_velocity_limit=0.02,
                    stiffness=0.0,
                    damping=30.0,
                ),
                "press": ImplicitActuatorCfg(
                    joint_names_expr=["press"],
                    joint_effort_limit=1000.0,
                    joint_velocity_limit=0.02,
                    stiffness=0.0,
                    damping=800.0,
                ),
                "press_carriage": ImplicitActuatorCfg(
                    joint_names_expr=["press_y"],
                    joint_effort_limit=1000.0,
                    joint_velocity_limit=0.10,
                    stiffness=3_000_000.0,
                    damping=15_000.0,
                ),
            },
        )
        sensors = [
            (
                "platen_contact",
                ContactSensorCfg,
                ContactSensorCfg(
                    prim_path="{ENV_REGEX_NS}/Machine/Platen",
                    filter_prim_paths_expr=["{ENV_REGEX_NS}/part_8"],
                    history_length=1,
                ),
            )
        ]
        for i in range(3):
            sensors.append(
                (
                    f"pad_{i}",
                    ContactSensorCfg,
                    ContactSensorCfg(prim_path=f"{{ENV_REGEX_NS}}/Machine/Jaw{i}", history_length=1),
                )
            )
        self.scene_config = make_configclass("AssemblyScene", [("robot", ArticulationCfg, robot), *sensors])()
        self.action_config = make_configclass(
            "AssemblyActions", [("machine", AssemblyActionCfg, AssemblyActionCfg(grip_force=grip_force))]
        )()
        self.observation_config = ObsCfg()
        self.event_config = make_configclass(
            "AssemblyEvents",
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
        for name, eye, target in [
            ("overview_cam", (1.3, -1.5, 1.9), (0.20, 0, 0.85)),
            ("arbor_cam", (0.35, -0.45, 1.30), (0, 0, 1.0)),
        ]:
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                height=720,
                width=1280,
                data_types=["rgb"],
                spawn=sim.PinholeCameraCfg(focal_length=18, clipping_range=(0.005, 10)),
                offset=CameraCfg.OffsetCfg(pos=eye, rot=camera_quat(eye, target), convention="world"),
            )
            cameras.append((name, CameraCfg, cfg))
        self.camera_config = make_configclass("AssemblyCameras", cameras, bases=(ArenaCameraCfg,))()


def make_env(device="cuda:0", grip_force=50, contact_bench=False):
    objects = [
        _box("floor", (3, 3, 0.1), (0, 0, -0.05), (0.22, 0.24, 0.26)),
        _box("bench", (1.10, 1.1, 0.06), (0.25, 0, 0.72), (0.22, 0.25, 0.28)),
        _box("shoulder", (0.14, 0.14, 0.05), (0, 0, 0.775), (0.35, 0.38, 0.42)),
        Object(
            name="arbor",
            object_type=ObjectType.BASE,
            initial_pose=Pose(position_xyz=(0, 0, 1.0)),
            spawner_cfg=sim.CylinderCfg(
                radius=spec.ARBOR_R,
                height=0.4,
                collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.00002, rest_offset=0),
                visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.48, 0.5, 0.53), roughness=0.35),
            ),
        ),
        DomeLight(spawner_cfg=sim.DomeLightCfg(intensity=800, color=(1.0, 1.0, 1.0))),
        DirectionalLight(
            initial_pose=Pose(position_xyz=(0, 0, 3), rotation_xyzw=(0, -0.382683, 0, 0.92388)),
            spawner_cfg=sim.DistantLightCfg(intensity=500, color=(1.0, 1.0, 1.0)),
        ),
    ]
    for i, x in enumerate((-0.23, 0.73)):
        for j, y in enumerate((-0.48, 0.48)):
            objects.append(_box(f"leg_{i}_{j}", (0.05, 0.05, 0.69), (x, y, 0.345), (0.16, 0.24, 0.33)))
    for part in spec.PARTS:
        x, y, z = part["source"]
        objects.append(
            Object(
                name="pedestal_" + part["name"],
                object_type=ObjectType.BASE,
                initial_pose=Pose(position_xyz=(x, y, 0.775)),
                spawner_cfg=sim.CylinderCfg(
                    radius=0.038,
                    height=0.05,
                    collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.00002, rest_offset=0),
                    visual_material=sim.PreviewSurfaceCfg(diffuse_color=(0.18, 0.28, 0.38)),
                ),
            )
        )
        objects.append(
            Object(
                name=part["name"],
                usd_path=str(PACKAGE / (part["name"] + ".usda")),
                object_type=ObjectType.RIGID,
                initial_pose=Pose(position_xyz=(0, 0, part["target_z"]) if contact_bench else (x, y, z)),
                spawn_cfg_addon={
                    "physics_material": sim.RigidBodyMaterialCfg(static_friction=0.65, dynamic_friction=0.5),
                    "collision_props": PhysxCollisionPropertiesCfg(
                        contact_offset=0.00002,
                        rest_offset=0,
                        mesh_collision_property=PhysxSDFMeshPropertiesCfg(
                            sdf_resolution=512,
                            sdf_subgrid_resolution=6,
                            sdf_narrow_band_thickness=0.01,
                            sdf_margin=0.01,
                        ),
                    ),
                    "rigid_props": PhysxRigidBodyPropertiesCfg(
                        disable_gravity=False,
                        max_depenetration_velocity=0.1,
                        solver_position_iteration_count=64,
                        solver_velocity_iteration_count=1,
                    ),
                },
            )
        )

    def configure(cfg):
        env_cfg_callback(spec.HZ)(cfg)
        # GPU PhysX does not support CCD. Use bounded joint speeds and smaller
        # discrete steps for the real 2 mm lower knives.
        cfg.sim.dt = 1 / 480
        cfg.decimation = 24
        cfg.sim.render_interval = 24
        cfg.sim.physics.solve_articulation_contact_last = True
        return cfg

    environment = IsaacLabArenaEnvironment(
        name="slitter_blade_spacer_recipe",
        scene=Scene(objects),
        embodiment=AssemblyMachine(grip_force),
        env_cfg_callback=configure,
    )
    return ArenaEnvBuilder(
        environment, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)
    ).make_registered()
