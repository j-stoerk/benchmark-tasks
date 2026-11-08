"""Check the hollow collision geometry, finite spindle fit and handling-history gate."""

import json
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from battery_roll.assets import PACKAGE, asset_texts, roll_segments
from battery_roll.embodiment import BatteryRollEmbodiment
from battery_roll.release import tools_clear
from battery_roll.scripted import RollScriptedPolicy
from inspect_robots.types import Action, ActionChunk
from yam_arena import ik, station
from yam_arena.demo import ArmPlan
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.scripted import YamScriptedPolicy
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module


def test_demo_feedback_uses_joints_bounds_compensation_and_preserves_gripper(monkeypatch):
    nominal = np.column_stack((np.tile((ik.LOWER + ik.UPPER) / 2, (2, 1)), [0.4, 0.4])).ravel()
    original = ActionChunk(actions=[Action(data=nominal, meta={"request_stop": True})], control_hz=20)
    monkeypatch.setattr(YamScriptedPolicy, "act", lambda self, observation: original)
    policy = RollScriptedPolicy()
    measured = nominal.reshape(2, 7).copy()
    measured[:, :6] -= 0.04
    observation = SimpleNamespace(state={"joint_pos": measured.ravel()})
    for _ in range(100):
        result = policy.act(observation)
    action = result.actions[0]
    np.testing.assert_allclose(action.data.reshape(2, 7)[:, :6], nominal.reshape(2, 7)[:, :6] + 0.15)
    np.testing.assert_allclose(action.data[[6, 13]], [0.4, 0.4])
    assert action.meta == original.actions[0].meta
    monkeypatch.setattr(YamScriptedPolicy, "reset", lambda self, scene: None)
    policy.reset(None)
    assert not policy._integral.any()


def test_demo_feedback_rejects_nonfinite_joint_measurements(monkeypatch):
    monkeypatch.setattr(
        YamScriptedPolicy, "act", lambda self, observation: ActionChunk(actions=[Action(data=np.zeros(14))])
    )
    with pytest.raises(ValueError, match="finite measured joints"):
        RollScriptedPolicy().act(SimpleNamespace(state={"joint_pos": np.full(14, np.nan)}))


def state(pos, yaw=0, roll=0, speed=0):
    x, z = math.sin(roll / 2), math.sin(yaw / 2)
    c, w = math.cos(roll / 2), math.cos(yaw / 2)
    return ObjectState(
        np.array(pos, float),
        np.array([w * x, z * x, z * c, w * c]),
        np.array([speed, 0, 0]),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
    )


def goal():
    return {"roll": state((*M.TARGET, M.SHAFT_Z - M.SAG)), "spindle": state((*M.TARGET, M.SHAFT_Z))}


def shift(obj, axis, delta):
    pos = obj.pos.copy()
    pos[axis] += delta
    return replace(obj, pos=pos)


def test_initial_objects_are_not_threaded_and_score_zero():
    objects = {
        "roll": state((*M.ROLL_XY, M.TOP + M.OUTER_R)),
        "spindle": state((*M.SHAFT_XY, M.TOP + M.SHAFT_HALF_HEIGHT)),
    }
    assert not M.threaded(objects) and M.score(objects) == 0


def test_goal_has_actual_sag_and_support_below_both_spindle_ends():
    objects = goal()
    assert M.threaded(objects) and M.on_rests(objects) and M.suspended(objects)
    assert M.success(objects) and M.score(objects) == 1
    assert objects["roll"].pos[2] - M.OUTER_R > M.TOP + 0.04


def test_threaded_source_and_transferred_floor_state_receive_only_partial_credit():
    objects = {
        "roll": state((*M.ROLL_XY, M.TOP + M.OUTER_R)),
        "spindle": state((*M.ROLL_XY, M.TOP + M.OUTER_R - M.SAG)),
    }
    assert M.score(objects) == 0.25
    objects = {name: shift(obj, 0, M.TARGET[0] - M.ROLL_XY[0]) for name, obj in objects.items()}
    objects = {name: shift(obj, 1, -M.ROLL_XY[1]) for name, obj in objects.items()}
    assert M.score(objects) == 0.5


@pytest.mark.parametrize("name", ["roll", "spindle"])
@pytest.mark.parametrize("yaw,roll", [(math.pi, 0), (0, math.pi)])
def test_axis_sign_reversals_are_equivalent(name, yaw, roll):
    objects = goal()
    objects[name] = state(objects[name].pos, yaw, roll)
    assert M.score(objects) == 1


@pytest.mark.parametrize("axis,delta", [(0, 0.025), (1, 0.100), (2, 0.030)])
def test_wrong_or_incomplete_bore_insertion_is_zero(axis, delta):
    objects = goal()
    objects["spindle"] = shift(objects["spindle"], axis, delta)
    assert not M.threaded(objects) and M.score(objects) == 0


def test_crosswise_spindle_cannot_pass_through_bore():
    objects = goal()
    objects["spindle"] = state(objects["spindle"].pos, yaw=math.pi / 2)
    assert M.score(objects) == 0


def test_a_grip_collar_cannot_intersect_the_bore_wall_even_when_the_tube_fits():
    objects = goal()
    objects["spindle"] = shift(objects["spindle"], 1, 0.065)
    assert not M.threaded(objects)
    # Raising the roll to the tube axis leaves room for the same collar.
    objects["roll"] = state((*M.TARGET, M.SHAFT_Z))
    assert M.threaded(objects)


def test_centered_floating_roll_does_not_count_as_suspended():
    objects = goal()
    objects["roll"] = state(objects["spindle"].pos)
    assert M.threaded(objects) and not M.suspended(objects) and M.score(objects) == 0.5


def test_a_spindle_across_the_rests_without_a_roll_scores_zero():
    objects = goal()
    objects["roll"] = state((*M.ROLL_XY, M.TOP + M.OUTER_R))
    assert M.score(objects) == 0


@pytest.mark.parametrize("bad", ["moving", "dropped", "nan", "bad_quaternion"])
def test_invalid_roll_states_are_zero(bad):
    objects = goal()
    obj = objects["roll"]
    if bad == "moving":
        obj = state(obj.pos, speed=0.02)
    elif bad == "dropped":
        obj = state((obj.pos[0], obj.pos[1], M.TOP - 0.1))
    elif bad == "nan":
        obj = state((float("nan"), 0, M.TOP + 0.1))
    else:
        obj = replace(obj, quat_xyzw=np.zeros(4))
    objects["roll"] = obj
    assert M.score(objects) == 0


def test_tcp_protection_and_release_are_separate():
    objects = goal()
    handles = np.array([[M.TARGET[0], 0.1, M.SHAFT_Z], [M.TARGET[0], -0.1, M.SHAFT_Z]])
    assert not M.handling_violation(objects, handles)
    assert not tools_clear(objects, handles, M.SIZES)
    handles[0] = objects["roll"].pos
    assert M.handling_violation(objects, handles)


def test_embodiment_latches_invalid_handling_and_reset_clears_history(monkeypatch):
    embodiment = BatteryRollEmbodiment()
    objects = goal()
    far = np.array([[0.1, 0.25, M.TOP + 0.15], [0.1, -0.25, M.TOP + 0.15]])
    embodiment._tools = far
    assert embodiment.task.score(objects) == 1
    embodiment._handling_violation = True
    assert embodiment.task.score(objects) == 0 and not embodiment.task.success(objects)
    monkeypatch.setattr(YamArenaEmbodiment, "reset", lambda self, scene, seed=None: "reset")
    assert embodiment.reset(SimpleNamespace(), seed=0) == "reset"
    assert not embodiment._handling_violation and np.isnan(embodiment._tools).all()
    assert embodiment.task.score(objects) == 0


def test_actual_step_readout_latches_a_violation_after_the_tools_retreat(monkeypatch):
    class Tensor:
        def __init__(self, values):
            self.values = np.asarray(values)

        def __getitem__(self, item):
            return Tensor(self.values[item])

        def detach(self):
            return self

        def cpu(self):
            return self

        def numpy(self):
            return self.values

    @dataclass
    class Result:
        info: dict

    class Scene(dict):
        env_origins = Tensor([[1, 2, 3]])

    def robot(tcp):
        return SimpleNamespace(
            body_names=["gripper"],
            data=SimpleNamespace(
                body_pos_w=Tensor([[np.asarray(tcp) + [1, 2, 3] - [0, 0, 0.134]]]), body_quat_w=Tensor([[[0, 0, 0, 1]]])
            ),
        )

    objects = goal()
    emb = BatteryRollEmbodiment()
    scene = Scene(left_robot=robot(objects["roll"].pos), right_robot=robot([0.1, -0.25, M.TOP + 0.15]))
    emb._base, emb._env = SimpleNamespace(scene=scene), object()
    monkeypatch.setitem(sys.modules, "yam_arena.scene", SimpleNamespace(object_states=lambda env, names: objects))
    monkeypatch.setattr(YamArenaEmbodiment, "_task_step", lambda self: Result({"score": self.task.score(objects)}))
    result = emb._task_step()
    assert result.info["score"] == 0.0 and result.info["handling_violation"]
    np.testing.assert_allclose(result.info["physics_readout"]["roll"]["pos"], objects["roll"].pos)
    scene["left_robot"] = robot([0.1, 0.25, M.TOP + 0.15])
    result = emb._task_step()
    assert tools_clear(objects, emb._tools, M.SIZES)
    assert result.info["score"] == 0.0 and result.info["handling_violation"]
    assert result.info["physics_readout"]["released"]


def test_usd_files_parse_have_compound_colliders_and_keep_the_core_empty():
    from pxr import Usd, UsdGeom, UsdPhysics

    texts = asset_texts()
    for filename, text in texts.items():
        assert (PACKAGE / filename).read_text(encoding="utf-8") == text
        stage = Usd.Stage.Open(str(PACKAGE / filename))
        assert stage and stage.GetDefaultPrim().HasAPI(UsdPhysics.RigidBodyAPI)
    stage = Usd.Stage.Open(str(PACKAGE / "roll.usda"))
    meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == M.SEGMENTS
    for mesh, vertices in zip(meshes, roll_segments()):
        assert mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.MeshCollisionAPI(mesh).GetApproximationAttr().Get() == "convexHull"
        np.testing.assert_allclose(mesh.GetPointsAttr().Get(), vertices, atol=1e-8)
        # Each convex sector has a separating inner chord outside the bore.
        center_angle = math.atan2(np.mean(np.array(vertices)[:, 2]), np.mean(np.array(vertices)[:, 0]))
        normal = np.array([math.cos(center_angle), math.sin(center_angle)])
        assert np.min(np.asarray(vertices)[:, [0, 2]] @ normal) > M.INNER_R - 0.001
    bbox = (
        UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    )
    np.testing.assert_allclose(np.array(bbox.GetMax()) - bbox.GetMin(), M.SIZES["roll"], atol=1e-6)
    stage = Usd.Stage.Open(str(PACKAGE / "spindle.usda"))
    collars = [p for p in stage.Traverse() if p.IsA(UsdGeom.Cube)]
    assert len(collars) == 2 and all(p.HasAPI(UsdPhysics.CollisionAPI) for p in collars)
    bbox = (
        UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    )
    np.testing.assert_allclose(np.array(bbox.GetMax()) - bbox.GetMin(), M.SIZES["spindle"], atol=1e-6)


def test_instruction_matches_metadata_and_both_candidate_arms_reach_20_hz_targets():
    spec = json.loads(Path(__file__).with_name("task.json").read_text(encoding="utf-8"))
    assert spec["instruction"] == M.INSTRUCTION
    plans = {side: ArmPlan(points, station.BASES[side]) for side, points in M.demo().items()}
    assert abs(plans["left"].duration - plans["right"].duration) < 1e-9
    for side, arm in plans.items():
        assert arm.duration + TASK.success_hold_s < 120
        actions = np.array([arm.action7(k / 20)[0] for k in range(round(arm.duration * 20) + 1)])
        assert arm.worst_ik_mm < 0.5, (side, arm.worst_ik_mm)
        assert arm.worst_ik_deg < 0.1, (side, arm.worst_ik_deg)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
