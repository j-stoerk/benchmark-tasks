"""Check real dimensions/load, finite support and adversarial retention outcomes."""

import copy
import math

import numpy as np
import pytest
from industrial_roll import spec
from industrial_roll.assets import PACKAGE


def goal():
    return {
        "roll_pos": [0, spec.TARGET_Y, spec.TARGET_Z],
        "roll_quat": [0, 0, 0, 1],
        "roll_lin_vel": [0, 0, 0],
        "roll_ang_vel": [0, 0, 0],
        "receiver_q": [-0.24, spec.TARGET_Y, spec.TARGET_Z - 1, 0.0064, 0.0064, 0.0064],
        "donor_q": -0.4,
        "donor_contact": 0,
        "jaw_forces": [2000, 2000, 2000],
    }


def test_geometry_is_full_size_and_mass_not_a_scaled_tabletop_roll():
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(PACKAGE / "roll.usda"))
    assert UsdPhysics.MassAPI(stage.GetDefaultPrim()).GetMassAttr().Get() == 200
    box = (
        UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    )
    np.testing.assert_allclose(np.array(box.GetMax()) - box.GetMin(), [0.370, 0.600, 0.600], atol=1e-7)
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == 48
    # Every convex sector remains outside its bore's separating chord.
    for p in meshes:
        points = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
        center = np.mean(points[:, 1:], axis=0)
        normal = center / np.linalg.norm(center)
        assert np.min(points[:, 1:] @ normal) >= spec.BORE * math.cos(math.pi / spec.SEGMENTS) - 1e-7


def test_receiver_has_three_physical_prismatic_jaws_and_real_strokes():
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(PACKAGE / "receiver.usda"))
    joints = [UsdPhysics.PrismaticJoint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.PrismaticJoint)]
    assert len(joints) == 6
    assert {j.GetPrim().GetName() for j in joints} == set(spec.JOINT_NAMES)
    assert sum(p.GetName().startswith("Jaw") for p in stage.Traverse()) == 3
    for joint in joints:
        drive = UsdPhysics.DriveAPI(joint.GetPrim(), "linear")
        assert drive
        if joint.GetPrim().GetName().startswith("jaw_"):
            assert drive.GetStiffnessAttr().Get() == 0  # Force-driven, not a pose lock.
        else:
            assert drive.GetStiffnessAttr().Get() > 0
        assert drive.GetMaxForceAttr().Get() > 0
    for index in (1, 2):
        follower = stage.GetPrimAtPath(f"/Machine/jaw_{index}")
        # The offline USD wheel lacks NVIDIA's schema registry; inspect the
        # authored list-op here and verify the actual constraint on the GPU.
        assert "PhysxMimicJointAPI:rotX" in follower.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
        assert str(follower.GetRelationship("physxMimicJoint:rotX:referenceJoint").GetTargets()[0]) == "/Machine/jaw_0"


def test_full_score_requires_an_entire_two_second_loaded_interval():
    p = spec.Proof()
    r = goal()
    for _ in range(39):
        score, done = p.update(r, True)
        assert score != 1 and not done
    assert p.update(r, True) == (1, True)


@pytest.mark.parametrize(
    "bad", ["donor_support", "short_insertion", "floating", "unexpanded", "one_pad_missing", "off_target"]
)
def test_held_or_unsupported_looking_end_states_cannot_pass(bad):
    r = goal()
    if bad == "donor_support":
        r["donor_q"] = 0
    elif bad == "short_insertion":
        r["receiver_q"][0] = -0.10
    elif bad == "floating":
        r["roll_pos"][2] += 0.012
    elif bad == "unexpanded":
        r["receiver_q"][3:] = [0, 0, 0]
    elif bad == "one_pad_missing":
        r["jaw_forces"][1] = 0
    elif bad == "off_target":
        r["roll_pos"][1] += 0.02
    p = spec.Proof()
    for _ in range(60):
        assert p.update(r, True)[0] != 1


def test_pose_alone_never_substitutes_for_measured_core_contact():
    r = goal()
    r["jaw_forces"] = [0, 0, 0]
    assert not spec.geometry(r)["clamped"]
    assert spec.Proof().update(r, True)[0] == 0


def test_preparation_credit_requires_measured_donor_support():
    r = goal()
    r["roll_pos"] = [0, 0, 1]
    r["receiver_q"] = [-0.24, 0, 0, 0, 0, 0]
    r["jaw_forces"] = [0, 0, 0]
    r["donor_q"] = 0
    r["donor_contact"] = 1962
    assert spec.Proof().update(r, False)[0] == 0.25
    r["donor_contact"] = 0
    assert spec.Proof().update(r, False)[0] == 0


@pytest.mark.parametrize("mode", ["axial", "radial", "rotation"])
def test_load_induced_slip_invalidates_proof_until_restarted(mode):
    r = goal()
    p = spec.Proof()
    p.update(r, True)
    if mode == "axial":
        r["roll_pos"][0] += 0.0035
    elif mode == "radial":
        r["roll_pos"][1] += 0.0035
    else:
        r["roll_quat"] = [math.sin(math.radians(4) / 2), 0, 0, math.cos(math.radians(4) / 2)]
    for _ in range(60):
        assert p.update(r, True)[0] != 1
    assert p.failed
    p.update(goal(), False)
    for _ in range(40):
        result = p.update(goal(), True)
    assert result == (1, True)


def test_no_proof_request_cannot_earn_full_score():
    p = spec.Proof()
    for _ in range(80):
        assert p.update(goal(), False)[0] == 0.5


def test_drop_is_sticky_even_if_a_pose_is_later_restored():
    p = spec.Proof()
    r = goal()
    r["roll_pos"][2] = 0.6
    assert p.update(r, False) == (0, False)
    for _ in range(60):
        assert p.update(goal(), True) == (0, False)


def test_a_new_test_after_a_pass_must_also_endure_the_full_interval():
    p = spec.Proof()
    for _ in range(40):
        p.update(goal(), True)
    p.update(goal(), False)
    assert p.update(goal(), True)[0] != 1


@pytest.mark.parametrize("field", ["roll_pos", "receiver_q", "roll_lin_vel", "roll_ang_vel", "jaw_forces", "roll_quat"])
def test_nonfinite_measurements_fail_closed(field):
    r = copy.deepcopy(goal())
    r[field][0] = float("nan")
    assert spec.Proof().update(r, True) == (0, False)
