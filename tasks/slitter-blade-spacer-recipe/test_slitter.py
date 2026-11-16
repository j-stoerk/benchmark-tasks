"""Geometry, recipe, load-path and adversarial retention contract checks."""

import copy
import math
from pathlib import Path

import numpy as np
import pytest
from slitter_stack import spec


def ready():
    return {
        "parts": {
            p["name"]: {"pos": [0, 0, p["target_z"]], "quat": [0, 0, 0, 1], "lin_vel": [0, 0, 0], "ang_vel": [0, 0, 0]}
            for p in spec.PARTS
        },
        "tool": [0, 0, 1.35, 0, 1, 0],
        "clamp_contact_N": 500,
        "clamp_z": spec.TOP + 0.01,
        "applied_proof": [100, 2],
    }


def test_full_width_and_actual_blade_recipe():
    assert sum(spec.HEIGHTS) == pytest.approx(0.330)
    assert spec.BLADE_H == 0.0007 and spec.BLADE_R == 0.050 and spec.BORE == 0.0325
    actual = [p["target_z"] - p["height"] / 2 - spec.BASE_Z for p in spec.PARTS if p["blade"]]
    np.testing.assert_allclose(actual, spec.POSITIONS)
    np.testing.assert_allclose(np.diff(actual), [0.075, 0.080, 0.075])


def test_bore_survives_convex_sector_inscription():
    minimum_radius = spec.BORE * math.cos(math.pi / spec.SEGMENTS)
    assert minimum_radius - spec.ARBOR_R > 0.000075


def test_actual_tooling_masses_are_not_enlarged_mocks():
    blades = [p for p in spec.PARTS if p["blade"]]
    assert all(0.04 < p["mass"] < 0.06 for p in blades)
    assert max(p["mass"] for p in spec.PARTS) > 1


def test_unclamped_stack_only_gets_preparation_credit():
    row = ready()
    row["clamp_contact_N"] = 0
    p = spec.Proof()
    for _ in range(100):
        assert p.update(row, True) == (0.25, False)


def test_clamped_without_proof_only_half_credit():
    row = ready()
    row["applied_proof"] = [0, 0]
    assert spec.Proof().update(row, True) == (0.5, False)


def test_complete_uninterrupted_native_retention_interval():
    p = spec.Proof()
    for _ in range(59):
        assert p.update(ready(), True) == (0.5, False)
    assert p.update(ready(), True) == (1, True)


def test_correct_total_width_with_wrong_internal_recipe_fails():
    row = ready()
    row["parts"]["part_3"]["pos"][2] += 0.005
    row["parts"]["part_4"]["pos"][2] += 0.005
    assert spec.geometry(row)["assembled"] is False
    assert spec.Proof().update(row, True) == (0, False)


def test_identical_blades_can_exchange_identity():
    row = ready()
    row["parts"]["part_1"], row["parts"]["part_7"] = row["parts"]["part_7"], row["parts"]["part_1"]
    assert spec.geometry(row)["assembled"] is True


@pytest.mark.parametrize(
    "mutation",
    [
        ("off_arbor", 0.002),
        ("hover", 0.00015),
        ("held", 0.83),
        ("platen_hover", 0.002),
        ("moving", 0.01),
        ("tilted", 0.04),
    ],
)
def test_pose_and_support_shortcuts_fail(mutation):
    row = ready()
    name, value = mutation
    if name == "off_arbor":
        row["parts"]["part_1"]["pos"][0] = value
    if name == "hover":
        row["parts"]["part_1"]["pos"][2] += value
    if name == "held":
        row["tool"][2] = value
    if name == "platen_hover":
        row["clamp_z"] += value
    if name == "moving":
        row["parts"]["part_1"]["lin_vel"][0] = value
    if name == "tilted":
        row["parts"]["part_1"]["quat"] = [math.sin(value / 2), 0, 0, math.cos(value / 2)]
    p = spec.Proof()
    for _ in range(80):
        assert p.update(row, True)[1] is False


def test_actual_translation_breaks_retention_and_requires_proof_reset():
    p = spec.Proof()
    row = ready()
    for _ in range(40):
        p.update(row, True)
    changed = copy.deepcopy(row)
    changed["parts"]["part_1"]["pos"][0] = 0.0001  # inside recipe centering, outside retention motion limit
    assert p.update(changed, True)[1] is False
    for _ in range(80):
        assert p.update(row, True)[1] is False
    p.update(row, False)
    for _ in range(60):
        result = p.update(row, True)
    assert result == (1, True)


def test_rotational_slip_breaks_retention():
    p = spec.Proof()
    row = ready()
    for _ in range(40):
        p.update(row, True)
    angle = math.radians(2)
    row["parts"]["part_1"]["quat"] = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]
    assert p.update(row, True) == (0.5, False)
    assert p.failed


@pytest.mark.parametrize("bad", [{}, {"pos": [float("nan"), 0, 1]}, {"pos": [0, 0, 0.5]}, {"quat": [0, 0, 0, 0]}])
def test_invalid_missing_ejected_and_nonfinite_states_latch_zero(bad):
    row = ready()
    if not bad:
        del row["parts"]["part_1"]
    else:
        row["parts"]["part_1"].update(bad)
    p = spec.Proof()
    assert p.update(row, True) == (0, False)
    for _ in range(80):
        assert p.update(ready(), True) == (0, False)


def test_original_native_ring_geometry_and_hollow_open_centre_head():
    from pxr import Usd, UsdPhysics

    root = Path(__file__).parent / "slitter_stack"
    for p in spec.PARTS:
        stage = Usd.Stage.Open(str(root / (p["name"] + ".usda")))
        assert stage
        assert UsdPhysics.MassAPI(stage.GetDefaultPrim()).GetMassAttr().Get() == pytest.approx(p["mass"])
        meshes = [prim for prim in stage.Traverse() if prim.GetTypeName() == "Mesh"]
        assert len(meshes) == spec.SEGMENTS
        for mesh in meshes:
            xy = np.asarray(mesh.GetAttribute("points").Get())[:, :2]
            assert np.min(np.linalg.norm(xy, axis=1)) >= spec.BORE - 1e-7
            assert mesh.HasAPI(UsdPhysics.CollisionAPI)
    machine = Usd.Stage.Open(str(root / "machine.usda"))
    assert machine.GetPrimAtPath("/Machine/anchor")
    assert not machine.GetPrimAtPath("/Machine/part_0")  # no task object is attached to the machine
    for i in range(1, 3):
        follower = machine.GetPrimAtPath(f"/Machine/jaw_{i}")
        # The offline USD wheel lacks NVIDIA's schema registry. Inspect the
        # authored schema and reference, then confirm the mechanism natively.
        assert "PhysxMimicJointAPI:rotX" in follower.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
        assert str(follower.GetRelationship("physxMimicJoint:rotX:referenceJoint").GetTargets()[0]) == "/Machine/jaw_0"
