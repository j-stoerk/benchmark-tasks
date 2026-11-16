"""Offline contract/geometry/adversarial checks; not substitutes for native trials."""

import math
from pathlib import Path

import numpy as np
import pytest
from calender_nip import spec


def reading(gaps=None, force=None, q=None, velocity=None):
    return {
        "joint_positions": [0.0002, 0] if q is None else q,
        "joint_velocities": [0, 0] if velocity is None else velocity,
        "gaps_um": [200] * 5 if gaps is None else gaps,
        "applied_proof_forces_N": spec.PROOF_FORCES.tolist() if force is None else force,
    }


def test_true_mass_and_scale():
    assert spec.ROLLER_MASS == pytest.approx(73.9845, abs=0.001)
    assert (spec.RADIUS, spec.FACE) == (0.1, 0.3)


def test_parallel_cylinder_gap():
    np.testing.assert_allclose(spec.gaps(0.0002, 0), [200] * 5)


def test_tilt_and_surface_radius_are_measured():
    angle = 0.001
    actual = spec.gaps(0.0002, angle)
    assert actual[0] < actual[2] < actual[-1]
    assert actual[2] < 200  # true surface, not just axis spacing
    assert actual[-1] - actual[0] == pytest.approx(280.00009, abs=1e-4)


def test_independent_bearing_targets():
    q = spec.joint_targets([100, 300])
    assert q[0] == pytest.approx(0.0002)
    assert math.tan(q[1]) * 0.35 == pytest.approx(0.0002)


def test_proof_changes_both_height_and_tilt():
    unloaded = spec.equilibrium([200, 200], [0, 0])
    loaded = spec.equilibrium([200, 200])
    assert loaded[0] - unloaded[0] == pytest.approx(0.0005)
    assert loaded[1] - unloaded[1] == pytest.approx(0.00057142857)
    assert max(spec.gaps(*loaded)) > 700


def test_unloaded_cannot_get_full_credit():
    proof = spec.Proof()
    for _ in range(200):
        score, success = proof.update(reading(force=[0, 0]), True)
    assert (score, success, proof.elapsed) == (0.5, False, 0)


def test_center_only_partial():
    assert spec.Proof().update(reading(gaps=[100, 150, 200, 250, 300], force=[0, 0]), False) == (0.25, False)


def test_continuous_loaded_hold():
    proof = spec.Proof()
    for _ in range(59):
        assert proof.update(reading(), True) == (0, False)
    assert proof.update(reading(), True) == (1, True)


@pytest.mark.parametrize(
    "bad",
    [
        reading(gaps=[200, 200, 200, 200, 211]),
        reading(force=[8000, 0]),
        reading(force=[0, 12000]),
        reading(force=[10000, 10000]),
        reading(velocity=[0.001, 0]),
        reading(velocity=[0, 0.01]),
    ],
)
def test_bad_hold_does_not_accumulate(bad):
    proof = spec.Proof()
    for _ in range(40):
        proof.update(reading(), True)
    assert proof.elapsed == pytest.approx(2)
    assert proof.update(bad, True)[1] is False
    assert proof.elapsed == 0
    for _ in range(40):
        assert proof.update(reading(), True)[1] is False


def test_turning_proof_off_clears_even_passed_interval():
    proof = spec.Proof()
    for _ in range(60):
        proof.update(reading(), True)
    assert proof.update(reading(), False) == (0, False)
    assert proof.elapsed == 0


@pytest.mark.parametrize(
    "bad",
    [
        reading(gaps=[0, 200, 200, 200, 200]),
        reading(gaps=[-5] * 5),
        reading(q=[0.004, 0]),
        reading(q=[0.0002, 0.01]),
        reading(force=[8000, float("nan")]),
        reading(q=[float("inf"), 0]),
        reading(gaps=[200] * 4),
        {},
    ],
)
def test_invalid_state_latches_zero_until_reset(bad):
    proof = spec.Proof()
    assert proof.update(bad, True) == (0, False)
    for _ in range(80):
        assert proof.update(reading(), True) == (0, False)
    assert spec.Proof().invalid is False


def test_native_assets_have_finite_spring_drives_and_real_scale():
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(Path(__file__).parent / "calender_nip/calender.usda"))
    assert stage
    for name in ["base/lower_roll", "Upper/roll"]:
        roll = UsdGeom.Cylinder.Get(stage, "/Calender/" + name)
        assert roll.GetRadiusAttr().Get() == pytest.approx(0.1)
        assert roll.GetHeightAttr().Get() == pytest.approx(0.3)
        assert roll.GetAxisAttr().Get() == "Y"
        assert roll.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    expected = [
        ("elevation", "linear", 2 * spec.SPRING),
        ("inclination", "angular", 2 * spec.SPRING * spec.BEARING_HALF_SPAN**2),
    ]
    for name, axis, stiffness in expected:
        drive = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/Calender/" + name), axis)
        assert drive.GetStiffnessAttr().Get() == pytest.approx(stiffness)
        assert drive.GetMaxForceAttr().Get() > 20_000
    assert stage.GetPrimAtPath("/Calender/anchor")
