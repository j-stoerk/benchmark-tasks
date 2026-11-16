import numpy as np
from stirbar_recovery.process import attraction
from stirbar_recovery.spec import Proof, geometry


def reading(pos, quat=(0, 0, 0, 1), tool=(0, 0, 0.9)):
    return {"tool_pos": list(tool), "parts": {"stirbar": {"pos": list(pos), "quat": list(quat), "lin_vel": [0, 0, 0]}}}


def test_far_magnet_cannot_lift_bar():
    force, torque, distance = attraction((0, 0, 0.90), (0, 0, 0.7555), (0, 0, 0, 1))
    assert distance > 0.03 and not force.any() and not torque.any()


def test_force_is_finite_at_close_contact():
    force, torque, _ = attraction((0.010, 0, 0.001), (0, 0, 0), (0, 0, 0, 1))
    assert np.linalg.norm(force) <= 0.40 + 1e-12 and np.isfinite(torque).all()


def test_magnetic_capture_is_not_an_attachment():
    near = attraction((0.010, 0, 0.009), (0, 0, 0), (0, 0, 0, 1))[0]
    far = attraction((0.010, 0, 0.04), (0, 0, 0), (0, 0, 0, 1))[0]
    assert np.linalg.norm(near) > 0.001 and not far.any()


def test_no_magnet_control_has_zero_force():
    assert not attraction((0.010, 0, 0.009), (0, 0, 0), (0, 0, 0, 1), False)[0].any()


def test_bar_centre_above_rim_with_end_inside_is_not_success():
    # X-axis bar rotated vertical; its lower end remains below the rim.
    q = (0, np.sqrt(0.5), 0, np.sqrt(0.5))
    r = reading((0, 0, 0.860), q, tool=(0, 0, 0.870))
    assert geometry(r)["bottom_z"] < 0.855
    proof = Proof()
    for _ in range(80):
        score, success = proof.update(r, np.zeros(7))
    assert score < 1 and not success


def test_unretained_bar_outside_flask_cannot_pass():
    r = reading((0.2, 0, 0.92), tool=(0, 0, 0.92))
    proof = Proof()
    for _ in range(80):
        score, success = proof.update(r, np.zeros(7))
    assert score == 0 and not success
