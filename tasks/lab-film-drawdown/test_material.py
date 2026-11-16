"""Adversarial material/scorer checks; native demos remain independently required."""

import numpy as np
from film_drawdown.process import Paint
from film_drawdown.spec import SURFACE_Z, Proof


def stroke(force=5.0, height=SURFACE_Z, enabled=True):
    paint = Paint()
    for y in np.linspace(-0.115, 0.110, 901):
        paint.update(np.array([0, y, height]), np.eye(3), force, 0.005, enabled)
    return paint


def test_contact_stroke_conserves_finite_paint():
    p = stroke()
    assert p.grid.all()
    assert abs(p.reservoir_ml - 1.4) < 1e-9


def test_airborne_stroke_cannot_paint():
    assert not stroke(height=SURFACE_Z + 0.001).grid.any()


def test_geometric_proximity_without_force_cannot_paint():
    assert not stroke(force=0.0).grid.any()


def test_disabled_material_never_gets_credit():
    assert not stroke(enabled=False).grid.any()


def test_pretty_coverage_without_lifting_tool_is_not_full_score():
    proof = Proof()
    reading = {"process": stroke().readout(), "tool_pos": [0, 0.11, SURFACE_Z]}
    for _ in range(100):
        score, success = proof.update(reading, np.zeros(7))
    assert score == 0.5 and not success


def test_visible_dry_stripe_blocks_full_score():
    p = stroke()
    p.grid[:, 3:5] = False
    reading = {"process": p.readout(), "tool_pos": [0, 0.11, 0.9]}
    proof = Proof()
    for _ in range(100):
        score, success = proof.update(reading, np.zeros(7))
    assert p.readout()["largest_dry_stripe_mm"] == 10 and not success
