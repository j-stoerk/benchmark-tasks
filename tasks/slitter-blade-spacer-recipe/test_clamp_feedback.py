"""An impact spike cannot start the retention proof on an unseated stack."""

from types import SimpleNamespace

from slitter_stack.policy import Demo


def observation(time, heights, force=400):
    return SimpleNamespace(
        state_time=time,
        state={
            "machine_encoders": [0.6, 0.4, 1.35, 0, 1, 0],
            "clamp_force": [force],
            "blade_heights_mm": heights,
        },
    )


def test_wrong_recipe_or_transient_force_does_not_start_proof():
    policy = Demo(variant="contact-bench")
    policy.reset(None)
    policy.phase, policy.entered = "clamp", 0
    policy.act(observation(2, [50, 125, 205, 280]))
    policy.act(observation(2.5, [50, 125.2, 205, 280]))
    policy.act(observation(3, [50, 125, 205, 280]))
    policy.act(observation(3.5, [50, 125, 205, 280]))
    assert policy.phase == "clamp"
    policy.act(observation(4.1, [50, 125, 205, 280]))
    assert policy.phase == "proof"
