"""Control progression must use measured stops rather than biased setpoints."""

import unittest
from types import SimpleNamespace

from split_valve.control import Demo


class Feedback(unittest.TestCase):
    def test_full_open_feedback_can_advance_at_the_retaining_stop(self):
        policy = Demo()
        policy.reset(None)
        policy.phase, policy.entered = "open", 10.0
        observation = SimpleNamespace(
            state={"encoders": [0, 0, 1.0001, 30, 89.9, 89.9], "seal_normals": [0] * 16}, state_time=20.0
        )
        policy.act(observation)
        self.assertEqual(policy.phase, "close")

    def test_closed_disc_without_seat_contact_does_not_unlock(self):
        policy = Demo()
        policy.reset(None)
        policy.phase, policy.entered = "close", 10.0
        for time in (11.0, 13.0):
            observation = SimpleNamespace(
                state={"encoders": [0, 0, 1.0001, 30, 0, 0], "seal_normals": [0] + [2] * 15}, state_time=time
            )
            policy.act(observation)
        self.assertEqual(policy.phase, "close")


if __name__ == "__main__":
    unittest.main()
