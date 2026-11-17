from __future__ import annotations

import unittest

from manual_control.brake_current_profile import (
    BrakeCurrentProfile,
    BrakeCurrentProfileConfig,
)


def make_profile() -> BrakeCurrentProfile:
    return BrakeCurrentProfile(
        BrakeCurrentProfileConfig(
            maximum_brake_current_amps=8.0,
            rise_amps_per_sec=16.0,
            fall_amps_per_sec=32.0,
            pedal_deadzone=0.01,
        )
    )


class BrakeCurrentProfileTest(unittest.TestCase):
    def test_full_lt_reaches_eight_amps_in_half_a_second(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(1.0, 0.25), 4.0)
        self.assertAlmostEqual(profile.update(1.0, 0.25), 8.0)

    def test_lt_travel_sets_a_proportional_target(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(0.5, 1.0), 4.0)

    def test_release_ramps_current_to_zero(self) -> None:
        profile = make_profile()
        profile.update(1.0, 0.5)

        self.assertAlmostEqual(profile.update(0.0, 0.125), 4.0)
        self.assertAlmostEqual(profile.update(0.0, 0.125), 0.0)

    def test_deadzone_and_reset_release_brake(self) -> None:
        profile = make_profile()
        self.assertEqual(profile.update(0.009, 1.0), 0.0)
        profile.update(1.0, 0.25)

        profile.reset()

        self.assertEqual(profile.command_current_amps, 0.0)


if __name__ == "__main__":
    unittest.main()
