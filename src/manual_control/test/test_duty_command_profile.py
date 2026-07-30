from __future__ import annotations

import unittest

from manual_control.duty_command_profile import (
    DutyCommandProfile,
    DutyProfileConfig,
    Gear,
)


def make_profile() -> DutyCommandProfile:
    return DutyCommandProfile(
        DutyProfileConfig(
            forward_max_duty=0.07,
            reverse_max_duty=0.04,
            start_duty=0.03,
            acceleration_duty_per_sec=0.01,
            coast_deceleration_duty_per_sec=0.02,
            brake_duty_per_sec=0.07,
            pedal_deadzone=0.03,
        )
    )


class DutyCommandProfileTest(unittest.TestCase):
    def test_starts_stopped_in_forward_gear(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.gear, Gear.FORWARD)
        self.assertEqual(profile.command_duty, 0.0)

    def test_accelerator_starts_at_start_duty_then_uses_ramp_rate(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(1.0, 0.0, 1.0), 0.03)
        self.assertAlmostEqual(profile.update(1.0, 0.0, 1.0), 0.04)

    def test_accelerator_is_proportional_and_clamped_to_forward_limit(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(0.5, 0.0, 1.0), 0.03)
        self.assertAlmostEqual(profile.update(1.0, 0.0, 100.0), 0.07)

    def test_releasing_both_pedals_coasts_toward_zero(self) -> None:
        profile = make_profile()
        profile.update(1.0, 0.0, 1.0)

        self.assertAlmostEqual(profile.update(0.0, 0.0, 0.5), 0.02)
        self.assertEqual(profile.update(0.0, 0.0, 5.0), 0.0)

    def test_brake_has_priority_and_decreases_duty(self) -> None:
        profile = make_profile()
        profile.update(1.0, 0.0, 2.0)
        profile.update(1.0, 0.0, 1.0)

        self.assertAlmostEqual(profile.update(1.0, 0.5, 0.5), 0.0225)
        self.assertEqual(profile.update(0.0, 1.0, 1.0), 0.0)

    def test_y_toggle_changes_to_reverse_only_while_stopped(self) -> None:
        profile = make_profile()

        self.assertTrue(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)
        profile.update(1.0, 0.0, 1.0)
        self.assertFalse(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)

    def test_reverse_accelerator_is_clamped_to_reverse_limit(self) -> None:
        profile = make_profile()
        profile.toggle_gear()

        self.assertAlmostEqual(profile.update(1.0, 0.0, 1.0), -0.03)
        self.assertAlmostEqual(profile.update(1.0, 0.0, 100.0), -0.04)

    def test_brake_moves_reverse_duty_toward_zero(self) -> None:
        profile = make_profile()
        profile.toggle_gear()
        profile.update(1.0, 0.0, 2.0)
        profile.update(1.0, 0.0, 1.0)

        self.assertAlmostEqual(profile.update(0.0, 0.5, 0.5), -0.0225)
        self.assertEqual(profile.update(0.0, 1.0, 1.0), 0.0)

    def test_coasting_moves_reverse_duty_toward_zero(self) -> None:
        profile = make_profile()
        profile.toggle_gear()
        profile.update(1.0, 0.0, 1.0)

        self.assertAlmostEqual(profile.update(0.0, 0.0, 0.5), -0.02)

    def test_pedal_deadzone_ignores_small_input(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.update(0.029, 0.0, 1.0), 0)


if __name__ == "__main__":
    unittest.main()
