import unittest

from auto_control.automatic_brake_profile import (
    AutomaticBrakeProfile,
    AutomaticBrakeProfileConfig,
)


def make_profile() -> AutomaticBrakeProfile:
    return AutomaticBrakeProfile(
        AutomaticBrakeProfileConfig(
            entry_speed_error_mps=0.10,
            exit_speed_error_mps=0.03,
            minimum_vehicle_speed_mps=0.20,
            minimum_brake_current_amps=1.0,
            maximum_brake_current_amps=4.0,
            current_gain_amps_per_mps=8.0,
            rise_amps_per_sec=8.0,
            fall_amps_per_sec=16.0,
        )
    )


class AutomaticBrakeProfileTest(unittest.TestCase):
    def test_enters_only_after_speed_error_threshold(self) -> None:
        profile = make_profile()

        self.assertEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=0.69,
                dt_sec=0.1,
            ),
            0.0,
        )
        self.assertAlmostEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=0.70,
                dt_sec=0.1,
            ),
            0.8,
        )
        self.assertTrue(profile.braking_requested)

    def test_hysteresis_prevents_motor_mode_chatter(self) -> None:
        profile = make_profile()
        profile.update(
            target_speed_mps=0.60,
            current_speed_mps=0.80,
            dt_sec=1.0,
        )

        command = profile.update(
            target_speed_mps=0.60,
            current_speed_mps=0.65,
            dt_sec=0.01,
        )
        self.assertTrue(profile.braking_requested)
        self.assertGreater(command, 0.0)

        profile.update(
            target_speed_mps=0.60,
            current_speed_mps=0.62,
            dt_sec=0.01,
        )
        self.assertFalse(profile.braking_requested)

    def test_current_is_gain_clamped_and_ramp_limited(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=1.60,
                dt_sec=0.25,
            ),
            2.0,
        )
        self.assertAlmostEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=1.60,
                dt_sec=0.25,
            ),
            4.0,
        )

    def test_release_ramp_reaches_zero(self) -> None:
        profile = make_profile()
        profile.update(
            target_speed_mps=0.60,
            current_speed_mps=1.10,
            dt_sec=1.0,
        )

        self.assertAlmostEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=0.62,
                dt_sec=0.10,
            ),
            2.4,
        )
        self.assertEqual(
            profile.update(
                target_speed_mps=0.60,
                current_speed_mps=0.62,
                dt_sec=0.20,
            ),
            0.0,
        )

    def test_low_speed_and_reset_release_brake(self) -> None:
        profile = make_profile()
        profile.update(
            target_speed_mps=0.0,
            current_speed_mps=1.0,
            dt_sec=1.0,
        )
        command = profile.update(
            target_speed_mps=0.0,
            current_speed_mps=0.20,
            dt_sec=1.0,
        )
        self.assertEqual(command, 0.0)
        self.assertFalse(profile.braking_requested)

        profile.update(
            target_speed_mps=0.0,
            current_speed_mps=1.0,
            dt_sec=1.0,
        )
        profile.reset()
        self.assertEqual(profile.command_current_amps, 0.0)
        self.assertFalse(profile.braking_requested)

    def test_rejects_invalid_hysteresis(self) -> None:
        with self.assertRaises(ValueError):
            AutomaticBrakeProfileConfig(
                entry_speed_error_mps=0.10,
                exit_speed_error_mps=0.10,
                minimum_vehicle_speed_mps=0.20,
                minimum_brake_current_amps=1.0,
                maximum_brake_current_amps=4.0,
                current_gain_amps_per_mps=8.0,
                rise_amps_per_sec=8.0,
                fall_amps_per_sec=16.0,
            )


if __name__ == "__main__":
    unittest.main()
