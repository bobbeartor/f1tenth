from __future__ import annotations

import unittest

from manual_control.duty_command_profile import (
    DutyCommandProfile,
    DutyProfileConfig,
    Gear,
    GearRequestResult,
)


def make_profile(
    immediate_stop_on_accelerator_release: bool = True,
) -> DutyCommandProfile:
    return DutyCommandProfile(
        DutyProfileConfig(
            forward_max_duty=0.10,
            reverse_max_duty=0.08,
            start_duty=0.06,
            reverse_start_duty=0.05,
            acceleration_duty_per_sec=0.03,
            coast_deceleration_duty_per_sec=0.02,
            pedal_deadzone=0.03,
            immediate_stop_on_accelerator_release=(
                immediate_stop_on_accelerator_release
            ),
        )
    )


class DutyCommandProfileTest(unittest.TestCase):
    def test_starts_stopped_in_forward_gear(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.gear, Gear.FORWARD)
        self.assertEqual(profile.command_duty, 0.0)

    def test_accelerator_starts_at_start_duty_then_uses_ramp_rate(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(1.0, 1.0), 0.06)
        self.assertAlmostEqual(profile.update(1.0, 1.0), 0.09)

    def test_accelerator_is_proportional_and_clamped_to_forward_limit(self) -> None:
        profile = make_profile()

        self.assertAlmostEqual(profile.update(0.5, 1.0), 0.06)
        self.assertAlmostEqual(profile.update(1.0, 100.0), 0.10)

    def test_releasing_accelerator_stops_duty_immediately(self) -> None:
        profile = make_profile()
        profile.update(1.0, 1.0)

        self.assertEqual(profile.update(0.0, 0.5), 0.0)

    def test_long_held_accelerator_is_not_replayed_after_release(self) -> None:
        profile = make_profile()
        for _ in range(500):
            profile.update(1.0, 0.0125)

        self.assertEqual(profile.command_duty, 0.10)
        self.assertEqual(profile.update(0.0, 0.0125), 0.0)

    def test_gear_toggle_changes_to_reverse_only_while_stopped(self) -> None:
        profile = make_profile()

        self.assertTrue(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)
        profile.update(1.0, 1.0)
        self.assertFalse(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)

    def test_gear_toggle_is_available_immediately_after_pedal_release(self) -> None:
        profile = make_profile()
        profile.update(1.0, 1.0)

        self.assertFalse(profile.toggle_gear())
        self.assertEqual(profile.update(0.0, 0.0125), 0.0)
        self.assertEqual(profile.gear, Gear.FORWARD)
        self.assertTrue(profile.toggle_gear())
        self.assertEqual(profile.gear, Gear.REVERSE)

    def test_reverse_accelerator_is_clamped_to_reverse_limit(self) -> None:
        profile = make_profile()
        profile.toggle_gear()

        self.assertAlmostEqual(profile.update(1.0, 1.0), -0.05)
        self.assertAlmostEqual(profile.update(1.0, 100.0), -0.08)

    def test_optional_coasting_moves_reverse_duty_toward_zero(self) -> None:
        profile = make_profile(immediate_stop_on_accelerator_release=False)
        profile.toggle_gear()
        profile.update(1.0, 1.0)

        self.assertAlmostEqual(profile.update(0.0, 0.5), -0.04)

    def test_gear_request_while_stopped_changes_immediately(self) -> None:
        profile = make_profile()

        self.assertEqual(
            profile.request_gear_toggle(), GearRequestResult.CHANGED
        )
        self.assertEqual(profile.gear, Gear.REVERSE)
        self.assertIsNone(profile.pending_gear)

    def test_gear_request_while_moving_is_reserved_until_duty_zero(
        self,
    ) -> None:
        profile = make_profile(immediate_stop_on_accelerator_release=False)
        profile.update(1.0, 1.0)  # +0.06

        self.assertEqual(
            profile.request_gear_toggle(), GearRequestResult.PENDING
        )
        self.assertEqual(profile.gear, Gear.FORWARD)
        self.assertEqual(profile.pending_gear, Gear.REVERSE)

        # 0.02/s 감속: 1초 뒤 0.04, 아직 전진 유지
        self.assertAlmostEqual(profile.update(0.0, 1.0), 0.04)
        self.assertEqual(profile.gear, Gear.FORWARD)

        # duty가 0이 되는 주기에 예약이 적용된다
        self.assertEqual(profile.update(0.0, 10.0), 0.0)
        self.assertEqual(profile.gear, Gear.REVERSE)
        self.assertIsNone(profile.pending_gear)
        self.assertAlmostEqual(profile.update(1.0, 0.0125), -0.05)

    def test_reserved_gear_does_not_flip_sign_while_accelerator_held(
        self,
    ) -> None:
        profile = make_profile()
        profile.update(1.0, 1.0)
        profile.request_gear_toggle()

        self.assertGreater(profile.update(1.0, 0.0125), 0.0)
        self.assertEqual(profile.gear, Gear.FORWARD)

    def test_second_request_cancels_reservation(self) -> None:
        profile = make_profile()
        profile.update(1.0, 1.0)
        profile.request_gear_toggle()

        self.assertEqual(
            profile.request_gear_toggle(), GearRequestResult.CANCELLED
        )
        self.assertEqual(profile.update(0.0, 0.0125), 0.0)
        self.assertEqual(profile.gear, Gear.FORWARD)

    def test_request_is_reserved_when_immediate_change_not_allowed(
        self,
    ) -> None:
        profile = make_profile()

        self.assertEqual(
            profile.request_gear_toggle(apply_now_allowed=False),
            GearRequestResult.PENDING,
        )
        self.assertEqual(profile.gear, Gear.FORWARD)
        profile.reset_speed()  # 제동 경로는 예약을 지우지 않는다
        self.assertEqual(profile.pending_gear, Gear.REVERSE)
        profile.update(0.0, 0.0125)
        self.assertEqual(profile.gear, Gear.REVERSE)

    def test_clear_gear_request_drops_reservation(self) -> None:
        profile = make_profile()
        profile.update(1.0, 1.0)
        profile.request_gear_toggle()
        profile.clear_gear_request()

        profile.update(0.0, 0.0125)
        self.assertEqual(profile.gear, Gear.FORWARD)

    def test_pedal_deadzone_ignores_small_input(self) -> None:
        profile = make_profile()

        self.assertEqual(profile.update(0.029, 1.0), 0)


if __name__ == "__main__":
    unittest.main()
