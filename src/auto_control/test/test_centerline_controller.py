import unittest

from auto_control.centerline_controller import (
    CenterlineController,
    CenterlinePath,
    ControllerConfig,
    DEFAULT_SERVO_CENTER,
    DEFAULT_SERVO_LEFT,
    DEFAULT_SERVO_RIGHT,
    StartupDutyConfig,
    StartupDutyProfile,
    steering_to_servo,
)


class CenterlineControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = CenterlineController(ControllerConfig())

    def test_invalid_path_stops(self):
        command = self.controller.update(self._path(valid=False), 1.0 / 60.0)

        self.assertEqual(command.state, "LANE_LOST")
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.duty, 0.0)

    def test_centered_path_drives_straight(self):
        command = self.controller.update(self._path(), 1.0 / 60.0)

        self.assertEqual(command.state, "TRACKING_BOTH")
        self.assertAlmostEqual(command.steering, 0.0)
        self.assertAlmostEqual(command.duty, 0.055)

    def test_centerline_to_right_commands_right_steering(self):
        command = self.controller.update(
            self._path(coefficients=(0.0, 0.0, 100.0)),
            1.0 / 60.0,
        )

        self.assertGreater(command.steering, 0.0)
        self.assertLessEqual(command.steering, 3.0 / 60.0)

    def test_centerline_to_left_commands_left_steering(self):
        command = self.controller.update(
            self._path(coefficients=(0.0, 0.0, 60.0)),
            1.0 / 60.0,
        )

        self.assertLess(command.steering, 0.0)

    def test_large_cross_track_error_receives_stronger_gain(self):
        config = ControllerConfig(
            preview_gain=0.0,
            derivative_gain=0.0,
            steering_deadband=0.0,
            steering_filter_alpha=1.0,
            maximum_steering_rate_per_sec=1000.0,
        )
        near = CenterlineController(config).update(
            self._path(coefficients=(0.0, 0.0, 90.0)),
            1.0 / 100.0,
        )
        far = CenterlineController(config).update(
            self._path(coefficients=(0.0, 0.0, 120.0)),
            1.0 / 100.0,
        )

        near_effective_gain = near.steering / near.cross_track_error
        far_effective_gain = far.steering / far.cross_track_error
        self.assertGreater(far_effective_gain, near_effective_gain)

    def test_curved_centerline_uses_lookahead(self):
        # x(90)=80 but x(68)=47: centered at the car and curving left ahead.
        command = self.controller.update(
            self._path(coefficients=(0.0, 1.5, -55.0)),
            1.0 / 60.0,
        )

        self.assertAlmostEqual(command.cross_track_error, 0.0)
        self.assertLess(command.preview_error, 0.0)
        self.assertLess(command.steering, 0.0)

    def test_single_boundary_centerline_is_driveable(self):
        command = self.controller.update(
            self._path(mode="LEFT_ONLY", confidence=0.60),
            1.0 / 60.0,
        )

        self.assertEqual(command.state, "TRACKING_SINGLE")
        self.assertGreaterEqual(command.duty, 0.050)

    def test_servo_mapping_matches_physical_directions(self):
        self.assertLess(DEFAULT_SERVO_LEFT, DEFAULT_SERVO_CENTER)
        self.assertGreater(DEFAULT_SERVO_RIGHT, DEFAULT_SERVO_CENTER)
        self.assertAlmostEqual(steering_to_servo(-1.0), DEFAULT_SERVO_LEFT)
        self.assertAlmostEqual(steering_to_servo(0.0), DEFAULT_SERVO_CENTER)
        self.assertAlmostEqual(steering_to_servo(1.0), DEFAULT_SERVO_RIGHT)

    @staticmethod
    def _path(
        coefficients=(0.0, 0.0, 80.0),
        confidence=1.0,
        valid=True,
        mode="BOTH",
    ):
        return CenterlinePath(
            image_width=160,
            roi_y_min=60,
            roi_y_max=90,
            coefficients=coefficients,
            confidence=confidence,
            valid=valid,
            mode=mode,
        )


class StartupDutyProfileTest(unittest.TestCase):
    def test_waits_for_stable_tracking_then_boosts_and_ramps_down(self):
        profile = StartupDutyProfile(
            StartupDutyConfig(
                boost_duty=0.060,
                boost_duration_sec=0.20,
                ramp_down_sec=0.10,
                stable_tracking_sec=0.20,
                rearm_stop_sec=0.50,
            )
        )

        for _ in range(19):
            duty, state = profile.update(True, 0.050, 0.01)
            self.assertEqual(duty, 0.0)
            self.assertEqual(state, StartupDutyProfile.WAITING)

        duty, state = profile.update(True, 0.050, 0.01)
        self.assertAlmostEqual(duty, 0.060)
        self.assertEqual(state, StartupDutyProfile.BOOSTING)

        samples = []
        for _ in range(40):
            samples.append(profile.update(True, 0.050, 0.01))
        ramp_duties = [
            duty
            for duty, phase in samples
            if phase == StartupDutyProfile.RAMPING
        ]
        self.assertTrue(ramp_duties)
        self.assertGreater(max(ramp_duties), 0.050)
        self.assertLessEqual(max(ramp_duties), 0.060)
        self.assertEqual(samples[-1][1], StartupDutyProfile.RUNNING)
        self.assertAlmostEqual(samples[-1][0], 0.050)

    def test_safety_loss_stops_immediately_during_boost(self):
        profile = StartupDutyProfile(
            StartupDutyConfig(stable_tracking_sec=0.0)
        )
        duty, state = profile.update(True, 0.050, 0.01)
        self.assertEqual(state, StartupDutyProfile.BOOSTING)
        self.assertAlmostEqual(duty, 0.060)

        duty, state = profile.update(False, 0.050, 0.01)

        self.assertEqual(state, StartupDutyProfile.STOPPED)
        self.assertEqual(duty, 0.0)

    def test_short_stop_does_not_repeat_boost(self):
        profile = StartupDutyProfile(
            StartupDutyConfig(
                boost_duration_sec=0.0,
                ramp_down_sec=0.0,
                stable_tracking_sec=0.0,
                rearm_stop_sec=0.50,
            )
        )
        duty, state = profile.update(True, 0.050, 0.01)
        self.assertEqual(state, StartupDutyProfile.RUNNING)
        self.assertAlmostEqual(duty, 0.050)

        profile.update(False, 0.0, 0.20)
        duty, state = profile.update(True, 0.050, 0.01)

        self.assertEqual(state, StartupDutyProfile.RUNNING)
        self.assertAlmostEqual(duty, 0.050)

    def test_long_stop_rearms_stable_tracking_wait(self):
        profile = StartupDutyProfile(
            StartupDutyConfig(
                boost_duration_sec=0.0,
                ramp_down_sec=0.0,
                stable_tracking_sec=0.20,
                rearm_stop_sec=0.50,
            )
        )
        for _ in range(20):
            profile.update(True, 0.050, 0.01)
        self.assertEqual(profile.phase, StartupDutyProfile.RUNNING)

        for _ in range(3):
            profile.update(False, 0.0, 0.20)
        duty, state = profile.update(True, 0.050, 0.01)

        self.assertEqual(state, StartupDutyProfile.WAITING)
        self.assertEqual(duty, 0.0)


if __name__ == "__main__":
    unittest.main()
