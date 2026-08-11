import unittest

from auto_control.centerline_controller import (
    CenterlineController,
    CenterlinePath,
    ControllerConfig,
    DEFAULT_SERVO_CENTER,
    DEFAULT_SERVO_LEFT,
    DEFAULT_SERVO_RIGHT,
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

    def test_curved_centerline_uses_lookahead(self):
        # x(83)=80 but x(68)=50: centered at the car and curving left ahead.
        command = self.controller.update(
            self._path(coefficients=(0.0, 2.0, -86.0)),
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
            roi_y_max=83,
            coefficients=coefficients,
            confidence=confidence,
            valid=valid,
            mode=mode,
        )


if __name__ == "__main__":
    unittest.main()
