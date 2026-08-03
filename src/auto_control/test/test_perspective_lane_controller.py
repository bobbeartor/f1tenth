import unittest

from auto_control.perspective_lane_controller import (
    ControllerConfig,
    LaneObservation,
    PerspectiveLaneController,
)


class PerspectiveLaneControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = PerspectiveLaneController(ControllerConfig())

    def test_invalid_observation_stops(self):
        command = self.controller.update(LaneObservation(640), 0.1)

        self.assertEqual(command.state, "LANE_LOST")
        self.assertEqual(command.steering, 0.0)
        self.assertEqual(command.duty, 0.0)

    def test_centered_straight_lane_drives_forward(self):
        observation = LaneObservation(640, 320.0, 320.0, 1.0, True)
        command = self.controller.update(observation, 0.1)

        self.assertEqual(command.state, "TRACKING")
        self.assertAlmostEqual(command.steering, 0.0)
        self.assertAlmostEqual(command.duty, 0.055)

    def test_lane_to_right_commands_right_steering(self):
        observation = LaneObservation(640, 380.0, 380.0, 1.0, True)
        command = self.controller.update(observation, 0.1)

        self.assertGreater(command.steering, 0.0)

    def test_perspective_convergence_does_not_reverse_lateral_correction(self):
        # The near centre is right of the car while the far centre converges
        # to the optical centre on a straight road. Steering must remain right.
        observation = LaneObservation(640, 380.0, 320.0, 1.0, True)
        command = self.controller.update(observation, 0.1)

        self.assertGreater(command.steering, 0.0)

    def test_left_curve_commands_left_steering(self):
        observation = LaneObservation(640, 320.0, 260.0, 1.0, True)
        command = self.controller.update(observation, 0.1)

        self.assertLess(command.steering, 0.0)

    def test_curve_reduces_duty(self):
        straight = LaneObservation(640, 320.0, 320.0, 1.0, True)
        curve = LaneObservation(640, 420.0, 500.0, 1.0, True)
        straight_command = self.controller.update(straight, 0.1)
        curve_command = self.controller.update(curve, 0.1)

        self.assertLess(curve_command.duty, straight_command.duty)

    def test_valid_tracking_duty_never_drops_below_start_duty(self):
        sharp_curve = LaneObservation(640, 500.0, 560.0, 1.0, True)
        command = self.controller.update(sharp_curve, 0.1)

        self.assertGreaterEqual(command.duty, 0.050)

    def test_confidence_below_threshold_stops(self):
        observation = LaneObservation(640, 320.0, 320.0, 0.44, True)
        command = self.controller.update(observation, 0.1)

        self.assertEqual(command.state, "LANE_LOST")
        self.assertEqual(command.duty, 0.0)


if __name__ == "__main__":
    unittest.main()
