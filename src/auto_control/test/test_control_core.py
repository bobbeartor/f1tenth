import math
import unittest

import numpy as np

from auto_control.control_core import (
    SpeedPid,
    build_path_model,
    centerline_points_from_mono8,
    curvature_target_speed,
    erpm_to_speed_mps,
    representative_curvature,
    speed_feedforward_duty,
    stanley_control,
    steering_angle_to_servo,
)


class ControlCoreTest(unittest.TestCase):
    def test_centerline_image_uses_bev_vehicle_coordinates(self) -> None:
        image = np.zeros((300, 120), dtype=np.uint8)
        image[150:260, 59:61] = 255

        x_m, y_m = centerline_points_from_mono8(
            image,
            x_max_m=3.0,
            y_max_m=0.60,
            meter_per_pixel=0.01,
            threshold=128,
        )

        self.assertAlmostEqual(float(x_m[0]), 0.405)
        self.assertAlmostEqual(float(x_m[-1]), 1.495)
        self.assertTrue(np.allclose(y_m, 0.0))

    def test_centerline_image_rejects_stale_lateral_margin_width(self) -> None:
        image = np.zeros((300, 260), dtype=np.uint8)

        with self.assertRaisesRegex(ValueError, "expected 120x300"):
            centerline_points_from_mono8(
                image,
                x_max_m=3.0,
                y_max_m=0.60,
                meter_per_pixel=0.01,
                threshold=128,
            )

    def test_straight_centerline_steers_toward_its_image_side(self) -> None:
        for center_column, expected_sign in ((49, 1.0), (69, -1.0)):
            with self.subTest(center_column=center_column):
                image = np.zeros((300, 120), dtype=np.uint8)
                end_column = center_column + 2
                image[80:280, center_column:end_column] = 255
                x_m, y_m = centerline_points_from_mono8(
                    image,
                    x_max_m=3.0,
                    y_max_m=0.60,
                    meter_per_pixel=0.01,
                    threshold=128,
                )
                path = build_path_model(
                    x_m,
                    y_m,
                    minimum_points=20,
                    minimum_span_m=0.35,
                    minimum_x_m=0.20,
                    maximum_x_m=2.2,
                    local_smoothing_window_m=0.08,
                    outlier_threshold_m=0.04,
                    geometry_window_m=0.14,
                )
                self.assertIsNotNone(path)

                result = stanley_control(
                    path,
                    speed_mps=0.8,
                    gain=1.5,
                    softening_speed_mps=0.35,
                    heading_lookahead_m=0.22,
                    maximum_steering_angle_rad=math.radians(30.0),
                    corner_heading_threshold_rad=math.radians(6.0),
                    corner_opposing_correction_ratio=0.80,
                )

                self.assertGreater(
                    expected_sign * result.cross_track_error_m, 0.0
                )
                self.assertGreater(expected_sign * result.steering_angle_rad, 0.0)

    def test_direct_path_and_stanley_steer_toward_left_path(self) -> None:
        x_m = np.linspace(0.3, 2.0, 80)
        y_m = 0.10 + 0.08 * np.square(x_m)
        path = build_path_model(
            x_m,
            y_m,
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.20,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=2.0,
            gain=1.2,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.10,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )

        self.assertGreater(result.cross_track_error_m, 0.0)
        self.assertGreater(result.heading_error_rad, 0.0)
        self.assertGreater(result.steering_angle_rad, 0.0)
        servo = steering_angle_to_servo(
            result.steering_angle_rad,
            maximum_steering_angle_rad=math.radians(30.0),
            servo_left=0.98,
            servo_center=0.46,
            servo_right=0.02,
        )
        self.assertGreater(servo, 0.46)

    def test_corner_guard_prevents_cross_track_from_reversing_turn(self) -> None:
        # The path is currently left of the axle but its heading is clearly a
        # right turn. At low speed the unbounded Stanley cross-track term is
        # large enough to request left steering, opposite the path heading.
        x_m = np.linspace(0.2, 1.5, 100)
        path = build_path_model(
            x_m,
            0.60 - 0.40 * x_m,
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.20,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)
        result = stanley_control(
            path,
            speed_mps=0.8,
            gain=1.2,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.10,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )

        self.assertGreater(result.cross_track_error_m, 0.0)
        self.assertLess(result.heading_error_rad, 0.0)
        self.assertTrue(result.direction_guard_used)
        self.assertLess(result.steering_angle_rad, 0.0)
        servo = steering_angle_to_servo(
            result.steering_angle_rad,
            maximum_steering_angle_rad=math.radians(30.0),
            servo_left=0.98,
            servo_center=0.46,
            servo_right=0.02,
        )
        self.assertLess(servo, 0.46)

    def test_curvature_profile_is_bounded_by_requested_speeds(self) -> None:
        x_m = np.linspace(0.2, 2.2, 160)
        path = build_path_model(
            x_m,
            0.50 * np.square(x_m),
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.20,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)
        curvature = representative_curvature(
            path,
            lookahead_minimum_x_m=0.5,
            lookahead_maximum_x_m=1.6,
            percentile=90.0,
        )
        target_speed = curvature_target_speed(
            curvature,
            maximum_lateral_acceleration_mps2=0.8,
            minimum_speed_mps=0.8,
            maximum_speed_mps=1.2,
        )
        self.assertGreaterEqual(target_speed, 0.8)
        self.assertLessEqual(target_speed, 1.2)
        self.assertLess(target_speed, 1.2)

    def test_far_corner_does_not_bend_current_heading(self) -> None:
        x_m = np.linspace(0.2, 2.2, 201)
        y_m = np.zeros_like(x_m)
        corner = x_m > 1.2
        y_m[corner] = -0.45 * np.square(x_m[corner] - 1.2)
        path = build_path_model(
            x_m,
            y_m,
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.20,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=0.0,
            gain=1.2,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.10,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )

        self.assertAlmostEqual(result.cross_track_error_m, 0.0, places=4)
        self.assertAlmostEqual(result.heading_error_rad, 0.0, places=4)
        self.assertAlmostEqual(result.steering_angle_rad, 0.0, places=4)

    def test_heading_lookahead_is_measured_from_front_axle_origin(self) -> None:
        # The path passes closest to the axle at X=0.40m. Its heading is right
        # at the requested front-axle lookahead X=0.20m, but left at the old
        # closest-point-plus-lookahead location X=0.60m.
        x_m = np.linspace(0.10, 0.90, 81)
        path = build_path_model(
            x_m,
            np.square(x_m - 0.40),
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.05,
            maximum_x_m=1.0,
            local_smoothing_window_m=0.0,
            outlier_threshold_m=0.0,
            geometry_window_m=0.08,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=1.0,
            gain=0.0,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.20,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )

        self.assertLess(result.heading_error_rad, 0.0)

    def test_front_axle_lookahead_uses_first_measured_tangent_in_blind_zone(
        self,
    ) -> None:
        # X=0.10m is requested from the axle, but the camera path starts at
        # X=0.30m. The controller must use the first available local tangent
        # rather than adding another 0.10m to the blind-zone distance.
        x_m = np.linspace(0.30, 1.00, 71)
        path = build_path_model(
            x_m,
            0.25 * np.square(x_m),
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.05,
            maximum_x_m=1.0,
            local_smoothing_window_m=0.0,
            outlier_threshold_m=0.0,
            geometry_window_m=0.08,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=1.0,
            gain=0.0,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.10,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )
        expected_heading = math.atan(float(path.first_derivative(0.30)))

        self.assertAlmostEqual(
            result.heading_error_rad,
            expected_heading,
            places=9,
        )

    def test_isolated_centerline_bump_is_rejected(self) -> None:
        x_m = np.linspace(0.2, 1.2, 101)
        y_m = np.zeros_like(x_m)
        y_m[10] = 0.20
        path = build_path_model(
            x_m,
            y_m,
            minimum_points=20,
            minimum_span_m=0.35,
            minimum_x_m=0.20,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)
        self.assertLess(float(np.max(np.abs(path.y_m))), 1.0e-6)

    def test_short_tight_corner_centerline_remains_usable(self) -> None:
        x_m = np.linspace(0.10, 0.36, 27)
        y_m = -0.20 * np.square((x_m - 0.10) / 0.26)
        path = build_path_model(
            x_m,
            y_m,
            minimum_points=12,
            minimum_span_m=0.20,
            minimum_x_m=0.10,
            maximum_x_m=2.2,
            local_smoothing_window_m=0.12,
            outlier_threshold_m=0.04,
            geometry_window_m=0.16,
        )
        self.assertIsNotNone(path)

        result = stanley_control(
            path,
            speed_mps=0.8,
            gain=1.2,
            softening_speed_mps=0.5,
            heading_lookahead_m=0.18,
            maximum_steering_angle_rad=math.radians(30.0),
            corner_heading_threshold_rad=math.radians(6.0),
            corner_opposing_correction_ratio=0.80,
        )
        self.assertLess(result.heading_error_rad, 0.0)
        self.assertLess(result.steering_angle_rad, 0.0)

    def test_inverted_servo_mapping_reverses_only_actuator_output(self) -> None:
        left_steering_rad = math.radians(10.0)
        normal = steering_angle_to_servo(
            left_steering_rad,
            maximum_steering_angle_rad=math.radians(30.0),
            servo_left=0.98,
            servo_center=0.46,
            servo_right=0.02,
        )
        inverted = steering_angle_to_servo(
            left_steering_rad,
            maximum_steering_angle_rad=math.radians(30.0),
            servo_left=0.98,
            servo_center=0.46,
            servo_right=0.02,
            inverted=True,
        )

        self.assertGreater(normal, 0.46)
        self.assertLess(inverted, 0.46)

    def test_erpm_conversion_matches_existing_vehicle_geometry(self) -> None:
        speed_mps = erpm_to_speed_mps(
            10000,
            wheel_diameter_m=0.1095,
            motor_pole_pairs=2,
            motor_pinion_teeth=13,
            spur_gear_teeth=54,
            differential_pinion_teeth=13,
            differential_ring_teeth=37,
            direction_sign=1.0,
            scale_correction=1.0,
        )
        self.assertAlmostEqual(speed_mps, 2.425, places=3)

    def test_speed_pid_never_exceeds_duty_limits(self) -> None:
        pid = SpeedPid(
            kp=0.012,
            ki=0.004,
            kd=0.0,
            integral_limit=1.0,
            minimum_duty=0.05,
            maximum_duty=0.06,
        )
        feedforward = speed_feedforward_duty(
            1.0,
            minimum_speed_mps=0.8,
            maximum_speed_mps=1.2,
            minimum_duty=0.05,
            maximum_duty=0.06,
        )
        self.assertAlmostEqual(feedforward, 0.055)
        self.assertEqual(
            pid.update(
                target_speed_mps=1.2,
                current_speed_mps=0.0,
                feedforward_duty=feedforward,
                dt_sec=0.0125,
            ),
            0.06,
        )
        self.assertEqual(
            pid.update(
                target_speed_mps=0.8,
                current_speed_mps=4.0,
                feedforward_duty=feedforward,
                dt_sec=0.0125,
            ),
            0.05,
        )


if __name__ == "__main__":
    unittest.main()
