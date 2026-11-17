import math
import unittest

from vehicle_dynamics_monitor.dynamics_estimator import (
    EstimatorConfig,
    VehicleDynamicsEstimator,
)


class VehicleDynamicsEstimatorTest(unittest.TestCase):
    @staticmethod
    def _fast_estimator() -> VehicleDynamicsEstimator:
        return VehicleDynamicsEstimator(
            EstimatorConfig(
                speed_filter_time_constant_sec=0.0001,
                acceleration_filter_time_constant_sec=0.0001,
                maximum_longitudinal_acceleration_mps2=1000.0,
            )
        )

    def test_erpm_conversion_matches_vehicle_geometry(self) -> None:
        estimator = self._fast_estimator()
        expected_ratio = (54.0 / 13.0) * (37.0 / 13.0)
        self.assertAlmostEqual(estimator.total_gear_ratio, expected_ratio)
        estimator.update_erpm(0, 1.0)
        estimator.update_erpm(20620, 1.0125)

        sample = estimator.sample(1.0125)

        self.assertTrue(sample.valid)
        self.assertGreater(sample.speed_mps, 4.9)
        self.assertLess(sample.speed_mps, 5.1)
        self.assertGreater(sample.longitudinal_acceleration_mps2, 0.0)
        self.assertAlmostEqual(sample.motor_rpm, 10310.0)
        self.assertAlmostEqual(
            sample.wheel_rpm, sample.motor_rpm / expected_ratio
        )

    def test_imu_yaw_rate_is_preferred_and_generates_lateral_acceleration(self) -> None:
        estimator = self._fast_estimator()
        estimator.update_erpm(20620, 1.0)
        estimator.update_imu_yaw_rate(0.4, 1.0)
        estimator.update_servo_position(0.98, 1.0)

        sample = estimator.sample(1.01, "auto")

        self.assertEqual(sample.yaw_rate_source, "imu")
        self.assertAlmostEqual(sample.yaw_rate_radps, 0.4)
        self.assertAlmostEqual(
            sample.lateral_acceleration_mps2, sample.speed_mps * 0.4
        )

    def test_steering_is_used_when_imu_is_stale(self) -> None:
        estimator = self._fast_estimator()
        estimator.update_erpm(10000, 1.0)
        estimator.update_imu_yaw_rate(-0.8, 0.5)
        estimator.update_servo_position(0.98, 1.0)

        sample = estimator.sample(1.01, "auto")

        expected_yaw_rate = (
            sample.speed_mps * math.tan(math.radians(30.0)) / 0.324
        )
        self.assertEqual(sample.yaw_rate_source, "steering")
        self.assertAlmostEqual(sample.steering_angle_rad, math.radians(30.0))
        self.assertAlmostEqual(sample.yaw_rate_radps, expected_yaw_rate)
        self.assertGreater(sample.lateral_acceleration_mps2, 0.0)

    def test_reverse_motion_keeps_signed_speed(self) -> None:
        estimator = self._fast_estimator()
        estimator.update_erpm(-20620, 1.0)

        sample = estimator.sample(1.01)

        self.assertLess(sample.speed_mps, -4.9)
        self.assertGreater(sample.speed_mps, -5.1)

    def test_stale_erpm_invalidates_output_acceleration(self) -> None:
        estimator = self._fast_estimator()
        estimator.update_erpm(10000, 1.0)
        estimator.update_imu_yaw_rate(0.5, 1.0)

        sample = estimator.sample(2.0)

        self.assertFalse(sample.valid)
        self.assertEqual(sample.longitudinal_acceleration_mps2, 0.0)
        self.assertEqual(sample.lateral_acceleration_mps2, 0.0)

    def test_standstill_snaps_filtered_tail_to_exact_zero(self) -> None:
        estimator = VehicleDynamicsEstimator()
        sample_period_sec = 1.0 / 80.0
        estimator.update_erpm(10000, 1.0)

        timestamp_sec = 1.0
        for _ in range(80):
            timestamp_sec += sample_period_sec
            estimator.update_erpm(0, timestamp_sec)
            if estimator.sample(timestamp_sec).speed_mps == 0.0:
                break

        sample = estimator.sample(timestamp_sec)
        self.assertLess(timestamp_sec - 1.0, 0.5)
        self.assertEqual(sample.raw_speed_mps, 0.0)
        self.assertEqual(sample.speed_mps, 0.0)
        self.assertEqual(sample.longitudinal_acceleration_mps2, 0.0)

        timestamp_sec += sample_period_sec
        estimator.update_erpm(0, timestamp_sec)
        self.assertEqual(estimator.sample(timestamp_sec).speed_mps, 0.0)


if __name__ == "__main__":
    unittest.main()
