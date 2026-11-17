from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class EstimatorConfig:
    wheel_diameter_m: float = 0.1095
    motor_pole_pairs: int = 2
    motor_pinion_teeth: int = 13
    spur_gear_teeth: int = 54
    differential_pinion_teeth: int = 13
    differential_ring_teeth: int = 37
    erpm_direction_sign: float = 1.0
    speed_scale_correction: float = 1.0
    speed_deadband_mps: float = 0.03
    speed_filter_time_constant_sec: float = 0.05
    acceleration_filter_time_constant_sec: float = 0.12
    maximum_speed_mps: float = 6.0
    maximum_longitudinal_acceleration_mps2: float = 15.0
    maximum_lateral_acceleration_mps2: float = 15.0
    maximum_yaw_rate_radps: float = 8.0
    maximum_sample_interval_sec: float = 0.10
    sample_timeout_sec: float = 0.15
    imu_timeout_sec: float = 0.10
    steering_timeout_sec: float = 0.15
    wheelbase_m: float = 0.324
    maximum_left_steering_angle_deg: float = 30.0
    maximum_right_steering_angle_deg: float = 30.0
    servo_left: float = 1.0
    servo_center: float = 0.46
    servo_right: float = 0.0

    def validate(self) -> None:
        positive_values = {
            "wheel_diameter_m": self.wheel_diameter_m,
            "motor_pole_pairs": self.motor_pole_pairs,
            "motor_pinion_teeth": self.motor_pinion_teeth,
            "spur_gear_teeth": self.spur_gear_teeth,
            "differential_pinion_teeth": self.differential_pinion_teeth,
            "differential_ring_teeth": self.differential_ring_teeth,
            "speed_scale_correction": self.speed_scale_correction,
            "speed_filter_time_constant_sec": self.speed_filter_time_constant_sec,
            "acceleration_filter_time_constant_sec": (
                self.acceleration_filter_time_constant_sec
            ),
            "maximum_speed_mps": self.maximum_speed_mps,
            "maximum_longitudinal_acceleration_mps2": (
                self.maximum_longitudinal_acceleration_mps2
            ),
            "maximum_lateral_acceleration_mps2": (
                self.maximum_lateral_acceleration_mps2
            ),
            "maximum_yaw_rate_radps": self.maximum_yaw_rate_radps,
            "maximum_sample_interval_sec": self.maximum_sample_interval_sec,
            "sample_timeout_sec": self.sample_timeout_sec,
            "imu_timeout_sec": self.imu_timeout_sec,
            "steering_timeout_sec": self.steering_timeout_sec,
            "wheelbase_m": self.wheelbase_m,
            "maximum_left_steering_angle_deg": (
                self.maximum_left_steering_angle_deg
            ),
            "maximum_right_steering_angle_deg": (
                self.maximum_right_steering_angle_deg
            ),
        }
        for name, value in positive_values.items():
            if not math.isfinite(float(value)) or float(value) <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if not math.isfinite(self.speed_deadband_mps) or self.speed_deadband_mps < 0:
            raise ValueError("speed_deadband_mps must be finite and non-negative")
        if not math.isfinite(self.erpm_direction_sign) or abs(
            self.erpm_direction_sign
        ) < 1.0e-12:
            raise ValueError("erpm_direction_sign must be finite and non-zero")
        for name, value in {
            "servo_left": self.servo_left,
            "servo_center": self.servo_center,
            "servo_right": self.servo_right,
        }.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if self.servo_left == self.servo_center or self.servo_right == self.servo_center:
            raise ValueError("servo endpoints must differ from servo_center")


@dataclass(frozen=True)
class DynamicsSample:
    timestamp_sec: float
    valid: bool
    measured_erpm: int
    motor_rpm: float
    wheel_rpm: float
    raw_speed_mps: float
    speed_mps: float
    longitudinal_acceleration_mps2: float
    steering_angle_rad: float
    yaw_rate_radps: float
    lateral_acceleration_mps2: float
    yaw_rate_source: str


class VehicleDynamicsEstimator:
    def __init__(self, config: EstimatorConfig | None = None) -> None:
        self.config = config or EstimatorConfig()
        self.config.validate()

        self.total_gear_ratio = (
            float(self.config.spur_gear_teeth)
            / float(self.config.motor_pinion_teeth)
            * float(self.config.differential_ring_teeth)
            / float(self.config.differential_pinion_teeth)
        )
        self.meters_per_second_per_erpm = (
            math.copysign(1.0, self.config.erpm_direction_sign)
            * self.config.speed_scale_correction
            * math.pi
            * self.config.wheel_diameter_m
            / (
                60.0
                * float(self.config.motor_pole_pairs)
                * self.total_gear_ratio
            )
        )

        self._measured_erpm = 0
        self._raw_speed_mps = 0.0
        self._speed_mps = 0.0
        self._longitudinal_acceleration_mps2 = 0.0
        self._last_motion_time_sec: float | None = None
        self._imu_yaw_rate_radps = 0.0
        self._last_imu_time_sec: float | None = None
        self._servo_position = self.config.servo_center
        self._last_steering_time_sec: float | None = None

    def update_erpm(self, measured_erpm: int, timestamp_sec: float) -> None:
        self._require_finite_time(timestamp_sec)
        raw_speed_mps = self._speed_from_erpm(measured_erpm)
        previous_time_sec = self._last_motion_time_sec

        self._measured_erpm = int(measured_erpm)
        self._raw_speed_mps = raw_speed_mps
        self._last_motion_time_sec = timestamp_sec

        if previous_time_sec is None:
            self._speed_mps = raw_speed_mps
            self._longitudinal_acceleration_mps2 = 0.0
            return

        dt_sec = timestamp_sec - previous_time_sec
        if dt_sec <= 0.0:
            return
        if dt_sec > self.config.maximum_sample_interval_sec:
            self._speed_mps = raw_speed_mps
            self._longitudinal_acceleration_mps2 = 0.0
            return

        previous_speed_mps = self._speed_mps
        speed_gain = self._filter_gain(
            dt_sec, self.config.speed_filter_time_constant_sec
        )
        self._speed_mps += speed_gain * (raw_speed_mps - self._speed_mps)
        raw_acceleration_mps2 = self._clamp(
            (self._speed_mps - previous_speed_mps) / dt_sec,
            -self.config.maximum_longitudinal_acceleration_mps2,
            self.config.maximum_longitudinal_acceleration_mps2,
        )
        acceleration_gain = self._filter_gain(
            dt_sec, self.config.acceleration_filter_time_constant_sec
        )
        self._longitudinal_acceleration_mps2 += acceleration_gain * (
            raw_acceleration_mps2 - self._longitudinal_acceleration_mps2
        )

        # The low-pass filters approach zero asymptotically and otherwise leave
        # denormal values such as 1e-41 on the published speed topic. Once the
        # raw ERPM-derived speed and the filtered speed are both inside the
        # configured physical deadband, the remaining filter state no longer
        # represents measurable vehicle motion. Snap both states to an exact
        # standstill instead of waiting for floating-point underflow.
        if (
            raw_speed_mps == 0.0
            and abs(self._speed_mps) <= self.config.speed_deadband_mps
        ):
            self._speed_mps = 0.0
            self._longitudinal_acceleration_mps2 = 0.0

    def update_imu_yaw_rate(
        self, yaw_rate_radps: float, timestamp_sec: float
    ) -> None:
        self._require_finite_time(timestamp_sec)
        if not math.isfinite(yaw_rate_radps):
            raise ValueError("yaw_rate_radps must be finite")
        self._imu_yaw_rate_radps = self._clamp(
            yaw_rate_radps,
            -self.config.maximum_yaw_rate_radps,
            self.config.maximum_yaw_rate_radps,
        )
        self._last_imu_time_sec = timestamp_sec

    def update_servo_position(
        self, servo_position: float, timestamp_sec: float
    ) -> None:
        self._require_finite_time(timestamp_sec)
        if not math.isfinite(servo_position):
            raise ValueError("servo_position must be finite")
        minimum = min(
            self.config.servo_left,
            self.config.servo_center,
            self.config.servo_right,
        )
        maximum = max(
            self.config.servo_left,
            self.config.servo_center,
            self.config.servo_right,
        )
        self._servo_position = self._clamp(servo_position, minimum, maximum)
        self._last_steering_time_sec = timestamp_sec

    def sample(self, timestamp_sec: float, yaw_rate_source: str = "auto") -> DynamicsSample:
        self._require_finite_time(timestamp_sec)
        if yaw_rate_source not in {"auto", "imu", "steering"}:
            raise ValueError("yaw_rate_source must be auto, imu, or steering")

        motion_is_fresh = self._is_fresh(
            self._last_motion_time_sec,
            timestamp_sec,
            self.config.sample_timeout_sec,
        )
        imu_is_fresh = self._is_fresh(
            self._last_imu_time_sec,
            timestamp_sec,
            self.config.imu_timeout_sec,
        )
        steering_is_fresh = self._is_fresh(
            self._last_steering_time_sec,
            timestamp_sec,
            self.config.steering_timeout_sec,
        )

        steering_angle_rad = self._steering_angle_from_servo(self._servo_position)
        steering_yaw_rate_radps = self._clamp(
            self._speed_mps * math.tan(steering_angle_rad) / self.config.wheelbase_m,
            -self.config.maximum_yaw_rate_radps,
            self.config.maximum_yaw_rate_radps,
        )

        selected_source = "none"
        yaw_rate_radps = 0.0
        if yaw_rate_source in {"auto", "imu"} and imu_is_fresh:
            selected_source = "imu"
            yaw_rate_radps = self._imu_yaw_rate_radps
        elif yaw_rate_source in {"auto", "steering"} and steering_is_fresh:
            selected_source = "steering"
            yaw_rate_radps = steering_yaw_rate_radps

        if not motion_is_fresh:
            yaw_rate_radps = 0.0
        lateral_acceleration_mps2 = self._clamp(
            self._speed_mps * yaw_rate_radps,
            -self.config.maximum_lateral_acceleration_mps2,
            self.config.maximum_lateral_acceleration_mps2,
        )

        motor_rpm = float(self._measured_erpm) / float(
            self.config.motor_pole_pairs
        )
        return DynamicsSample(
            timestamp_sec=timestamp_sec,
            valid=motion_is_fresh,
            measured_erpm=self._measured_erpm,
            motor_rpm=motor_rpm,
            wheel_rpm=motor_rpm / self.total_gear_ratio,
            raw_speed_mps=self._raw_speed_mps,
            speed_mps=self._speed_mps,
            longitudinal_acceleration_mps2=(
                self._longitudinal_acceleration_mps2 if motion_is_fresh else 0.0
            ),
            steering_angle_rad=steering_angle_rad,
            yaw_rate_radps=yaw_rate_radps,
            lateral_acceleration_mps2=lateral_acceleration_mps2,
            yaw_rate_source=selected_source,
        )

    def _speed_from_erpm(self, measured_erpm: int) -> float:
        speed_mps = float(measured_erpm) * self.meters_per_second_per_erpm
        if abs(speed_mps) < self.config.speed_deadband_mps:
            speed_mps = 0.0
        return self._clamp(
            speed_mps,
            -self.config.maximum_speed_mps,
            self.config.maximum_speed_mps,
        )

    def _steering_angle_from_servo(self, position: float) -> float:
        center = self.config.servo_center
        if (position - center) * (self.config.servo_left - center) >= 0.0:
            fraction = (position - center) / (self.config.servo_left - center)
            return self._clamp(fraction, 0.0, 1.0) * math.radians(
                self.config.maximum_left_steering_angle_deg
            )
        fraction = (position - center) / (self.config.servo_right - center)
        return -self._clamp(fraction, 0.0, 1.0) * math.radians(
            self.config.maximum_right_steering_angle_deg
        )

    @staticmethod
    def _is_fresh(
        last_timestamp_sec: float | None,
        timestamp_sec: float,
        timeout_sec: float,
    ) -> bool:
        if last_timestamp_sec is None:
            return False
        age_sec = timestamp_sec - last_timestamp_sec
        return 0.0 <= age_sec <= timeout_sec

    @staticmethod
    def _filter_gain(dt_sec: float, time_constant_sec: float) -> float:
        return -math.expm1(-dt_sec / time_constant_sec)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _require_finite_time(timestamp_sec: float) -> None:
        if not math.isfinite(timestamp_sec):
            raise ValueError("timestamp_sec must be finite")
