#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from line_detactor.msg import LaneResult
from std_msgs.msg import Bool, Float32, Int32

from auto_control.automatic_brake_profile import (
    AutomaticBrakeProfile,
    AutomaticBrakeProfileConfig,
)
from auto_control.control_core import (
    OrderedPathModel,
    SpeedPid,
    build_ordered_path_model,
    clamp,
    curvature_target_speed,
    erpm_to_speed_mps,
    move_toward,
    representative_curvature,
    speed_feedforward_duty,
    stanley_control,
    steering_angle_to_servo,
)
from auto_control.performance_measurement import PerformanceMeasurement


class AutoControlNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_control")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        command_qos = QoSProfile(depth=1)
        command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        command_qos.durability = DurabilityPolicy.VOLATILE
        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._duty_pub = self.create_publisher(
            Float32, self.duty_topic, command_qos
        )
        self._brake_current_pub = self.create_publisher(
            Float32, self.brake_current_topic, command_qos
        )
        self._servo_pub = self.create_publisher(
            Float32, self.servo_position_topic, command_qos
        )
        self._command_duty_pub = self.create_publisher(
            Float32, self.command_duty_topic, command_qos
        )
        self._command_brake_current_pub = self.create_publisher(
            Float32, self.command_brake_current_topic, command_qos
        )
        self._target_speed_pub = self.create_publisher(
            Float32, self.target_speed_topic, command_qos
        )
        self._current_speed_pub = self.create_publisher(
            Float32, self.current_speed_topic, command_qos
        )
        self._curvature_pub = self.create_publisher(
            Float32, self.curvature_topic, command_qos
        )
        self._steering_angle_pub = self.create_publisher(
            Float32, self.steering_angle_topic, command_qos
        )
        self._cross_track_error_pub = self.create_publisher(
            Float32, self.cross_track_error_topic, command_qos
        )
        self._heading_error_pub = self.create_publisher(
            Float32, self.heading_error_topic, command_qos
        )
        self._raw_steering_angle_pub = self.create_publisher(
            Float32, self.raw_steering_angle_topic, command_qos
        )
        self._command_servo_position_pub = self.create_publisher(
            Float32, self.command_servo_position_topic, command_qos
        )

        self._lane_sub = self.create_subscription(
            LaneResult, self.lane_result_topic, self._on_lane_result, sensor_qos
        )
        self._erpm_sub = self.create_subscription(
            Int32, self.measured_erpm_topic, self._on_measured_erpm, sensor_qos
        )
        self._connection_sub = self.create_subscription(
            Bool,
            self.connection_status_topic,
            self._on_connection_status,
            connection_qos,
        )
        self._enable_sub = self.create_subscription(
            Bool, self.enable_topic, self._on_enable, command_qos
        )

        self._path: OrderedPathModel | None = None
        self._last_path_received_time: Time | None = None
        self._path_capture_time: Time | None = None
        self._lane_result_count = 0
        self._lane_result_status = "waiting_for_lane_result"
        self._lane_result_points = 0
        self._lane_capture_age_sec: float | None = None
        self._last_erpm_time: Time | None = None
        self._current_speed_mps = 0.0
        self._vesc_connected = False
        self._command_duty = 0.0
        self._command_brake_current = 0.0
        self._brake_mode_active = False
        self._last_motor_mode = "stop"
        self._steering_angle_rad = 0.0
        self._last_control_time = self.get_clock().now()
        self._last_stop_reason = "startup"
        self._latest_target_speed_mps = 0.0
        self._latest_curvature_per_m = 0.0
        self._latest_cross_track_error_m = 0.0
        self._latest_heading_error_rad = 0.0
        self._latest_raw_steering_angle_rad = 0.0
        self._latest_servo_position = self.servo_center
        self._latest_direction_guard_used = False

        self._speed_pid = SpeedPid(
            kp=self.speed_pid_kp,
            ki=self.speed_pid_ki,
            kd=self.speed_pid_kd,
            integral_limit=self.speed_pid_integral_limit,
            minimum_duty=self.minimum_duty,
            maximum_duty=self.maximum_duty,
        )
        self._brake_profile = AutomaticBrakeProfile(
            AutomaticBrakeProfileConfig(
                entry_speed_error_mps=self.brake_entry_speed_error_mps,
                exit_speed_error_mps=self.brake_exit_speed_error_mps,
                minimum_vehicle_speed_mps=(
                    self.brake_minimum_vehicle_speed_mps
                ),
                minimum_brake_current_amps=(
                    self.brake_minimum_current_amps
                ),
                maximum_brake_current_amps=(
                    self.brake_maximum_current_amps
                ),
                current_gain_amps_per_mps=(
                    self.brake_current_gain_amps_per_mps
                ),
                rise_amps_per_sec=self.brake_current_rise_amps_per_sec,
                fall_amps_per_sec=self.brake_current_fall_amps_per_sec,
            )
        )
        self._watchdog_timer = self.create_timer(
            1.0 / self.control_rate_hz, self._on_watchdog_timer
        )
        self._status_timer = self.create_timer(
            1.0 / self.status_log_rate_hz, self._log_status
        )
        self._performance_measurement: PerformanceMeasurement | None = None
        self._performance_timer = None
        self._performance_finished = False
        if self.performance_measurement_enabled:
            # Exercise the full centerline/Stanley/servo-position calculation
            # without publishing commands to physical actuators.
            self.enabled = True
            self.control_mode = "monitor_only"
            self._performance_measurement = PerformanceMeasurement(
                duration_sec=self.performance_measurement_duration_sec,
                startup_timeout_sec=(
                    self.performance_measurement_startup_timeout_sec
                ),
                log_directory=self.performance_measurement_log_directory,
                engine_precision=(
                    self.performance_measurement_engine_precision
                ),
                model_path=self.performance_measurement_model_path,
            )
            self._performance_timer = self.create_timer(
                self.performance_measurement_power_sample_interval_sec,
                self._on_performance_timer,
            )
            power_source = self._performance_measurement.power.source
            self.get_logger().warn(
                "Performance measurement enabled: monitor_only safety mode, "
                "preview disabled by auto_drive, duration=%.1fs after the "
                "first valid centerline, startup timeout=%.1fs, power=%s"
                % (
                    self.performance_measurement_duration_sec,
                    self.performance_measurement_startup_timeout_sec,
                    (
                        f"{power_source.rail_name} ({power_source.source})"
                        if power_source is not None
                        else "unavailable (timing will still be recorded)"
                    ),
                )
            )

        if self.enabled and self.control_mode == "drive":
            self.get_logger().warn(
                "Automatic control is armed at launch. The vehicle will move when "
                "VESC telemetry and a valid ML centerline are both available."
            )
        elif self.enabled:
            self.get_logger().warn(
                "Automatic control starts in %s mode; suppressed actuator topics "
                "will not be published." % self.control_mode
            )
        else:
            self.get_logger().info("Automatic control starts disabled.")
        self.get_logger().info(
            "Auto control ready: mode=%s, lane=%s, speed=%.2f..%.2fm/s, "
            "duty=%.3f..%.3f, auto_brake=%s/%.1fA, control=on_lane_result, watchdog=%.1fHz"
            % (
                self.control_mode,
                self.lane_result_topic,
                self.minimum_speed_mps,
                self.maximum_speed_mps,
                self.minimum_duty,
                self.maximum_duty,
                "on" if self.electrical_brake_enabled else "off",
                self.brake_maximum_current_amps,
                self.control_rate_hz,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("enabled", True)
        self.declare_parameter("control_mode", "drive")
        self.declare_parameter("enable_topic", "/auto/enabled")
        self.declare_parameter("lane_result_topic", "/line_detactor/result")
        self.declare_parameter("lane_result_frame_id", "front_axle_bev")
        self.declare_parameter("path_maximum_gap_m", 0.15)
        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("brake_current_topic", "/vesc/brake_current")
        self.declare_parameter(
            "servo_position_topic", "/vesc/servo_position"
        )
        self.declare_parameter("command_duty_topic", "/auto/current_duty")
        self.declare_parameter(
            "command_brake_current_topic", "/auto/current_brake_current"
        )
        self.declare_parameter("target_speed_topic", "/auto/target_speed")
        self.declare_parameter("current_speed_topic", "/auto/current_speed")
        self.declare_parameter("curvature_topic", "/auto/path_curvature")
        self.declare_parameter(
            "steering_angle_topic", "/auto/steering_angle_rad"
        )
        self.declare_parameter(
            "cross_track_error_topic", "/auto/cross_track_error_m"
        )
        self.declare_parameter(
            "heading_error_topic", "/auto/heading_error_rad"
        )
        self.declare_parameter(
            "raw_steering_angle_topic", "/auto/raw_steering_angle_rad"
        )
        self.declare_parameter(
            "command_servo_position_topic", "/auto/current_servo_position"
        )

        self.declare_parameter("control_rate_hz", 80.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("path_timeout_sec", 0.15)
        self.declare_parameter("path_capture_maximum_age_sec", 0.20)
        self.declare_parameter("erpm_timeout_sec", 0.15)

        self.declare_parameter("bev_x_max_m", 3.0)
        self.declare_parameter("bev_y_max_m", 0.60)
        self.declare_parameter("bev_meter_per_pixel", 0.01)
        self.declare_parameter("path_minimum_x_m", 0.05)
        self.declare_parameter("path_maximum_x_m", 2.20)
        self.declare_parameter("path_minimum_points", 8)
        self.declare_parameter("path_minimum_span_m", 0.12)
        self.declare_parameter("path_geometry_window_m", 0.14)

        self.declare_parameter("stanley_gain", 1.40)
        self.declare_parameter("stanley_softening_speed_mps", 0.40)
        self.declare_parameter("stanley_heading_lookahead_m", 0.15)
        self.declare_parameter(
            "stanley_corner_heading_threshold_deg", 4.0
        )
        self.declare_parameter(
            "stanley_corner_opposing_correction_ratio", 0.45
        )
        self.declare_parameter("maximum_steering_angle_deg", 30.0)
        self.declare_parameter("steering_current_weight", 0.47)
        self.declare_parameter("steering_rate_limit_deg_per_sec", 240.0)
        self.declare_parameter("servo_left", 1.0)
        self.declare_parameter("servo_center", 0.46)
        self.declare_parameter("servo_right", 0.0)
        # The installed servo linkage reverses the mathematical steering
        # direction. Keep desired vehicle steering signs unchanged and invert
        # only the final actuator mapping.
        self.declare_parameter("steering_servo_inverted", True)

        self.declare_parameter("minimum_speed_mps", 0.80)
        self.declare_parameter("maximum_speed_mps", 1.8)
        self.declare_parameter("maximum_lateral_acceleration_mps2", 0.6)
        self.declare_parameter("curvature_lookahead_minimum_x_m", 0.50)
        self.declare_parameter("curvature_lookahead_maximum_x_m", 1.60)
        self.declare_parameter("curvature_percentile", 90.0)
        self.declare_parameter("minimum_duty", 0.070)
        self.declare_parameter("maximum_duty", 0.090)
        self.declare_parameter("duty_rise_rate_per_sec", 0.04)
        self.declare_parameter("duty_fall_rate_per_sec", 0.08)
        self.declare_parameter("speed_pid_kp", 0.012)
        self.declare_parameter("speed_pid_ki", 0.004)
        self.declare_parameter("speed_pid_kd", 0.0)
        self.declare_parameter("speed_pid_integral_limit", 1.0)
        self.declare_parameter("speed_filter_time_constant_sec", 0.05)
        self.declare_parameter("electrical_brake_enabled", True)
        self.declare_parameter("brake_entry_speed_error_mps", 0.10)
        self.declare_parameter("brake_exit_speed_error_mps", 0.03)
        self.declare_parameter("brake_minimum_vehicle_speed_mps", 0.20)
        self.declare_parameter("brake_minimum_current_amps", 0.5)
        self.declare_parameter("brake_maximum_current_amps", 2.5)
        self.declare_parameter("brake_current_gain_amps_per_mps", 6.0)
        self.declare_parameter("brake_current_rise_amps_per_sec", 6.0)
        self.declare_parameter("brake_current_fall_amps_per_sec", 16.0)

        self.declare_parameter("wheel_diameter_m", 0.1095)
        self.declare_parameter("motor_pole_pairs", 2)
        self.declare_parameter("motor_pinion_teeth", 13)
        self.declare_parameter("spur_gear_teeth", 54)
        self.declare_parameter("differential_pinion_teeth", 13)
        self.declare_parameter("differential_ring_teeth", 37)
        self.declare_parameter("erpm_direction_sign", 1.0)
        self.declare_parameter("speed_scale_correction", 1.0)

        self.declare_parameter("performance_measurement_enabled", False)
        self.declare_parameter("performance_measurement_duration_sec", 30.0)
        self.declare_parameter(
            "performance_measurement_startup_timeout_sec", 300.0
        )
        self.declare_parameter(
            "performance_measurement_power_sample_interval_sec", 0.1
        )
        self.declare_parameter(
            "performance_measurement_log_directory", "performance_logs"
        )
        self.declare_parameter(
            "performance_measurement_engine_precision", "unknown"
        )
        self.declare_parameter("performance_measurement_model_path", "")

    def _read_parameters(self) -> None:
        string_parameters = (
            "control_mode",
            "enable_topic",
            "lane_result_topic",
            "lane_result_frame_id",
            "measured_erpm_topic",
            "connection_status_topic",
            "duty_topic",
            "brake_current_topic",
            "servo_position_topic",
            "command_duty_topic",
            "command_brake_current_topic",
            "target_speed_topic",
            "current_speed_topic",
            "curvature_topic",
            "steering_angle_topic",
            "cross_track_error_topic",
            "heading_error_topic",
            "raw_steering_angle_topic",
            "command_servo_position_topic",
            "performance_measurement_log_directory",
            "performance_measurement_engine_precision",
            "performance_measurement_model_path",
        )
        float_parameters = (
            "control_rate_hz",
            "status_log_rate_hz",
            "path_timeout_sec",
            "path_capture_maximum_age_sec",
            "erpm_timeout_sec",
            "bev_x_max_m",
            "bev_y_max_m",
            "bev_meter_per_pixel",
            "path_minimum_x_m",
            "path_maximum_x_m",
            "path_minimum_span_m",
            "path_maximum_gap_m",
            "path_geometry_window_m",
            "stanley_gain",
            "stanley_softening_speed_mps",
            "stanley_heading_lookahead_m",
            "stanley_corner_heading_threshold_deg",
            "stanley_corner_opposing_correction_ratio",
            "maximum_steering_angle_deg",
            "steering_current_weight",
            "steering_rate_limit_deg_per_sec",
            "servo_left",
            "servo_center",
            "servo_right",
            "minimum_speed_mps",
            "maximum_speed_mps",
            "maximum_lateral_acceleration_mps2",
            "curvature_lookahead_minimum_x_m",
            "curvature_lookahead_maximum_x_m",
            "curvature_percentile",
            "minimum_duty",
            "maximum_duty",
            "duty_rise_rate_per_sec",
            "duty_fall_rate_per_sec",
            "speed_pid_kp",
            "speed_pid_ki",
            "speed_pid_kd",
            "speed_pid_integral_limit",
            "speed_filter_time_constant_sec",
            "brake_entry_speed_error_mps",
            "brake_exit_speed_error_mps",
            "brake_minimum_vehicle_speed_mps",
            "brake_minimum_current_amps",
            "brake_maximum_current_amps",
            "brake_current_gain_amps_per_mps",
            "brake_current_rise_amps_per_sec",
            "brake_current_fall_amps_per_sec",
            "wheel_diameter_m",
            "erpm_direction_sign",
            "speed_scale_correction",
            "performance_measurement_duration_sec",
            "performance_measurement_startup_timeout_sec",
            "performance_measurement_power_sample_interval_sec",
        )
        int_parameters = (
            "path_minimum_points",
            "motor_pole_pairs",
            "motor_pinion_teeth",
            "spur_gear_teeth",
            "differential_pinion_teeth",
            "differential_ring_teeth",
        )
        self.enabled = bool(self.get_parameter("enabled").value)
        self.electrical_brake_enabled = bool(
            self.get_parameter("electrical_brake_enabled").value
        )
        self.steering_servo_inverted = bool(
            self.get_parameter("steering_servo_inverted").value
        )
        self.performance_measurement_enabled = bool(
            self.get_parameter("performance_measurement_enabled").value
        )
        for name in string_parameters:
            setattr(self, name, str(self.get_parameter(name).value))
        for name in float_parameters:
            setattr(self, name, float(self.get_parameter(name).value))
        for name in int_parameters:
            setattr(self, name, int(self.get_parameter(name).value))
        self.maximum_steering_angle_rad = math.radians(
            self.maximum_steering_angle_deg
        )
        self.steering_rate_limit_rad_per_sec = math.radians(
            self.steering_rate_limit_deg_per_sec
        )
        self.stanley_corner_heading_threshold_rad = math.radians(
            self.stanley_corner_heading_threshold_deg
        )

    def _validate_parameters(self) -> None:
        if self.control_mode not in ("drive", "steering_only", "monitor_only"):
            raise ValueError(
                "control_mode must be drive, steering_only, or monitor_only"
            )
        if not self.lane_result_topic or not self.lane_result_frame_id:
            raise ValueError("lane result topic and expected frame ID must not be empty")
        if not math.isfinite(self.path_maximum_gap_m) or self.path_maximum_gap_m <= 0:
            raise ValueError("path_maximum_gap_m must be finite and positive")
        if self.control_rate_hz <= 0.0 or self.status_log_rate_hz <= 0.0:
            raise ValueError("control and status rates must be positive")
        if min(
            self.performance_measurement_duration_sec,
            self.performance_measurement_startup_timeout_sec,
            self.performance_measurement_power_sample_interval_sec,
        ) <= 0.0:
            raise ValueError("performance measurement durations must be positive")
        if not self.performance_measurement_log_directory:
            raise ValueError("performance measurement log directory must not be empty")
        if min(
            self.path_timeout_sec,
            self.path_capture_maximum_age_sec,
            self.erpm_timeout_sec,
            self.bev_x_max_m,
            self.bev_y_max_m,
            self.bev_meter_per_pixel,
            self.path_minimum_span_m,
            self.path_geometry_window_m,
            self.maximum_steering_angle_rad,
            self.minimum_speed_mps,
            self.maximum_lateral_acceleration_mps2,
            self.duty_rise_rate_per_sec,
            self.duty_fall_rate_per_sec,
            self.speed_filter_time_constant_sec,
            self.wheel_diameter_m,
        ) <= 0.0:
            raise ValueError("positive auto-control parameter is not positive")
        if self.maximum_speed_mps < self.minimum_speed_mps:
            raise ValueError("maximum_speed_mps must be >= minimum_speed_mps")
        if not 0.0 < self.minimum_duty <= self.maximum_duty <= 1.0:
            raise ValueError("duty limits must satisfy 0 < minimum <= maximum <= 1")
        if not 0.0 < self.steering_current_weight <= 1.0:
            raise ValueError("steering_current_weight must be in (0, 1]")
        if self.steering_rate_limit_rad_per_sec <= 0.0:
            raise ValueError("steering_rate_limit_deg_per_sec must be positive")
        if self.speed_pid_integral_limit < 0.0:
            raise ValueError("speed_pid_integral_limit must not be negative")
        if self.speed_scale_correction <= 0.0:
            raise ValueError("speed_scale_correction must be positive")
        if not 0.0 <= self.curvature_percentile <= 100.0:
            raise ValueError("curvature_percentile must be in [0, 100]")
        if self.stanley_corner_heading_threshold_deg < 0.0:
            raise ValueError(
                "stanley_corner_heading_threshold_deg must not be negative"
            )
        if not 0.0 <= self.stanley_corner_opposing_correction_ratio < 1.0:
            raise ValueError(
                "stanley_corner_opposing_correction_ratio must be in [0, 1)"
            )
        if self.path_minimum_x_m >= self.path_maximum_x_m:
            raise ValueError("path X limits are reversed")
        if self.path_minimum_points < 3:
            raise ValueError("path_minimum_points must be at least 3")
        if min(
            self.motor_pole_pairs,
            self.motor_pinion_teeth,
            self.spur_gear_teeth,
            self.differential_pinion_teeth,
            self.differential_ring_teeth,
        ) <= 0:
            raise ValueError("motor and gear parameters must be positive")

    def _on_lane_result(self, message: LaneResult) -> None:
        callback_started_ns = (
            time.perf_counter_ns()
            if self.performance_measurement_enabled else 0
        )
        now = self.get_clock().now()
        self._lane_result_count += 1
        self._lane_result_points = len(message.centerline_points)
        self._lane_capture_age_sec = None
        # Invalid new frames must invalidate the old path immediately.
        self._path = None
        self._last_path_received_time = now
        try:
            capture_time = self._message_time_or_none(message)
            if capture_time is None:
                raise ValueError("missing source capture timestamp")
            age = (now - capture_time).nanoseconds / 1e9
            self._lane_capture_age_sec = age
            if age < -0.05 or age > self.path_capture_maximum_age_sec:
                raise ValueError("source capture timestamp is stale or in the future")
            if self._path_capture_time is not None and capture_time.nanoseconds <= self._path_capture_time.nanoseconds:
                raise ValueError("duplicate or out-of-order lane result")
            self._path_capture_time = capture_time
            if message.centerline_sample_limit_reached:
                self._lane_result_status = "centerline_sample_limit"
                return
            if not message.centerline_valid:
                self._lane_result_status = "no_centerline(state=%d)" % int(message.state)
                return
            if int(message.state) not in (1, 2, 3):
                raise ValueError("centerline has no observed lane support")
            if message.header.frame_id != self.lane_result_frame_id:
                raise ValueError(f"unexpected lane result frame: {message.header.frame_id}")
            width, height = int(message.source_width), int(message.source_height)
            pad_left, pad_right = int(message.padding_left), int(message.padding_right)
            sx = float(message.centerline_bev_width_m) / max(1, width)
            sy = float(message.centerline_bev_height_m) / max(1, height)
            if width <= 0 or height <= 0 or not all(math.isfinite(v) and v > 0 for v in (sx, sy)):
                raise ValueError("invalid source geometry")
            if not (math.isclose(sx, self.bev_meter_per_pixel, rel_tol=1e-4) and
                    math.isclose(sy, self.bev_meter_per_pixel, rel_tol=1e-4) and
                    math.isclose(width*sx, 2*self.bev_y_max_m, rel_tol=1e-4) and
                    math.isclose(height*sy, self.bev_x_max_m, rel_tol=1e-4)):
                raise ValueError("lane result scale disagrees with the configured symmetric BEV")
            count = len(message.centerline_points)
            if not 2 <= count <= 10000 or len(message.centerline_support) != count:
                raise ValueError("invalid centerline point/support lengths")
            if any(int(v) not in (1, 2, 3, 4) for v in message.centerline_support):
                raise ValueError("unknown centerline support value")
            points = np.asarray([(p.x, p.y, p.z) for p in message.centerline_points], dtype=float)
            if (not np.all(np.isfinite(points)) or np.any(np.abs(points[:, 2]) > 1e-6) or
                    np.any(points[:, 0] < 0) or np.any(points[:, 0] >= width+pad_left+pad_right) or
                    np.any(points[:, 1] < 0) or np.any(points[:, 1] >= height)):
                raise ValueError("centerline points are outside their result canvas")
            # BEV pixel centers: +X forward, +Y left. Remove output-only horizontal padding.
            x_m = self.bev_x_max_m - (points[:, 1] + 0.5) * sy
            y_m = self.bev_y_max_m - (points[:, 0] - pad_left + 0.5) * sx
            self._path = build_ordered_path_model(
                x_m, y_m,
                minimum_points=self.path_minimum_points,
                minimum_span_m=self.path_minimum_span_m,
                minimum_x_m=self.path_minimum_x_m,
                maximum_x_m=self.path_maximum_x_m,
                maximum_gap_m=self.path_maximum_gap_m,
                geometry_window_m=self.path_geometry_window_m,
            )
            self._lane_result_status = (
                "accepted" if self._path is not None else "insufficient_contiguous_path"
            )
        except (TypeError, ValueError, OverflowError) as exception:
            self._path = None
            self._lane_result_status = str(exception)
            self.get_logger().warn(f"Rejected ML centerline: {exception}", throttle_duration_sec=1.0)
        finally:
            # Every result, including invalid/empty frames, immediately decides the command.
            servo_position_calculated = self._update_control_from_path()
            if self.performance_measurement_enabled:
                callback_finished_ns = time.perf_counter_ns()
                self._record_performance_frame(
                    message=message,
                    callback_received_ros_ns=now.nanoseconds,
                    callback_started_ns=callback_started_ns,
                    callback_finished_ns=callback_finished_ns,
                    valid_centerline=self._path is not None,
                    servo_position_calculated=servo_position_calculated,
                )

    def _message_time_or_none(self, message: LaneResult) -> Time | None:
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            return None
        return Time.from_msg(message.header.stamp, clock_type=self.get_clock().clock_type)

    @staticmethod
    def _stamp_nanoseconds(stamp) -> int | None:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
        if seconds == 0 and nanoseconds == 0:
            return None
        return seconds * 1_000_000_000 + nanoseconds

    @staticmethod
    def _latency_milliseconds(
        started_ns: int | None, finished_ns: int | None
    ) -> float | None:
        if started_ns is None or finished_ns is None:
            return None
        elapsed_ns = finished_ns - started_ns
        if elapsed_ns < 0:
            return None
        return elapsed_ns / 1_000_000.0

    def _record_performance_frame(
        self,
        *,
        message: LaneResult,
        callback_received_ros_ns: int,
        callback_started_ns: int,
        callback_finished_ns: int,
        valid_centerline: bool,
        servo_position_calculated: bool,
    ) -> None:
        measurement = self._performance_measurement
        if measurement is None or self._performance_finished:
            return
        if not measurement.active:
            if not valid_centerline or not servo_position_calculated:
                return
            measurement.start(callback_started_ns)
            self.get_logger().info(
                "First valid ML centerline reached servo-position calculation; "
                "starting %.1fs performance measurement."
                % self.performance_measurement_duration_sec
            )

        callback_finished_ros_ns = self.get_clock().now().nanoseconds
        source_stamp_ns = self._stamp_nanoseconds(message.header.stamp)
        detector_input_stamp_ns = self._stamp_nanoseconds(
            message.detector_input_received_stamp
        )
        result_ready_stamp_ns = self._stamp_nanoseconds(
            message.detector_result_ready_stamp
        )
        auto_control_compute_ms = (
            callback_finished_ns - callback_started_ns
        ) / 1_000_000.0
        detector_total_compute_ms = (
            int(message.detector_total_compute_nanoseconds) / 1_000_000.0
        )
        lane_postprocess_total_ms = (
            int(message.label_export_nanoseconds)
            + int(message.backend_postprocess_nanoseconds)
            + int(message.lane_geometry_nanoseconds)
            + int(message.result_message_build_nanoseconds)
        ) / 1_000_000.0
        frame = {
            "measurement_elapsed_sec": measurement.elapsed_sec(
                callback_finished_ns
            ),
            "detector_sequence": int(message.detector_sequence),
            "actual_engine_precision": str(message.engine_precision),
            "valid_centerline": valid_centerline,
            "servo_position_calculated": servo_position_calculated,
            "centerline_point_count": len(message.centerline_points),
            "steering_angle_rad": float(self._steering_angle_rad),
            "servo_position": float(self._latest_servo_position),
            "h2d_preprocess_ms": (
                int(message.h2d_preprocess_nanoseconds) / 1_000_000.0
            ),
            "pure_inference_ms": (
                int(message.pure_inference_nanoseconds) / 1_000_000.0
            ),
            "label_export_ms": (
                int(message.label_export_nanoseconds) / 1_000_000.0
            ),
            "backend_postprocess_ms": (
                int(message.backend_postprocess_nanoseconds) / 1_000_000.0
            ),
            "lane_geometry_ms": (
                int(message.lane_geometry_nanoseconds) / 1_000_000.0
            ),
            "result_message_build_ms": (
                int(message.result_message_build_nanoseconds) / 1_000_000.0
            ),
            "lane_postprocess_total_ms": lane_postprocess_total_ms,
            "detector_queue_ms": (
                int(message.detector_queue_nanoseconds) / 1_000_000.0
            ),
            "detector_total_compute_ms": detector_total_compute_ms,
            "lane_result_transport_ms": self._latency_milliseconds(
                result_ready_stamp_ns, callback_received_ros_ns
            ),
            "source_to_detector_input_ms": self._latency_milliseconds(
                source_stamp_ns, detector_input_stamp_ns
            ),
            "auto_control_compute_ms": auto_control_compute_ms,
            "compute_only_total_ms": (
                detector_total_compute_ms + auto_control_compute_ms
            ),
            "detector_input_to_control_complete_ms": (
                self._latency_milliseconds(
                    detector_input_stamp_ns, callback_finished_ros_ns
                )
            ),
            "source_capture_to_control_complete_ms": (
                self._latency_milliseconds(
                    source_stamp_ns, callback_finished_ros_ns
                )
            ),
        }
        measurement.add_frame(frame)

    def _on_performance_timer(self) -> None:
        measurement = self._performance_measurement
        if measurement is None or self._performance_finished:
            return
        if measurement.startup_timed_out():
            self._finish_performance_measurement("startup_timeout")
            return
        if not measurement.active:
            return
        measurement.sample_power()
        if measurement.due_to_finish():
            self._finish_performance_measurement("complete")

    def _finish_performance_measurement(self, status: str) -> None:
        measurement = self._performance_measurement
        if measurement is None or self._performance_finished:
            return
        self._performance_finished = True
        try:
            output_path = measurement.write(status)
            self.get_logger().info(
                "PERFORMANCE_MEASUREMENT_COMPLETE status=%s file=%s frames=%d"
                % (status, output_path, len(measurement.frames))
            )
        except (OSError, ValueError, TypeError) as exception:
            self.get_logger().error(
                f"Failed to write performance measurement: {exception}"
            )
        self.stop_actuators()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_measured_erpm(self, message: Int32) -> None:
        now = self.get_clock().now()
        raw_speed_mps = erpm_to_speed_mps(
            int(message.data),
            wheel_diameter_m=self.wheel_diameter_m,
            motor_pole_pairs=self.motor_pole_pairs,
            motor_pinion_teeth=self.motor_pinion_teeth,
            spur_gear_teeth=self.spur_gear_teeth,
            differential_pinion_teeth=self.differential_pinion_teeth,
            differential_ring_teeth=self.differential_ring_teeth,
            direction_sign=self.erpm_direction_sign,
            scale_correction=self.speed_scale_correction,
        )
        if self._last_erpm_time is None:
            self._current_speed_mps = raw_speed_mps
        else:
            dt_sec = self._elapsed_sec(self._last_erpm_time, now)
            if dt_sec <= 0.0 or dt_sec > self.erpm_timeout_sec:
                self._current_speed_mps = raw_speed_mps
            else:
                gain = 1.0 - math.exp(
                    -dt_sec / max(1.0e-4, self.speed_filter_time_constant_sec)
                )
                self._current_speed_mps += gain * (
                    raw_speed_mps - self._current_speed_mps
                )
        self._last_erpm_time = now

    def _on_connection_status(self, message: Bool) -> None:
        self._vesc_connected = bool(message.data)

    def _on_enable(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        if not self.enabled:
            self._stop_control("disabled")
            self._publish_commands(
                duty=0.0,
                brake_current=0.0,
                servo_position=self.servo_center,
                motor_mode="stop",
            )
        self.get_logger().warn(
            f"Automatic control {'enabled' if self.enabled else 'disabled'}."
        )

    def _on_watchdog_timer(self) -> None:
        # No repeated PID/steering updates against the same image. The timer only
        # stops/repeats zero commands when input or vehicle state becomes invalid.
        reason = self._stop_reason(self.get_clock().now())
        if reason is not None:
            self._stop_control(reason)
            self._publish_commands(
                duty=0.0,
                brake_current=0.0,
                servo_position=self.servo_center,
                motor_mode="stop",
            )

    def _update_control_from_path(self) -> bool:
        now = self.get_clock().now()
        dt_sec = clamp(
            self._elapsed_sec(self._last_control_time, now),
            1.0e-6,
            min(self.path_timeout_sec, self.path_capture_maximum_age_sec, self.erpm_timeout_sec),
        )
        self._last_control_time = now

        stop_reason = self._stop_reason(now)
        if stop_reason is not None:
            self._stop_control(stop_reason)
            self._publish_commands(
                duty=0.0,
                brake_current=0.0,
                servo_position=self.servo_center,
                motor_mode="stop",
            )
            return False

        assert self._path is not None
        curvature_per_m = representative_curvature(
            self._path,
            lookahead_minimum_x_m=self.curvature_lookahead_minimum_x_m,
            lookahead_maximum_x_m=self.curvature_lookahead_maximum_x_m,
            percentile=self.curvature_percentile,
        )
        target_speed_mps = curvature_target_speed(
            curvature_per_m,
            maximum_lateral_acceleration_mps2=(
                self.maximum_lateral_acceleration_mps2
            ),
            minimum_speed_mps=self.minimum_speed_mps,
            maximum_speed_mps=self.maximum_speed_mps,
        )
        stanley = stanley_control(
            self._path,
            speed_mps=self._current_speed_mps,
            gain=self.stanley_gain,
            softening_speed_mps=self.stanley_softening_speed_mps,
            heading_lookahead_m=self.stanley_heading_lookahead_m,
            maximum_steering_angle_rad=self.maximum_steering_angle_rad,
            corner_heading_threshold_rad=(
                self.stanley_corner_heading_threshold_rad
            ),
            corner_opposing_correction_ratio=(
                self.stanley_corner_opposing_correction_ratio
            ),
        )

        corner_sign_reset_used = (
            abs(stanley.heading_error_rad)
            >= self.stanley_corner_heading_threshold_rad
            and stanley.steering_angle_rad * self._steering_angle_rad < 0.0
        )
        if corner_sign_reset_used:
            # Do not let temporal smoothing retain a left command after the
            # fitted path has clearly entered a right corner, or vice versa.
            self._steering_angle_rad = 0.0
            filtered_steering_rad = stanley.steering_angle_rad
        else:
            # Keep the YAML weight's response at its reference control_rate_hz
            # while accounting for the actual interval between image results.
            current_weight = 1.0 - (1.0 - self.steering_current_weight) ** (
                dt_sec * self.control_rate_hz
            )
            filtered_steering_rad = (
                current_weight * stanley.steering_angle_rad
                + (1.0 - current_weight)
                * self._steering_angle_rad
            )
        self._steering_angle_rad = move_toward(
            self._steering_angle_rad,
            filtered_steering_rad,
            self.steering_rate_limit_rad_per_sec * dt_sec,
        )
        servo_position = steering_angle_to_servo(
            self._steering_angle_rad,
            maximum_steering_angle_rad=self.maximum_steering_angle_rad,
            servo_left=self.servo_left,
            servo_center=self.servo_center,
            servo_right=self.servo_right,
            inverted=self.steering_servo_inverted,
        )

        if self.control_mode != "drive":
            # Passive modes must never contend with a manual controller for the
            # motor topics. Keep command diagnostics explicitly at zero.
            self._command_duty = 0.0
            self._command_brake_current = 0.0
            self._brake_mode_active = False
            self._speed_pid.reset()
            self._brake_profile.reset()
            motor_mode = "suppressed"
        elif self.electrical_brake_enabled:
            self._command_brake_current = self._brake_profile.update(
                target_speed_mps=target_speed_mps,
                current_speed_mps=self._current_speed_mps,
                dt_sec=dt_sec,
            )
        else:
            self._brake_profile.reset()
            self._command_brake_current = 0.0

        if self.control_mode == "drive" and self._command_brake_current > 0.0:
            # Brake current owns the VESC motor mode. Discard propulsion PID
            # state so positive duty can never fight an active brake command.
            self._command_duty = 0.0
            self._speed_pid.reset()
            motor_mode = "brake"
            self._brake_mode_active = True
        elif self.control_mode == "drive" and self._brake_mode_active:
            # Explicitly release COMM_SET_CURRENT_BRAKE, then wait one control
            # cycle before returning to positive duty.
            self._command_duty = 0.0
            self._speed_pid.reset()
            motor_mode = "brake_release"
            self._brake_mode_active = False
        elif self.control_mode == "drive":
            feedforward_duty = speed_feedforward_duty(
                target_speed_mps,
                minimum_speed_mps=self.minimum_speed_mps,
                maximum_speed_mps=self.maximum_speed_mps,
                minimum_duty=self.minimum_duty,
                maximum_duty=self.maximum_duty,
            )
            desired_duty = self._speed_pid.update(
                target_speed_mps=target_speed_mps,
                current_speed_mps=self._current_speed_mps,
                feedforward_duty=feedforward_duty,
                dt_sec=dt_sec,
            )
            if self._command_duty < self.minimum_duty:
                # Match manual driving's start-duty behavior when a path appears.
                self._command_duty = self.minimum_duty
            else:
                rate = (
                    self.duty_rise_rate_per_sec
                    if desired_duty >= self._command_duty
                    else self.duty_fall_rate_per_sec
                )
                self._command_duty = move_toward(
                    self._command_duty, desired_duty, rate * dt_sec
                )
            motor_mode = "duty"

        self._last_stop_reason = "running"
        self._latest_target_speed_mps = target_speed_mps
        self._latest_curvature_per_m = curvature_per_m
        self._latest_cross_track_error_m = stanley.cross_track_error_m
        self._latest_heading_error_rad = stanley.heading_error_rad
        self._latest_raw_steering_angle_rad = stanley.steering_angle_rad
        self._latest_direction_guard_used = (
            stanley.direction_guard_used or corner_sign_reset_used
        )
        self._publish_commands(
            duty=self._command_duty,
            brake_current=self._command_brake_current,
            servo_position=servo_position,
            motor_mode=motor_mode,
        )
        return True

    def _stop_reason(self, now: Time) -> str | None:
        if not self.enabled:
            return "disabled"
        # Lateral-error monitoring is camera/path based. It intentionally stays
        # available while the vehicle is pushed by hand without VESC telemetry.
        if self.control_mode != "monitor_only":
            if not self._vesc_connected:
                return "vesc_disconnected"
            if self._last_erpm_time is None:
                return "waiting_for_erpm"
            if self._elapsed_sec(self._last_erpm_time, now) > self.erpm_timeout_sec:
                return "erpm_timeout"
        if self._path is None or self._last_path_received_time is None:
            return "centerline_missing"
        if (
            self._elapsed_sec(self._last_path_received_time, now)
            > self.path_timeout_sec
        ):
            return "centerline_timeout"
        if self._path_capture_time is not None and (
            self._elapsed_sec(self._path_capture_time, now)
            > self.path_capture_maximum_age_sec
        ):
            return "centerline_stale"
        return None

    def _stop_control(self, reason: str) -> None:
        # Stopped time must not accumulate into a large first steering/PID step.
        self._last_control_time = self.get_clock().now()
        self._command_duty = 0.0
        self._command_brake_current = 0.0
        self._brake_mode_active = False
        self._steering_angle_rad = 0.0
        self._latest_target_speed_mps = 0.0
        self._latest_curvature_per_m = 0.0
        self._latest_cross_track_error_m = 0.0
        self._latest_heading_error_rad = 0.0
        self._latest_raw_steering_angle_rad = 0.0
        self._latest_servo_position = self.servo_center
        self._latest_direction_guard_used = False
        self._speed_pid.reset()
        self._brake_profile.reset()
        if reason != self._last_stop_reason:
            self.get_logger().warn(
                f"Automatic drive stopped: {reason}. Releasing motor command."
            )
        self._last_stop_reason = reason

    def _publish_commands(
        self,
        *,
        duty: float,
        brake_current: float,
        servo_position: float,
        motor_mode: str,
    ) -> None:
        if self.control_mode == "drive":
            if motor_mode == "duty":
                self._duty_pub.publish(Float32(data=float(duty)))
            elif motor_mode == "brake":
                self._brake_current_pub.publish(
                    Float32(data=float(brake_current))
                )
            elif motor_mode == "brake_release":
                self._brake_current_pub.publish(Float32(data=0.0))
            elif motor_mode == "stop":
                # Either zero command safely supersedes a previous drive mode.
                # Publish both so the bridge state and diagnostics are explicit.
                self._brake_current_pub.publish(Float32(data=0.0))
                self._duty_pub.publish(Float32(data=0.0))
            else:
                raise ValueError(f"unsupported motor mode: {motor_mode}")
        elif motor_mode not in ("suppressed", "stop"):
            raise ValueError(f"unsupported passive motor mode: {motor_mode}")
        self._last_motor_mode = motor_mode
        self._latest_servo_position = float(servo_position)
        if self.control_mode != "monitor_only":
            self._servo_pub.publish(Float32(data=float(servo_position)))
        self._command_duty_pub.publish(Float32(data=float(duty)))
        self._command_brake_current_pub.publish(
            Float32(data=float(brake_current))
        )
        self._target_speed_pub.publish(
            Float32(data=float(self._latest_target_speed_mps))
        )
        self._current_speed_pub.publish(
            Float32(data=float(self._current_speed_mps))
        )
        self._curvature_pub.publish(
            Float32(data=float(self._latest_curvature_per_m))
        )
        self._steering_angle_pub.publish(
            Float32(data=float(self._steering_angle_rad))
        )
        self._cross_track_error_pub.publish(
            Float32(data=float(self._latest_cross_track_error_m))
        )
        self._heading_error_pub.publish(
            Float32(data=float(self._latest_heading_error_rad))
        )
        self._raw_steering_angle_pub.publish(
            Float32(data=float(self._latest_raw_steering_angle_rad))
        )
        self._command_servo_position_pub.publish(
            Float32(data=self._latest_servo_position)
        )

    def _log_status(self) -> None:
        path_points = self._path.point_count if self._path is not None else 0
        receive_age = (
            "never" if self._last_path_received_time is None else
            "%.3fs" % self._elapsed_sec(self._last_path_received_time, self.get_clock().now())
        )
        capture_age = (
            "unknown" if self._lane_capture_age_sec is None else
            "%.3fs" % self._lane_capture_age_sec
        )
        self.get_logger().info(
            "Auto status | state=%s | path_points=%d | speed=%.2f/%.2fm/s | "
            "curvature=%.3f/m | cte=%+.3fm | heading=%+.1fdeg | guard=%s | "
            "raw/final_steering=%+.1f/%+.1fdeg | servo=%.3f | "
            "motor=%s | duty=%.4f | brake=%.2fA | "
            "lane_rx=%d | lane_status=%s | source_points=%d | "
            "last_rx_age=%s | capture_age_at_rx=%s"
            % (
                self._last_stop_reason,
                path_points,
                self._current_speed_mps,
                self._latest_target_speed_mps,
                self._latest_curvature_per_m,
                self._latest_cross_track_error_m,
                math.degrees(self._latest_heading_error_rad),
                "on" if self._latest_direction_guard_used else "off",
                math.degrees(self._latest_raw_steering_angle_rad),
                math.degrees(self._steering_angle_rad),
                self._latest_servo_position,
                self._last_motor_mode,
                self._command_duty,
                self._command_brake_current,
                self._lane_result_count,
                self._lane_result_status,
                self._lane_result_points,
                receive_age,
                capture_age,
            )
        )

    @staticmethod
    def _elapsed_sec(start: Time, end: Time) -> float:
        return max(0.0, (end - start).nanoseconds / 1_000_000_000.0)

    def stop_actuators(self) -> None:
        # main() may drain one queued callback while shutting down. A pending
        # lane result must not re-arm frame-triggered control after the stop.
        self.enabled = False
        self._watchdog_timer.cancel()
        self._status_timer.cancel()
        if self._performance_timer is not None:
            self._performance_timer.cancel()
        self._stop_control("shutdown")
        self._publish_commands(
            duty=0.0,
            brake_current=0.0,
            servo_position=self.servo_center,
            motor_mode="stop",
        )

    def write_interrupted_performance_measurement(self) -> None:
        measurement = self._performance_measurement
        if measurement is None or self._performance_finished:
            return
        self._performance_finished = True
        try:
            output_path = measurement.write("interrupted")
            self.get_logger().warn(
                "Interrupted performance measurement written to %s"
                % output_path
            )
        except (OSError, ValueError, TypeError) as exception:
            self.get_logger().error(
                f"Failed to write interrupted performance measurement: {exception}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.write_interrupted_performance_measurement()
        if rclpy.ok():
            node.stop_actuators()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
