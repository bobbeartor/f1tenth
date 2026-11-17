#!/usr/bin/env python3

from __future__ import annotations

import math
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32

from vehicle_dynamics_monitor.dynamics_estimator import (
    DynamicsSample,
    EstimatorConfig,
    VehicleDynamicsEstimator,
)
from vehicle_dynamics_monitor.vesc_can import (
    SocketCanReceiver,
    decode_vesc_can_frame,
)
from vehicle_dynamics_monitor.vesc_slcan import VescSlcanInterface


class VehicleDynamicsNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_dynamics_node")
        self._declare_parameters()

        self.input_mode = str(self.get_parameter("input_mode").value).lower()
        self.yaw_rate_source = str(
            self.get_parameter("yaw_rate_source").value
        ).lower()
        if self.input_mode not in {"ros_topic", "socketcan", "slcan"}:
            raise ValueError("input_mode must be ros_topic, socketcan, or slcan")
        if self.yaw_rate_source not in {"auto", "imu", "steering"}:
            raise ValueError("yaw_rate_source must be auto, imu, or steering")

        self.publish_rate_hz = self._positive_parameter("publish_rate_hz")
        self.status_log_rate_hz = self._nonnegative_parameter(
            "status_log_rate_hz"
        )
        self.diagnostic_rate_hz = self._positive_parameter(
            "diagnostic_rate_hz"
        )
        self.vehicle_frame_id = str(
            self.get_parameter("vehicle_frame_id").value
        )
        self.imu_yaw_axis = str(
            self.get_parameter("imu_yaw_axis").value
        ).lower()
        if self.imu_yaw_axis not in {"x", "y", "z"}:
            raise ValueError("imu_yaw_axis must be x, y, or z")
        self.imu_yaw_rate_sign = float(
            self.get_parameter("imu_yaw_rate_sign").value
        )
        self.imu_yaw_rate_bias_radps = float(
            self.get_parameter("imu_yaw_rate_bias_radps").value
        )
        if not math.isfinite(self.imu_yaw_rate_sign) or abs(
            self.imu_yaw_rate_sign
        ) < 1.0e-12:
            raise ValueError("imu_yaw_rate_sign must be finite and non-zero")
        if not math.isfinite(self.imu_yaw_rate_bias_radps):
            raise ValueError("imu_yaw_rate_bias_radps must be finite")

        self.estimator = VehicleDynamicsEstimator(self._estimator_config())
        self._latest_telemetry: dict[str, int | float] = {}
        self._connection_status: bool | None = None
        self._commanded_duty: float | None = None
        self._last_status_log_sec = 0.0
        self._last_diagnostic_sec = 0.0
        self._last_can_open_attempt_sec = 0.0
        self._can_frames_received = 0
        self._vesc_frames_decoded = 0
        self._can_receiver: SocketCanReceiver | VescSlcanInterface | None = None
        self._can_controller_id = int(
            self.get_parameter("can_controller_id").value
        )
        if not 0 <= self._can_controller_id <= 255:
            raise ValueError("can_controller_id must be in [0, 255]")

        self._create_publishers()
        self._create_subscriptions()

        if self.input_mode == "socketcan":
            self._can_receiver = SocketCanReceiver(
                str(self.get_parameter("can_interface").value)
            )
            self._try_open_can()
        elif self.input_mode == "slcan":
            self._can_receiver = VescSlcanInterface(
                channel=str(self.get_parameter("slcan_channel").value),
                bitrate=int(self.get_parameter("slcan_bitrate").value),
            )
            self._try_open_can()

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._on_publish_timer
        )
        self.get_logger().info(
            "Vehicle dynamics monitor ready | "
            f"input={self.input_mode} | yaw_source={self.yaw_rate_source} | "
            f"gear_ratio={self.estimator.total_gear_ratio:.6f} | "
            "acceleration_frame=" + self.vehicle_frame_id
        )
        if self.input_mode == "ros_topic":
            self.get_logger().info(
                "Using VESC bridge telemetry topic. Set input_mode:=slcan for "
                "CANable 2 or input_mode:=socketcan for a kernel CAN interface."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_mode", "ros_topic")
        # Steering servo positions are not tire angles until the installed
        # linkage is measured. Use gyro-only lateral dynamics by default.
        self.declare_parameter("yaw_rate_source", "imu")
        self.declare_parameter("publish_rate_hz", 80.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("diagnostic_rate_hz", 2.0)
        self.declare_parameter("vehicle_frame_id", "base_link")

        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("imu_topic", "/camera/imu")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("commanded_duty_topic", "/manual/current_duty")
        self.declare_parameter(
            "acceleration_topic", "/vehicle/dynamics/acceleration"
        )
        self.declare_parameter("speed_topic", "/vehicle/dynamics/speed_mps")
        self.declare_parameter(
            "longitudinal_acceleration_topic",
            "/vehicle/dynamics/longitudinal_acceleration_mps2",
        )
        self.declare_parameter(
            "lateral_acceleration_topic",
            "/vehicle/dynamics/lateral_acceleration_mps2",
        )
        self.declare_parameter(
            "yaw_rate_topic", "/vehicle/dynamics/yaw_rate_radps"
        )
        self.declare_parameter(
            "motor_rpm_topic", "/vehicle/dynamics/motor_rpm"
        )
        self.declare_parameter("wheel_rpm_topic", "/vehicle/dynamics/wheel_rpm")
        self.declare_parameter(
            "motor_current_topic", "/vehicle/dynamics/motor_current_a"
        )
        self.declare_parameter(
            "input_current_topic", "/vehicle/dynamics/input_current_a"
        )
        self.declare_parameter(
            "input_voltage_topic", "/vehicle/dynamics/input_voltage_v"
        )
        self.declare_parameter(
            "duty_cycle_topic", "/vehicle/dynamics/duty_cycle"
        )
        self.declare_parameter(
            "fet_temperature_topic", "/vehicle/dynamics/fet_temperature_c"
        )
        self.declare_parameter(
            "motor_temperature_topic",
            "/vehicle/dynamics/motor_temperature_c",
        )
        self.declare_parameter(
            "diagnostics_topic", "/vehicle/dynamics/diagnostics"
        )

        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("can_controller_id", 0)
        self.declare_parameter("slcan_channel", "/dev/ttyACM0")
        self.declare_parameter("slcan_bitrate", 500_000)
        # /camera/imu uses camera optical coordinates: vehicle up is normally -Y.
        self.declare_parameter("imu_yaw_axis", "y")
        self.declare_parameter("imu_yaw_rate_sign", -1.0)
        self.declare_parameter("imu_yaw_rate_bias_radps", 0.0)

        defaults = EstimatorConfig()
        for name in (
            "wheel_diameter_m",
            "motor_pole_pairs",
            "motor_pinion_teeth",
            "spur_gear_teeth",
            "differential_pinion_teeth",
            "differential_ring_teeth",
            "erpm_direction_sign",
            "speed_scale_correction",
            "speed_deadband_mps",
            "speed_filter_time_constant_sec",
            "acceleration_filter_time_constant_sec",
            "maximum_speed_mps",
            "maximum_longitudinal_acceleration_mps2",
            "maximum_lateral_acceleration_mps2",
            "maximum_yaw_rate_radps",
            "maximum_sample_interval_sec",
            "sample_timeout_sec",
            "imu_timeout_sec",
            "steering_timeout_sec",
            "wheelbase_m",
            "maximum_left_steering_angle_deg",
            "maximum_right_steering_angle_deg",
            "servo_left",
            "servo_center",
            "servo_right",
        ):
            self.declare_parameter(name, getattr(defaults, name))

    def _estimator_config(self) -> EstimatorConfig:
        values: dict[str, Any] = {}
        for field_name in EstimatorConfig.__dataclass_fields__:
            values[field_name] = self.get_parameter(field_name).value
        return EstimatorConfig(**values)

    def _create_publishers(self) -> None:
        self.acceleration_pub = self.create_publisher(
            Vector3Stamped,
            str(self.get_parameter("acceleration_topic").value),
            qos_profile_sensor_data,
        )
        self.speed_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("speed_topic").value),
            qos_profile_sensor_data,
        )
        self.longitudinal_acceleration_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("longitudinal_acceleration_topic").value),
            qos_profile_sensor_data,
        )
        self.lateral_acceleration_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("lateral_acceleration_topic").value),
            qos_profile_sensor_data,
        )
        self.yaw_rate_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("yaw_rate_topic").value),
            qos_profile_sensor_data,
        )
        self.motor_rpm_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("motor_rpm_topic").value),
            qos_profile_sensor_data,
        )
        self.wheel_rpm_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("wheel_rpm_topic").value),
            qos_profile_sensor_data,
        )
        telemetry_topics = {
            "motor_current_a": "motor_current_topic",
            "input_current_a": "input_current_topic",
            "input_voltage_v": "input_voltage_topic",
            "duty_cycle": "duty_cycle_topic",
            "fet_temperature_c": "fet_temperature_topic",
            "motor_temperature_c": "motor_temperature_topic",
        }
        self.telemetry_publishers = {
            value_name: self.create_publisher(
                Float32,
                str(self.get_parameter(topic_parameter).value),
                qos_profile_sensor_data,
            )
            for value_name, topic_parameter in telemetry_topics.items()
        }
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )

    def _create_subscriptions(self) -> None:
        self.imu_sub = self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.servo_sub = self.create_subscription(
            Float32,
            str(self.get_parameter("servo_position_topic").value),
            self._on_servo_position,
            qos_profile_sensor_data,
        )
        self.commanded_duty_sub = self.create_subscription(
            Float32,
            str(self.get_parameter("commanded_duty_topic").value),
            self._on_commanded_duty,
            qos_profile_sensor_data,
        )
        self.measured_erpm_sub = None
        self.connection_sub = None
        if self.input_mode == "ros_topic":
            self.measured_erpm_sub = self.create_subscription(
                Int32,
                str(self.get_parameter("measured_erpm_topic").value),
                self._on_measured_erpm,
                qos_profile_sensor_data,
            )
            connection_qos = QoSProfile(depth=1)
            connection_qos.reliability = ReliabilityPolicy.RELIABLE
            connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.connection_sub = self.create_subscription(
                Bool,
                str(self.get_parameter("connection_status_topic").value),
                self._on_connection_status,
                connection_qos,
            )

    def _on_measured_erpm(self, message: Int32) -> None:
        now_sec = time.monotonic()
        self._latest_telemetry["measured_erpm"] = int(message.data)
        self.estimator.update_erpm(int(message.data), now_sec)

    def _on_connection_status(self, message: Bool) -> None:
        self._connection_status = bool(message.data)

    def _on_imu(self, message: Imu) -> None:
        axis_value = getattr(message.angular_velocity, self.imu_yaw_axis)
        yaw_rate_radps = self.imu_yaw_rate_sign * (
            float(axis_value) - self.imu_yaw_rate_bias_radps
        )
        self.estimator.update_imu_yaw_rate(yaw_rate_radps, time.monotonic())

    def _on_servo_position(self, message: Float32) -> None:
        self.estimator.update_servo_position(float(message.data), time.monotonic())

    def _on_commanded_duty(self, message: Float32) -> None:
        self._commanded_duty = float(message.data)

    def _try_open_can(self) -> None:
        if self._can_receiver is None or self._can_receiver.is_open:
            return
        now_sec = time.monotonic()
        if now_sec - self._last_can_open_attempt_sec < 1.0:
            return
        self._last_can_open_attempt_sec = now_sec
        try:
            self._can_receiver.open()
            if isinstance(self._can_receiver, VescSlcanInterface):
                self.get_logger().info(
                    f"Opened python-can SLCAN {self._can_receiver.channel} "
                    f"at {self._can_receiver.bitrate} bit/s (receive-only) for "
                    f"VESC controller ID {self._can_controller_id}."
                )
            else:
                self.get_logger().info(
                    "Opened receive-only SocketCAN interface "
                    f"{self._can_receiver.interface} for VESC controller "
                    f"ID {self._can_controller_id}."
                )
        except Exception as exc:
            self.get_logger().error(
                f"Cannot open {self.input_mode} CAN input: {exc}",
                throttle_duration_sec=5.0,
            )

    def _drain_can(self) -> None:
        if self._can_receiver is None:
            return
        self._try_open_can()
        if not self._can_receiver.is_open:
            return
        try:
            frames = self._can_receiver.drain()
        except Exception as exc:
            self.get_logger().error(
                f"{self.input_mode} CAN receive failed: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        now_sec = time.monotonic()
        self._can_frames_received += len(frames)
        for can_id, data in frames:
            update = decode_vesc_can_frame(
                can_id, data, self._can_controller_id
            )
            if update is None:
                continue
            self._vesc_frames_decoded += 1
            self._latest_telemetry.update(update.values)
            for name, publisher in self.telemetry_publishers.items():
                if name in update.values:
                    publisher.publish(Float32(data=float(update.values[name])))
            if "measured_erpm" in update.values:
                self.estimator.update_erpm(
                    int(update.values["measured_erpm"]), now_sec
                )

    def _on_publish_timer(self) -> None:
        if self.input_mode in {"socketcan", "slcan"}:
            self._drain_can()
        now_sec = time.monotonic()
        sample = self.estimator.sample(now_sec, self.yaw_rate_source)
        if sample.valid:
            self._publish_sample(sample)
        if now_sec - self._last_diagnostic_sec >= 1.0 / self.diagnostic_rate_hz:
            self._publish_diagnostics(sample)
            self._last_diagnostic_sec = now_sec
        if (
            self.status_log_rate_hz > 0.0
            and now_sec - self._last_status_log_sec
            >= 1.0 / self.status_log_rate_hz
        ):
            self._log_status(sample)
            self._last_status_log_sec = now_sec

    def _publish_sample(self, sample: DynamicsSample) -> None:
        acceleration = Vector3Stamped()
        acceleration.header.stamp = self.get_clock().now().to_msg()
        acceleration.header.frame_id = self.vehicle_frame_id
        acceleration.vector.x = sample.longitudinal_acceleration_mps2
        acceleration.vector.y = sample.lateral_acceleration_mps2
        acceleration.vector.z = 0.0
        self.acceleration_pub.publish(acceleration)
        self.speed_pub.publish(Float32(data=sample.speed_mps))
        self.longitudinal_acceleration_pub.publish(
            Float32(data=sample.longitudinal_acceleration_mps2)
        )
        self.lateral_acceleration_pub.publish(
            Float32(data=sample.lateral_acceleration_mps2)
        )
        self.yaw_rate_pub.publish(Float32(data=sample.yaw_rate_radps))
        self.motor_rpm_pub.publish(Float32(data=sample.motor_rpm))
        self.wheel_rpm_pub.publish(Float32(data=sample.wheel_rpm))

    def _publish_diagnostics(self, sample: DynamicsSample) -> None:
        connected = self._is_connected(sample)
        if not sample.valid:
            level = DiagnosticStatus.ERROR
            message = "ERPM telemetry missing or stale"
        elif not connected:
            level = DiagnosticStatus.ERROR
            message = "VESC connection is not healthy"
        elif sample.yaw_rate_source == "none":
            level = DiagnosticStatus.WARN
            message = "No fresh IMU yaw rate or steering fallback"
        else:
            level = DiagnosticStatus.OK
            message = "Vehicle dynamics valid"

        status = DiagnosticStatus()
        status.level = level
        status.name = "vehicle_dynamics_monitor"
        if self.input_mode == "ros_topic":
            status.hardware_id = "vesc-bridge"
        else:
            status.hardware_id = (
                f"vesc-{self.input_mode}-{self._can_controller_id}"
            )
        status.message = message
        values: dict[str, Any] = {
            "input_mode": self.input_mode,
            "connected": connected,
            "valid": sample.valid,
            "yaw_rate_source": sample.yaw_rate_source,
            "measured_erpm": sample.measured_erpm,
            "motor_rpm": f"{sample.motor_rpm:.3f}",
            "wheel_rpm": f"{sample.wheel_rpm:.3f}",
            "speed_mps": f"{sample.speed_mps:.4f}",
            "longitudinal_acceleration_mps2": (
                f"{sample.longitudinal_acceleration_mps2:.4f}"
            ),
            "lateral_acceleration_mps2": (
                f"{sample.lateral_acceleration_mps2:.4f}"
            ),
            "yaw_rate_radps": f"{sample.yaw_rate_radps:.4f}",
            "steering_angle_deg": f"{math.degrees(sample.steering_angle_rad):.3f}",
            "can_frames_received": self._can_frames_received,
            "vesc_frames_decoded": self._vesc_frames_decoded,
        }
        if self._commanded_duty is not None:
            values["commanded_duty"] = f"{self._commanded_duty:.5f}"
        values.update(self._latest_telemetry)
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in values.items()
        ]
        message_array = DiagnosticArray()
        message_array.header.stamp = self.get_clock().now().to_msg()
        message_array.status = [status]
        self.diagnostics_pub.publish(message_array)

    def _log_status(self, sample: DynamicsSample) -> None:
        if not sample.valid:
            self.get_logger().warn(
                "Vehicle dynamics | waiting for fresh "
                f"{self.input_mode} ERPM telemetry | "
                f"CAN frames={self._can_frames_received} | "
                f"VESC frames={self._vesc_frames_decoded}",
                throttle_duration_sec=1.0,
            )
            return
        optional_fields = []
        for key, unit in (
            ("input_voltage_v", "V"),
            ("motor_current_a", "A motor"),
            ("input_current_a", "A input"),
            ("fet_temperature_c", "C FET"),
            ("motor_temperature_c", "C motor"),
            ("duty_cycle", "duty"),
        ):
            if key in self._latest_telemetry:
                optional_fields.append(
                    f"{key}={float(self._latest_telemetry[key]):.2f}{unit}"
                )
        optional_text = " | " + " | ".join(optional_fields) if optional_fields else ""
        self.get_logger().info(
            f"Vehicle dynamics | ERPM={sample.measured_erpm} | "
            f"motor={sample.motor_rpm:.1f}rpm | wheel={sample.wheel_rpm:.1f}rpm | "
            f"speed={sample.speed_mps:.3f}m/s | "
            f"a_long={sample.longitudinal_acceleration_mps2:+.3f}m/s^2 | "
            f"a_lat={sample.lateral_acceleration_mps2:+.3f}m/s^2 | "
            f"yaw={sample.yaw_rate_radps:+.3f}rad/s({sample.yaw_rate_source})"
            f"{optional_text}"
        )

    def _is_connected(self, sample: DynamicsSample) -> bool:
        if self.input_mode in {"socketcan", "slcan"}:
            return sample.valid
        return self._connection_status is True

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than zero")
        return value

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def destroy_node(self) -> bool:
        if self._can_receiver is not None:
            try:
                self._can_receiver.close()
            except Exception as exc:
                self.get_logger().error(
                    f"Could not close {self.input_mode} CAN input: {exc}"
                )
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: VehicleDynamicsNode | None = None
    try:
        node = VehicleDynamicsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
