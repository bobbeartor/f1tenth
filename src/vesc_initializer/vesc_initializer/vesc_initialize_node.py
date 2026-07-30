#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32

from vesc_initializer.vesc_driver import (
    VescCommandIds,
    VescDriver,
    VescDriverError,
    VescScales,
)


class VescInitializeNode(Node):
    def __init__(self) -> None:
        super().__init__("vesc_initialize_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.1)
        self.declare_parameter("write_timeout", 2.0)
        self.declare_parameter("startup_delay", 0.2)
        self.declare_parameter("connect_on_startup", True)
        self.declare_parameter("verify_firmware_on_startup", True)

        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("min_duty", -1.0)
        self.declare_parameter("max_duty", 1.0)
        self.declare_parameter("min_erpm", -100000)
        self.declare_parameter("max_erpm", 100000)
        self.declare_parameter("servo_min", 0.0)
        self.declare_parameter("servo_max", 1.0)
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("log_commands", False)
        self.declare_parameter("telemetry_rate_hz", 10.0)
        self.declare_parameter("telemetry_log_rate_hz", 2.0)

        self.declare_parameter("packet.comm_get_firmware_version", 0)
        self.declare_parameter("packet.comm_get_values_selective", 50)
        self.declare_parameter("packet.comm_set_duty", 5)
        self.declare_parameter("packet.comm_set_erpm", 8)
        self.declare_parameter("packet.comm_set_servo_pos", 12)
        self.declare_parameter("packet.duty_scale", 100000)
        self.declare_parameter("packet.servo_scale", 1000)

        duty_topic = str(self.get_parameter("duty_topic").value)
        erpm_topic = str(self.get_parameter("erpm_topic").value)
        measured_erpm_topic = str(
            self.get_parameter("measured_erpm_topic").value
        )
        servo_topic = str(self.get_parameter("servo_position_topic").value)
        connection_status_topic = str(
            self.get_parameter("connection_status_topic").value
        )

        self.min_duty = float(self.get_parameter("min_duty").value)
        self.max_duty = float(self.get_parameter("max_duty").value)
        self.min_erpm = int(self.get_parameter("min_erpm").value)
        self.max_erpm = int(self.get_parameter("max_erpm").value)
        self.servo_min = float(self.get_parameter("servo_min").value)
        self.servo_max = float(self.get_parameter("servo_max").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.log_commands = bool(self.get_parameter("log_commands").value)
        self.telemetry_rate_hz = max(
            0.0,
            float(self.get_parameter("telemetry_rate_hz").value),
        )
        self.telemetry_log_rate_hz = max(
            0.0,
            float(self.get_parameter("telemetry_log_rate_hz").value),
        )
        self.verify_firmware_on_startup = bool(
            self.get_parameter("verify_firmware_on_startup").value
        )

        self._last_duty_time: Time | None = None
        self._last_duty_command = 0.0
        self._last_erpm_time: Time | None = None
        self._last_erpm_command = 0
        self._last_command_mode: str | None = None
        self._last_telemetry_log_time: Time | None = None
        self._connection_status: bool | None = None
        self._connection_verified = False

        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_pub = self.create_publisher(
            Bool,
            connection_status_topic,
            connection_qos,
        )
        self.measured_erpm_pub = self.create_publisher(
            Int32,
            measured_erpm_topic,
            10,
        )

        self.driver = VescDriver(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            timeout=float(self.get_parameter("serial_timeout").value),
            write_timeout=float(self.get_parameter("write_timeout").value),
            startup_delay=float(self.get_parameter("startup_delay").value),
            command_ids=VescCommandIds(
                get_firmware_version=int(
                    self.get_parameter("packet.comm_get_firmware_version").value
                ),
                get_values_selective=int(
                    self.get_parameter("packet.comm_get_values_selective").value
                ),
                set_duty=int(self.get_parameter("packet.comm_set_duty").value),
                set_erpm=int(self.get_parameter("packet.comm_set_erpm").value),
                set_servo_pos=int(
                    self.get_parameter("packet.comm_set_servo_pos").value
                ),
            ),
            scales=VescScales(
                duty=int(self.get_parameter("packet.duty_scale").value),
                servo=int(self.get_parameter("packet.servo_scale").value),
            ),
        )

        if bool(self.get_parameter("connect_on_startup").value):
            self._try_open_driver()

        # Duty/ERPM/servo commands are desired current states. If serial I/O
        # pauses the executor, discard superseded commands instead of replaying
        # them after the operator has released or changed an input.
        latest_command_qos = QoSProfile(depth=1)
        latest_command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_command_qos.durability = DurabilityPolicy.VOLATILE
        self.duty_sub = self.create_subscription(
            Float32,
            duty_topic,
            self._on_duty,
            latest_command_qos,
        )
        self.erpm_sub = self.create_subscription(
            Int32,
            erpm_topic,
            self._on_erpm,
            latest_command_qos,
        )
        self.servo_sub = self.create_subscription(
            Float32,
            servo_topic,
            self._on_servo_position,
            latest_command_qos,
        )

        if self.command_timeout > 0.0:
            self.timeout_timer = self.create_timer(0.05, self._check_command_timeout)
        else:
            self.timeout_timer = None
        self.connection_status_timer = self.create_timer(
            1.0,
            self._publish_connection_status,
        )
        if self.telemetry_rate_hz > 0.0:
            self.telemetry_timer = self.create_timer(
                1.0 / self.telemetry_rate_hz,
                self._poll_telemetry,
            )
        else:
            self.telemetry_timer = None

        self.get_logger().info(
            f"VESC initializer ready. duty_topic={duty_topic}, "
            f"erpm_topic={erpm_topic}, "
            f"measured_erpm_topic={measured_erpm_topic}, "
            f"servo_position_topic={servo_topic}, "
            f"connection_status_topic={connection_status_topic}"
        )

    def _try_open_driver(self) -> None:
        try:
            if self.verify_firmware_on_startup:
                firmware_major, firmware_minor = self.driver.get_firmware_version()
                firmware_text = f", firmware={firmware_major}.{firmware_minor}"
            else:
                self.driver.open()
                firmware_text = ""

            self.driver.set_duty(0.0)
            self._connection_verified = True
            self._set_connection_status(True)
            self.get_logger().info(
                "\n"
                "==================== VESC 연결 여부 ====================\n"
                f"VESC 연결완료: {self.driver.port}{firmware_text}\n"
                "펌웨어 응답 및 duty 0 시리얼 쓰기 확인완료\n"
                "========================================================"
            )
        except VescDriverError as exc:
            self._connection_verified = False
            self.driver.close()
            self._set_connection_status(False)
            self.get_logger().error(
                "\n"
                "==================== VESC 연결 여부 ====================\n"
                f"VESC 연결실패: {self.driver.port}\n"
                f"원인: {exc}\n"
                "========================================================"
            )

    def _on_duty(self, msg: Float32) -> None:
        target_duty = self._clamp_float(
            float(msg.data),
            self.min_duty,
            self.max_duty,
        )
        self._last_duty_time = self.get_clock().now()
        self._last_duty_command = target_duty
        self._last_command_mode = "duty"
        self._send_duty(target_duty)

    def _on_erpm(self, msg: Int32) -> None:
        target_erpm = self._clamp_int(
            int(msg.data),
            self.min_erpm,
            self.max_erpm,
        )
        self._last_erpm_time = self.get_clock().now()
        self._last_erpm_command = target_erpm
        self._last_command_mode = "erpm"
        self._send_erpm(target_erpm)

    def _on_servo_position(self, msg: Float32) -> None:
        position = self._clamp_float(float(msg.data), self.servo_min, self.servo_max)
        try:
            self.driver.set_servo_position(position)
            if self._connection_verified:
                self._set_connection_status(True)
            if self.log_commands:
                self.get_logger().info(
                    f"VESC serial write OK: servo_position={position:.3f}"
                )
        except VescDriverError as exc:
            self._connection_verified = False
            self.driver.close()
            self._set_connection_status(False)
            self.get_logger().error(
                f"Servo position command failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _poll_telemetry(self) -> None:
        try:
            measured_erpm = self.driver.get_measured_erpm()
        except VescDriverError as exc:
            self._connection_verified = False
            self._set_connection_status(False)
            self.get_logger().warn(
                f"Failed to read measured ERPM: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        self._connection_verified = True
        self._set_connection_status(True)
        self.measured_erpm_pub.publish(Int32(data=measured_erpm))

        if self.telemetry_log_rate_hz <= 0.0:
            return

        now = self.get_clock().now()
        if self._last_telemetry_log_time is not None:
            elapsed_sec = (
                now - self._last_telemetry_log_time
            ).nanoseconds / 1_000_000_000.0
            if elapsed_sec < 1.0 / self.telemetry_log_rate_hz:
                return

        self._last_telemetry_log_time = now
        if self._last_command_mode == "erpm":
            target_text = f"target_erpm={self._last_erpm_command}"
        else:
            target_text = f"target_duty={self._last_duty_command:.5f}"
        self.get_logger().info(
            f"VESC | {target_text} | measured_erpm={measured_erpm}"
        )

    def _check_command_timeout(self) -> None:
        if self._last_command_mode == "duty":
            last_time = self._last_duty_time
            command_is_zero = self._last_duty_command == 0.0
        elif self._last_command_mode == "erpm":
            last_time = self._last_erpm_time
            command_is_zero = self._last_erpm_command == 0
        else:
            return

        if last_time is None or command_is_zero:
            return

        elapsed_sec = (
            self.get_clock().now() - last_time
        ).nanoseconds / 1_000_000_000
        if elapsed_sec < self.command_timeout:
            return

        if self._last_command_mode == "duty":
            self.get_logger().warn(
                f"Duty command timeout ({elapsed_sec:.2f}s). Sending duty 0.",
                throttle_duration_sec=1.0,
            )
            self._last_duty_time = self.get_clock().now()
            self._last_duty_command = 0.0
            self._send_duty(0.0)
        else:
            self.get_logger().warn(
                f"ERPM command timeout ({elapsed_sec:.2f}s). Sending ERPM 0.",
                throttle_duration_sec=1.0,
            )
            self._last_erpm_time = self.get_clock().now()
            self._last_erpm_command = 0
            self._send_erpm(0)

    def _send_duty(self, target_duty: float) -> None:
        try:
            self.driver.set_duty(target_duty)
            if self._connection_verified:
                self._set_connection_status(True)
            if self.log_commands:
                self.get_logger().info(
                    f"VESC serial write OK: duty={target_duty:.5f}",
                    throttle_duration_sec=0.25,
                )
        except VescDriverError as exc:
            self._connection_verified = False
            self.driver.close()
            self._set_connection_status(False)
            self.get_logger().error(
                f"Duty command failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _send_erpm(self, target_erpm: int) -> None:
        try:
            self.driver.set_erpm(target_erpm)
            if self._connection_verified:
                self._set_connection_status(True)
            if self.log_commands:
                self.get_logger().info(
                    f"VESC serial write OK: erpm={target_erpm}",
                    throttle_duration_sec=0.25,
                )
        except VescDriverError as exc:
            self._connection_verified = False
            self.driver.close()
            self._set_connection_status(False)
            self.get_logger().error(
                f"ERPM command failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _set_connection_status(self, connected: bool) -> None:
        if self._connection_status == connected:
            return

        self._connection_status = connected
        self._publish_connection_status()

    def _publish_connection_status(self) -> None:
        connected = bool(self._connection_status and self.driver.is_open)
        self.connection_pub.publish(Bool(data=connected))

    def destroy_node(self) -> bool:
        try:
            if self.driver.is_open:
                self.driver.set_duty(0.0)
        except VescDriverError as exc:
            self.get_logger().warn(f"Failed to send duty 0 before shutdown: {exc}")
        finally:
            self.driver.close()
            self._set_connection_status(False)

        return super().destroy_node()

    @staticmethod
    def _clamp_int(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _clamp_float(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VescInitializeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
