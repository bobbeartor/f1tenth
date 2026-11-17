#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, Int32, String

from vesc_bridge.vesc_driver import (
    VescCommandIds,
    VescDriver,
    VescDriverError,
    VescScales,
)
from vesc_bridge.vesc_io_worker import VescIoResult, VescIoWorker


class VescBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vesc_bridge_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.02)
        self.declare_parameter("write_timeout", 0.05)
        self.declare_parameter("serial_exclusive", True)
        self.declare_parameter("startup_delay", 0.2)
        self.declare_parameter("connect_on_startup", True)
        self.declare_parameter("verify_firmware_on_startup", True)

        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("brake_current_topic", "/vesc/brake_current")
        self.declare_parameter("erpm_topic", "/vesc/erpm")
        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("min_duty", -1.0)
        self.declare_parameter("max_duty", 1.0)
        self.declare_parameter("max_brake_current_amps", 8.0)
        self.declare_parameter("min_erpm", -100000)
        self.declare_parameter("max_erpm", 100000)
        self.declare_parameter("servo_min", 0.0)
        self.declare_parameter("servo_max", 1.0)
        self.declare_parameter("command_timeout", 0.3)
        self.declare_parameter("log_commands", False)
        self.declare_parameter("telemetry_rate_hz", 80.0)
        self.declare_parameter("telemetry_log_rate_hz", 2.0)
        self.declare_parameter("telemetry_timeout_sec", 0.15)

        self.declare_parameter("packet.comm_get_firmware_version", 0)
        self.declare_parameter("packet.comm_get_values_selective", 50)
        self.declare_parameter("packet.comm_set_duty", 5)
        self.declare_parameter("packet.comm_set_current_brake", 7)
        self.declare_parameter("packet.comm_set_erpm", 8)
        self.declare_parameter("packet.comm_set_servo_pos", 12)
        self.declare_parameter("packet.duty_scale", 100000)
        self.declare_parameter("packet.brake_current_scale", 1000)
        self.declare_parameter("packet.servo_scale", 1000)

        duty_topic = str(self.get_parameter("duty_topic").value)
        brake_current_topic = str(
            self.get_parameter("brake_current_topic").value
        )
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
        self.max_brake_current_amps = float(
            self.get_parameter("max_brake_current_amps").value
        )
        if (
            not math.isfinite(self.max_brake_current_amps) or
            self.max_brake_current_amps <= 0.0
        ):
            raise ValueError("max_brake_current_amps must be positive")
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
        self.telemetry_timeout_sec = float(self.get_parameter("telemetry_timeout_sec").value)
        if not math.isfinite(self.telemetry_timeout_sec) or self.telemetry_timeout_sec <= 0.0:
            raise ValueError("telemetry_timeout_sec must be positive")
        self._last_valid_telemetry_sec: float | None = None

        self._last_duty_time: Time | None = None
        self._last_duty_command = 0.0
        self._last_brake_time: Time | None = None
        self._last_brake_current = 0.0
        self._last_erpm_time: Time | None = None
        self._last_erpm_command = 0
        self._last_command_mode: str | None = None
        self._connection_status: bool | None = None
        self._connection_verified = False
        self._last_link_fault = ""
        self._telemetry_stats_started_sec = time.monotonic()
        self._telemetry_samples_interval = 0
        self._telemetry_failures_interval = 0
        self._telemetry_rtt_sum_sec = 0.0
        self._telemetry_rtt_max_sec = 0.0

        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.connection_pub = self.create_publisher(
            Bool,
            connection_status_topic,
            connection_qos,
        )
        self.connection_diagnostics_pub = self.create_publisher(
            String, connection_status_topic.rstrip('/') + '/diagnostics', connection_qos)
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
            exclusive=bool(self.get_parameter("serial_exclusive").value),
            command_ids=VescCommandIds(
                get_firmware_version=int(
                    self.get_parameter("packet.comm_get_firmware_version").value
                ),
                get_values_selective=int(
                    self.get_parameter("packet.comm_get_values_selective").value
                ),
                set_duty=int(self.get_parameter("packet.comm_set_duty").value),
                set_current_brake=int(
                    self.get_parameter(
                        "packet.comm_set_current_brake"
                    ).value
                ),
                set_erpm=int(self.get_parameter("packet.comm_set_erpm").value),
                set_servo_pos=int(
                    self.get_parameter("packet.comm_set_servo_pos").value
                ),
            ),
            scales=VescScales(
                duty=int(self.get_parameter("packet.duty_scale").value),
                brake_current=int(
                    self.get_parameter("packet.brake_current_scale").value
                ),
                servo=int(self.get_parameter("packet.servo_scale").value),
            ),
        )

        if bool(self.get_parameter("connect_on_startup").value):
            self._try_open_driver()

        self.io_worker = VescIoWorker(self.driver, self.telemetry_rate_hz)
        self._telemetry_started_sec = time.monotonic()
        self.io_worker.start()

        # Duty/brake/ERPM/servo commands are desired current states. If serial
        # I/O pauses the executor, discard superseded commands instead of
        # replaying them after the operator has released or changed an input.
        latest_command_qos = QoSProfile(depth=1)
        latest_command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_command_qos.durability = DurabilityPolicy.VOLATILE
        self.duty_sub = self.create_subscription(
            Float32,
            duty_topic,
            self._on_duty,
            latest_command_qos,
        )
        self.brake_current_sub = self.create_subscription(
            Float32,
            brake_current_topic,
            self._on_brake_current,
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
        # UART runs independently; this short callback only transfers completed
        # results into ROS publishers and logging without ever blocking control.
        self.io_result_timer = self.create_timer(0.0025, self._drain_io_results)

        self.get_logger().info(
            f"VESC bridge ready. duty_topic={duty_topic}, "
            f"brake_current_topic={brake_current_topic} "
            f"(max={self.max_brake_current_amps:.1f}A), "
            f"erpm_topic={erpm_topic}, "
            f"measured_erpm_topic={measured_erpm_topic}, "
            f"servo_position_topic={servo_topic}, "
            f"connection_status_topic={connection_status_topic}, "
            f"telemetry={self.telemetry_rate_hz:.1f}Hz via dedicated UART worker | "
            f"write_timeout={self.driver.write_timeout*1000:.0f}ms exclusive={self.driver.exclusive}"
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
            self._publish_link_fault(str(exc))
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
        self.io_worker.submit_duty(target_duty)

    def _on_brake_current(self, msg: Float32) -> None:
        target_brake_current = self._clamp_float(
            float(msg.data),
            0.0,
            self.max_brake_current_amps,
        )
        self._last_brake_time = self.get_clock().now()
        self._last_brake_current = target_brake_current
        self._last_command_mode = "brake"
        self.io_worker.submit_brake_current(target_brake_current)

    def _on_erpm(self, msg: Int32) -> None:
        target_erpm = self._clamp_int(
            int(msg.data),
            self.min_erpm,
            self.max_erpm,
        )
        self._last_erpm_time = self.get_clock().now()
        self._last_erpm_command = target_erpm
        self._last_command_mode = "erpm"
        self.io_worker.submit_erpm(target_erpm)

    def _on_servo_position(self, msg: Float32) -> None:
        position = self._clamp_float(float(msg.data), self.servo_min, self.servo_max)
        self.io_worker.submit_servo(position)

    def _drain_io_results(self) -> None:
        for result in self.io_worker.drain_results():
            self._handle_io_result(result)
        # Also expire a silent/stalled worker; failed replies need not arrive
        # for the freshness deadline to take effect.
        if self.telemetry_rate_hz > 0.0 and self._connection_verified:
            anchor = self._last_valid_telemetry_sec
            if anchor is None:
                anchor = self._telemetry_started_sec
            if time.monotonic() - anchor > self.telemetry_timeout_sec:
                self._publish_link_fault('UART telemetry expired')
                self._connection_verified = False
                self._set_connection_status(False)
                self.get_logger().warn("VESC telemetry expired; connection marked unavailable.")

    def _telemetry_is_fresh(self, sample_sec: float | None) -> bool:
        return sample_sec is not None and 0.0 <= time.monotonic() - sample_sec <= self.telemetry_timeout_sec

    def _handle_io_result(self, result: VescIoResult) -> None:
        if not result.success:
            retain_connection = (
                result.operation == "telemetry"
                and result.recoverable_response_error
                and self._connection_verified
                and self.driver.is_open
                and self._telemetry_is_fresh(self._last_valid_telemetry_sec)
            )
            if not retain_connection:
                self._publish_link_fault(result.error or 'UART I/O failure', result.operation)
                self._connection_verified = False
                self._set_connection_status(False)
            if result.operation == "telemetry":
                self._telemetry_failures_interval += 1
                self.get_logger().warn(
                    f"Discarded ERPM response (retaining link={retain_connection}): {result.error}",
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().error(
                    f"VESC {result.operation} command failed: {result.error}",
                    throttle_duration_sec=1.0,
                )
            return

        if result.operation != "telemetry":
            # A successful serial write does not prove the VESC received it.
            # Only a response-bearing transaction can restore a failed link.
            if self._connection_verified:
                self._set_connection_status(True)
            if self.log_commands:
                self.get_logger().info(
                    f"VESC serial write OK: "
                    f"{result.operation}={result.value}",
                    throttle_duration_sec=0.25,
                )
            return

        # Do not turn delayed queued results into apparently fresh measurements.
        # Use request start as a conservative bound on measurement age.
        if not self._telemetry_is_fresh(result.started_at_sec):
            self._telemetry_failures_interval += 1
            return
        self._last_valid_telemetry_sec = result.started_at_sec
        self._connection_verified = True
        self._set_connection_status(True)
        measured_erpm = int(result.value)
        self.measured_erpm_pub.publish(Int32(data=measured_erpm))
        self._telemetry_samples_interval += 1
        self._telemetry_rtt_sum_sec += result.round_trip_sec
        self._telemetry_rtt_max_sec = max(
            self._telemetry_rtt_max_sec,
            result.round_trip_sec,
        )
        self._report_telemetry_if_due(measured_erpm)

    def _report_telemetry_if_due(self, measured_erpm: int) -> None:
        if self.telemetry_log_rate_hz <= 0.0:
            return

        now_sec = time.monotonic()
        elapsed_sec = now_sec - self._telemetry_stats_started_sec
        if elapsed_sec < 1.0 / self.telemetry_log_rate_hz:
            return

        sample_count = self._telemetry_samples_interval
        actual_hz = sample_count / elapsed_sec if elapsed_sec > 0.0 else 0.0
        average_rtt_ms = (
            1000.0 * self._telemetry_rtt_sum_sec / sample_count
            if sample_count > 0
            else 0.0
        )
        if self._last_command_mode == "erpm":
            target_text = f"target_erpm={self._last_erpm_command}"
        elif self._last_command_mode == "brake":
            target_text = f"brake_current={self._last_brake_current:.2f}A"
        else:
            target_text = f"target_duty={self._last_duty_command:.5f}"
        writes = self.driver.write_stats()
        write_average_ms = 1000.*writes.duration_sum_sec/max(1, writes.attempts)
        self.get_logger().info(
            f"VESC | {target_text} | measured_erpm={measured_erpm} | "
            f"telemetry={actual_hz:.1f}/{self.telemetry_rate_hz:.1f}Hz | "
            f"uart_rtt={average_rtt_ms:.2f}/"
            f"{1000.0 * self._telemetry_rtt_max_sec:.2f}ms avg/max | "
            f"failures={self._telemetry_failures_interval} | "
            f"uart_write={write_average_ms:.2f}/{1000.*writes.duration_max_sec:.2f}ms avg/max lifetime | "
            f"write_attempts/failures/timeouts={writes.attempts}/{writes.failures}/{writes.timeouts} "
            f"bytes={writes.bytes_sent}"
        )
        self._telemetry_stats_started_sec = now_sec
        self._telemetry_samples_interval = 0
        self._telemetry_failures_interval = 0
        self._telemetry_rtt_sum_sec = 0.0
        self._telemetry_rtt_max_sec = 0.0

    def _check_command_timeout(self) -> None:
        if self._last_command_mode == "duty":
            last_time = self._last_duty_time
            command_is_zero = self._last_duty_command == 0.0
        elif self._last_command_mode == "brake":
            last_time = self._last_brake_time
            command_is_zero = self._last_brake_current == 0.0
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
            self.io_worker.submit_duty(0.0)
        elif self._last_command_mode == "brake":
            self.get_logger().warn(
                f"Brake-current command timeout ({elapsed_sec:.2f}s). "
                "Releasing brake current.",
                throttle_duration_sec=1.0,
            )
            self._last_brake_time = self.get_clock().now()
            self._last_brake_current = 0.0
            self.io_worker.submit_brake_current(0.0)
        else:
            self.get_logger().warn(
                f"ERPM command timeout ({elapsed_sec:.2f}s). Sending ERPM 0.",
                throttle_duration_sec=1.0,
            )
            self._last_erpm_time = self.get_clock().now()
            self._last_erpm_command = 0
            self.io_worker.submit_erpm(0)

    def _publish_link_fault(self, error: str, operation: str = 'startup') -> None:
        lower = error.lower()
        reason = ('UART write timeout' if 'write' in lower and 'timeout' in lower else
                  'UART port already in use' if 'already open' in lower else
                  'UART telemetry expired' if 'telemetry expired' in lower else
                  'UART I/O failure')
        # Preserve the initiating fault through retries until a verified recovery.
        if not self._last_link_fault or self._connection_verified or self._connection_status is True:
            self._last_link_fault = reason
        self.connection_diagnostics_pub.publish(String(data='FAULT | ' + self._last_link_fault))
        stats = self.driver.write_stats()
        self.get_logger().warn(
            f'UART fault | operation={operation} | port={self.driver.port} | '
            f'last_write={stats.last_duration_sec*1000:.2f}ms | '
            f'write_failures/timeouts={stats.failures}/{stats.timeouts} | {error}',
            throttle_duration_sec=1.0)

    def _set_connection_status(self, connected: bool) -> None:
        if self._connection_status == connected:
            return

        self._connection_status = connected
        self._publish_connection_status()
        if connected and self._last_link_fault:
            self.connection_diagnostics_pub.publish(
                String(data='RECOVERED | ' + self._last_link_fault))

    def _publish_connection_status(self) -> None:
        connected = bool(self._connection_status and self.driver.is_open)
        self.connection_pub.publish(Bool(data=connected))

    def destroy_node(self) -> bool:
        stopped = self.io_worker.stop(send_duty_zero=True, timeout_sec=1.0)
        if not stopped:
            self.get_logger().warn(
                "VESC UART worker did not stop within 1.0s during shutdown."
            )
        self._connection_verified = False
        self._set_connection_status(False)

        return super().destroy_node()

    @staticmethod
    def _clamp_int(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _clamp_float(value: float, minimum: float, maximum: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VescBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
