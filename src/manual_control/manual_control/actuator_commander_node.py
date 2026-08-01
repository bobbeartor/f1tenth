#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String

from manual_control.duty_command_profile import (
    DutyCommandProfile,
    DutyProfileConfig,
)


class ActuatorCommanderNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_commander_node")

        self.declare_parameter("accelerator_topic", "/manual/accelerator")
        self.declare_parameter("brake_topic", "/manual/brake")
        self.declare_parameter("steering_topic", "/manual/steering")
        self.declare_parameter("gear_toggle_topic", "/manual/gear_toggle")
        self.declare_parameter("current_duty_topic", "/manual/current_duty")
        self.declare_parameter("gear_state_topic", "/manual/gear")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")

        self.declare_parameter("forward_max_duty", 0.10)
        self.declare_parameter("reverse_max_duty", 0.08)
        self.declare_parameter("start_duty", 0.05)
        self.declare_parameter("reverse_start_duty", 0.05)
        self.declare_parameter("acceleration_duty_per_sec", 0.03)
        self.declare_parameter("coast_deceleration_duty_per_sec", 0.005)
        self.declare_parameter("brake_duty_per_sec", 0.07)
        self.declare_parameter("control_rate_hz", 80.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("input_timeout_sec", 0.3)
        self.declare_parameter("immediate_stop_on_accelerator_release", True)

        self.declare_parameter("servo_left", 0.98)
        self.declare_parameter("servo_center", 0.46)
        self.declare_parameter("servo_right", 0.02)
        self.declare_parameter("pedal_deadzone", 0.01)
        self.declare_parameter("steering_deadzone", 0.04)

        accelerator_topic = str(self.get_parameter("accelerator_topic").value)
        brake_topic = str(self.get_parameter("brake_topic").value)
        steering_topic = str(self.get_parameter("steering_topic").value)
        gear_toggle_topic = str(self.get_parameter("gear_toggle_topic").value)
        current_duty_topic = str(self.get_parameter("current_duty_topic").value)
        gear_state_topic = str(self.get_parameter("gear_state_topic").value)
        duty_topic = str(self.get_parameter("duty_topic").value)
        servo_topic = str(self.get_parameter("servo_position_topic").value)

        self.control_rate_hz = max(
            1.0,
            float(self.get_parameter("control_rate_hz").value),
        )
        self.status_log_rate_hz = max(
            0.2,
            float(self.get_parameter("status_log_rate_hz").value),
        )
        self.input_timeout_sec = max(
            0.05,
            float(self.get_parameter("input_timeout_sec").value),
        )
        self.servo_left = float(self.get_parameter("servo_left").value)
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_right = float(self.get_parameter("servo_right").value)
        self.steering_deadzone = float(
            self.get_parameter("steering_deadzone").value
        )

        self.duty_profile = DutyCommandProfile(
            DutyProfileConfig(
                forward_max_duty=float(
                    self.get_parameter("forward_max_duty").value
                ),
                reverse_max_duty=float(
                    self.get_parameter("reverse_max_duty").value
                ),
                start_duty=float(self.get_parameter("start_duty").value),
                reverse_start_duty=float(
                    self.get_parameter("reverse_start_duty").value
                ),
                acceleration_duty_per_sec=float(
                    self.get_parameter("acceleration_duty_per_sec").value
                ),
                coast_deceleration_duty_per_sec=float(
                    self.get_parameter(
                        "coast_deceleration_duty_per_sec"
                    ).value
                ),
                brake_duty_per_sec=float(
                    self.get_parameter("brake_duty_per_sec").value
                ),
                pedal_deadzone=float(
                    self.get_parameter("pedal_deadzone").value
                ),
                immediate_stop_on_accelerator_release=bool(
                    self.get_parameter(
                        "immediate_stop_on_accelerator_release"
                    ).value
                ),
            )
        )

        self._accelerator = 0.0
        self._brake = 0.0
        self._steering = 0.0
        self._last_accelerator_time: Time | None = None
        self._last_brake_time: Time | None = None
        self._last_steering_time: Time | None = None
        self._last_control_time = self.get_clock().now()
        self._pedal_input_timed_out = True
        self._steering_input_timed_out = True

        # Manual commands are state values. Intermediate samples may be
        # discarded; every consumer should act on the newest sample only.
        latest_command_qos = QoSProfile(depth=1)
        latest_command_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_command_qos.durability = DurabilityPolicy.VOLATILE
        self.duty_pub = self.create_publisher(
            Float32,
            duty_topic,
            latest_command_qos,
        )
        self.servo_pub = self.create_publisher(
            Float32,
            servo_topic,
            latest_command_qos,
        )
        self.current_duty_pub = self.create_publisher(
            Float32,
            current_duty_topic,
            latest_command_qos,
        )
        self.gear_state_pub = self.create_publisher(String, gear_state_topic, 10)

        self.accelerator_sub = self.create_subscription(
            Float32,
            accelerator_topic,
            self._on_accelerator,
            latest_command_qos,
        )
        self.brake_sub = self.create_subscription(
            Float32,
            brake_topic,
            self._on_brake,
            latest_command_qos,
        )
        self.steering_sub = self.create_subscription(
            Float32,
            steering_topic,
            self._on_steering,
            latest_command_qos,
        )
        button_event_qos = QoSProfile(depth=10)
        button_event_qos.reliability = ReliabilityPolicy.RELIABLE
        button_event_qos.durability = DurabilityPolicy.VOLATILE
        self.gear_toggle_sub = self.create_subscription(
            Bool,
            gear_toggle_topic,
            self._on_gear_toggle,
            button_event_qos,
        )

        self.control_timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._on_control_timer,
        )
        self.status_timer = self.create_timer(
            1.0 / self.status_log_rate_hz,
            self._publish_status,
        )

        self.get_logger().info(
            "Manual drive waiting for joystick input. "
            f"gear={self.duty_profile.gear.name}, command_duty=0"
        )
        self._publish_gear_state()

    def _on_accelerator(self, msg: Float32) -> None:
        self._accelerator = self._clamp(float(msg.data), 0.0, 1.0)
        self._last_accelerator_time = self.get_clock().now()
        self._pedal_input_timed_out = False

    def _on_brake(self, msg: Float32) -> None:
        self._brake = self._clamp(float(msg.data), 0.0, 1.0)
        self._last_brake_time = self.get_clock().now()
        self._pedal_input_timed_out = False

    def _on_steering(self, msg: Float32) -> None:
        steering = self._clamp(float(msg.data), -1.0, 1.0)
        self._steering = self._apply_deadzone(
            steering,
            self.steering_deadzone,
        )
        self._last_steering_time = self.get_clock().now()
        self._steering_input_timed_out = False

    def _on_gear_toggle(self, msg: Bool) -> None:
        if not bool(msg.data):
            return

        if self.duty_profile.toggle_gear():
            self.get_logger().info(
                f"Gear changed: {self.duty_profile.gear.name}"
            )
            self._publish_gear_state()
        else:
            self.get_logger().warn(
                "Gear change rejected: release the accelerator and wait for "
                "duty 0 before pressing RB."
            )

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        nominal_period = 1.0 / self.control_rate_hz
        elapsed_sec = (now - self._last_control_time).nanoseconds / 1_000_000_000.0
        dt_sec = self._clamp(elapsed_sec, 0.0, nominal_period * 2.0)
        self._last_control_time = now

        if self._pedal_inputs_are_stale(now):
            self._handle_pedal_input_timeout()
            command_duty = 0.0
        else:
            command_duty = self.duty_profile.update(
                self._accelerator,
                self._brake,
                dt_sec,
            )

        if self._input_is_stale(self._last_steering_time, now):
            self._handle_steering_input_timeout()
            servo_position = self.servo_center
        else:
            servo_position = self._steering_to_servo(self._steering)

        self.duty_pub.publish(Float32(data=command_duty))
        self.current_duty_pub.publish(Float32(data=command_duty))
        self.servo_pub.publish(Float32(data=servo_position))

    def _pedal_inputs_are_stale(self, now: Time) -> bool:
        return self._input_is_stale(
            self._last_accelerator_time,
            now,
        ) or self._input_is_stale(self._last_brake_time, now)

    def _input_is_stale(self, last_input_time: Time | None, now: Time) -> bool:
        if last_input_time is None:
            return True
        elapsed_sec = (now - last_input_time).nanoseconds / 1_000_000_000.0
        return elapsed_sec >= self.input_timeout_sec

    def _handle_pedal_input_timeout(self) -> None:
        if not self._pedal_input_timed_out:
            self.get_logger().warn(
                f"Pedal input timeout ({self.input_timeout_sec:.2f}s). "
                "Sending duty 0."
            )
        self._pedal_input_timed_out = True
        self._accelerator = 0.0
        self._brake = 0.0
        self.duty_profile.reset_speed()

    def _handle_steering_input_timeout(self) -> None:
        if not self._steering_input_timed_out:
            self.get_logger().warn(
                f"Steering input timeout ({self.input_timeout_sec:.2f}s). "
                "Centering servo."
            )
        self._steering_input_timed_out = True
        self._steering = 0.0

    def _publish_status(self) -> None:
        self._publish_gear_state()
        self.get_logger().info(
            f"Manual status | gear={self.duty_profile.gear.name} | "
            f"command_duty={self.duty_profile.command_duty:.5f}"
        )

    def _publish_gear_state(self) -> None:
        self.gear_state_pub.publish(
            String(data=self.duty_profile.gear.name)
        )

    def stop_actuators(self) -> None:
        self.control_timer.cancel()
        self.status_timer.cancel()
        self.duty_profile.reset_speed()
        self.duty_pub.publish(Float32(data=0.0))
        self.current_duty_pub.publish(Float32(data=0.0))
        self.servo_pub.publish(Float32(data=self.servo_center))
        self._publish_gear_state()

    def _steering_to_servo(self, steering: float) -> float:
        steering = self._clamp(steering, -1.0, 1.0)
        if steering < 0.0:
            servo_position = self._lerp(
                self.servo_center,
                self.servo_left,
                -steering,
            )
        else:
            servo_position = self._lerp(
                self.servo_center,
                self.servo_right,
                steering,
            )

        servo_min = min(self.servo_left, self.servo_center, self.servo_right)
        servo_max = max(self.servo_left, self.servo_center, self.servo_right)
        return self._clamp(servo_position, servo_min, servo_max)

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def _lerp(start: float, end: float, amount: float) -> float:
        return start + (end - start) * amount

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActuatorCommanderNode()

    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
