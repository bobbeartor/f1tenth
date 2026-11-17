#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

from manual_control.brake_current_profile import (
    BrakeCurrentProfile,
    BrakeCurrentProfileConfig,
)
from manual_control.duty_command_profile import (
    DutyCommandProfile,
    DutyProfileConfig,
    GearRequestResult,
)


class ActuatorCommanderNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_commander_node")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("joy_steering_axis", 0)
        self.declare_parameter("joy_accelerator_axis", 5)
        self.declare_parameter("joy_brake_axis", 4)
        self.declare_parameter("joy_gear_button", 10)
        self.declare_parameter("joy_trigger_released", 0.0)
        self.declare_parameter("joy_trigger_pressed", 1.0)
        self.declare_parameter("current_duty_topic", "/manual/current_duty")
        self.declare_parameter(
            "current_brake_current_topic", "/manual/current_brake_current"
        )
        self.declare_parameter("gear_state_topic", "/manual/gear")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("brake_current_topic", "/vesc/brake_current")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")

        self.declare_parameter("forward_max_duty", 0.10)
        self.declare_parameter("reverse_max_duty", 0.08)
        self.declare_parameter("start_duty", 0.05)
        self.declare_parameter("reverse_start_duty", 0.05)
        self.declare_parameter("acceleration_duty_per_sec", 0.03)
        self.declare_parameter("coast_deceleration_duty_per_sec", 0.005)
        self.declare_parameter("maximum_brake_current_amps", 8.0)
        self.declare_parameter("brake_current_rise_amps_per_sec", 16.0)
        self.declare_parameter("brake_current_fall_amps_per_sec", 32.0)
        self.declare_parameter("control_rate_hz", 80.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("input_timeout_sec", 0.3)
        self.declare_parameter("immediate_stop_on_accelerator_release", True)

        self.declare_parameter("servo_left", 1.0)
        self.declare_parameter("servo_center", 0.46)
        self.declare_parameter("servo_right", 0.0)
        self.declare_parameter("pedal_deadzone", 0.01)
        self.declare_parameter("steering_deadzone", 0.04)

        joy_topic = str(self.get_parameter("joy_topic").value)
        self.joy_steering_axis = int(
            self.get_parameter("joy_steering_axis").value
        )
        self.joy_accelerator_axis = int(
            self.get_parameter("joy_accelerator_axis").value
        )
        self.joy_brake_axis = int(
            self.get_parameter("joy_brake_axis").value
        )
        self.joy_gear_button = int(
            self.get_parameter("joy_gear_button").value
        )
        self.joy_trigger_released = float(
            self.get_parameter("joy_trigger_released").value
        )
        self.joy_trigger_pressed = float(
            self.get_parameter("joy_trigger_pressed").value
        )
        current_duty_topic = str(self.get_parameter("current_duty_topic").value)
        current_brake_current_topic = str(
            self.get_parameter("current_brake_current_topic").value
        )
        gear_state_topic = str(self.get_parameter("gear_state_topic").value)
        duty_topic = str(self.get_parameter("duty_topic").value)
        brake_current_topic = str(
            self.get_parameter("brake_current_topic").value
        )
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
        self.pedal_deadzone = float(
            self.get_parameter("pedal_deadzone").value
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
                pedal_deadzone=self.pedal_deadzone,
                immediate_stop_on_accelerator_release=bool(
                    self.get_parameter(
                        "immediate_stop_on_accelerator_release"
                    ).value
                ),
            )
        )
        self.brake_profile = BrakeCurrentProfile(
            BrakeCurrentProfileConfig(
                maximum_brake_current_amps=float(
                    self.get_parameter("maximum_brake_current_amps").value
                ),
                rise_amps_per_sec=float(
                    self.get_parameter(
                        "brake_current_rise_amps_per_sec"
                    ).value
                ),
                fall_amps_per_sec=float(
                    self.get_parameter(
                        "brake_current_fall_amps_per_sec"
                    ).value
                ),
                pedal_deadzone=self.pedal_deadzone,
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
        self._gear_button_was_pressed = False
        self._invalid_joy_layout_reported = False
        self._brake_mode_active = False
        # 시작/입력 timeout(조이스틱 끊김) 직후에는 RT를 한 번 놓은 것이
        # 확인될 때까지 가속 입력을 무시한다. 재연결 순간 남아 있던 RT 값으로
        # 바로 출발하지 않게 한다. LT 제동과 조향은 막지 않는다.
        self._accelerator_requires_release = True
        self._accelerator_lock_reported = False

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
        self.brake_current_pub = self.create_publisher(
            Float32,
            brake_current_topic,
            latest_command_qos,
        )
        self.servo_pub = self.create_publisher(
            Float32,
            servo_topic,
            latest_command_qos,
        )
        self.current_brake_current_pub = self.create_publisher(
            Float32,
            current_brake_current_topic,
            latest_command_qos,
        )
        self.current_duty_pub = self.create_publisher(
            Float32,
            current_duty_topic,
            latest_command_qos,
        )
        self.gear_state_pub = self.create_publisher(
            String,
            gear_state_topic,
            latest_command_qos,
        )

        # Read every manual command from one current-state message. This avoids
        # four intermediate topics being received at different times or
        # replaying stale values after a controller reconnect.
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self._on_joy,
            latest_command_qos,
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
            "Manual drive waiting for direct joystick input. "
            f"RT=axis{self.joy_accelerator_axis}, "
            f"LT=axis{self.joy_brake_axis}, "
            f"steering=axis{self.joy_steering_axis}, "
            f"gear=button{self.joy_gear_button}; "
            f"initial_gear={self.duty_profile.gear.name}, command_duty=0, "
            f"LT_max_brake={self.brake_profile.config.maximum_brake_current_amps:.1f}A"
        )
        self._publish_gear_state()

    def _on_joy(self, msg: Joy) -> None:
        required_axis = max(
            self.joy_steering_axis,
            self.joy_accelerator_axis,
            self.joy_brake_axis,
        )
        if (
            required_axis < 0
            or len(msg.axes) <= required_axis
            or self.joy_gear_button < 0
            or len(msg.buttons) <= self.joy_gear_button
        ):
            if not self._invalid_joy_layout_reported:
                self.get_logger().error(
                    "Joystick message does not contain the configured axes "
                    "or gear button. Ignoring input so the safety timeout "
                    "stops the vehicle."
                )
                self._invalid_joy_layout_reported = True
            return

        self._invalid_joy_layout_reported = False
        now = self.get_clock().now()
        self._accelerator = self._gate_accelerator(
            self._normalize_trigger(
                float(msg.axes[self.joy_accelerator_axis])
            )
        )
        self._brake = self._normalize_trigger(
            float(msg.axes[self.joy_brake_axis])
        )
        steering = self._clamp(
            float(msg.axes[self.joy_steering_axis]),
            -1.0,
            1.0,
        )
        self._steering = self._apply_deadzone(
            steering,
            self.steering_deadzone,
        )
        self._last_accelerator_time = now
        self._last_brake_time = now
        self._last_steering_time = now
        self._pedal_input_timed_out = False
        self._steering_input_timed_out = False

        self._update_gear_button(
            bool(msg.buttons[self.joy_gear_button])
        )

    def _update_gear_button(self, gear_button_pressed: bool) -> None:
        gear_toggle_requested = (
            gear_button_pressed and not self._gear_button_was_pressed
        )
        self._gear_button_was_pressed = gear_button_pressed
        if not gear_toggle_requested:
            return

        braking = (
            self._brake >= self.pedal_deadzone or
            self.brake_profile.command_current_amps > 0.0
        )
        result = self.duty_profile.request_gear_toggle(
            apply_now_allowed=not braking
        )
        if result is GearRequestResult.CHANGED:
            self.get_logger().info(
                f"Gear changed: {self.duty_profile.gear.name}"
            )
            self._publish_gear_state()
        elif result is GearRequestResult.PENDING:
            self.get_logger().info(
                "Gear change reserved: "
                f"{self.duty_profile.pending_gear.name} will engage when "
                "duty reaches 0 (press RB again to cancel)."
            )
        else:
            self.get_logger().info(
                "Gear change reservation cancelled: staying in "
                f"{self.duty_profile.gear.name}."
            )

    def _gate_accelerator(self, accelerator: float) -> float:
        if not self._accelerator_requires_release:
            return accelerator
        if accelerator < self.pedal_deadzone:
            if self._accelerator_lock_reported:
                self.get_logger().info("RT released: accelerator enabled.")
            self._accelerator_requires_release = False
            self._accelerator_lock_reported = False
            return accelerator
        if not self._accelerator_lock_reported:
            self.get_logger().warn(
                "RT is held after startup or joystick input loss. Ignoring RT "
                "until it is released once."
            )
            self._accelerator_lock_reported = True
        return 0.0

    def _update_duty(self, accelerator: float, dt_sec: float) -> float:
        gear_before = self.duty_profile.gear
        command_duty = self.duty_profile.update(accelerator, dt_sec)
        if self.duty_profile.gear is not gear_before:
            self.get_logger().info(
                f"Reserved gear engaged: {self.duty_profile.gear.name}"
            )
            self._publish_gear_state()
        return command_duty

    def _normalize_trigger(self, value: float) -> float:
        released = self.joy_trigger_released
        pressed = self.joy_trigger_pressed
        if pressed == released:
            return 0.0
        normalized = (value - released) / (pressed - released)
        return self._clamp(normalized, 0.0, 1.0)

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        nominal_period = 1.0 / self.control_rate_hz
        elapsed_sec = (now - self._last_control_time).nanoseconds / 1_000_000_000.0
        dt_sec = self._clamp(elapsed_sec, 0.0, nominal_period * 2.0)
        self._last_control_time = now

        if self._pedal_inputs_are_stale(now):
            self._handle_pedal_input_timeout()
            command_duty = 0.0
            command_brake_current = 0.0
            self.brake_current_pub.publish(Float32(data=0.0))
            self._brake_mode_active = False
        else:
            command_brake_current = self.brake_profile.update(
                self._brake, dt_sec
            )
            if command_brake_current > 0.0:
                # Electrical braking owns the VESC motor-control mode. Clear
                # stored propulsion immediately so RT can never fight LT.
                self.duty_profile.reset_speed()
                command_duty = 0.0
                self.brake_current_pub.publish(
                    Float32(data=command_brake_current)
                )
                self._brake_mode_active = True
            elif self._brake_mode_active:
                # Send one explicit zero-current command before returning to
                # duty mode on the next control cycle.
                self.duty_profile.reset_speed()
                command_duty = 0.0
                self.brake_current_pub.publish(Float32(data=0.0))
                self._brake_mode_active = False
            else:
                command_duty = self._update_duty(self._accelerator, dt_sec)
                self.duty_pub.publish(Float32(data=command_duty))

        if self._input_is_stale(self._last_steering_time, now):
            self._handle_steering_input_timeout()
            servo_position = self.servo_center
        else:
            servo_position = self._steering_to_servo(self._steering)

        self.current_duty_pub.publish(Float32(data=command_duty))
        self.current_brake_current_pub.publish(
            Float32(data=command_brake_current)
        )
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
                "Releasing the motor command."
            )
        self._pedal_input_timed_out = True
        self._accelerator = 0.0
        self._brake = 0.0
        self.duty_profile.reset_speed()
        # 컨트롤러가 끊긴 뒤 재연결 시 예전 RB 예약이 적용되지 않게 한다.
        self.duty_profile.clear_gear_request()
        self.brake_profile.reset()
        self._accelerator_requires_release = True

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
            f"command_duty={self.duty_profile.command_duty:.5f} | "
            f"brake_current={self.brake_profile.command_current_amps:.2f}A"
        )

    def _publish_gear_state(self) -> None:
        self.gear_state_pub.publish(
            String(data=self.duty_profile.gear.name)
        )

    def stop_actuators(self) -> None:
        self.control_timer.cancel()
        self.status_timer.cancel()
        self.duty_profile.reset_speed()
        self.brake_profile.reset()
        self.brake_current_pub.publish(Float32(data=0.0))
        self.duty_pub.publish(Float32(data=0.0))
        self.current_duty_pub.publish(Float32(data=0.0))
        self.current_brake_current_pub.publish(Float32(data=0.0))
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
