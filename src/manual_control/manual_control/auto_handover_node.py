"""Own actuator outputs while switching from automatic to manual driving."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32, String

from manual_control.actuator_commander_node import ActuatorCommanderNode
from manual_control.handover_state import HandoverState


class AutoHandoverNode(ActuatorCommanderNode):
    """Single actuator publisher; automatic commands never bypass the handover."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("handover_button", 2)  # SDL logical X
        self.declare_parameter("handover_stopped_erpm", 100.0)
        self.declare_parameter("handover_stop_hold_sec", 0.5)
        self.declare_parameter("handover_telemetry_timeout_sec", 0.3)
        self.declare_parameter("measured_erpm_topic", "/vesc/measured_erpm")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("auto_enable_topic", "/auto/enabled")
        self.declare_parameter("drive_mode_topic", "drive/mode")
        for channel in ("duty", "brake_current", "servo_position"):
            self.declare_parameter(f"auto_{channel}_topic", f"handover/auto/{channel}")

        self.handover_button = int(self.get_parameter("handover_button").value)
        if self.handover_button < 0:
            raise ValueError("handover_button must be non-negative")
        self.handover = HandoverState(
            float(self.get_parameter("handover_stopped_erpm").value),
            float(self.get_parameter("handover_stop_hold_sec").value),
            float(self.get_parameter("handover_telemetry_timeout_sec").value),
        )
        self._neutral = False
        self._joy_received_sec: float | None = None
        self._auto_servo = self.servo_center
        self._auto_brake_current = 0.0
        self._last_handover_tick = time.monotonic()
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter("drive_mode_topic").value), state_qos,
        )
        self.auto_enable_pub = self.create_publisher(
            Bool, str(self.get_parameter("auto_enable_topic").value), command_qos,
        )
        self.auto_subscriptions = [
            self.create_subscription(
                Float32, str(self.get_parameter(f"auto_{channel}_topic").value),
                lambda msg, channel=channel: self._on_auto_command(channel, msg),
                command_qos,
            ) for channel in ("duty", "brake_current", "servo_position")
        ]
        self.erpm_sub = self.create_subscription(
            Int32, str(self.get_parameter("measured_erpm_topic").value),
            lambda msg: self.handover.on_erpm(msg.data, time.monotonic()), command_qos,
        )
        self.connection_sub = self.create_subscription(
            Bool, str(self.get_parameter("connection_status_topic").value),
            self._on_connection, state_qos,
        )
        self._publish_mode()
        self.get_logger().info(self.handover_help())

    def handover_help(self):
        return (
            f"AUTO active; X (button {self.handover_button}) brakes, then switches "
            "to MANUAL after fresh ERPM confirms a stop and controls are released. "
            "BEV preview remains running."
        )

    def _on_connection(self, msg: Bool) -> None:
        if not msg.data:
            self.handover.invalidate_telemetry()

    def _on_auto_command(self, channel: str, msg: Float32) -> None:
        if self.handover.mode != "AUTO" or not math.isfinite(msg.data):
            return
        if channel == "servo_position":
            self._auto_servo = self._clamp(
                msg.data, min(self.servo_left, self.servo_right),
                max(self.servo_left, self.servo_right),
            )
            self.servo_pub.publish(Float32(data=self._auto_servo))
        elif channel == "brake_current":
            self._auto_brake_current = self._clamp(
                msg.data, 0.0, self.brake_profile.config.maximum_brake_current_amps,
            )
            self.brake_current_pub.publish(msg)
            self.current_duty_pub.publish(Float32(data=0.0))
            self.current_brake_current_pub.publish(msg)
        else:
            self._auto_brake_current = 0.0
            self.duty_pub.publish(msg)
            self.current_duty_pub.publish(msg)
            self.current_brake_current_pub.publish(Float32(data=0.0))

    def _on_joy(self, msg: Joy) -> None:
        if self.handover.mode == "MANUAL":
            super()._on_joy(msg)
            return
        # A valid X must stop automatic output even if other input slots are absent.
        if len(msg.buttons) > self.handover_button:
            if self.handover.on_x(bool(msg.buttons[self.handover_button])):
                self.duty_profile.reset_speed()
                self.brake_profile.current_amps = self._auto_brake_current
                self._publish_stopping(1.0 / self.control_rate_hz)
                self._publish_mode()
                self.get_logger().warn(
                    "X pressed: automatic commands blocked, braking. Release X, "
                    "RT, LT, RB and center steering to enter MANUAL after stopping."
                )
        axes = (self.joy_accelerator_axis, self.joy_brake_axis, self.joy_steering_axis)
        valid = (min(axes) >= 0 and len(msg.axes) > max(axes)
                 and self.joy_gear_button >= 0
                 and len(msg.buttons) > max(self.handover_button, self.joy_gear_button)
                 and all(math.isfinite(msg.axes[index]) for index in axes))
        self._neutral = bool(valid and (
            self._normalize_trigger(msg.axes[self.joy_accelerator_axis]) <= self.pedal_deadzone
            and self._normalize_trigger(msg.axes[self.joy_brake_axis]) <= self.pedal_deadzone
            and abs(msg.axes[self.joy_steering_axis]) <= self.steering_deadzone
            and not msg.buttons[self.joy_gear_button]
            and not msg.buttons[self.handover_button]
        ))
        self._joy_received_sec = time.monotonic() if valid else None

    def _on_control_timer(self) -> None:
        now_sec = time.monotonic()
        dt = min(max(now_sec - self._last_handover_tick, 0.0), 2.0 / self.control_rate_hz)
        self._last_handover_tick = now_sec
        if self.handover.mode == "AUTO":
            return  # Only forward fresh automatic commands; never replay them.
        if self.handover.mode == "MANUAL":
            super()._on_control_timer()
            return
        neutral = (self._neutral and self._joy_received_sec is not None
                   and 0.0 <= now_sec - self._joy_received_sec < self.input_timeout_sec)
        if self.handover.update(now_sec, neutral):
            # Base manual state has never consumed AUTO/STOPPING joystick frames.
            # Release braking now; only a subsequent Joy frame can command motion.
            self.brake_profile.reset()
            self.duty_profile.reset_speed()
            self.duty_profile.clear_gear_request()
            # STOPPING 단계에서 RT 해제(neutral)를 이미 확인했다.
            self._accelerator_requires_release = False
            self._last_control_time = self.get_clock().now()
            self.brake_current_pub.publish(Float32(data=0.0))
            self.current_brake_current_pub.publish(Float32(data=0.0))
            self.servo_pub.publish(Float32(data=self.servo_center))
            self._publish_mode()
            self.get_logger().warn("Stopped: MANUAL active. RT/LT/steering/RB now control the vehicle.")
        else:
            self._publish_stopping(dt)

    def _publish_stopping(self, dt: float) -> None:
        current = self.brake_profile.update(1.0, dt)
        # Never publish duty during braking: it would replace VESC brake mode.
        self.brake_current_pub.publish(Float32(data=current))
        self.current_duty_pub.publish(Float32(data=0.0))
        self.current_brake_current_pub.publish(Float32(data=current))
        self.servo_pub.publish(Float32(data=self._auto_servo))

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.handover.mode))
        if self.handover.mode != "AUTO":
            # Advisory disable, retried by status timer. Output gating above is
            # authoritative even if this best-effort message is lost/re-enabled.
            self.auto_enable_pub.publish(Bool(data=False))

    def _publish_status(self) -> None:
        self._publish_mode()
        self._publish_gear_state()
        self.get_logger().info(
            f"Drive mode={self.handover.mode} | gear={self.duty_profile.gear.name} | "
            f"manual_duty={self.duty_profile.command_duty:.5f} | "
            f"brake={self.brake_profile.command_current_amps:.2f}A"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoHandoverNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
