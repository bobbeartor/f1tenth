"""Reversible GUI handover using the existing manual controller and stop gate."""

import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from manual_control.actuator_commander_node import ActuatorCommanderNode
from manual_control.auto_handover_node import AutoHandoverNode


class VfhHandoverNode(AutoHandoverNode):
    def __init__(self):
        self._request_id = ""
        self._destination = "AUTO"
        self._last_mode_publish = -1.0
        self._last_motor_command = None
        self._stop_current = 2.0
        super().__init__()
        self.declare_parameter("mode_request_topic", "/roi_vfh_test/drive_mode_request")
        self.declare_parameter("mode_ack_topic", "/roi_vfh_test/drive_mode_ack")
        self.declare_parameter("auto_command_timeout_sec", 0.30)
        self.declare_parameter("handover_stop_brake_current_amps", 2.0)
        self._command_timeout = float(self.get_parameter("auto_command_timeout_sec").value)
        self._default_stop_current = float(self.get_parameter("handover_stop_brake_current_amps").value)
        self._stop_current = self._default_stop_current
        if not math.isfinite(self._command_timeout) or not .05 <= self._command_timeout <= 1:
            raise ValueError("auto_command_timeout_sec must be in .05..1 seconds")
        if not math.isfinite(self._stop_current) or not 0 < self._stop_current <= self.brake_profile.config.maximum_brake_current_amps:
            raise ValueError("handover stop current must be positive and within the manual brake limit")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ack_pub = self.create_publisher(
            String, str(self.get_parameter("mode_ack_topic").value), qos)
        self.mode_request_sub = self.create_subscription(
            String, str(self.get_parameter("mode_request_topic").value),
            self._on_mode_request, qos)

    def handover_help(self):
        return (
            "VFH GUI owns mode selection: Stop -> brake -> MANUAL; Start -> brake -> AUTO. "
            "X does not switch modes in this node. Release RT/LT/RB and center steering to switch.")

    def _publish_stopping(self, dt):
        fraction = self._stop_current / self.brake_profile.config.maximum_brake_current_amps
        current = self.brake_profile.update(fraction, dt)
        self.brake_current_pub.publish(Float32(data=current))
        self.current_duty_pub.publish(Float32(data=0.0))
        self.current_brake_current_pub.publish(Float32(data=current))
        self.servo_pub.publish(Float32(data=self._auto_servo))

    def _on_mode_request(self, msg):
        fields = msg.data.split("|")
        if len(fields) not in (2, 3):
            return
        token, destination = fields[:2]
        if not token or len(token) > 96 or destination not in ("AUTO", "MANUAL"):
            return
        current = self._default_stop_current
        if len(fields) == 3:
            try:
                current = float(fields[2])
            except ValueError:
                return
            if not math.isfinite(current) or not 0 < current <= self.brake_profile.config.maximum_brake_current_amps:
                return
        if token == self._request_id:
            self._publish_mode()
            return
        self._stop_current = current
        self._request_id, self._destination = token, destination
        if self.handover.mode != destination or destination == "MANUAL":
            # The existing state machine requires fresh post-request standstill
            # telemetry and neutral inputs. It is reused for both directions.
            self.handover.mode = "AUTO"
            self.handover.on_x(False)
            self.handover.on_x(True)
            self.handover.on_x(False)
            self.duty_profile.reset_speed()
            self.brake_profile.current_amps = self._auto_brake_current
            self._neutral = False
            self._joy_received_sec = None
            self._publish_stopping(1.0 / self.control_rate_hz)
        self._publish_mode()

    def _on_auto_command(self, channel, msg):
        if self.handover.mode == "AUTO" and math.isfinite(msg.data):
            if channel in ("duty", "brake_current"):
                self._last_motor_command = time.monotonic()
            super()._on_auto_command(channel, msg)

    def _on_joy(self, msg):
        axes = (self.joy_accelerator_axis, self.joy_brake_axis, self.joy_steering_axis)
        valid = (min(axes) >= 0 and len(msg.axes) > max(axes)
                 and self.joy_gear_button >= 0 and len(msg.buttons) > self.joy_gear_button
                 and all(math.isfinite(msg.axes[index]) for index in axes))
        self._neutral = bool(valid and (
            self._normalize_trigger(msg.axes[self.joy_accelerator_axis]) <= self.pedal_deadzone
            and self._normalize_trigger(msg.axes[self.joy_brake_axis]) <= self.pedal_deadzone
            and abs(msg.axes[self.joy_steering_axis]) <= self.steering_deadzone
            and not msg.buttons[self.joy_gear_button]))
        self._joy_received_sec = time.monotonic() if valid else None
        if self.handover.mode == "MANUAL":
            ActuatorCommanderNode._on_joy(self, msg)

    def _on_control_timer(self):
        clock = time.monotonic()
        dt = min(max(clock - self._last_handover_tick, 0.0), 2.0 / self.control_rate_hz)
        self._last_handover_tick = clock
        if clock - self._last_mode_publish >= .1:
            self._publish_mode()
            self._last_mode_publish = clock
        if self.handover.mode == "AUTO":
            if self._last_motor_command is None or clock - self._last_motor_command > self._command_timeout:
                self._publish_stopping(dt)
            return
        if self.handover.mode == "MANUAL":
            ActuatorCommanderNode._on_control_timer(self)
            now = self.get_clock().now()
            self._auto_servo = (self.servo_center if self._input_is_stale(self._last_steering_time, now)
                                else self._steering_to_servo(self._steering))
            self._auto_brake_current = self.brake_profile.command_current_amps
            return
        neutral = (self._neutral and self._joy_received_sec is not None
                   and 0 <= clock - self._joy_received_sec < self.input_timeout_sec)
        if not self.handover.update(clock, neutral):
            self._publish_stopping(dt)
            return
        self.handover.mode = self._destination
        self.duty_profile.reset_speed()
        self.duty_profile.clear_gear_request()
        # handover 정지 게이트가 RT 해제(neutral)를 이미 확인했다.
        self._accelerator_requires_release = False
        self.brake_profile.reset()
        self._accelerator = self._brake = self._steering = 0.0
        self._last_accelerator_time = self._last_brake_time = self._last_steering_time = None
        self._gear_button_was_pressed = False
        self._brake_mode_active = False
        self._last_control_time = self.get_clock().now()
        self._last_motor_command = None
        if self.handover.mode == "MANUAL":
            self.brake_current_pub.publish(Float32(data=0.0))
            self.current_brake_current_pub.publish(Float32(data=0.0))
            self.servo_pub.publish(Float32(data=self.servo_center))
            self._auto_servo = self.servo_center
        # AUTO retains braking until a new automatic motor command arrives.
        # No cached duty or joystick sample is replayed after either transition.
        self._publish_mode()

    def _publish_mode(self):
        mode = self.handover.mode
        if mode == "STOPPING":
            mode += "_" + self._destination
        self.mode_pub.publish(String(data=mode))
        if hasattr(self, "ack_pub"):
            self.ack_pub.publish(String(data=self._request_id + "|" + mode))


def main(args=None):
    rclpy.init(args=args)
    node = VfhHandoverNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
