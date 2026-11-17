#!/usr/bin/env python3

import json
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, String


class JoyParamsConverterNode(Node):
    def __init__(self) -> None:
        super().__init__("joy_params_converter_node")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("accelerator_topic", "/manual/accelerator")
        self.declare_parameter("brake_topic", "/manual/brake")
        self.declare_parameter("steering_topic", "/manual/steering")
        self.declare_parameter("gear_toggle_topic", "/manual/gear_toggle")
        self.declare_parameter("debug_topic", "/manual/controller_debug")
        self.declare_parameter("keymap_path", "")
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("trigger_deadzone", 0.01)
        self.declare_parameter("steering_deadzone", 0.05)

        self.keymap = self._load_keymap()
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.trigger_deadzone = float(
            self.get_parameter("trigger_deadzone").value
        )
        self.steering_deadzone = float(self.get_parameter("steering_deadzone").value)

        joy_topic = str(self.get_parameter("joy_topic").value)
        accelerator_topic = str(self.get_parameter("accelerator_topic").value)
        brake_topic = str(self.get_parameter("brake_topic").value)
        steering_topic = str(self.get_parameter("steering_topic").value)
        gear_toggle_topic = str(self.get_parameter("gear_toggle_topic").value)
        debug_topic = str(self.get_parameter("debug_topic").value)

        # Controller data describes current state, not an event stream. Keeping
        # only one sample prevents held inputs from building a stale backlog.
        latest_state_qos = QoSProfile(depth=1)
        latest_state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_state_qos.durability = DurabilityPolicy.VOLATILE
        self.accelerator_pub = self.create_publisher(
            Float32,
            accelerator_topic,
            latest_state_qos,
        )
        self.brake_pub = self.create_publisher(
            Float32,
            brake_topic,
            latest_state_qos,
        )
        self.steering_pub = self.create_publisher(
            Float32,
            steering_topic,
            latest_state_qos,
        )
        # Do not queue gear changes. A delayed RB event must never be replayed
        # after the operator has released the button.
        self.gear_toggle_pub = self.create_publisher(
            Bool,
            gear_toggle_topic,
            latest_state_qos,
        )
        self.debug_pub = self.create_publisher(
            String,
            debug_topic,
            latest_state_qos,
        )
        self.joy_sub = self.create_subscription(
            Joy,
            joy_topic,
            self._on_joy,
            latest_state_qos,
        )

        self.get_logger().info(
            "Subscribing to %s, publishing accelerator=%s, brake=%s, "
            "steering=%s, gear_toggle=%s (latest-only, no delayed replay)"
            % (
                joy_topic,
                accelerator_topic,
                brake_topic,
                steering_topic,
                gear_toggle_topic,
            )
        )

    def _load_keymap(self) -> dict[str, Any]:
        keymap_path = str(self.get_parameter("keymap_path").value)
        if not keymap_path:
            raise RuntimeError(
                "keymap_path parameter is required; vehicle_bringup passes it "
                "from controller_keymap.yaml"
            )

        with open(keymap_path, "r", encoding="utf-8") as keymap_file:
            loaded = yaml.safe_load(keymap_file) or {}

        keymap = loaded.get("manual_keymap", loaded)
        self.get_logger().info(f"Loaded manual keymap: {keymap_path}")
        return keymap

    def _on_joy(self, msg: Joy) -> None:
        controller_state = self._controller_state(msg)

        accelerator = controller_state["triggers"]["rt"]["value"]
        brake = controller_state["triggers"]["lt"]["value"]
        steering = controller_state["axes"]["left_stick_x"]
        gear_button_pressed = controller_state["buttons"].get("rb", False)

        self.accelerator_pub.publish(Float32(data=accelerator))
        self.brake_pub.publish(Float32(data=brake))
        self.steering_pub.publish(Float32(data=steering))
        # Publish the current RB state, including release. With KEEP_LAST(1),
        # release overwrites a stale press before it can execute later.
        self.gear_toggle_pub.publish(Bool(data=gear_button_pressed))

        if self.publish_debug:
            self.debug_pub.publish(
                String(data=json.dumps(controller_state, sort_keys=True))
            )

    def _controller_state(self, msg: Joy) -> dict[str, Any]:
        return {
            "axes": {
                "left_stick_x": self._stick_axis(msg, "left_stick", "x"),
                "left_stick_y": self._stick_axis(msg, "left_stick", "y"),
                "right_stick_x": self._stick_axis(msg, "right_stick", "x"),
                "right_stick_y": self._stick_axis(msg, "right_stick", "y"),
            },
            "triggers": {
                "lt": self._trigger_state(msg, "lt"),
                "rt": self._trigger_state(msg, "rt"),
            },
            "buttons": {
                name: self._button_pressed(msg, name)
                for name in sorted(self.keymap.get("buttons", {}).keys())
            },
            "dpad": {
                name: self._dpad_pressed(msg, name)
                for name in sorted(self.keymap.get("dpad", {}).keys())
            },
        }

    def _stick_axis(self, msg: Joy, stick_name: str, axis_name: str) -> float:
        axis_config = (
            self.keymap.get("axes", {})
            .get(stick_name, {})
            .get(axis_name, {})
        )
        value = self._axis_value(msg, axis_config.get("number"))
        return self._apply_deadzone(value, self.steering_deadzone)

    def _trigger_state(self, msg: Joy, trigger_name: str) -> dict[str, Any]:
        trigger_config = self.keymap.get("triggers", {}).get(trigger_name, {})
        axis_config = trigger_config.get("axis", {})
        button_config = trigger_config.get("button", {})

        raw_axis = self._axis_value(msg, axis_config.get("number"), default=-1.0)
        released = float(axis_config.get("normalized_released", -1.0))
        pressed = float(axis_config.get("normalized_pressed", 1.0))
        value = self._normalize_range(raw_axis, released, pressed)
        value = self._apply_deadzone(value, self.trigger_deadzone)

        button_pressed = self._button_pressed_from_config(msg, button_config)
        return {
            "value": value,
            "axis": raw_axis,
            "pressed": button_pressed or value > 0.0,
        }

    def _button_pressed(self, msg: Joy, button_name: str) -> bool:
        button_config = self.keymap.get("buttons", {}).get(button_name, {})
        return self._button_pressed_from_config(msg, button_config)

    def _button_pressed_from_config(self, msg: Joy, button_config: dict[str, Any]) -> bool:
        number = button_config.get("number")
        if number is None:
            return False

        index = int(number)
        if index < 0 or index >= len(msg.buttons):
            return False

        pressed_value = int(button_config.get("pressed_value", 1))
        return int(msg.buttons[index]) == pressed_value

    def _dpad_pressed(self, msg: Joy, dpad_name: str) -> bool:
        dpad_config = self.keymap.get("dpad", {}).get(dpad_name, {})
        if dpad_config.get("type") == "button":
            return self._button_pressed_from_config(msg, dpad_config)

        raw_axis = self._axis_value(msg, dpad_config.get("number"))
        pressed_value = float(dpad_config.get("normalized_pressed", 0.0))
        if pressed_value < 0.0:
            return raw_axis <= -0.5
        if pressed_value > 0.0:
            return raw_axis >= 0.5
        return abs(raw_axis) < 0.5

    def _axis_value(self, msg: Joy, number: Any, default: float = 0.0) -> float:
        if number is None:
            return default

        index = int(number)
        if index < 0 or index >= len(msg.axes):
            return default

        return float(msg.axes[index])

    @staticmethod
    def _normalize_range(value: float, released: float, pressed: float) -> float:
        if pressed == released:
            return 0.0

        normalized = (value - released) / (pressed - released)
        return max(0.0, min(1.0, normalized))

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyParamsConverterNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
