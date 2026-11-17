"""Exercise real node callbacks with an in-memory ROS transport/clock double.

This verifies command ownership and the manual profile handoff without hardware;
it is not a substitute for ROS discovery or a powered vehicle check.
"""

import importlib
import sys
import types
import unittest
from unittest.mock import Mock, patch


class Message:
    def __init__(self, data=None, axes=None, buttons=None):
        self.data, self.axes, self.buttons = data, axes, buttons


class Stamp:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return types.SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)


class Publisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


class FakeNode:
    def __init__(self, name):
        self.params = {}
        self.clock_sec = 0.0
        self.logger = Mock()

    def declare_parameter(self, name, default):
        self.params[name] = default

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.params[name])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return args

    def create_timer(self, *args):
        return Mock()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: Stamp(self.clock_sec))

    def get_logger(self):
        return self.logger


def ros_doubles():
    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "rclpy.time",
        "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg",
    )}
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.time"].Time = Stamp
    modules["rclpy.qos"].QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    modules["rclpy.qos"].DurabilityPolicy = types.SimpleNamespace(VOLATILE=0, TRANSIENT_LOCAL=1)
    modules["rclpy.qos"].ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0, RELIABLE=1)
    modules["sensor_msgs.msg"].Joy = Message
    for name in ("Bool", "Float32", "Int32", "String"):
        setattr(modules["std_msgs.msg"], name, Message)
    return modules


class AutoHandoverNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Restore module cache after this suite so ROS tests can run in the same process.
        doubles = ros_doubles()
        for name in ("manual_control.actuator_commander_node", "manual_control.auto_handover_node"):
            doubles[name] = None
        cls.module_patch = patch.dict(sys.modules, doubles)
        cls.module_patch.start()
        for name, value in doubles.items():
            if value is None:
                del sys.modules[name]
        cls.module = importlib.import_module("manual_control.auto_handover_node")

    @classmethod
    def tearDownClass(cls):
        cls.module_patch.stop()

    def setUp(self):
        self.now = 0.0
        self.time_patch = patch.object(self.module.time, "monotonic", side_effect=lambda: self.now)
        self.time_patch.start()
        self.addCleanup(self.time_patch.stop)
        self.node = self.module.AutoHandoverNode()

    def joy(self, x=0, rt=0.0, lt=0.0, steering=0.0, rb=0):
        buttons = [0] * 15
        buttons[2], buttons[10] = x, rb
        self.node._on_joy(Message(axes=[steering, 0, 0, 0, lt, rt], buttons=buttons))

    def tick(self, now, erpm=None, **joy):
        self.now = now
        self.node.clock_sec = now
        if erpm is not None:
            self.node.erpm_sub[2](Message(data=erpm))
        self.joy(**joy)
        self.node._on_control_timer()

    def stop_and_switch(self):
        self.joy(x=1)
        for now in (0.1, 0.3, 0.5, 0.7):
            self.tick(now, erpm=0)
        self.assertEqual(self.node.handover.mode, "MANUAL")

    def test_auto_passthrough_ignores_manual_pedals_and_gear(self):
        self.node._on_auto_command("duty", Message(data=0.08))
        self.node._on_auto_command("servo_position", Message(data=0.6))
        self.tick(0.1, rt=1.0, rb=1, steering=-1.0)
        self.assertEqual(self.node.duty_pub.values, [0.08])
        self.assertEqual(self.node.servo_pub.values, [0.6])
        self.assertEqual(self.node.duty_profile.command_duty, 0.0)
        self.assertEqual(self.node.duty_profile.gear.name, "FORWARD")

    def test_x_blocks_late_auto_commands_and_only_brakes_until_stopped(self):
        self.node._on_auto_command("servo_position", Message(data=0.6))
        self.node._on_auto_command("brake_current", Message(data=4.0))
        self.joy(x=1, rt=1.0)
        for channel, value in (("duty", 0.1), ("brake_current", 0.0), ("servo_position", 1.0)):
            self.node._on_auto_command(channel, Message(data=value))
        for now in (0.1, 0.3, 0.5, 0.7):
            self.tick(now, erpm=500, rt=1.0)
        self.assertEqual(self.node.handover.mode, "STOPPING")
        self.assertEqual(self.node.duty_pub.values, [])
        self.assertTrue(all(value >= 4.0 for value in self.node.brake_current_pub.values))
        self.assertTrue(all(value == 0.6 for value in self.node.servo_pub.values))
        self.assertEqual(self.node.auto_enable_pub.values, [False])

    def test_held_controls_prevent_switch_then_manual_starts_with_fresh_input(self):
        self.joy(x=1, rt=1.0, rb=1)
        for now in (0.1, 0.3, 0.5, 0.7):
            self.tick(now, erpm=0, rt=1.0, rb=1)
        self.assertEqual(self.node.handover.mode, "STOPPING")
        self.tick(0.8, erpm=0)
        self.assertEqual(self.node.handover.mode, "MANUAL")
        self.assertEqual(self.node.duty_pub.values, [])
        self.assertEqual(self.node.brake_current_pub.values[-1], 0.0)
        self.assertEqual(self.node.duty_profile.gear.name, "FORWARD")
        self.tick(0.82, rt=1.0)
        self.assertAlmostEqual(self.node.duty_pub.values[-1], 0.05)
        self.assertEqual(self.node.mode_pub.values, ["AUTO", "STOPPING", "MANUAL"])

    def test_missing_telemetry_or_joystick_keeps_braking(self):
        self.joy(x=1)
        for now in (0.1, 0.3, 0.5, 0.7):
            self.tick(now)
        self.assertEqual(self.node.handover.mode, "STOPPING")
        # Stationary telemetry alone cannot use a stale neutral joystick frame.
        for now in (1.0, 1.2, 1.4, 1.6):
            self.now = now
            self.node.erpm_sub[2](Message(data=0))
            self.node._on_control_timer()
        self.assertEqual(self.node.handover.mode, "STOPPING")
        self.assertEqual(self.node.duty_pub.values, [])

    def test_manual_mode_drops_auto_even_if_auto_is_reenabled_and_x_is_pressed(self):
        self.stop_and_switch()
        before = list(self.node.duty_pub.values)
        self.node._on_auto_command("duty", Message(data=0.9))
        self.assertEqual(self.node.duty_pub.values, before)
        self.tick(0.8, x=1)
        self.assertEqual(self.node.handover.mode, "MANUAL")

    def test_malformed_joy_can_request_stop_but_cannot_allow_manual(self):
        self.node._on_joy(Message(axes=[], buttons=[0, 0, 1]))
        self.assertEqual(self.node.handover.mode, "STOPPING")
        self.assertFalse(self.node._neutral)
        self.assertGreater(self.node.brake_current_pub.values[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
