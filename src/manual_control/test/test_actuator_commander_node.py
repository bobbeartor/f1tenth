"""Manual actuator node: stale RT must not drive after joystick input loss.

Uses the same in-memory ROS doubles as test_auto_handover_node; no hardware.
"""

import importlib
import sys
import unittest
from unittest.mock import patch

from test_auto_handover_node import Message, ros_doubles


class ActuatorCommanderNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        doubles = ros_doubles()
        doubles["manual_control.actuator_commander_node"] = None
        cls.module_patch = patch.dict(sys.modules, doubles)
        cls.module_patch.start()
        del sys.modules["manual_control.actuator_commander_node"]
        cls.module = importlib.import_module(
            "manual_control.actuator_commander_node"
        )

    @classmethod
    def tearDownClass(cls):
        cls.module_patch.stop()

    def setUp(self):
        self.node = self.module.ActuatorCommanderNode()
        self.now = 0.0

    def step(self, dt=0.0125, rt=0.0, lt=0.0):
        self.now += dt
        self.node.clock_sec = self.now
        buttons = [0] * 15
        self.node._on_joy(
            Message(axes=[0.0, 0, 0, 0, lt, rt], buttons=buttons)
        )
        self.node._on_control_timer()

    def lose_input(self, seconds=0.5):
        self.now += seconds
        self.node.clock_sec = self.now
        self.node._on_control_timer()

    def test_rt_held_at_startup_is_ignored_until_released(self):
        for _ in range(10):
            self.step(rt=1.0)
        self.assertEqual(self.node.duty_profile.command_duty, 0.0)
        self.step(rt=0.0)
        self.step(rt=1.0)
        self.assertGreater(self.node.duty_profile.command_duty, 0.0)

    def test_rt_held_after_reconnect_does_not_drive(self):
        self.step(rt=0.0)
        for _ in range(20):
            self.step(rt=1.0)
        self.assertGreater(self.node.duty_profile.command_duty, 0.0)

        self.lose_input()  # longer than input_timeout_sec (0.3s)
        self.assertEqual(self.node.duty_profile.command_duty, 0.0)

        # Reconnected while RT is still reported as pressed.
        for _ in range(10):
            self.step(rt=1.0)
        self.assertEqual(self.node.duty_profile.command_duty, 0.0)
        self.assertEqual(self.node.duty_pub.values[-1], 0.0)

        self.step(rt=0.0)
        self.step(rt=1.0)
        self.assertGreater(self.node.duty_profile.command_duty, 0.0)

    def test_brake_still_works_while_rt_is_locked(self):
        self.lose_input()
        for _ in range(10):
            self.step(rt=1.0, lt=1.0)
        self.assertGreater(self.node.brake_current_pub.values[-1], 0.0)


if __name__ == "__main__":
    unittest.main()
