"""Check launch routing without requiring a ROS installation."""

import ast
from pathlib import Path
from types import SimpleNamespace
import unittest


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context.launch_configurations[self.name]


class HandoverLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = Path(__file__).parents[1] / "launch" / "auto_drive.launch.py"
        tree = ast.parse(source.read_text())
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "_configure_handover")
        import os
        namespace = {
            "os": os, "LaunchConfiguration": LaunchConfiguration,
            "get_package_share_directory": lambda name: f"/share/{name}",
            "PythonLaunchDescriptionSource": lambda path: path,
            "IncludeLaunchDescription": lambda source, **kw: {"source": source, **kw},
            "Node": lambda **kw: kw,
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), "exec"), namespace)
        cls.configure = staticmethod(namespace["_configure_handover"])

    def context(self, **overrides):
        values = {
            "manual_handover_enabled": "true", "manual_test": "false",
            "auto_control_mode": "drive",
            "controller_name_contains": "8BitDo",
            **{f"effective_{name}_topic": f"/test_vehicle/vesc/{name}" for name in (
                "duty", "brake_current", "servo_position", "measured_erpm", "connection_status")},
        }
        values.update(overrides)
        return SimpleNamespace(launch_configurations=values)

    def test_auto_commands_are_private_and_only_handover_targets_the_bridge(self):
        context = self.context()
        actions = self.configure(context, {"enable_topic": "/test_auto/enabled"}, "car03")
        self.assertEqual(len(actions), 2)
        self.assertEqual(actions[1]["executable"], "auto_handover_node")
        parameters = actions[1]["parameters"][-1]
        for channel in ("duty", "brake_current", "servo_position"):
            self.assertEqual(parameters[f"{channel}_topic"], f"/test_vehicle/vesc/{channel}")
            self.assertEqual(parameters[f"auto_{channel}_topic"], f"/car03/handover/auto/{channel}")
            self.assertEqual(context.launch_configurations[f"effective_{channel}_topic"],
                             parameters[f"auto_{channel}_topic"])
            self.assertEqual(context.launch_configurations[f"bridge_{channel}_topic"],
                             parameters[f"{channel}_topic"])
        self.assertEqual(parameters["auto_enable_topic"], "/test_auto/enabled")
        self.assertEqual(parameters["joy_topic"], "/car03/joy")
        self.assertEqual(context.launch_configurations["dynamics_commanded_duty_topic"],
                         parameters["current_duty_topic"])
        self.assertEqual(dict(actions[0]["launch_arguments"])["vehicle_namespace"], "car03")

    def test_passive_modes_and_existing_manual_test_do_not_start_another_controller(self):
        for overrides in (
            {"manual_handover_enabled": "false"}, {"manual_test": "true"},
            {"auto_control_mode": "monitor_only"},
            {"auto_control_mode": "steering_only"},
        ):
            with self.subTest(overrides=overrides):
                context = self.context(**overrides)
                self.assertEqual(self.configure(context, {}, "car03"), [])
                self.assertEqual(context.launch_configurations["effective_duty_topic"],
                                 context.launch_configurations["bridge_duty_topic"])

    def test_empty_namespace_produces_root_topics(self):
        actions = self.configure(self.context(), {}, "")
        self.assertEqual(actions[1]["parameters"][-1]["joy_topic"], "/joy")
        self.assertEqual(actions[1]["parameters"][-1]["auto_duty_topic"], "/handover/auto/duty")

    def test_relative_controller_topics_keep_root_resolution(self):
        actions = self.configure(self.context(effective_duty_topic="vesc/duty"),
                                 {"enable_topic": "auto/enabled"}, "car03")
        self.assertEqual(actions[1]["parameters"][-1]["duty_topic"], "/vesc/duty")
        self.assertEqual(actions[1]["parameters"][-1]["auto_enable_topic"], "/auto/enabled")


if __name__ == "__main__":
    unittest.main()
