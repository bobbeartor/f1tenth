import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import tempfile
import sys
import unittest
from unittest.mock import patch

import yaml

from auto_control_tunnel.parameter_preset import parameter_values, write_preset


def load_runtime_tools():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.parameter', 'auto_control_tunnel.remote_parameters',
        'rclpy.qos', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv')}
    for mod, attrs in {
        'rclpy.node': ['Node'], 'rclpy.parameter': ['Parameter'],
        'auto_control_tunnel.remote_parameters': ['RemoteParameters'],
        'rclpy.qos': ['QoSProfile', 'DurabilityPolicy'],
        'std_msgs.msg': ['Bool', 'String'], 'std_srvs.srv': ['Trigger'],
    }.items():
        for attr in attrs:
            setattr(modules[mod], attr, object)
    path = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/runtime_tools.py'
    spec = importlib.util.spec_from_file_location('auto_control_tunnel._runtime_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    sys.modules[spec.name] = module
    return module.RuntimeTools


class PresetTests(unittest.TestCase):
    def test_all_ros_types_survive_yaml_round_trip(self):
        samples = [
            (1, 'bool_value', True), (2, 'integer_value', 9007199254740993),
            (3, 'double_value', 0.125), (4, 'string_value', '카메라: on'),
            (5, 'byte_array_value', [b'\x00', b'\xff']),
            (6, 'bool_array_value', [True, False]),
            (7, 'integer_array_value', [1, 2]),
            (8, 'double_array_value', [0.1, 0.2]),
            (9, 'string_array_value', ['on', '001', 'true']),
            (9, 'string_array_value', []),
        ]
        names = [f'p{i}' for i in range(len(samples))]
        values = [NS(type=t, **{field: value}) for t, field, value in samples]
        converted = parameter_values(names, values)
        self.assertEqual(converted['p4'], [0, 255])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '설정.yaml'
            write_preset(path, {'/auto_control': converted, '/camera_driver': {'sensor_fps': 80.0}})
            actual = yaml.safe_load(path.read_text())
        self.assertEqual(actual['auto_control']['ros__parameters'], converted)
        self.assertEqual(actual['camera_driver']['ros__parameters']['sensor_fps'], 80.0)
        self.assertIs(type(actual['auto_control']['ros__parameters']['p1']), int)

    def test_missing_or_invalid_responses_are_rejected(self):
        with self.assertRaises(ValueError):
            parameter_values(['missing'], [])
        for kind in (0, 99):
            with self.assertRaises(ValueError):
                parameter_values(['bad'], [NS(type=kind)])

    def test_failed_write_preserves_existing_preset_and_removes_temp_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'preset.yaml'
            path.write_text('existing')
            with patch('auto_control_tunnel.parameter_preset.os.replace', side_effect=OSError('disk')):
                with self.assertRaises(OSError):
                    write_preset(path, {'/auto_control': {'maximum_speed_mps': 1.0}})
            self.assertEqual(path.read_text(), 'existing')
            self.assertEqual(list(Path(directory).iterdir()), [path])


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        self.cls = load_runtime_tools()
        self.node = object.__new__(self.cls)
        self.node.windows = {'parameters': object()}
        self.messages = []
        self.node.preset_status = NS(set=self.messages.append)
        self.node.update_preset_status = lambda: None
        self.request = {'path': '/unused.yaml', 'remaining': {'/a', '/b'}, 'snapshot': {}}
        self.node.preset_request = self.request

    def response(self, value):
        return NS(result=lambda: NS(values=[NS(type=3, double_value=value)]))

    def test_waits_for_every_node_and_writes_fresh_applied_values(self):
        with patch('auto_control_tunnel._runtime_test.write_preset') as write:
            # GUI entries are intentionally not consulted; these are service replies.
            self.node.preset_loaded(self.request, '/b', ['speed'], self.response(0.4))
            write.assert_not_called()
            self.node.preset_loaded(self.request, '/a', ['speed'], self.response(1.2))
            write.assert_called_once_with('/unused.yaml', {'/b': {'speed': 0.4}, '/a': {'speed': 1.2}})
        self.assertIsNone(self.node.preset_request)
        self.assertIn('저장 완료', self.messages[-1])

    def test_timeout_discards_partial_snapshot_and_ignores_late_reply(self):
        with patch('auto_control_tunnel._runtime_test.write_preset') as write:
            self.node.preset_loaded(self.request, '/a', ['speed'], self.response(1.2))
            self.node.preset_failed(self.request, 'timeout')
            self.node.preset_loaded(self.request, '/b', ['speed'], self.response(0.4))
            write.assert_not_called()
        self.assertIn('저장 실패', self.messages[-1])

    def test_service_error_does_not_save_partial_snapshot(self):
        with patch('auto_control_tunnel._runtime_test.write_preset') as write:
            self.node.preset_loaded(self.request, '/a', ['speed'], NS(result=lambda: NS(values=[])))
            write.assert_not_called()
        self.assertIsNone(self.node.preset_request)
        self.assertIn('저장 실패', self.messages[-1])

    def test_closed_window_cancels_save_and_late_callbacks(self):
        self.node.preset_request = None
        self.node.windows = {}
        with patch('auto_control_tunnel._runtime_test.write_preset') as write:
            self.node.preset_loaded(self.request, '/a', ['speed'], self.response(1.2))
            self.node.preset_failed(self.request, 'timeout')
            write.assert_not_called()
        self.assertEqual(self.messages, [])


if __name__ == '__main__':
    unittest.main()
