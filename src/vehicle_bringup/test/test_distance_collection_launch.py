"""Validate collection startup/exit routing without ROS or vehicle hardware."""
import ast
import os
from pathlib import Path
import re
import tempfile
import unittest


class Config:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


def action(*args, **kwargs):
    return {'args': args, **kwargs}


class CollectionLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).parents[1]/'launch/manual_drive_distance_collection.launch.py'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_launch_collection')
        namespace = dict(Path=Path, os=os, re=re, LaunchConfiguration=Config,
                         ParameterValue=action, Node=action, IncludeLaunchDescription=action,
                         PythonLaunchDescriptionSource=action, PathJoinSubstitution=action,
                         FindPackageShare=action, RegisterEventHandler=action,
                         OnProcessExit=action, EmitEvent=action, Shutdown=action)
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        cls.configure = staticmethod(namespace['_launch_collection'])

    def test_single_camera_collector_and_manual_routing(self):
        with tempfile.TemporaryDirectory() as tmp:
            context = dict(sequence='manual_test_1', output_directory=tmp)
            actions = self.configure(context)
            self.assertEqual(len(actions), 3)
            handler, collector, manual = actions
            self.assertEqual(collector['package'], 'multiclass_yolo')
            self.assertEqual(collector['executable'], 'yolo_distance_node')
            self.assertEqual(collector['parameters'][0]['mode'], 'collect')
            self.assertEqual(collector['parameters'][0]['sequence'], 'manual_test_1')
            self.assertIs(handler['args'][0]['target_action'], collector)
            self.assertEqual(handler['args'][0]['on_exit'][0]['event']['reason'], 'Distance collector exited')
            self.assertEqual(set(dict(manual['launch_arguments'])),
                             {'vehicle_namespace', 'vesc_port', 'controller_name_contains'})
            self.assertIn('manual_drive.launch.py', str(manual))
            self.assertNotIn('camera_driver', str(actions))
            self.assertNotIn('auto_drive.launch.py', str(actions))

    def test_existing_sequence_refused_before_nodes_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp)/'used').mkdir()
            with self.assertRaisesRegex(RuntimeError, 'already exists'):
                self.configure(dict(sequence='used', output_directory=tmp))

    def test_invalid_sequence_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            for sequence in ['', '../outside', 'run/name', 'run name']:
                with self.subTest(sequence=sequence), self.assertRaises(RuntimeError):
                    self.configure(dict(sequence=sequence, output_directory=tmp))

if __name__ == '__main__':
    unittest.main()
