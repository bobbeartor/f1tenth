"""Exercise real node callbacks with transport/clock/GPU doubles, without ROS."""
import ast
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch
import unittest
from collections import deque
import numpy as np
from tunnel_tcn_test.loading_status import LoadingStatus
from tunnel_tcn_test.features import FeatureStream, EventDecoder


def load_node():
    modules = {}
    for name in ['rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg',
                 'std_msgs', 'std_msgs.msg', 'ament_index_python', 'ament_index_python.packages']:
        modules[name] = ModuleType(name)
    modules['rclpy.node'].Node = object
    for name in ['QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy', 'HistoryPolicy']:
        setattr(modules['rclpy.qos'], name, object)
    modules['sensor_msgs.msg'].Image = object
    modules['std_msgs.msg'].String = SimpleNamespace
    modules['ament_index_python.packages'].get_package_share_directory = lambda _: ''
    path = Path(__file__).resolve().parents[1] / 'tunnel_tcn_test/node.py'
    spec = importlib.util.spec_from_file_location('tunnel_tcn_test._node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class NodeTests(unittest.TestCase):
    def test_startup_trial_reports_failure_instead_of_ready(self):
        # Execute the real constructor's model-check block with a GPU double.
        path = Path(__file__).resolve().parents[1] / 'tunnel_tcn_test/node.py'
        tree = ast.parse(path.read_text())
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TunnelNode')
        constructor = next(n for n in node_class.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        start = next(i for i, n in enumerate(constructor.body)
                     if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Attribute)
                     and n.targets[0].attr == 'model_status')
        end = next(i for i, n in enumerate(constructor.body[start:], start)
                   if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
                   and n.targets[0].id == 'qos')
        code = compile(ast.Module(body=constructor.body[start:end], type_ignores=[]), str(path), 'exec')
        for output, error in ((np.zeros((1, 2), np.float32), None),
                              (np.full((1, 2), np.nan), RuntimeError),
                              (np.zeros((2,), np.float32), RuntimeError),
                              (None, RuntimeError)):
            reports, errors = [], []
            def infer(features):
                self.assertEqual(features.shape, (1, 17, 32))
                if output is None:
                    raise RuntimeError('engine load/inference failure')
                return output
            node = SimpleNamespace(
                create_publisher=lambda *a: SimpleNamespace(publish=lambda m: reports.append(m.data)),
                get_logger=lambda: SimpleNamespace(info=lambda m: None, error=errors.append))
            namespace = dict(LoadingStatus=LoadingStatus, self=node, model_path='/model.onnx', cache_dir='/cache',
                provenance={'onnx_sha256': 'test'}, np=np, String=SimpleNamespace,
                QoSProfile=lambda **kw: kw,
                ReliabilityPolicy=SimpleNamespace(RELIABLE='reliable'),
                DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL='latched'),
                TensorRTBackend=lambda *a, **kw: SimpleNamespace(infer=infer, engine_path='/engine'))
            if error:
                with self.assertRaises(error):
                    exec(code, namespace)
                self.assertTrue(reports[-1].startswith('FAILED | '))
                self.assertFalse(any(item.startswith('READY | ') for item in reports))
            else:
                exec(code, namespace)
                self.assertTrue(reports[-1].startswith('READY | '))

    def test_headless_main_runs_inference_loop_without_gui_calls(self):
        module = load_node()
        running = [True]
        calls = []
        fake_node = SimpleNamespace(
            declare_parameter=lambda *_: SimpleNamespace(value=False),
            process=lambda: calls.append('process'),
            backend=SimpleNamespace(close=lambda: calls.append('close')),
            destroy_node=lambda: calls.append('destroy'))
        def spin_once(*_, **__):
            running[0] = False
        def unexpected_gui(*_, **__):
            self.fail('Headless TCN must not access a GUI')
        with patch.object(module, 'TunnelNode', return_value=fake_node), \
             patch.object(module, 'rclpy', SimpleNamespace(init=lambda **_: None,
                          ok=lambda: running[0], spin_once=spin_once)), \
             patch.object(module, 'cv2', SimpleNamespace(namedWindow=unexpected_gui,
                          imshow=unexpected_gui, waitKey=unexpected_gui,
                          destroyAllWindows=unexpected_gui)):
            module.main()
        self.assertEqual(calls, ['process', 'close', 'destroy'])

    def setUp(self):
        module = load_node()
        self.node = n = module.TunnelNode.__new__(module.TunnelNode)
        n.fps, n.max_age = 30., .25
        n.stream, n.decoder = FeatureStream(), EventDecoder()
        n.origin_ns = n.last_stamp = n.last_received = None
        n.next_inference = 0.
        n.model_id = "20261023_140042_191723_aug"
        n.last_event = None
        n.times, n.dropped = deque(maxlen=60), 0
        n.probabilities, n.status = np.zeros(2), 'WAITING'
        n.get_logger = lambda: SimpleNamespace(info=lambda _: None, warning=lambda _: None)
        self.clock_ns = 0
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.clock_ns))
        self.outputs = []
        n.result = SimpleNamespace(publish=lambda x: self.outputs.append(json.loads(x.data)))
        self.calls = []
        def infer(x):
            self.calls.append(x)
            return np.array([[10., -10.]], np.float32)
        n.backend = SimpleNamespace(infer=infer)

    def frame(self, frame, age=0.):
        stamp = 1_000_000_000 + round(frame * 1e9 / 30)
        self.clock_ns = stamp + round(age * 1e9)
        message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp // 1_000_000_000,
                                  nanosec=stamp % 1_000_000_000)),
                                  encoding='mono8', width=64, height=40, step=64, data=bytes(64 * 40))
        self.node.on_image(message)
        self.node.process()

    def test_warmup_holds_events_then_emits_one_entry(self):
        for frame in range(80):
            self.frame(frame)
        events = [(i, r['event']) for i, r in enumerate(self.outputs) if r['event']]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], 25)
        self.assertEqual(events[0][1]['type'], 'entry')
        self.assertEqual(self.node.decoder.state, 'inside')
        self.assertEqual(self.calls[-1].shape, (1, 17, 32))

    def test_history_reset_preserves_state_unless_new_drive(self):
        n = self.node
        for frame in range(80):
            self.frame(frame)
        n.clear_history()
        self.assertEqual(n.decoder.state, 'inside')
        self.assertIsNone(n.stream.started)
        self.frame(80)
        self.assertIsNone(self.outputs[-1]['event'])
        self.assertIn('WARMUP', n.status)
        n.clear_history(new_drive=True)
        self.assertEqual(n.decoder.state, 'unknown')
        self.assertIsNone(n.last_event)

    def test_stale_and_duplicate_images_do_not_infer(self):
        self.frame(0)
        self.frame(0)
        self.assertEqual(len(self.calls), 1)
        self.frame(1, age=.3)
        self.assertEqual(len(self.calls), 1)
        self.assertIn('STALE', self.node.status)
        self.frame(3)
        self.assertEqual(len(self.calls), 2)
        self.assertFalse(self.node.stream.ready)

    def test_missing_frames_are_counted_and_long_gap_holds_events(self):
        for frame in range(80):
            self.frame(frame)
        self.frame(112)
        self.assertEqual(self.node.dropped, 32)
        self.assertIn('WARMUP', self.node.status)
        self.assertEqual(self.node.decoder.state, 'inside')
        self.assertIsNone(self.outputs[-1]['event'])


if __name__ == '__main__':
    unittest.main()
