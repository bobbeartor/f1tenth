import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
from auto_control_tunnel.model_check import ModelChecks


def load_monitor():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.node'].Node = object
    for name in ('QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy'):
        setattr(modules['rclpy.qos'], name, object)
    modules['std_msgs.msg'].String = NS
    modules['std_msgs.msg'].Bool = NS
    path = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/model_monitor.py'
    spec = importlib.util.spec_from_file_location('auto_control_tunnel._monitor_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class ModelCheckTests(unittest.TestCase):
    def test_all_enabled_models_need_trial_success(self):
        checks = ModelChecks(['rgb', 'ir', 'tcn', 'traffic'])
        for name in ('rgb', 'ir', 'tcn'):
            checks.update(name, 'READY | trial inference=PASS')
        self.assertFalse(checks.ready)
        checks.update('traffic', 'LOADING | engine')
        self.assertEqual(checks.pending, ['traffic'])
        checks.update('traffic', 'FAILED | corrupt engine')
        self.assertFalse(checks.ready)
        checks.update('traffic', 'READY | trial inference=PASS')
        self.assertTrue(checks.ready)
        checks.update('rgb', 'LOADING | restarted')
        self.assertFalse(checks.ready)

    def test_unknown_invalid_and_duplicate_reports_do_not_change_state(self):
        checks = ModelChecks(['rgb'])
        self.assertFalse(checks.update('unknown', 'READY | pass'))
        self.assertFalse(checks.update('rgb', 'file exists'))
        self.assertTrue(checks.update('rgb', 'READY | trial inference=PASS'))
        self.assertFalse(checks.update('rgb', 'READY | trial inference=PASS'))
        self.assertTrue(checks.ready)

    def test_timeout_and_late_success_are_visible(self):
        module = load_monitor()
        monitor = module.ModelStartupMonitor.__new__(module.ModelStartupMonitor)
        monitor.checks = ModelChecks(['rgb', 'tcn'])
        monitor.timeout, monitor.started, monitor.last_wait_log = 300., 0., 0.
        monitor.timed_out = monitor.all_ready_reported = False
        infos, errors, ready = [], [], []
        monitor.ready_pub = NS(publish=lambda message: ready.append(message.data))
        monitor.get_logger = lambda: NS(info=infos.append, error=errors.append)
        monitor.on_status('rgb', NS(data='READY | trial inference=PASS'))
        with patch.object(module.time, 'monotonic', return_value=301.):
            monitor.tick()
            monitor.tick()
        self.assertEqual(len(errors), 1)
        self.assertIn('NOT READY', errors[0])
        self.assertIn('tcn', errors[0])
        self.assertFalse(any('ALL READY' in item for item in infos))
        monitor.on_status('tcn', NS(data='FAILED | bad output'))
        self.assertIn('FAIL tcn', errors[-1])
        monitor.on_status('tcn', NS(data='READY | trial inference=PASS'))
        monitor.on_status('tcn', NS(data='READY | trial inference=PASS'))
        self.assertEqual(sum('ALL READY' in item for item in infos), 1)
        self.assertEqual(ready, [False, False, False, False, True])
        monitor.on_status('rgb', NS(data='LOADING | restarted'))
        self.assertFalse(ready[-1])
        with patch.object(module.time, 'monotonic', return_value=302.):
            monitor.tick()
        self.assertFalse(ready[-1])


if __name__ == '__main__':
    unittest.main()
