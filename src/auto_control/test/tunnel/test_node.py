"""Transport/clock doubles exercise the real selector callbacks without ROS."""
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch
from auto_control_tunnel.selection import LaneSelector
from auto_control_tunnel.ir_length_exit import IrLengthExit
from lane_fixture import lane, ns


def load_node():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.serialization', 'std_msgs', 'std_msgs.msg',
        'line_detactor', 'line_detactor.msg')}
    modules['rclpy.node'].Node = object
    for name in ('QoSProfile', 'ReliabilityPolicy', 'DurabilityPolicy'):
        setattr(modules['rclpy.qos'], name, object)
    modules['std_msgs.msg'].String = NS
    modules['rclpy.serialization'].deserialize_message = lambda raw, _type: None
    modules['line_detactor.msg'].LaneResult = object
    path = Path(__file__).resolve().parents[2] / 'auto_control_tunnel/node.py'
    spec = importlib.util.spec_from_file_location('auto_control_tunnel._node_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict('sys.modules', modules):
        spec.loader.exec_module(module)
    return module


class CallbackTests(unittest.TestCase):
    def setUp(self):
        self.module = load_node()
        self.node = self.module.TunnelLaneSelector.__new__(self.module.TunnelLaneSelector)
        n = self.node
        n.preview = self.module.TunnelPreviewTelemetry()
        self.message = lane(10, valid=False)
        n.policy = LaneSelector()
        n.last_tcn_received, n.tcn_live = None, False
        n.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_010_000_000))
        n.get_logger = lambda: NS(warning=lambda _: None, info=lambda _: None)
        n.publish_status = lambda: None
        self.outputs = []
        n.output = {source: NS(publish=self.outputs.append) for source in ('RGB', 'IR')}

    def report(self, **overrides):
        report = dict(source_stamp_ns=10_000_000_000, status='LIVE', state='outside', event=None)
        report.update(overrides)
        self.node.on_event(NS(data=json.dumps(report)))

    def test_live_tcn_routes_original_capture_without_rewriting_stamp(self):
        self.report()
        self.node.on_lane('RGB', self.message)
        self.assertEqual(self.outputs, [self.message])
        self.assertFalse(self.outputs[0].centerline_valid)
        self.assertEqual(self.outputs[0].header.stamp.sec, 10)

    def test_raw_lane_is_forwarded_as_original_bytes_without_decoding(self):
        import struct
        decoded = []
        self.module.deserialize_message = lambda raw, _type: decoded.append(raw) or self.message
        data = b'\x00\x01\x00\x00' + struct.pack('<iI', 10, 5) + b'payload'
        self.report()
        self.node.on_raw_lane('RGB', data)
        self.assertEqual(self.outputs, [data])
        self.assertEqual(decoded, [])
        self.assertEqual(self.node.policy.last_input_ns['RGB'], 10_000_000_005)

    def test_warmup_and_tcn_silence_withhold_lane_results(self):
        self.node.on_lane('RGB', self.message)
        self.report(status='WARMUP 2.5s')
        self.node.on_lane('RGB', self.message)
        self.report()
        with patch.object(self.module.time, 'monotonic', return_value=self.node.last_tcn_received + .6):
            self.node.on_lane('RGB', self.message)
        self.assertEqual(self.outputs, [])

    def test_validation_routes_rgb_during_tcn_warmup_then_keeps_ir_during_tcn_loss(self):
        self.node.policy = LaneSelector(validation_mode=True)
        self.node.on_lane('RGB', self.message)
        self.assertEqual(self.outputs, [self.message])
        self.report(status='WARMUP 2.5s')
        # Validation accepts an old LIVE event, then prepares a new target path.
        self.report(state='inside', source_stamp_ns=9_000_000_000)
        self.assertEqual(self.node.policy.state, 'ENTERING')
        self.node.tcn_live = False
        with patch.object(self.module.time, 'monotonic', return_value=100000.):
            self.node.on_lane('IR', self.message)
        self.assertEqual(self.outputs, [self.message])
        valid_ir = lane(10.02)
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=ns(10.025)))
        self.node.on_lane('IR', valid_ir)
        self.assertEqual(self.node.policy.selected, 'IR')
        self.assertEqual(self.outputs, [self.message, valid_ir])
        # Does not fabricate a new result or repeat one already delivered.
        self.node.on_lane('IR', valid_ir)
        self.assertEqual(len(self.outputs), 2)

    def test_real_tcn_event_schema_requests_entry(self):
        self.report(state='inside', event=dict(type='entry', score=.9, time_s=3.))
        self.assertEqual(self.node.policy.state, 'ENTERING')
        self.assertEqual(self.node.policy.selected, 'RGB')
        self.assertEqual(self.node.policy.compute, 'BOTH')

    def test_callbacks_keep_rgb_until_first_valid_ir_and_publish_original_immediately(self):
        self.node.policy = LaneSelector(validation_mode=True)
        rgb = lane(10.)
        self.node.on_lane('RGB', rgb)
        self.report(state='inside')
        self.node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=ns(10.04)))
        self.node.on_lane('IR', lane(10.02, valid=False))
        rgb_next = lane(10.03)
        self.node.on_lane('RGB', rgb_next)
        self.assertEqual(self.outputs, [rgb, rgb_next])
        target = lane(10.035, lateral=.1)
        self.node.on_lane('IR', target)
        self.assertIs(self.outputs[-1], target)
        self.assertEqual(target.lane_source, 'IR')
        self.assertEqual(self.node.policy.state, 'INSIDE')
        self.assertEqual(self.node.policy.compute, 'IR')

    def test_retained_tcn_state_recovers_lost_event_without_repeating_switch(self):
        self.report(state='inside')
        self.assertEqual(self.node.policy.state, 'ENTERING')
        event_stamp = self.node.policy.last_event_ns
        self.report(state='inside', source_stamp_ns=10_005_000_000)
        self.assertEqual(self.node.policy.last_event_ns, event_stamp)

    def test_length_exit_callback_is_not_reversed_by_retained_tcn_inside(self):
        self.node.policy = LaneSelector(validation_mode=True,
            ir_length_exit=IrLengthExit(enabled=True))
        self.report(state='inside')
        self.node.on_lane('IR',lane(10.001))
        for index,length in enumerate((2.,2.,2.,0.,0.,0.)):
            t=10.02+index*.03
            self.node.get_clock=lambda t=t: NS(now=lambda: NS(nanoseconds=ns(t+.005)))
            message=lane(t,valid=length>0)
            message.observed_lane_length_valid=True
            message.observed_lane_length_m=length
            self.node.on_lane('IR',message)
        self.assertEqual(self.node.policy.state,'EXITING')
        self.report(state='inside',source_stamp_ns=ns(10.17))
        self.assertEqual(self.node.policy.state,'EXITING')
        rgb=lane(10.171)
        self.node.on_lane('RGB',rgb)
        self.assertIs(self.outputs[-1],rgb)
        self.assertEqual(self.node.policy.state,'OUTSIDE')
        self.report(state='inside',source_stamp_ns=ns(10.172))
        self.assertEqual(self.node.policy.state,'OUTSIDE')

    def test_stale_or_malformed_tcn_does_not_start_motion_or_transition(self):
        for data in ('[]', '{', '{"source_stamp_ns": true}',
                     json.dumps(dict(source_stamp_ns=9_000_000_000, status='LIVE',
                                     event=dict(type='entry')))):
            self.node.on_event(NS(data=data))
        self.assertFalse(self.node.tcn_live)
        self.assertEqual(self.node.policy.state, 'OUTSIDE')


if __name__ == '__main__':
    unittest.main()
