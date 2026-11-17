import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from line_detactor.msg import LaneResult
from .selection import LaneSelector
from .ir_length_exit import IrLengthExit
from .path_validation import PathRules
from .telemetry import TunnelPreviewTelemetry
from .raw_lane import RawLane


class TunnelLaneSelector(Node):
    def __init__(self):
        super().__init__('tunnel_lane_selector')
        param = lambda key, default: self.declare_parameter(key, default).value
        rules = PathRules(
            frame_id=param('lane_result_frame_id', 'front_axle_bev'),
            minimum_points=param('path_minimum_points', 8), minimum_span=param('path_minimum_span_m', .12),
            minimum_x=param('path_minimum_x_m', .05), maximum_x=param('path_maximum_x_m', 2.20),
            maximum_gap=param('path_maximum_gap_m', .15), x_max=param('bev_x_max_m', 3.),
            y_max=param('bev_y_max_m', .6), mpp=param('bev_meter_per_pixel', .01))
        length_exit = IrLengthExit(
            enabled=param('ir_length_exit_enabled', False),
            ratio=param('ir_length_exit_ratio', .15),
            confirm_frames=param('ir_length_exit_confirm_frames', 3),
            minimum_reference_m=param('ir_length_exit_minimum_reference_m', .5))
        self.policy = LaneSelector(param('max_age_sec', .2), param('validation_mode', False),
                                   rules=rules, ir_length_exit=length_exit)
        self.last_ir_length_log = 0.
        self.get_logger().info(f'IR length exit: enabled={length_exit.enabled} '
            f'ratio={length_exit.threshold:.3f} confirm_frames={length_exit.confirm_frames} '
            f'minimum_reference_m={length_exit.minimum_reference:.2f}')
        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        status = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # One output per camera: the controller learns the source from the
        # topic, so detector bytes are forwarded unmodified (no decode/encode).
        self.output = {source: self.create_publisher(
            LaneResult, '/tunnel/lane_result/' + source.lower(), sensor) for source in ('RGB', 'IR')}
        self.status = {key: self.create_publisher(String, '/tunnel/' + key, status)
                       for key in ('state', 'target_source', 'selected_source', 'compute_source')}
        self.preview = TunnelPreviewTelemetry()
        self.preview_pub = self.create_publisher(String, '/tunnel/preview_status', status)
        self.preview_timer = self.create_timer(.2, self.publish_preview)
        self.subscriptions_owned = [
            self.create_subscription(String, '/tunnel_tcn/result', self.on_event, sensor),
            self.create_subscription(LaneResult, param('rgb_result_topic', '/line_detactor/result'),
                                     lambda data: self.on_raw_lane('RGB', data), sensor, raw=True),
            self.create_subscription(LaneResult, '/ir_bev_line_detection/result',
                                     lambda data: self.on_raw_lane('IR', data), sensor, raw=True)]
        self.published = None
        self.last_tcn_received = None
        self.tcn_live = False
        self.publish_status()

    def publish_status(self):
        values = (self.policy.state, self.policy.camera, self.policy.selected, self.policy.compute)
        if values == self.published:
            return
        for key, value in zip(self.status, values):
            self.status[key].publish(String(data=value))
        self.published = values
        self.get_logger().info(f'Tunnel {values[0]} | selected={values[2]} compute={values[3]} '
                              f'camera={values[1]} reason={self.policy.reason}')
        if self.policy.last_handover and not self.policy.transitioning:
            self.get_logger().info('Tunnel handover ' + json.dumps(self.policy.last_handover))
        self.publish_preview()

    def publish_preview(self):
        self.preview_pub.publish(String(data='\n'.join(
            self.preview.lines(self.policy.state, time.monotonic()))))

    def on_event(self, message):
        try:
            report = json.loads(message.data)
            stamp = report['source_stamp_ns']
            if type(stamp) is not int or (not self.policy.validation_mode and
                    not 0 <= (self.get_clock().now().nanoseconds - stamp) / 1e9 <= .25):
                return
            self.preview.update(report, time.monotonic())
            self.last_tcn_received = time.monotonic()
            state = report.get('state')
            self.tcn_live = str(report.get('status', '')).startswith('LIVE') and state in ('inside', 'outside')
            event = report.get('event')
            if not self.tcn_live:
                return
            if not self.policy.observe_tcn_state(state, stamp, self.get_clock().now().nanoseconds):
                return
            requested = 'IR' if state == 'inside' else 'RGB'
            # Event packets may be lost with sensor QoS. The TCN retains its
            # accepted state in subsequent reports, so reconcile that state too.
            # Matching reports do not repeat the same source change.
            if requested == self.policy.requested:
                return
            kind = 'entry' if requested == 'IR' else 'exit'
            if isinstance(event, dict) and event.get('type') != kind:
                return
            if self.policy.event(kind, stamp, self.get_clock().now().nanoseconds):
                self.publish_status()
        except (ValueError, TypeError, KeyError, AttributeError):
            self.get_logger().warning('Ignoring malformed TCN result')

    def on_raw_lane(self, source, data):
        try:
            message = RawLane(data, lambda raw: deserialize_message(raw, LaneResult))
        except Exception:
            self.get_logger().warning('Ignoring undecodable lane result')
            return
        self.on_lane(source, message)

    def on_lane(self, source, message):
        # Validation follows the last selected source independently of TCN
        # health, like running that detector/controller alone. TCN only switches.
        if not self.policy.validation_mode and (not self.tcn_live or self.last_tcn_received is None or
                time.monotonic() - self.last_tcn_received > .5):
            return
        previous_latch = self.policy.ir_exit_latched_at
        result = self.policy.lane(source, message, self.get_clock().now().nanoseconds)
        monitor = self.policy.ir_length_exit
        triggered = previous_latch is None and self.policy.ir_exit_latched_at is not None
        if monitor.enabled and source == 'IR' and (triggered or
                time.monotonic() - getattr(self, 'last_ir_length_log', 0.) >= 1.):
            self.last_ir_length_log = time.monotonic()
            current = '--' if monitor.current is None else f'{monitor.current:.3f}'
            ratio = '--' if monitor.ratio is None else f'{monitor.ratio:.3f}'
            self.get_logger().info(f'[IR LENGTH EXIT] current_m={current} '
                f'max_m={monitor.reference:.3f} ratio={ratio} threshold={monitor.threshold:.3f} '
                f'low_frames={monitor.below_count}/{monitor.confirm_frames} triggered={triggered}')
        self.publish_status()
        if result is not None:
            # RawLane forwards the detector's own bytes; plain messages (tests,
            # non-raw transports) are published as they are.
            self.output[source].publish(getattr(result, 'raw', result))


def main(args=None):
    rclpy.init(args=args)
    node = TunnelLaneSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
