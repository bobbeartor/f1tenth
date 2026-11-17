"""Summarize actual engine load + trial-inference reports at auto_drive startup."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, String
from .model_check import ModelChecks


class ModelStartupMonitor(Node):
    def __init__(self):
        super().__init__('model_startup_check')
        expected = self.declare_parameter('expected_nodes', ['line_detactor']).value
        self.checks = ModelChecks(expected)
        self.timeout = float(self.declare_parameter('timeout_sec', 300.).value)
        if self.timeout <= 0:
            raise ValueError('Model check timeout must be positive')
        self.started = time.monotonic()
        self.last_wait_log = self.started
        self.timed_out = False
        self.all_ready_reported = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, '/perception/models_ready', qos)
        self.ready_pub.publish(Bool(data=False))
        self.subscriptions_owned = [self.create_subscription(
            String, f'/{name}/model_status', lambda message, name=name: self.on_status(name, message), qos)
            for name in expected]
        self.timer = self.create_timer(1., self.tick)
        self.get_logger().info('[MODEL CHECK] Checking: ' + ', '.join(expected))

    def on_status(self, name, message):
        if not self.checks.update(name, message.data):
            return
        if message.data.startswith('FAILED | '):
            self.get_logger().error(f'[MODEL CHECK] FAIL {name}: {message.data}')
        elif message.data.startswith('READY | '):
            self.get_logger().info(f'[MODEL CHECK] PASS {name}: {message.data}')
        self.report_complete()

    def report_complete(self):
        self.ready_pub.publish(Bool(data=self.checks.ready))
        if self.checks.ready and not self.all_ready_reported:
            self.get_logger().info(f'[MODEL CHECK] ALL READY ({len(self.checks.status)} models)')
        self.all_ready_reported = self.checks.ready

    def tick(self):
        self.ready_pub.publish(Bool(data=self.checks.ready))
        if self.checks.ready:
            return
        now = time.monotonic()
        pending = ', '.join(self.checks.pending)
        if now - self.started >= self.timeout and not self.timed_out:
            self.get_logger().error(
                f'[MODEL CHECK] NOT READY after {self.timeout:.0f}s: {pending}; '
                'check model/engine errors or failed component loading. Automatic driving remains blocked; monitoring continues.')
            self.timed_out = True
        if now - self.last_wait_log >= 10.:
            self.get_logger().info(f'[MODEL CHECK] Waiting: {pending}')
            self.last_wait_log = now


def main(args=None):
    rclpy.init(args=args)
    node = ModelStartupMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
