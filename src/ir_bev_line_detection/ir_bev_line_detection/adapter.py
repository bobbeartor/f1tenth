"""Preserve camera capture stamps so existing stale-path stops still work."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .image import mono_to_bgr


class MonoBevAdapter(Node):
    def __init__(self):
        super().__init__('ir_bev_adapter')
        topic = self.declare_parameter('input_topic', '/camera/image_bev_ir').value
        output = self.declare_parameter('output_topic', '/ir_bev_line_detection/image_bgr').value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Image, output, qos)
        self.subscription = self.create_subscription(Image, topic, self.on_image, qos)

    def on_image(self, message):
        try:
            if message.encoding != 'mono8' or message.header.frame_id != 'front_axle_bev':
                raise ValueError('Expected mono8 in front_axle_bev')
            data = mono_to_bgr(message.data, message.width, message.height, message.step)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return
        result = Image()
        result.header = message.header
        result.width, result.height = message.width, message.height
        result.encoding, result.is_bigendian = 'bgr8', False
        result.step, result.data = message.width * 3, data
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = MonoBevAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
