#!/usr/bin/env python3
"""ROS 2 node for lane following directly in perspective camera images."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from auto_control.perspective_lane_controller import (
    ControlCommand,
    ControllerConfig,
    PerspectiveLaneController,
)
from auto_control.perspective_lane_estimator import (
    LaneEstimatorConfig,
    PerspectiveLaneEstimator,
)


class PerspectiveLaneNode(Node):
    def __init__(self) -> None:
        super().__init__("perspective_lane_node")
        self._declare_parameters()

        self.drive_enabled = bool(self.get_parameter("drive_enabled").value)
        self.publish_to_vesc = bool(
            self.get_parameter("publish_to_vesc").value
        )
        self.require_vesc_connection = bool(
            self.get_parameter("require_vesc_connection").value
        )
        self.force_lane_mask_input = bool(
            self.get_parameter("force_lane_mask_input").value
        )
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.image_timeout_sec = max(
            0.05,
            float(self.get_parameter("image_timeout_sec").value),
        )
        self.servo_left = float(self.get_parameter("servo_left").value)
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_right = float(self.get_parameter("servo_right").value)

        scan_y_ratios = tuple(
            float(value)
            for value in self.get_parameter("scan_y_ratios").value
        )
        estimator_config = LaneEstimatorConfig(
            processing_width=int(
                self.get_parameter("processing_width").value
            ),
            roi_top_ratio=float(self.get_parameter("roi_top_ratio").value),
            roi_bottom_ratio=float(
                self.get_parameter("roi_bottom_ratio").value
            ),
            roi_top_left_ratio=float(
                self.get_parameter("roi_top_left_ratio").value
            ),
            roi_top_right_ratio=float(
                self.get_parameter("roi_top_right_ratio").value
            ),
            roi_bottom_left_ratio=float(
                self.get_parameter("roi_bottom_left_ratio").value
            ),
            roi_bottom_right_ratio=float(
                self.get_parameter("roi_bottom_right_ratio").value
            ),
            scan_y_ratios=scan_y_ratios,
            scan_band_height_ratio=float(
                self.get_parameter("scan_band_height_ratio").value
            ),
            minimum_band_occupancy=float(
                self.get_parameter("minimum_band_occupancy").value
            ),
            maximum_segment_width_ratio=float(
                self.get_parameter("maximum_segment_width_ratio").value
            ),
            lane_width_top_ratio=float(
                self.get_parameter("lane_width_top_ratio").value
            ),
            lane_width_bottom_ratio=float(
                self.get_parameter("lane_width_bottom_ratio").value
            ),
            pair_minimum_width_scale=float(
                self.get_parameter("pair_minimum_width_scale").value
            ),
            pair_maximum_width_scale=float(
                self.get_parameter("pair_maximum_width_scale").value
            ),
            single_boundary_maximum_error_scale=float(
                self.get_parameter(
                    "single_boundary_maximum_error_scale"
                ).value
            ),
            minimum_brightness=int(
                self.get_parameter("minimum_brightness").value
            ),
            tophat_threshold=int(
                self.get_parameter("tophat_threshold").value
            ),
            tophat_kernel=int(self.get_parameter("tophat_kernel").value),
            blur_kernel=int(self.get_parameter("blur_kernel").value),
            morphology_kernel=int(
                self.get_parameter("morphology_kernel").value
            ),
        )
        controller_config = ControllerConfig(
            lateral_gain=float(self.get_parameter("lateral_gain").value),
            heading_gain=float(self.get_parameter("heading_gain").value),
            derivative_gain=float(
                self.get_parameter("derivative_gain").value
            ),
            steering_deadband=float(
                self.get_parameter("steering_deadband").value
            ),
            steering_filter_alpha=float(
                self.get_parameter("steering_filter_alpha").value
            ),
            maximum_steering_rate_per_sec=float(
                self.get_parameter(
                    "maximum_steering_rate_per_sec"
                ).value
            ),
            minimum_confidence=float(
                self.get_parameter("minimum_confidence").value
            ),
            base_duty=float(self.get_parameter("base_duty").value),
            minimum_duty=float(self.get_parameter("minimum_duty").value),
            steering_slowdown=float(
                self.get_parameter("steering_slowdown").value
            ),
        )
        self.estimator = PerspectiveLaneEstimator(estimator_config)
        self.controller = PerspectiveLaneController(controller_config)

        image_topic = str(self.get_parameter("image_topic").value)
        steering_topic = str(self.get_parameter("steering_topic").value)
        throttle_topic = str(self.get_parameter("throttle_topic").value)
        confidence_topic = str(self.get_parameter("confidence_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        duty_topic = str(self.get_parameter("duty_topic").value)
        servo_topic = str(self.get_parameter("servo_position_topic").value)
        connection_topic = str(
            self.get_parameter("connection_status_topic").value
        )

        latest_qos = QoSProfile(depth=1)
        latest_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_qos.durability = DurabilityPolicy.VOLATILE
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.steering_pub = self.create_publisher(
            Float32, steering_topic, latest_qos
        )
        self.throttle_pub = self.create_publisher(
            Float32, throttle_topic, latest_qos
        )
        self.confidence_pub = self.create_publisher(
            Float32, confidence_topic, latest_qos
        )
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.duty_pub = self.create_publisher(Float32, duty_topic, latest_qos)
        self.servo_pub = self.create_publisher(
            Float32, servo_topic, latest_qos
        )
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                str(self.get_parameter("debug_image_topic").value),
                latest_qos,
            )
            self.debug_mask_pub = self.create_publisher(
                Image,
                str(self.get_parameter("debug_mask_topic").value),
                latest_qos,
            )
        else:
            self.debug_image_pub = None
            self.debug_mask_pub = None

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            latest_qos,
        )
        self.connection_sub = self.create_subscription(
            Bool,
            connection_topic,
            self._on_connection,
            connection_qos,
        )

        self._vesc_connected = False
        self._last_image_time: Time | None = None
        self._last_processed_time: Time | None = None
        self._latest_command = ControlCommand(
            0.0, 0.0, 0.0, 0.0, "WAITING_FOR_IMAGE"
        )
        self._latest_confidence = 0.0
        self._processing_error_count = 0
        self._last_actual_state = "STARTING"

        control_rate_hz = max(
            1.0,
            float(self.get_parameter("control_rate_hz").value),
        )
        status_log_rate_hz = max(
            0.2,
            float(self.get_parameter("status_log_rate_hz").value),
        )
        self.control_timer = self.create_timer(
            1.0 / control_rate_hz,
            self._on_control_timer,
        )
        self.status_timer = self.create_timer(
            1.0 / status_log_rate_hz,
            self._publish_status,
        )

        mode = "ARMED" if self.drive_enabled else "DRY-RUN"
        self.get_logger().info(
            f"Perspective lane follower started in {mode} mode. "
            f"image={image_topic}, BEV=disabled, debug={self.publish_debug}"
        )
        if not self.drive_enabled:
            self.get_logger().warn(
                "drive_enabled=false: lane tracking runs, but no VESC command "
                "is published. Set drive_enabled:=true only after checking "
                "the debug output with the wheels lifted."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/camera/image_rect")
        self.declare_parameter("force_lane_mask_input", False)
        self.declare_parameter("steering_topic", "/auto/steering")
        self.declare_parameter("throttle_topic", "/auto/throttle")
        self.declare_parameter("confidence_topic", "/auto/lane_confidence")
        self.declare_parameter("status_topic", "/auto/status")
        self.declare_parameter("debug_image_topic", "/auto/lane_debug")
        self.declare_parameter("debug_mask_topic", "/auto/lane_mask")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter(
            "servo_position_topic", "/vesc/servo_position"
        )
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("publish_to_vesc", True)
        self.declare_parameter("require_vesc_connection", True)
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("image_timeout_sec", 0.25)

        self.declare_parameter("processing_width", 640)
        self.declare_parameter("roi_top_ratio", 0.44)
        self.declare_parameter("roi_bottom_ratio", 0.80)
        self.declare_parameter("roi_top_left_ratio", 0.33)
        self.declare_parameter("roi_top_right_ratio", 0.67)
        self.declare_parameter("roi_bottom_left_ratio", 0.04)
        self.declare_parameter("roi_bottom_right_ratio", 0.96)
        self.declare_parameter("scan_y_ratios", [0.50, 0.58, 0.67, 0.76])
        self.declare_parameter("scan_band_height_ratio", 0.030)
        self.declare_parameter("minimum_band_occupancy", 0.20)
        self.declare_parameter("maximum_segment_width_ratio", 0.16)
        self.declare_parameter("lane_width_top_ratio", 0.10)
        self.declare_parameter("lane_width_bottom_ratio", 0.62)
        self.declare_parameter("pair_minimum_width_scale", 0.48)
        self.declare_parameter("pair_maximum_width_scale", 1.55)
        self.declare_parameter(
            "single_boundary_maximum_error_scale", 0.30
        )
        self.declare_parameter("minimum_brightness", 130)
        self.declare_parameter("tophat_threshold", 40)
        self.declare_parameter("tophat_kernel", 21)
        self.declare_parameter("blur_kernel", 5)
        self.declare_parameter("morphology_kernel", 3)

        self.declare_parameter("lateral_gain", 0.90)
        self.declare_parameter("heading_gain", 1.35)
        self.declare_parameter("derivative_gain", 0.04)
        self.declare_parameter("steering_deadband", 0.015)
        self.declare_parameter("steering_filter_alpha", 0.35)
        self.declare_parameter("maximum_steering_rate_per_sec", 2.5)
        self.declare_parameter("minimum_confidence", 0.45)
        self.declare_parameter("base_duty", 0.055)
        self.declare_parameter("minimum_duty", 0.050)
        self.declare_parameter("steering_slowdown", 0.55)
        self.declare_parameter("servo_left", 0.98)
        self.declare_parameter("servo_center", 0.46)
        self.declare_parameter("servo_right", 0.02)

    def _on_image(self, msg: Image) -> None:
        now = self.get_clock().now()
        try:
            image, image_is_mask = self._decode_image(msg)
            image_is_mask = image_is_mask or self.force_lane_mask_input
            estimate = self.estimator.estimate(image, image_is_mask)
            dt_sec = 1.0 / 30.0
            if self._last_processed_time is not None:
                dt_sec = (
                    now - self._last_processed_time
                ).nanoseconds / 1_000_000_000.0
            self._latest_command = self.controller.update(
                estimate.observation,
                dt_sec,
            )
            self._latest_confidence = estimate.observation.confidence
            self._last_image_time = now
            self._last_processed_time = now

            if self.publish_debug:
                debug = self.estimator.create_debug_image(image, estimate)
                self.debug_image_pub.publish(
                    self._array_to_image(debug, "bgr8", msg)
                )
                self.debug_mask_pub.publish(
                    self._array_to_image(estimate.mask, "mono8", msg)
                )
        except (ValueError, cv2.error) as exc:
            self._processing_error_count += 1
            self._latest_command = ControlCommand(
                0.0, 0.0, 0.0, 0.0, "IMAGE_ERROR"
            )
            self._latest_confidence = 0.0
            self.get_logger().error(
                f"Lane image processing failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _on_connection(self, msg: Bool) -> None:
        self._vesc_connected = bool(msg.data)

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        image_fresh = False
        if self._last_image_time is not None:
            age_sec = (
                now - self._last_image_time
            ).nanoseconds / 1_000_000_000.0
            image_fresh = age_sec < self.image_timeout_sec

        connection_ok = (
            not self.require_vesc_connection or self._vesc_connected
        )
        tracking_ok = self._latest_command.state in {
            "TRACKING",
            "LOW_CONFIDENCE",
        }
        permitted = (
            self.drive_enabled
            and image_fresh
            and connection_ok
            and tracking_ok
        )

        if permitted:
            steering = self._latest_command.steering
            duty = self._latest_command.duty
            state = self._latest_command.state
        else:
            steering = 0.0
            duty = 0.0
            if not self.drive_enabled:
                state = "DISARMED"
            elif not image_fresh:
                state = "IMAGE_TIMEOUT"
            elif not connection_ok:
                state = "VESC_DISCONNECTED"
            else:
                state = self._latest_command.state

        self._last_actual_state = state
        self.steering_pub.publish(Float32(data=float(steering)))
        self.throttle_pub.publish(Float32(data=float(duty)))
        self.confidence_pub.publish(
            Float32(data=float(self._latest_confidence))
        )

        # In dry-run mode this node never takes ownership of the VESC topics,
        # so it cannot fight an accidentally running manual command publisher.
        if self.publish_to_vesc and self.drive_enabled:
            self.duty_pub.publish(Float32(data=float(duty)))
            self.servo_pub.publish(
                Float32(data=float(self._steering_to_servo(steering)))
            )

    def _publish_status(self) -> None:
        command = self._latest_command
        status = (
            f"state={self._last_actual_state} "
            f"confidence={self._latest_confidence:.2f} "
            f"steering={command.steering:+.3f} duty={command.duty:.4f} "
            f"lateral={command.lateral_error:+.3f} "
            f"heading={command.heading_error:+.3f} "
            f"vesc_connected={self._vesc_connected} "
            f"image_errors={self._processing_error_count}"
        )
        self.status_pub.publish(String(data=status))
        self.get_logger().info(status)

    def _decode_image(self, msg: Image) -> tuple[np.ndarray, bool]:
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)
        if width <= 0 or height <= 0 or step <= 0:
            raise ValueError("image width, height, and step must be positive")
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        encoding = msg.encoding.lower()

        if encoding == "nv12":
            rows = height * 3 // 2
            required = rows * step
            if buffer.size < required:
                raise ValueError(
                    f"short NV12 buffer: {buffer.size} < {required}"
                )
            nv12 = buffer[:required].reshape(rows, step)[:, :width]
            bgr = cv2.cvtColor(
                np.ascontiguousarray(nv12),
                cv2.COLOR_YUV2BGR_NV12,
            )
            return bgr, False

        if encoding in {"mono8", "8uc1"}:
            required = height * step
            if buffer.size < required:
                raise ValueError(
                    f"short mono8 buffer: {buffer.size} < {required}"
                )
            mono = buffer[:required].reshape(height, step)[:, :width]
            return np.ascontiguousarray(mono), True

        if encoding in {"bgr8", "rgb8"}:
            required = height * step
            if buffer.size < required or step < width * 3:
                raise ValueError("invalid packed color image buffer")
            packed = buffer[:required].reshape(height, step)[:, : width * 3]
            color = np.ascontiguousarray(packed.reshape(height, width, 3))
            if encoding == "rgb8":
                color = cv2.cvtColor(color, cv2.COLOR_RGB2BGR)
            return color, False

        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    @staticmethod
    def _array_to_image(
        array: np.ndarray,
        encoding: str,
        source: Image,
    ) -> Image:
        contiguous = np.ascontiguousarray(array)
        output = Image()
        output.header = source.header
        output.height = int(contiguous.shape[0])
        output.width = int(contiguous.shape[1])
        output.encoding = encoding
        output.is_bigendian = False
        output.step = int(contiguous.strides[0])
        output.data = contiguous.tobytes()
        return output

    def _steering_to_servo(self, steering: float) -> float:
        steering = max(-1.0, min(1.0, steering))
        if steering < 0.0:
            return self.servo_center + (
                self.servo_left - self.servo_center
            ) * -steering
        return self.servo_center + (
            self.servo_right - self.servo_center
        ) * steering

    def stop_actuators(self) -> None:
        if not (self.publish_to_vesc and self.drive_enabled):
            return
        for _ in range(3):
            self.duty_pub.publish(Float32(data=0.0))
            self.servo_pub.publish(Float32(data=float(self.servo_center)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerspectiveLaneNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.stop_actuators()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
