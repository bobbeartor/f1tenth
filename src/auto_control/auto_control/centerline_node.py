#!/usr/bin/env python3
"""ROS 2 node for high-rate centerline-following vehicle control."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from auto_control.centerline_controller import (
    CenterlineController,
    CenterlinePath,
    ControlCommand,
    ControllerConfig,
    DEFAULT_SERVO_CENTER,
    DEFAULT_SERVO_LEFT,
    DEFAULT_SERVO_RIGHT,
    StartupDutyConfig,
    StartupDutyProfile,
    steering_to_servo,
)
from auto_control.lane_model import LaneModel, LaneModelConfig


class CenterlineNode(Node):
    def __init__(self) -> None:
        super().__init__("centerline_node")
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
        self.debug_background_topic = str(
            self.get_parameter("debug_background_topic").value
        )
        self.image_timeout_sec = max(
            0.05,
            float(self.get_parameter("image_timeout_sec").value),
        )
        self.servo_left = float(self.get_parameter("servo_left").value)
        self.servo_center = float(self.get_parameter("servo_center").value)
        self.servo_right = float(self.get_parameter("servo_right").value)

        model_config = LaneModelConfig(
            processing_width=int(
                self.get_parameter("processing_width").value
            ),
            processing_height=int(
                self.get_parameter("processing_height").value
            ),
            roi_y_min=int(self.get_parameter("roi_y_min").value),
            roi_y_max=int(self.get_parameter("roi_y_max").value),
            white_threshold=int(
                self.get_parameter("white_threshold").value
            ),
            morphology_kernel=int(
                self.get_parameter("morphology_kernel").value
            ),
            minimum_points_per_boundary=int(
                self.get_parameter("minimum_points_per_boundary").value
            ),
            tracking_margin_px=float(
                self.get_parameter("tracking_margin_px").value
            ),
            trace_seed_search_rows=int(
                self.get_parameter("trace_seed_search_rows").value
            ),
            trace_centering_max_run_width=int(
                self.get_parameter("trace_centering_max_run_width").value
            ),
            expected_lane_width_y_ratios=tuple(
                float(value)
                for value in self.get_parameter(
                    "expected_lane_width_y_ratios"
                ).value
            ),
            expected_lane_width_ratios=tuple(
                float(value)
                for value in self.get_parameter(
                    "expected_lane_width_ratios"
                ).value
            ),
            lane_width_minimum_scale=float(
                self.get_parameter("lane_width_minimum_scale").value
            ),
            lane_width_maximum_scale=float(
                self.get_parameter("lane_width_maximum_scale").value
            ),
            maximum_fit_residual_px=float(
                self.get_parameter("maximum_fit_residual_px").value
            ),
            lane_width_learning_alpha=float(
                self.get_parameter("lane_width_learning_alpha").value
            ),
            single_lane_confidence_scale=float(
                self.get_parameter("single_lane_confidence_scale").value
            ),
            maximum_extrapolation_rows=int(
                self.get_parameter("maximum_extrapolation_rows").value
            ),
            single_lane_curvature_gain=float(
                self.get_parameter("single_lane_curvature_gain").value
            ),
            single_lane_maximum_offset_scale=float(
                self.get_parameter(
                    "single_lane_maximum_offset_scale"
                ).value
            ),
            single_lane_direction_guard_enabled=bool(
                self.get_parameter(
                    "single_lane_direction_guard_enabled"
                ).value
            ),
            single_lane_convexity_tolerance=float(
                self.get_parameter(
                    "single_lane_convexity_tolerance"
                ).value
            ),
        )
        controller_config = ControllerConfig(
            lookahead_y=int(self.get_parameter("lookahead_y").value),
            cross_track_gain=float(
                self.get_parameter("cross_track_gain").value
            ),
            cross_track_error_boost_gain=float(
                self.get_parameter("cross_track_error_boost_gain").value
            ),
            preview_gain=float(self.get_parameter("preview_gain").value),
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
        self.model = LaneModel(model_config)
        self.controller = CenterlineController(controller_config)
        self.startup_duty_profile = StartupDutyProfile(
            StartupDutyConfig(
                boost_duty=float(
                    self.get_parameter("startup_boost_duty").value
                ),
                boost_duration_sec=float(
                    self.get_parameter(
                        "startup_boost_duration_sec"
                    ).value
                ),
                ramp_down_sec=float(
                    self.get_parameter("startup_ramp_down_sec").value
                ),
                stable_tracking_sec=float(
                    self.get_parameter(
                        "startup_stable_tracking_sec"
                    ).value
                ),
                rearm_stop_sec=float(
                    self.get_parameter("startup_rearm_stop_sec").value
                ),
            )
        )

        latest_qos = QoSProfile(depth=1)
        latest_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        latest_qos.durability = DurabilityPolicy.VOLATILE
        connection_qos = QoSProfile(depth=1)
        connection_qos.reliability = ReliabilityPolicy.RELIABLE
        connection_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.steering_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("steering_topic").value),
            latest_qos,
        )
        self.throttle_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("throttle_topic").value),
            latest_qos,
        )
        self.confidence_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("confidence_topic").value),
            latest_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.duty_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("duty_topic").value),
            latest_qos,
        )
        self.servo_pub = self.create_publisher(
            Float32,
            str(self.get_parameter("servo_position_topic").value),
            latest_qos,
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

        self._latest_debug_background: np.ndarray | None = None
        self._latest_debug_background_stamp_ns: int | None = None
        if self.publish_debug and self.debug_background_topic:
            self.debug_background_sub = self.create_subscription(
                Image,
                self.debug_background_topic,
                self._on_debug_background,
                latest_qos,
            )
        else:
            self.debug_background_sub = None

        self.image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            latest_qos,
        )
        self.connection_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("connection_status_topic").value),
            self._on_connection,
            connection_qos,
        )

        self._vesc_connected = False
        self._last_image_time: Time | None = None
        self._last_control_time: Time | None = None
        self._latest_path = CenterlinePath(
            image_width=model_config.processing_width,
            roi_y_min=model_config.roi_y_min,
            roi_y_max=model_config.roi_y_max,
        )
        self._latest_command = ControlCommand(
            0.0,
            0.0,
            0.0,
            0.0,
            "WAITING_FOR_IMAGE",
        )
        self._processing_error_count = 0
        self._last_actual_state = "STARTING"
        self._last_actual_steering = 0.0
        self._last_actual_duty = 0.0

        control_rate_hz = max(
            1.0,
            float(self.get_parameter("control_rate_hz").value),
        )
        self._control_period_sec = 1.0 / control_rate_hz
        status_log_rate_hz = max(
            0.2,
            float(self.get_parameter("status_log_rate_hz").value),
        )
        self.control_timer = self.create_timer(
            self._control_period_sec,
            self._on_control_timer,
        )
        self.status_timer = self.create_timer(
            1.0 / status_log_rate_hz,
            self._publish_status,
        )

        mode = "ARMED" if self.drive_enabled else "DRY-RUN"
        self.get_logger().info(
            f"Centerline follower started: mode={mode}, "
            f"control={control_rate_hz:.1f}Hz, "
            f"ROI=y[{model_config.roi_y_min},{model_config.roi_y_max}]"
        )
        if not self.drive_enabled:
            self.get_logger().warn(
                "drive_enabled=false: perception runs but VESC commands are "
                "disabled. Inspect debug output before enabling the vehicle."
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/lane_mask")
        self.declare_parameter("force_lane_mask_input", True)
        self.declare_parameter("debug_background_topic", "/camera/image_rect")
        self.declare_parameter("steering_topic", "/auto/steering")
        self.declare_parameter("throttle_topic", "/auto/throttle")
        self.declare_parameter("confidence_topic", "/auto/lane_confidence")
        self.declare_parameter("status_topic", "/auto/status")
        self.declare_parameter("debug_image_topic", "/auto/lane_debug")
        self.declare_parameter("debug_mask_topic", "/auto/lane_mask")
        self.declare_parameter("duty_topic", "/vesc/duty")
        self.declare_parameter("servo_position_topic", "/vesc/servo_position")
        self.declare_parameter("connection_status_topic", "/vesc/connected")
        self.declare_parameter("drive_enabled", False)
        self.declare_parameter("publish_to_vesc", True)
        self.declare_parameter("require_vesc_connection", True)
        self.declare_parameter("publish_debug", False)
        self.declare_parameter("control_rate_hz", 100.0)
        self.declare_parameter("status_log_rate_hz", 2.0)
        self.declare_parameter("image_timeout_sec", 0.15)

        self.declare_parameter("processing_width", 160)
        self.declare_parameter("processing_height", 100)
        self.declare_parameter("roi_y_min", 50)
        self.declare_parameter("roi_y_max", 74)
        self.declare_parameter("white_threshold", 127)
        self.declare_parameter("morphology_kernel", 3)
        self.declare_parameter("minimum_points_per_boundary", 8)
        self.declare_parameter("tracking_margin_px", 22.0)
        self.declare_parameter("trace_seed_search_rows", 12)
        self.declare_parameter("trace_centering_max_run_width", 12)
        self.declare_parameter(
            "expected_lane_width_y_ratios", [0.50, 0.60, 0.70]
        )
        self.declare_parameter(
            "expected_lane_width_ratios", [0.297, 0.469, 0.641]
        )
        self.declare_parameter("lane_width_minimum_scale", 0.45)
        self.declare_parameter("lane_width_maximum_scale", 1.60)
        self.declare_parameter("maximum_fit_residual_px", 3.5)
        self.declare_parameter("lane_width_learning_alpha", 0.15)
        self.declare_parameter("single_lane_confidence_scale", 0.78)
        self.declare_parameter("maximum_extrapolation_rows", 5)
        self.declare_parameter("single_lane_curvature_gain", 6.0)
        self.declare_parameter("single_lane_maximum_offset_scale", 1.30)
        self.declare_parameter("single_lane_direction_guard_enabled", True)
        self.declare_parameter("single_lane_convexity_tolerance", 0.002)

        self.declare_parameter("lookahead_y", 68)
        self.declare_parameter("cross_track_gain", 0.75)
        self.declare_parameter("cross_track_error_boost_gain", 1.50)
        self.declare_parameter("preview_gain", 1.25)
        self.declare_parameter("derivative_gain", 0.025)
        self.declare_parameter("steering_deadband", 0.015)
        self.declare_parameter("steering_filter_alpha", 0.45)
        self.declare_parameter("maximum_steering_rate_per_sec", 3.0)
        self.declare_parameter("minimum_confidence", 0.45)
        self.declare_parameter("base_duty", 0.055)
        self.declare_parameter("minimum_duty", 0.050)
        self.declare_parameter("steering_slowdown", 0.55)
        self.declare_parameter("startup_boost_duty", 0.060)
        self.declare_parameter("startup_boost_duration_sec", 0.20)
        self.declare_parameter("startup_ramp_down_sec", 0.10)
        self.declare_parameter("startup_stable_tracking_sec", 0.20)
        self.declare_parameter("startup_rearm_stop_sec", 0.50)
        self.declare_parameter("servo_left", DEFAULT_SERVO_LEFT)
        self.declare_parameter("servo_center", DEFAULT_SERVO_CENTER)
        self.declare_parameter("servo_right", DEFAULT_SERVO_RIGHT)

    def _on_image(self, msg: Image) -> None:
        now = self.get_clock().now()
        try:
            image, image_is_mask = self._decode_image(msg)
            estimate = self.model.estimate(
                image,
                image_is_mask or self.force_lane_mask_input,
            )
            self._latest_path = estimate.path
            self._last_image_time = now

            if self.publish_debug:
                debug_source = self._matching_debug_background(msg)
                if debug_source is None:
                    debug_source = image
                debug = self.model.create_debug_image(debug_source, estimate)
                self.debug_image_pub.publish(
                    self._array_to_image(debug, "bgr8", msg)
                )
                self.debug_mask_pub.publish(
                    self._array_to_image(estimate.mask, "mono8", msg)
                )
        except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
            self._processing_error_count += 1
            self._latest_path = CenterlinePath(
                image_width=self.model.config.processing_width,
                roi_y_min=self.model.config.roi_y_min,
                roi_y_max=self.model.config.roi_y_max,
            )
            self.get_logger().error(
                f"Lane model failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        dt_sec = self._control_period_sec
        if self._last_control_time is not None:
            dt_sec = (
                now - self._last_control_time
            ).nanoseconds / 1_000_000_000.0
        self._last_control_time = now

        image_fresh = False
        if self._last_image_time is not None:
            age_sec = (
                now - self._last_image_time
            ).nanoseconds / 1_000_000_000.0
            image_fresh = age_sec < self.image_timeout_sec

        if image_fresh:
            self._latest_command = self.controller.update(
                self._latest_path,
                dt_sec,
            )
        else:
            self.controller.reset()
            self._latest_command = ControlCommand(
                0.0,
                0.0,
                0.0,
                0.0,
                "IMAGE_TIMEOUT",
            )

        connection_ok = (
            not self.require_vesc_connection or self._vesc_connected
        )
        tracking_ok = self._latest_command.state.startswith("TRACKING_")
        permitted = (
            self.drive_enabled
            and image_fresh
            and connection_ok
            and tracking_ok
        )
        if permitted:
            steering = self._latest_command.steering
            duty, startup_state = self.startup_duty_profile.update(
                True,
                self._latest_command.duty,
                dt_sec,
            )
            state = (
                self._latest_command.state
                if startup_state == StartupDutyProfile.RUNNING
                else startup_state
            )
        else:
            self.startup_duty_profile.update(False, 0.0, dt_sec)
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
        self._last_actual_steering = steering
        self._last_actual_duty = duty
        self.steering_pub.publish(Float32(data=float(steering)))
        self.throttle_pub.publish(Float32(data=float(duty)))
        self.confidence_pub.publish(
            Float32(data=float(self._latest_path.confidence))
        )
        if self.publish_to_vesc and self.drive_enabled:
            self.duty_pub.publish(Float32(data=float(duty)))
            self.servo_pub.publish(
                Float32(data=float(self._steering_to_servo(steering)))
            )

    def _on_debug_background(self, msg: Image) -> None:
        try:
            image, _ = self._decode_image(msg)
            self._latest_debug_background = image
            self._latest_debug_background_stamp_ns = self._stamp_ns(msg)
        except (ValueError, cv2.error) as exc:
            self.get_logger().warn(
                f"Debug background decode failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _on_connection(self, msg: Bool) -> None:
        self._vesc_connected = bool(msg.data)

    def _matching_debug_background(self, msg: Image) -> np.ndarray | None:
        if (
            self._latest_debug_background is None
            or self._latest_debug_background_stamp_ns is None
        ):
            return None
        if abs(
            self._latest_debug_background_stamp_ns - self._stamp_ns(msg)
        ) > 100_000_000:
            return None
        return self._latest_debug_background

    def _publish_status(self) -> None:
        command = self._latest_command
        status = (
            f"state={self._last_actual_state} "
            f"lane_mode={self._latest_path.mode} "
            f"confidence={self._latest_path.confidence:.2f} "
            f"steering={self._last_actual_steering:+.3f} "
            f"duty={self._last_actual_duty:.4f} "
            f"requested_duty={command.duty:.4f} "
            f"cross_track={command.cross_track_error:+.3f} "
            f"preview={command.preview_error:+.3f} "
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
                raise ValueError(f"short NV12 buffer: {buffer.size} < {required}")
            nv12 = buffer[:required].reshape(rows, step)[:, :width]
            bgr = cv2.cvtColor(
                np.ascontiguousarray(nv12),
                cv2.COLOR_YUV2BGR_NV12,
            )
            return bgr, False

        if encoding in {"mono8", "8uc1"}:
            required = height * step
            if buffer.size < required:
                raise ValueError(f"short mono8 buffer: {buffer.size} < {required}")
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

    def _steering_to_servo(self, steering: float) -> float:
        return steering_to_servo(
            steering,
            self.servo_left,
            self.servo_center,
            self.servo_right,
        )

    def stop_actuators(self) -> None:
        self.startup_duty_profile.reset()
        if not (self.publish_to_vesc and self.drive_enabled):
            return
        for _ in range(3):
            self.duty_pub.publish(Float32(data=0.0))
            self.servo_pub.publish(Float32(data=float(self.servo_center)))

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

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


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CenterlineNode()
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
