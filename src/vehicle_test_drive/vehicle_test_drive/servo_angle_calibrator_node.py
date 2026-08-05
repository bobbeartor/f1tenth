#!/usr/bin/env python3

"""Hold an arbitrary VESC servo value for steering-angle measurement."""

from __future__ import annotations

import math
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from vesc_initializer.vesc_driver import (
    VescCommandIds,
    VescDriver,
    VescDriverError,
    VescScales,
)


class ServoAngleCalibratorNode(Node):
    """Own the VESC serial port and hold one steering-servo position."""

    def __init__(self) -> None:
        super().__init__("servo_angle_calibrator")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.1)
        self.declare_parameter("write_timeout", 2.0)
        self.declare_parameter("startup_delay", 0.2)
        self.declare_parameter("verify_firmware_on_startup", True)

        self.declare_parameter("servo_value", 0.46)
        self.declare_parameter("servo_min", 0.02)
        self.declare_parameter("servo_max", 0.98)
        self.declare_parameter("center_servo_value", 0.46)
        self.declare_parameter("command_refresh_rate_hz", 10.0)
        self.declare_parameter("return_to_center_on_shutdown", True)
        self.declare_parameter("interactive", True)

        self.declare_parameter("packet.comm_get_firmware_version", 0)
        self.declare_parameter("packet.comm_set_duty", 5)
        self.declare_parameter("packet.comm_set_servo_pos", 12)
        self.declare_parameter("packet.duty_scale", 100000)
        self.declare_parameter("packet.servo_scale", 1000)

        self._servo_min = float(self.get_parameter("servo_min").value)
        self._servo_max = float(self.get_parameter("servo_max").value)
        self._center_servo_value = float(
            self.get_parameter("center_servo_value").value
        )
        self._servo_value = float(self.get_parameter("servo_value").value)
        self._refresh_rate_hz = float(
            self.get_parameter("command_refresh_rate_hz").value
        )
        self._return_to_center = bool(
            self.get_parameter("return_to_center_on_shutdown").value
        )
        self._interactive = bool(self.get_parameter("interactive").value)
        self._validate_startup_parameters()

        self._driver_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._closed = False
        self._communication_failed = False
        self._driver = VescDriver(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            timeout=float(self.get_parameter("serial_timeout").value),
            write_timeout=float(self.get_parameter("write_timeout").value),
            startup_delay=float(self.get_parameter("startup_delay").value),
            command_ids=VescCommandIds(
                get_firmware_version=int(
                    self.get_parameter(
                        "packet.comm_get_firmware_version"
                    ).value
                ),
                set_duty=int(
                    self.get_parameter("packet.comm_set_duty").value
                ),
                set_servo_pos=int(
                    self.get_parameter("packet.comm_set_servo_pos").value
                ),
            ),
            scales=VescScales(
                duty=int(self.get_parameter("packet.duty_scale").value),
                servo=int(self.get_parameter("packet.servo_scale").value),
            ),
        )

        firmware_text = self._connect_and_apply_initial_value()
        self._parameter_callback_handle = (
            self.add_on_set_parameters_callback(self._on_parameter_change)
        )
        self._refresh_timer = self.create_timer(
            1.0 / self._refresh_rate_hz,
            self._refresh_command,
        )

        self.get_logger().warn(
            "This node owns the VESC serial port directly. Do not run "
            "vesc_initializer, manual control, or autonomous control at the "
            "same time. Motor duty is held at 0."
        )
        self.get_logger().info(
            f"VESC connected: port={self._driver.port}{firmware_text}, "
            f"servo_value={self._servo_value:.3f}, "
            f"allowed=[{self._servo_min:.3f}, {self._servo_max:.3f}]"
        )

        self._input_thread: threading.Thread | None = None
        if self._interactive:
            self._input_thread = threading.Thread(
                target=self._interactive_loop,
                name="servo-value-input",
                daemon=True,
            )
            self._input_thread.start()

    def _validate_startup_parameters(self) -> None:
        if (
            not math.isfinite(self._servo_min)
            or not math.isfinite(self._servo_max)
            or self._servo_min < 0.0
            or self._servo_max > 1.0
            or self._servo_min >= self._servo_max
        ):
            raise ValueError(
                "servo_min and servo_max must define a range within [0, 1]"
            )
        for name, value in (
            ("servo_value", self._servo_value),
            ("center_servo_value", self._center_servo_value),
        ):
            if not self._servo_value_is_valid(value):
                raise ValueError(
                    f"{name}={value} is outside "
                    f"[{self._servo_min}, {self._servo_max}]"
                )
        if not math.isfinite(self._refresh_rate_hz) or not (
            1.0 <= self._refresh_rate_hz <= 50.0
        ):
            raise ValueError(
                "command_refresh_rate_hz must be in the range [1, 50]"
            )

    def _connect_and_apply_initial_value(self) -> str:
        try:
            if bool(
                self.get_parameter("verify_firmware_on_startup").value
            ):
                major, minor = self._driver.get_firmware_version()
                firmware_text = f", firmware={major}.{minor}"
            else:
                self._driver.open()
                firmware_text = ""
            self._write_fixed_command(self._servo_value)
            return firmware_text
        except VescDriverError as exc:
            self._driver.close()
            raise RuntimeError(
                f"Could not initialize VESC on {self._driver.port}: {exc}"
            ) from exc

    def _on_parameter_change(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        servo_parameters = [
            parameter
            for parameter in parameters
            if parameter.name == "servo_value"
        ]
        unsupported = [
            parameter.name
            for parameter in parameters
            if parameter.name != "servo_value"
        ]
        if unsupported:
            return SetParametersResult(
                successful=False,
                reason=(
                    "Only servo_value can be changed while running. "
                    f"Restart to change: {', '.join(unsupported)}"
                ),
            )
        if not servo_parameters:
            return SetParametersResult(successful=True)

        parameter = servo_parameters[-1]
        if parameter.type_ != Parameter.Type.DOUBLE:
            return SetParametersResult(
                successful=False,
                reason="servo_value must be a floating-point number",
            )
        value = float(parameter.value)
        if not self._servo_value_is_valid(value):
            return SetParametersResult(
                successful=False,
                reason=(
                    f"servo_value must be within "
                    f"[{self._servo_min}, {self._servo_max}]"
                ),
            )

        try:
            self._write_fixed_command(value)
        except VescDriverError as exc:
            return SetParametersResult(
                successful=False,
                reason=f"VESC write failed: {exc}",
            )

        self._servo_value = value
        self.get_logger().info(f"Holding servo_value={value:.3f}")
        return SetParametersResult(successful=True)

    def _refresh_command(self) -> None:
        if self._communication_failed or self._closed:
            return
        try:
            self._write_fixed_command(self._servo_value)
        except VescDriverError as exc:
            self._communication_failed = True
            self.get_logger().error(f"VESC communication failed: {exc}")
            if rclpy.ok():
                rclpy.shutdown()

    def _write_fixed_command(self, servo_value: float) -> None:
        with self._driver_lock:
            self._driver.set_duty(0.0)
            self._driver.set_servo_position(servo_value)

    def _interactive_loop(self) -> None:
        print(
            "\nEnter a servo value and press Enter "
            f"({self._servo_min:.3f} to {self._servo_max:.3f}).\n"
            "Examples: 0.46, 0.40, 0.52\n"
            "Enter q to center the steering and quit.\n",
            flush=True,
        )
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                raw_value = input("servo_value> ").strip()
            except (EOFError, KeyboardInterrupt):
                return

            if raw_value.lower() in {"q", "quit", "exit"}:
                if rclpy.ok():
                    rclpy.shutdown()
                return
            try:
                value = float(raw_value)
            except ValueError:
                print("Invalid number. Example: 0.46", flush=True)
                continue

            result = self.set_parameters(
                [
                    Parameter(
                        "servo_value",
                        Parameter.Type.DOUBLE,
                        value,
                    )
                ]
            )[0]
            if not result.successful:
                print(f"Rejected: {result.reason}", flush=True)

    def _servo_value_is_valid(self, value: float) -> bool:
        return (
            math.isfinite(value)
            and self._servo_min <= value <= self._servo_max
        )

    def close_driver(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()

        try:
            with self._driver_lock:
                if self._driver.is_open:
                    self._driver.set_duty(0.0)
                    if self._return_to_center:
                        self._driver.set_servo_position(
                            self._center_servo_value
                        )
        except VescDriverError as exc:
            self.get_logger().warn(
                f"Failed to send safe shutdown command: {exc}"
            )
        finally:
            self._driver.close()

        if self._return_to_center:
            self.get_logger().info(
                "Motor duty is 0 and steering returned to "
                f"center_servo_value={self._center_servo_value:.3f}"
            )

    def destroy_node(self) -> bool:
        self.close_driver()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ServoAngleCalibratorNode | None = None
    exit_code = 0
    try:
        node = ServoAngleCalibratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Servo calibration interrupted by user.")
    except ExternalShutdownException:
        pass
    except (RuntimeError, ValueError) as exc:
        print(f"Servo calibrator failed: {exc}", flush=True)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
