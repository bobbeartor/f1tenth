"""Pure steering and speed control for perspective-image lane tracking.

This module deliberately has no ROS, OpenCV, or NumPy dependency.  Keeping the
controller independent from image processing makes its sign convention and
fail-safe behaviour easy to unit test.
"""

from __future__ import annotations

from dataclasses import dataclass


DEFAULT_SERVO_LEFT = 0.02
DEFAULT_SERVO_CENTER = 0.46
DEFAULT_SERVO_RIGHT = 0.98


def steering_to_servo(
    steering: float,
    servo_left: float = DEFAULT_SERVO_LEFT,
    servo_center: float = DEFAULT_SERVO_CENTER,
    servo_right: float = DEFAULT_SERVO_RIGHT,
) -> float:
    """Convert conventional -left/+right steering to a VESC servo value."""
    steering = max(-1.0, min(1.0, steering))
    if steering < 0.0:
        return servo_center + (servo_left - servo_center) * -steering
    return servo_center + (servo_right - servo_center) * steering


@dataclass(frozen=True)
class LaneObservation:
    """Lane centre measured directly in a perspective camera image."""

    image_width: int
    near_center_x: float = 0.0
    far_center_x: float = 0.0
    confidence: float = 0.0
    valid: bool = False


@dataclass(frozen=True)
class ControllerConfig:
    lateral_gain: float = 0.90
    heading_gain: float = 1.35
    derivative_gain: float = 0.04
    steering_deadband: float = 0.015
    steering_filter_alpha: float = 0.35
    maximum_steering_rate_per_sec: float = 2.5
    minimum_confidence: float = 0.45
    base_duty: float = 0.055
    minimum_duty: float = 0.050
    steering_slowdown: float = 0.55


@dataclass(frozen=True)
class ControlCommand:
    """Normalised steering and a forward VESC duty recommendation."""

    steering: float
    duty: float
    lateral_error: float
    heading_error: float
    state: str


class PerspectiveLaneController:
    """Feedback controller using near/far centres from a perspective image.

    Steering uses the conventional normalised sign: -1 is left and +1 is
    right.  Image x grows to the right, so the signs require no inversion.
    """

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self._previous_lateral_error: float | None = None
        self._filtered_steering = 0.0

    def reset(self) -> None:
        self._previous_lateral_error = None
        self._filtered_steering = 0.0

    def update(
        self,
        observation: LaneObservation,
        dt_sec: float,
    ) -> ControlCommand:
        dt_sec = self._clamp(dt_sec, 0.001, 0.2)
        if (
            not observation.valid
            or observation.image_width <= 0
            or observation.confidence < self.config.minimum_confidence
        ):
            self.reset()
            return ControlCommand(0.0, 0.0, 0.0, 0.0, "LANE_LOST")

        half_width = observation.image_width * 0.5
        image_center = half_width
        lateral_error = self._clamp(
            (observation.near_center_x - image_center) / half_width,
            -1.0,
            1.0,
        )
        # On a straight road, perspective makes a laterally offset near centre
        # converge back toward the optical centre.  Therefore far-minus-near
        # is not a heading error unless a BEV transform is used.  The far
        # centre's displacement from the optical centre is the useful direct
        # perspective-image heading/preview term.
        heading_error = self._clamp(
            (observation.far_center_x - image_center) / half_width,
            -1.0,
            1.0,
        )

        derivative = 0.0
        if self._previous_lateral_error is not None:
            derivative = self._clamp(
                (lateral_error - self._previous_lateral_error) / dt_sec,
                -4.0,
                4.0,
            )
        self._previous_lateral_error = lateral_error

        target = (
            self.config.lateral_gain * lateral_error
            + self.config.heading_gain * heading_error
            + self.config.derivative_gain * derivative
        )
        target = self._clamp(target, -1.0, 1.0)
        if abs(target) < self.config.steering_deadband:
            target = 0.0

        alpha = self._clamp(self.config.steering_filter_alpha, 0.0, 1.0)
        filtered_target = (
            alpha * target + (1.0 - alpha) * self._filtered_steering
        )
        maximum_change = max(
            0.0,
            self.config.maximum_steering_rate_per_sec * dt_sec,
        )
        steering = self._clamp(
            filtered_target,
            self._filtered_steering - maximum_change,
            self._filtered_steering + maximum_change,
        )
        steering = self._clamp(steering, -1.0, 1.0)
        self._filtered_steering = steering

        confidence_scale = self._clamp(
            (observation.confidence - self.config.minimum_confidence)
            / max(1.0e-6, 1.0 - self.config.minimum_confidence),
            0.0,
            1.0,
        )
        curve_scale = self._clamp(
            1.0 - self.config.steering_slowdown * abs(steering),
            0.0,
            1.0,
        )
        requested_duty = self.config.base_duty * curve_scale
        requested_duty *= 0.65 + 0.35 * confidence_scale
        duty = self._clamp(
            requested_duty,
            self.config.minimum_duty,
            self.config.base_duty,
        )
        state = "TRACKING" if observation.confidence >= 0.70 else "LOW_CONFIDENCE"
        return ControlCommand(
            steering,
            duty,
            lateral_error,
            heading_error,
            state,
        )

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
