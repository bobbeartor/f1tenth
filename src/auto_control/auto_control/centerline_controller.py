"""Pure control logic for following a fitted lane centerline."""

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
    """Map conventional -left/+right steering to the VESC servo range."""
    steering = _clamp(steering, -1.0, 1.0)
    if steering < 0.0:
        return servo_center + (servo_left - servo_center) * -steering
    return servo_center + (servo_right - servo_center) * steering


@dataclass(frozen=True)
class CenterlinePath:
    """Quadratic center path expressed as x(y) in the working image."""

    image_width: int
    roi_y_min: int
    roi_y_max: int
    coefficients: tuple[float, float, float] = (0.0, 0.0, 0.0)
    confidence: float = 0.0
    valid: bool = False
    mode: str = "NONE"

    def x_at(self, y: float) -> float:
        a, b, c = self.coefficients
        return a * y * y + b * y + c


@dataclass(frozen=True)
class ControllerConfig:
    lookahead_y: int = 68
    cross_track_gain: float = 0.75
    preview_gain: float = 1.25
    derivative_gain: float = 0.025
    steering_deadband: float = 0.015
    steering_filter_alpha: float = 0.45
    maximum_steering_rate_per_sec: float = 3.0
    minimum_confidence: float = 0.45
    base_duty: float = 0.055
    minimum_duty: float = 0.050
    steering_slowdown: float = 0.55


@dataclass(frozen=True)
class ControlCommand:
    steering: float
    duty: float
    cross_track_error: float
    preview_error: float
    state: str


class CenterlineController:
    """Track only the center path; lane boundaries never enter this control law."""

    def __init__(self, config: ControllerConfig) -> None:
        self.config = config
        self._previous_cross_track_error: float | None = None
        self._filtered_steering = 0.0

    def reset(self) -> None:
        self._previous_cross_track_error = None
        self._filtered_steering = 0.0

    def update(self, path: CenterlinePath, dt_sec: float) -> ControlCommand:
        dt_sec = _clamp(dt_sec, 0.001, 0.2)
        if (
            not path.valid
            or path.image_width <= 0
            or path.confidence < self.config.minimum_confidence
        ):
            self.reset()
            return ControlCommand(0.0, 0.0, 0.0, 0.0, "LANE_LOST")

        half_width = path.image_width * 0.5
        image_center = half_width
        near_y = float(path.roi_y_max)
        lookahead_y = float(
            max(path.roi_y_min, min(path.roi_y_max, self.config.lookahead_y))
        )
        cross_track_error = _clamp(
            (path.x_at(near_y) - image_center) / half_width,
            -1.0,
            1.0,
        )
        preview_error = _clamp(
            (path.x_at(lookahead_y) - image_center) / half_width,
            -1.0,
            1.0,
        )

        derivative = 0.0
        if self._previous_cross_track_error is not None:
            derivative = _clamp(
                (cross_track_error - self._previous_cross_track_error) / dt_sec,
                -4.0,
                4.0,
            )
        self._previous_cross_track_error = cross_track_error

        target = (
            self.config.cross_track_gain * cross_track_error
            + self.config.preview_gain * preview_error
            + self.config.derivative_gain * derivative
        )
        target = _clamp(target, -1.0, 1.0)
        if abs(target) < self.config.steering_deadband:
            target = 0.0

        alpha = _clamp(self.config.steering_filter_alpha, 0.0, 1.0)
        filtered_target = alpha * target + (1.0 - alpha) * self._filtered_steering
        maximum_change = max(
            0.0,
            self.config.maximum_steering_rate_per_sec * dt_sec,
        )
        steering = _clamp(
            filtered_target,
            self._filtered_steering - maximum_change,
            self._filtered_steering + maximum_change,
        )
        steering = _clamp(steering, -1.0, 1.0)
        self._filtered_steering = steering

        confidence_scale = _clamp(
            (path.confidence - self.config.minimum_confidence)
            / max(1.0e-6, 1.0 - self.config.minimum_confidence),
            0.0,
            1.0,
        )
        curve_scale = _clamp(
            1.0 - self.config.steering_slowdown * abs(steering),
            0.0,
            1.0,
        )
        requested_duty = self.config.base_duty * curve_scale
        requested_duty *= 0.65 + 0.35 * confidence_scale
        duty = _clamp(
            requested_duty,
            self.config.minimum_duty,
            self.config.base_duty,
        )
        state = "TRACKING_BOTH" if path.mode == "BOTH" else "TRACKING_SINGLE"
        return ControlCommand(
            steering,
            duty,
            cross_track_error,
            preview_error,
            state,
        )


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
