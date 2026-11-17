from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class PathModel:
    """Locally smoothed measured centerline in the vehicle frame."""

    x_m: np.ndarray
    y_m: np.ndarray
    geometry_window_m: float

    @property
    def minimum_x_m(self) -> float:
        return float(self.x_m[0])

    @property
    def maximum_x_m(self) -> float:
        return float(self.x_m[-1])

    @property
    def point_count(self) -> int:
        return int(self.x_m.size)

    def lateral_position(self, x_m: float | np.ndarray) -> float | np.ndarray:
        values = np.interp(x_m, self.x_m, self.y_m)
        return float(values) if np.ndim(x_m) == 0 else values

    def first_derivative(self, x_m: float | np.ndarray) -> float | np.ndarray:
        query = np.asarray(x_m, dtype=float)
        half_window_m = 0.5 * max(self.geometry_window_m, 1.0e-3)
        lower = np.maximum(self.minimum_x_m, query - half_window_m)
        upper = np.minimum(self.maximum_x_m, query + half_window_m)

        # Keep a full local chord at either end instead of extrapolating the
        # measured centerline behind the camera-visible range.
        short = upper - lower < half_window_m
        lower = np.where(
            short,
            np.maximum(self.minimum_x_m, upper - 2.0 * half_window_m),
            lower,
        )
        upper = np.where(
            short,
            np.minimum(self.maximum_x_m, lower + 2.0 * half_window_m),
            upper,
        )
        distance = upper - lower
        derivative = np.divide(
            np.interp(upper, self.x_m, self.y_m)
            - np.interp(lower, self.x_m, self.y_m),
            distance,
            out=np.zeros_like(query, dtype=float),
            where=distance > 1.0e-6,
        )
        return float(derivative) if query.ndim == 0 else derivative

    def curvature(self, x_m: float | np.ndarray) -> float | np.ndarray:
        query = np.asarray(x_m, dtype=float)
        flat_query = np.atleast_1d(query).astype(float).ravel()
        result = np.zeros_like(flat_query)
        window_m = min(
            max(self.geometry_window_m, 1.0e-3),
            0.5 * (self.maximum_x_m - self.minimum_x_m),
        )
        if window_m <= 1.0e-6:
            return float(result[0]) if query.ndim == 0 else result.reshape(query.shape)

        for index, requested_x_m in enumerate(flat_query):
            center_x_m = clamp(
                float(requested_x_m),
                self.minimum_x_m + window_m,
                self.maximum_x_m - window_m,
            )
            first = np.asarray(
                (
                    center_x_m - window_m,
                    float(self.lateral_position(center_x_m - window_m)),
                )
            )
            middle = np.asarray(
                (center_x_m, float(self.lateral_position(center_x_m)))
            )
            last = np.asarray(
                (
                    center_x_m + window_m,
                    float(self.lateral_position(center_x_m + window_m)),
                )
            )
            first_length = float(np.linalg.norm(middle - first))
            second_length = float(np.linalg.norm(last - middle))
            chord_length = float(np.linalg.norm(last - first))
            denominator = first_length * second_length * chord_length
            if denominator <= 1.0e-9:
                continue
            first_to_middle = middle - first
            first_to_last = last - first
            cross = float(
                first_to_middle[0] * first_to_last[1]
                - first_to_middle[1] * first_to_last[0]
            )
            result[index] = 2.0 * cross / denominator
        return float(result[0]) if query.ndim == 0 else result.reshape(query.shape)


@dataclass(frozen=True)
class OrderedPathModel:
    """Near-to-far polyline: preserve corners even when vehicle X stops increasing."""

    x_m: np.ndarray
    y_m: np.ndarray
    arc_m: np.ndarray
    geometry_window_m: float

    @property
    def point_count(self) -> int:
        return int(self.x_m.size)

    def point(self, arc_m):
        return np.stack((np.interp(arc_m, self.arc_m, self.x_m),
                         np.interp(arc_m, self.arc_m, self.y_m)), axis=-1)

    def heading(self, arc_m: float) -> float:
        half = self.geometry_window_m / 2.0
        lo = max(0.0, arc_m - half)
        hi = min(float(self.arc_m[-1]), arc_m + half)
        if hi - lo < half:
            lo = max(0.0, hi - self.geometry_window_m)
            hi = min(float(self.arc_m[-1]), lo + self.geometry_window_m)
        delta = self.point(hi) - self.point(lo)
        return math.atan2(float(delta[1]), float(delta[0]))

    def curvature(self, arc_m):
        query = np.asarray(arc_m, dtype=float)
        window = min(self.geometry_window_m, float(self.arc_m[-1]) / 2.0)
        middle_s = np.clip(query, window, self.arc_m[-1] - window)
        first = self.point(middle_s - window)
        middle = self.point(middle_s)
        last = self.point(middle_s + window)
        a, b = middle - first, last - first
        denominator = (np.linalg.norm(a, axis=-1) * np.linalg.norm(last-middle, axis=-1)
                       * np.linalg.norm(b, axis=-1))
        value = np.divide(2.0 * (a[..., 0]*b[..., 1] - a[..., 1]*b[..., 0]),
                          denominator, out=np.zeros_like(query), where=denominator > 1e-9)
        return float(value) if query.ndim == 0 else value


def build_ordered_path_model(
    x_m, y_m, *, minimum_points: int, minimum_span_m: float,
    minimum_x_m: float, maximum_x_m: float, maximum_gap_m: float,
    geometry_window_m: float,
) -> OrderedPathModel | None:
    x, y = np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
    if x.ndim != 1 or x.shape != y.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("ordered centerline must contain finite matching coordinate arrays")
    # Only retain the first contiguous forward ROI segment; never sort X or
    # join across cropped/gapped regions. Sideways and curved segments stay ordered.
    points = []
    for point in zip(x, y):
        if not minimum_x_m <= point[0] <= maximum_x_m:
            if points:
                break
            continue
        if points:
            gap = float(np.linalg.norm(np.asarray(point) - points[-1]))
            if gap < 1e-5:
                continue
            if gap > maximum_gap_m:
                break
        points.append(point)
    if len(points) < minimum_points:
        return None
    xy = np.asarray(points)
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    if arc[-1] < minimum_span_m:
        return None
    # Spatial smoothing already happens in line_detactor; do not refit it as y(X).
    return OrderedPathModel(xy[:, 0], xy[:, 1], arc, geometry_window_m)


@dataclass(frozen=True)
class StanleyResult:
    steering_angle_rad: float
    cross_track_error_m: float
    heading_error_rad: float
    direction_guard_used: bool


def centerline_points_from_mono8(
    image: np.ndarray,
    *,
    x_max_m: float,
    y_max_m: float,
    meter_per_pixel: float,
    threshold: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Return one metric center point per occupied image row.

    The BEV image is ordered with +X toward the top and +Y toward the left.
    The lane image has the same metric extent as the BEV image. Its horizontal
    center is vehicle Y=0; no additional lateral image margin is present.
    """

    if image.ndim != 2:
        raise ValueError("centerline image must be two-dimensional")
    if meter_per_pixel <= 0.0:
        raise ValueError("meter_per_pixel must be positive")
    expected_height = int(round(x_max_m / meter_per_pixel))
    expected_width = int(round(2.0 * y_max_m / meter_per_pixel))
    if image.shape != (expected_height, expected_width):
        raise ValueError(
            "centerline image dimensions do not match BEV geometry: "
            f"received {image.shape[1]}x{image.shape[0]}, expected "
            f"{expected_width}x{expected_height}"
        )

    rows, columns = np.nonzero(image >= threshold)
    if rows.size == 0:
        return np.empty(0, dtype=float), np.empty(0, dtype=float)

    row_counts = np.bincount(rows, minlength=image.shape[0])
    column_sums = np.bincount(
        rows,
        weights=columns.astype(float),
        minlength=image.shape[0],
    )
    occupied_rows = np.flatnonzero(row_counts)
    center_columns = column_sums[occupied_rows] / row_counts[occupied_rows]

    x_m = x_max_m - (occupied_rows.astype(float) + 0.5) * meter_per_pixel
    y_m = (
        y_max_m
        - (center_columns + 0.5) * meter_per_pixel
    )

    # Calculations below expect increasing forward distance.
    order = np.argsort(x_m)
    return x_m[order], y_m[order]


def build_path_model(
    x_m: Iterable[float],
    y_m: Iterable[float],
    *,
    minimum_points: int,
    minimum_span_m: float,
    minimum_x_m: float,
    maximum_x_m: float,
    local_smoothing_window_m: float,
    outlier_threshold_m: float,
    geometry_window_m: float,
) -> PathModel | None:
    x = np.asarray(tuple(x_m), dtype=float)
    y = np.asarray(tuple(y_m), dtype=float)
    finite = np.isfinite(x) & np.isfinite(y)
    selected = finite & (x >= minimum_x_m) & (x <= maximum_x_m)
    x = x[selected]
    y = y[selected]

    if x.size < minimum_points:
        return None
    span_m = float(np.max(x) - np.min(x))
    if span_m < minimum_span_m:
        return None

    order = np.argsort(x)
    x = x[order]
    y = y[order]
    unique_x, inverse = np.unique(x, return_inverse=True)
    if unique_x.size != x.size:
        y = np.asarray(
            [np.median(y[inverse == index]) for index in range(unique_x.size)]
        )
        x = unique_x
    if x.size < minimum_points or float(x[-1] - x[0]) < minimum_span_m:
        return None

    # Suppress isolated lateral spikes first, then replace each point with a
    # local straight-segment estimate. This preserves the measured polyline
    # and a real corner while preventing centimetre-scale roughness from
    # becoming steering commands. There is deliberately no global polynomial.
    half_window_m = 0.5 * max(0.0, local_smoothing_window_m)
    filtered_y = y.copy()
    if half_window_m > 0.0:
        left = np.searchsorted(x, x - half_window_m, side="left")
        right = np.searchsorted(x, x + half_window_m, side="right")
        for index, (first, last) in enumerate(zip(left, right)):
            if last - first < 3:
                continue
            median_y_m = float(np.median(y[first:last]))
            if (
                outlier_threshold_m > 0.0
                and abs(float(y[index]) - median_y_m) > outlier_threshold_m
            ):
                filtered_y[index] = median_y_m

        def interval_sum(values: np.ndarray) -> np.ndarray:
            prefix = np.concatenate((np.zeros(1), np.cumsum(values)))
            return prefix[right] - prefix[left]

        counts = (right - left).astype(float)
        sum_x = interval_sum(x)
        sum_y = interval_sum(filtered_y)
        sum_xx = interval_sum(x * x)
        sum_xy = interval_sum(x * filtered_y)
        denominator = counts * sum_xx - sum_x * sum_x
        valid = (counts >= 3.0) & (np.abs(denominator) > 1.0e-12)
        smoothed_y = filtered_y.copy()
        slope = np.zeros_like(x)
        slope[valid] = (
            counts[valid] * sum_xy[valid]
            - sum_x[valid] * sum_y[valid]
        ) / denominator[valid]
        intercept = np.zeros_like(x)
        intercept[valid] = (
            sum_y[valid] - slope[valid] * sum_x[valid]
        ) / counts[valid]
        smoothed_y[valid] = slope[valid] * x[valid] + intercept[valid]
        y = smoothed_y

    return PathModel(
        x_m=x,
        y_m=y,
        geometry_window_m=max(geometry_window_m, 1.0e-3),
    )


def closest_path_geometry(path: PathModel | OrderedPathModel) -> tuple[float, float]:
    """Return signed cross-track error and closest arc length (legacy model: X)."""

    start_x = path.x_m[:-1]
    start_y = path.y_m[:-1]
    delta_x = path.x_m[1:] - start_x
    delta_y = path.y_m[1:] - start_y
    length_squared = np.square(delta_x) + np.square(delta_y)
    projection = np.divide(
        -(start_x * delta_x + start_y * delta_y),
        length_squared,
        out=np.zeros_like(delta_x),
        where=length_squared > 1.0e-12,
    )
    projection = np.clip(projection, 0.0, 1.0)
    closest_x = start_x + projection * delta_x
    closest_y = start_y + projection * delta_y
    squared_distance = np.square(closest_x) + np.square(closest_y)
    segment = int(np.argmin(squared_distance))
    segment_length = math.sqrt(max(float(length_squared[segment]), 1.0e-12))
    tangent_x = float(delta_x[segment]) / segment_length
    tangent_y = float(delta_y[segment]) / segment_length
    cross_track_error_m = (
        -tangent_y * float(closest_x[segment])
        + tangent_x * float(closest_y[segment])
    )
    if isinstance(path, OrderedPathModel):
        closest_s = path.arc_m[segment] + projection[segment] * segment_length
        return cross_track_error_m, float(closest_s)
    return cross_track_error_m, float(closest_x[segment])


def stanley_control(
    path: PathModel | OrderedPathModel,
    *,
    speed_mps: float,
    gain: float,
    softening_speed_mps: float,
    heading_lookahead_m: float,
    maximum_steering_angle_rad: float,
    corner_heading_threshold_rad: float,
    corner_opposing_correction_ratio: float,
) -> StanleyResult:
    """Calculate a front-axle Stanley steering command.

    Positive steering is left because the vehicle frame uses +Y left.
    Cross-track error is the signed normal distance from the real front-axle
    origin to the centerline. Ordered ML paths use arc-length lookahead from
    the closest projection; the legacy X-based model uses absolute vehicle X.
    Heading is clamped to the available path instead of extrapolated.
    """

    cross_track_error_m, closest_coordinate = closest_path_geometry(path)
    if isinstance(path, OrderedPathModel):
        # Arc lookahead from the closest front-axle projection handles 90-degree corners.
        heading_s = min(float(path.arc_m[-1]), closest_coordinate + max(0.0, heading_lookahead_m))
        heading_error_rad = path.heading(heading_s)
    else:
        heading_x_m = clamp(max(0.0, heading_lookahead_m), path.minimum_x_m, path.maximum_x_m)
        heading_error_rad = math.atan(float(path.first_derivative(heading_x_m)))
    correction_rad = math.atan2(
        max(0.0, gain) * cross_track_error_m,
        max(0.0, abs(speed_mps)) + max(0.0, softening_speed_mps),
    )
    direction_guard_used = False
    if (
        abs(heading_error_rad) >= max(0.0, corner_heading_threshold_rad)
        and heading_error_rad * correction_rad < 0.0
    ):
        maximum_opposing_correction_rad = (
            abs(heading_error_rad)
            * clamp(corner_opposing_correction_ratio, 0.0, 0.99)
        )
        limited_correction_rad = clamp(
            correction_rad,
            -maximum_opposing_correction_rad,
            maximum_opposing_correction_rad,
        )
        direction_guard_used = limited_correction_rad != correction_rad
        correction_rad = limited_correction_rad
    steering_angle_rad = clamp(
        heading_error_rad + correction_rad,
        -maximum_steering_angle_rad,
        maximum_steering_angle_rad,
    )
    return StanleyResult(
        steering_angle_rad=steering_angle_rad,
        cross_track_error_m=cross_track_error_m,
        heading_error_rad=heading_error_rad,
        direction_guard_used=direction_guard_used,
    )


def representative_curvature(
    path: PathModel | OrderedPathModel,
    *,
    lookahead_minimum_x_m: float,
    lookahead_maximum_x_m: float,
    percentile: float,
    sample_count: int = 32,
) -> float:
    if isinstance(path, OrderedPathModel):
        samples = np.linspace(0.0, float(path.arc_m[-1]), max(2, sample_count))
        forward_x = path.point(samples)[:, 0]
        selected = (forward_x >= lookahead_minimum_x_m) & (forward_x <= lookahead_maximum_x_m)
        if np.any(selected):
            samples = samples[selected]
        values = np.abs(path.curvature(samples))
        if not np.all(np.isfinite(values)):
            raise ValueError("nonfinite ordered path curvature")
        return float(np.percentile(values, clamp(percentile, 0.0, 100.0)))
    minimum_x_m = max(path.minimum_x_m, lookahead_minimum_x_m)
    maximum_x_m = min(path.maximum_x_m, lookahead_maximum_x_m)
    if maximum_x_m <= minimum_x_m:
        minimum_x_m = path.minimum_x_m
        maximum_x_m = path.maximum_x_m
    samples = np.linspace(minimum_x_m, maximum_x_m, max(2, sample_count))
    curvature = np.abs(path.curvature(samples))
    finite_curvature = curvature[np.isfinite(curvature)]
    if finite_curvature.size == 0:
        return 0.0
    return float(np.percentile(finite_curvature, clamp(percentile, 0.0, 100.0)))


def curvature_target_speed(
    curvature_per_m: float,
    *,
    maximum_lateral_acceleration_mps2: float,
    minimum_speed_mps: float,
    maximum_speed_mps: float,
) -> float:
    curvature = abs(float(curvature_per_m))
    if curvature <= 1.0e-6:
        return maximum_speed_mps
    safe_speed_mps = math.sqrt(
        max(0.0, maximum_lateral_acceleration_mps2) / curvature
    )
    return clamp(safe_speed_mps, minimum_speed_mps, maximum_speed_mps)


def erpm_to_speed_mps(
    measured_erpm: int,
    *,
    wheel_diameter_m: float,
    motor_pole_pairs: int,
    motor_pinion_teeth: int,
    spur_gear_teeth: int,
    differential_pinion_teeth: int,
    differential_ring_teeth: int,
    direction_sign: float,
    scale_correction: float,
) -> float:
    gear_ratio = (
        float(spur_gear_teeth) / float(motor_pinion_teeth)
        * float(differential_ring_teeth) / float(differential_pinion_teeth)
    )
    return (
        float(measured_erpm)
        * (1.0 if direction_sign >= 0.0 else -1.0)
        * scale_correction
        * math.pi
        * wheel_diameter_m
        / (60.0 * float(motor_pole_pairs) * gear_ratio)
    )


def steering_angle_to_servo(
    steering_angle_rad: float,
    *,
    maximum_steering_angle_rad: float,
    servo_left: float,
    servo_center: float,
    servo_right: float,
    inverted: bool = False,
) -> float:
    if inverted:
        steering_angle_rad = -steering_angle_rad
    normalized = clamp(
        steering_angle_rad / maximum_steering_angle_rad,
        -1.0,
        1.0,
    )
    if normalized >= 0.0:
        return lerp(servo_center, servo_left, normalized)
    return lerp(servo_center, servo_right, -normalized)


class SpeedPid:
    def __init__(
        self,
        *,
        kp: float,
        ki: float,
        kd: float,
        integral_limit: float,
        minimum_duty: float,
        maximum_duty: float,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.minimum_duty = minimum_duty
        self.maximum_duty = maximum_duty
        self._integral = 0.0
        self._previous_error: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None

    def update(
        self,
        *,
        target_speed_mps: float,
        current_speed_mps: float,
        feedforward_duty: float,
        dt_sec: float,
    ) -> float:
        dt_sec = max(1.0e-4, float(dt_sec))
        error = target_speed_mps - current_speed_mps
        derivative = 0.0
        if self._previous_error is not None:
            derivative = (error - self._previous_error) / dt_sec

        candidate_integral = clamp(
            self._integral + error * dt_sec,
            -self.integral_limit,
            self.integral_limit,
        )
        candidate = (
            feedforward_duty
            + self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )
        output = clamp(candidate, self.minimum_duty, self.maximum_duty)

        # Conditional integration prevents further wind-up while saturated.
        saturated_high = candidate > self.maximum_duty and error > 0.0
        saturated_low = candidate < self.minimum_duty and error < 0.0
        if not saturated_high and not saturated_low:
            self._integral = candidate_integral
        self._previous_error = error
        return output


def speed_feedforward_duty(
    target_speed_mps: float,
    *,
    minimum_speed_mps: float,
    maximum_speed_mps: float,
    minimum_duty: float,
    maximum_duty: float,
) -> float:
    if maximum_speed_mps <= minimum_speed_mps:
        return minimum_duty
    amount = clamp(
        (target_speed_mps - minimum_speed_mps)
        / (maximum_speed_mps - minimum_speed_mps),
        0.0,
        1.0,
    )
    return lerp(minimum_duty, maximum_duty, amount)


def move_toward(value: float, target: float, maximum_step: float) -> float:
    maximum_step = max(0.0, maximum_step)
    if value < target:
        return min(target, value + maximum_step)
    if value > target:
        return max(target, value - maximum_step)
    return target


def lerp(start: float, end: float, amount: float) -> float:
    return start + (end - start) * amount


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))
