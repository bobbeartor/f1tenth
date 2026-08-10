"""Quadratic lane-boundary fitting and centerline reconstruction."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from auto_control.centerline_controller import CenterlinePath


@dataclass(frozen=True)
class LaneModelConfig:
    processing_width: int = 160
    processing_height: int = 100
    roi_y_min: int = 60
    roi_y_max: int = 98
    white_threshold: int = 127
    morphology_kernel: int = 3
    maximum_line_width_px: int = 12
    minimum_points_per_boundary: int = 8
    tracking_margin_px: float = 22.0
    expected_lane_width_top_px: float = 60.0
    expected_lane_width_bottom_px: float = 120.0
    lane_width_minimum_scale: float = 0.55
    lane_width_maximum_scale: float = 1.45
    maximum_fit_residual_px: float = 3.5
    lane_width_learning_alpha: float = 0.15
    single_lane_confidence_scale: float = 0.78


@dataclass(frozen=True)
class BoundaryFit:
    coefficients: tuple[float, float, float]
    points: tuple[tuple[float, float], ...]
    rms_error: float

    def x_at(self, y: float) -> float:
        a, b, c = self.coefficients
        return a * y * y + b * y + c


@dataclass(frozen=True)
class LaneModelEstimate:
    path: CenterlinePath
    mask: np.ndarray
    left: BoundaryFit | None
    right: BoundaryFit | None


class LaneModel:
    """Fit curved boundaries only inside rows 60..98 of a 160x100 mask."""

    def __init__(self, config: LaneModelConfig) -> None:
        self.config = config
        self._validate_config()
        self._previous_center_coefficients: np.ndarray | None = None
        self._lane_width_coefficients = self._configured_width_coefficients()

    def reset(self) -> None:
        self._previous_center_coefficients = None
        self._lane_width_coefficients = self._configured_width_coefficients()

    def estimate(
        self,
        image: np.ndarray,
        image_is_mask: bool = False,
    ) -> LaneModelEstimate:
        mask = self._prepare_mask(image, image_is_mask)
        left_points, right_points = self._collect_boundary_points(mask)
        left = self._fit_boundary(left_points)
        right = self._fit_boundary(right_points)

        if left is not None and right is not None:
            left, right = self._validate_pair(left, right)

        if left is not None and right is not None:
            left_coefficients = np.asarray(left.coefficients, dtype=np.float64)
            right_coefficients = np.asarray(right.coefficients, dtype=np.float64)
            center_coefficients = 0.5 * (
                left_coefficients + right_coefficients
            )
            measured_width = right_coefficients - left_coefficients
            alpha = max(
                0.0,
                min(1.0, self.config.lane_width_learning_alpha),
            )
            self._lane_width_coefficients = (
                (1.0 - alpha) * self._lane_width_coefficients
                + alpha * measured_width
            )
            mode = "BOTH"
            confidence = self._confidence(left, right, single=False)
        elif left is not None:
            center_coefficients = (
                np.asarray(left.coefficients, dtype=np.float64)
                + 0.5 * self._lane_width_coefficients
            )
            mode = "LEFT_ONLY"
            confidence = self._confidence(left, None, single=True)
        elif right is not None:
            center_coefficients = (
                np.asarray(right.coefficients, dtype=np.float64)
                - 0.5 * self._lane_width_coefficients
            )
            mode = "RIGHT_ONLY"
            confidence = self._confidence(None, right, single=True)
        else:
            self._previous_center_coefficients = None
            return LaneModelEstimate(
                path=CenterlinePath(
                    image_width=self.config.processing_width,
                    roi_y_min=self.config.roi_y_min,
                    roi_y_max=self.config.roi_y_max,
                ),
                mask=mask,
                left=None,
                right=None,
            )

        center_coefficients = self._clip_path(center_coefficients)
        self._previous_center_coefficients = center_coefficients.copy()
        path = CenterlinePath(
            image_width=self.config.processing_width,
            roi_y_min=self.config.roi_y_min,
            roi_y_max=self.config.roi_y_max,
            coefficients=tuple(float(value) for value in center_coefficients),
            confidence=confidence,
            valid=True,
            mode=mode,
        )
        return LaneModelEstimate(path, mask, left, right)

    def create_debug_image(
        self,
        image: np.ndarray,
        estimate: LaneModelEstimate,
    ) -> np.ndarray:
        debug = self._resize_image(image)
        if debug.ndim == 2:
            debug = cv2.cvtColor(debug, cv2.COLOR_GRAY2BGR)

        mask_overlay = np.zeros_like(debug)
        mask_overlay[:, :, 1] = estimate.mask
        debug = cv2.addWeighted(debug, 0.75, mask_overlay, 0.35, 0.0)
        cv2.rectangle(
            debug,
            (0, self.config.roi_y_min),
            (self.config.processing_width - 1, self.config.roi_y_max),
            (0, 180, 255),
            1,
        )
        self._draw_fit(debug, estimate.left, (255, 0, 0), 1)
        self._draw_fit(debug, estimate.right, (0, 0, 255), 1)

        if estimate.path.valid:
            points = self._curve_points(estimate.path.coefficients)
            cv2.polylines(debug, [points], False, (0, 255, 255), 2)
        cv2.line(
            debug,
            (self.config.processing_width // 2, self.config.roi_y_min),
            (self.config.processing_width // 2, self.config.roi_y_max),
            (255, 0, 255),
            1,
        )
        cv2.putText(
            debug,
            f"{estimate.path.mode} conf={estimate.path.confidence:.2f}",
            (3, 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.32,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return debug

    def _prepare_mask(self, image: np.ndarray, image_is_mask: bool) -> np.ndarray:
        if image.size == 0:
            raise ValueError("input image is empty")
        resized = self._resize_image(image, preserve_binary=image_is_mask)
        if resized.ndim == 2:
            gray = resized
        else:
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        # INTER_AREA can turn a thin white line into a low non-zero value
        # while shrinking. lane_detect already removed background noise, so
        # retain every non-zero mask contribution instead of aliasing it away.
        threshold = 0 if image_is_mask else self.config.white_threshold
        _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)

        roi_mask = np.zeros_like(mask)
        roi_mask[
            self.config.roi_y_min : self.config.roi_y_max + 1,
            :,
        ] = mask[
            self.config.roi_y_min : self.config.roi_y_max + 1,
            :,
        ]
        kernel_size = self._odd(self.config.morphology_kernel)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (kernel_size, kernel_size),
        )
        roi_mask = cv2.morphologyEx(roi_mask, cv2.MORPH_CLOSE, kernel)
        roi_mask[: self.config.roi_y_min, :] = 0
        roi_mask[self.config.roi_y_max + 1 :, :] = 0
        return roi_mask

    def _resize_image(
        self,
        image: np.ndarray,
        preserve_binary: bool = False,
    ) -> np.ndarray:
        target_size = (
            self.config.processing_width,
            self.config.processing_height,
        )
        if image.shape[1] == target_size[0] and image.shape[0] == target_size[1]:
            return image.copy()
        shrinking = (
            image.shape[1] > target_size[0]
            or image.shape[0] > target_size[1]
        )
        if preserve_binary:
            interpolation = cv2.INTER_AREA if shrinking else cv2.INTER_NEAREST
        else:
            interpolation = cv2.INTER_AREA if shrinking else cv2.INTER_LINEAR
        return cv2.resize(image, target_size, interpolation=interpolation)

    def _collect_boundary_points(
        self,
        mask: np.ndarray,
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
        left_points: list[tuple[float, float]] = []
        right_points: list[tuple[float, float]] = []
        center_hint = self.config.processing_width * 0.5

        for y in range(self.config.roi_y_max, self.config.roi_y_min - 1, -1):
            if self._previous_center_coefficients is not None:
                center_hint = float(
                    np.polyval(self._previous_center_coefficients, y)
                )
            expected_width = self._width_at(y)
            candidates = self._row_candidates(mask[y])
            left_x, right_x = self._select_candidates(
                candidates,
                center_hint,
                expected_width,
            )
            if left_x is not None:
                left_points.append((float(y), left_x))
            if right_x is not None:
                right_points.append((float(y), right_x))
            if left_x is not None and right_x is not None:
                center_hint = 0.5 * (left_x + right_x)
            elif left_x is not None:
                center_hint = left_x + 0.5 * expected_width
            elif right_x is not None:
                center_hint = right_x - 0.5 * expected_width

        return left_points, right_points

    def _row_candidates(self, row: np.ndarray) -> list[float]:
        active = row > 0
        candidates: list[float] = []
        start: int | None = None
        width = int(row.shape[0])
        for x in range(width + 1):
            is_active = x < width and bool(active[x])
            if is_active and start is None:
                start = x
            elif not is_active and start is not None:
                run_width = x - start
                if 0 < run_width <= self.config.maximum_line_width_px:
                    candidates.append(0.5 * (start + x - 1))
                start = None
        return candidates

    def _select_candidates(
        self,
        candidates: Sequence[float],
        center_hint: float,
        expected_width: float,
    ) -> tuple[float | None, float | None]:
        expected_left = center_hint - 0.5 * expected_width
        expected_right = center_hint + 0.5 * expected_width
        best_pair: tuple[float, float] | None = None
        best_pair_cost = float("inf")
        ordered = sorted(candidates)
        for index, left_x in enumerate(ordered):
            for right_x in ordered[index + 1 :]:
                measured_width = right_x - left_x
                if not (
                    expected_width * self.config.lane_width_minimum_scale
                    <= measured_width
                    <= expected_width * self.config.lane_width_maximum_scale
                ):
                    continue
                cost = (
                    abs(left_x - expected_left)
                    + abs(right_x - expected_right)
                    + abs(measured_width - expected_width)
                )
                if cost < best_pair_cost:
                    best_pair_cost = cost
                    best_pair = (left_x, right_x)
        if best_pair is not None:
            return best_pair

        margin = max(self.config.tracking_margin_px, expected_width * 0.30)
        left_options = [
            (abs(x - expected_left), x)
            for x in ordered
            if abs(x - expected_left) <= margin
        ]
        right_options = [
            (abs(x - expected_right), x)
            for x in ordered
            if abs(x - expected_right) <= margin
        ]
        left = min(left_options)[1] if left_options else None
        right = min(right_options)[1] if right_options else None
        if left is not None and right is not None and left == right:
            if abs(left - expected_left) <= abs(right - expected_right):
                right = None
            else:
                left = None
        if left is not None and right is not None and left >= right:
            return None, None
        return left, right

    def _fit_boundary(
        self,
        points: Sequence[tuple[float, float]],
    ) -> BoundaryFit | None:
        minimum_points = max(3, self.config.minimum_points_per_boundary)
        if len(points) < minimum_points:
            return None
        data = np.asarray(points, dtype=np.float64)
        inliers = np.ones(data.shape[0], dtype=bool)
        coefficients: np.ndarray | None = None
        for _ in range(3):
            if int(np.count_nonzero(inliers)) < minimum_points:
                return None
            coefficients = np.polyfit(data[inliers, 0], data[inliers, 1], 2)
            residuals = np.abs(
                data[:, 1] - np.polyval(coefficients, data[:, 0])
            )
            updated = residuals <= self.config.maximum_fit_residual_px
            if np.array_equal(updated, inliers):
                break
            inliers = updated
        if coefficients is None or int(np.count_nonzero(inliers)) < minimum_points:
            return None
        inlier_data = data[inliers]
        residuals = (
            inlier_data[:, 1]
            - np.polyval(coefficients, inlier_data[:, 0])
        )
        rms_error = float(np.sqrt(np.mean(residuals * residuals)))
        if rms_error > self.config.maximum_fit_residual_px:
            return None
        return BoundaryFit(
            coefficients=tuple(float(value) for value in coefficients),
            points=tuple(
                (float(y), float(x)) for y, x in inlier_data
            ),
            rms_error=rms_error,
        )

    def _validate_pair(
        self,
        left: BoundaryFit,
        right: BoundaryFit,
    ) -> tuple[BoundaryFit | None, BoundaryFit | None]:
        for y in np.linspace(
            self.config.roi_y_min,
            self.config.roi_y_max,
            7,
        ):
            expected_width = self._configured_width_at(float(y))
            measured_width = right.x_at(float(y)) - left.x_at(float(y))
            if not (
                expected_width * self.config.lane_width_minimum_scale
                <= measured_width
                <= expected_width * self.config.lane_width_maximum_scale
            ):
                if len(left.points) >= len(right.points):
                    return left, None
                return None, right
        return left, right

    def _confidence(
        self,
        left: BoundaryFit | None,
        right: BoundaryFit | None,
        single: bool,
    ) -> float:
        fits = [fit for fit in (left, right) if fit is not None]
        roi_rows = self.config.roi_y_max - self.config.roi_y_min + 1
        coverage = min(
            1.0,
            min(len(fit.points) for fit in fits) / max(1.0, roi_rows),
        )
        residual_score = max(
            0.0,
            1.0
            - max(fit.rms_error for fit in fits)
            / max(1.0e-6, self.config.maximum_fit_residual_px),
        )
        confidence = 0.55 + 0.45 * coverage * residual_score
        if single:
            confidence *= self.config.single_lane_confidence_scale
        return max(0.0, min(1.0, confidence))

    def _configured_width_coefficients(self) -> np.ndarray:
        y_span = self.config.roi_y_max - self.config.roi_y_min
        slope = (
            self.config.expected_lane_width_bottom_px
            - self.config.expected_lane_width_top_px
        ) / y_span
        intercept = (
            self.config.expected_lane_width_top_px
            - slope * self.config.roi_y_min
        )
        return np.asarray((0.0, slope, intercept), dtype=np.float64)

    def _configured_width_at(self, y: float) -> float:
        return float(np.polyval(self._configured_width_coefficients(), y))

    def _width_at(self, y: float) -> float:
        learned = float(np.polyval(self._lane_width_coefficients, y))
        configured = self._configured_width_at(y)
        return max(
            configured * self.config.lane_width_minimum_scale,
            min(
                configured * self.config.lane_width_maximum_scale,
                learned,
            ),
        )

    def _clip_path(self, coefficients: np.ndarray) -> np.ndarray:
        y_values = np.linspace(
            self.config.roi_y_min,
            self.config.roi_y_max,
            9,
        )
        x_values = np.polyval(coefficients, y_values)
        if np.all((x_values >= 0.0) & (x_values < self.config.processing_width)):
            return coefficients
        clipped = np.clip(
            x_values,
            0.0,
            float(self.config.processing_width - 1),
        )
        return np.polyfit(y_values, clipped, 2)

    def _draw_fit(
        self,
        image: np.ndarray,
        fit: BoundaryFit | None,
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        if fit is None:
            return
        cv2.polylines(
            image,
            [self._curve_points(fit.coefficients)],
            False,
            color,
            thickness,
        )

    def _curve_points(
        self,
        coefficients: Sequence[float],
    ) -> np.ndarray:
        y_values = np.arange(
            self.config.roi_y_min,
            self.config.roi_y_max + 1,
        )
        x_values = np.polyval(coefficients, y_values)
        x_values = np.clip(
            np.rint(x_values),
            0,
            self.config.processing_width - 1,
        ).astype(np.int32)
        return np.column_stack((x_values, y_values.astype(np.int32)))

    def _validate_config(self) -> None:
        if self.config.processing_width <= 0 or self.config.processing_height <= 0:
            raise ValueError("processing dimensions must be positive")
        if not (
            0 <= self.config.roi_y_min < self.config.roi_y_max
            < self.config.processing_height
        ):
            raise ValueError("ROI must satisfy 0 <= y_min < y_max < height")
        if (
            self.config.expected_lane_width_top_px <= 0.0
            or self.config.expected_lane_width_bottom_px <= 0.0
        ):
            raise ValueError("expected lane widths must be positive")

    @staticmethod
    def _odd(value: int) -> int:
        value = max(1, int(value))
        return value if value % 2 == 1 else value + 1
