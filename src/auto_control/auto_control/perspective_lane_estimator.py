"""Lane-centre estimation without a bird's-eye-view transformation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from auto_control.perspective_lane_controller import LaneObservation


@dataclass(frozen=True)
class LaneEstimatorConfig:
    processing_width: int = 640
    scan_y_ratios: tuple[float, ...] = (0.50, 0.58, 0.67, 0.76)
    scan_band_height_ratio: float = 0.030
    minimum_band_occupancy: float = 0.20
    maximum_segment_width_ratio: float = 0.16
    lane_width_far_ratio: float = 0.187
    lane_width_near_ratio: float = 0.562
    pair_minimum_width_scale: float = 0.48
    pair_maximum_width_scale: float = 1.55
    single_boundary_maximum_error_scale: float = 0.30
    minimum_brightness: int = 130
    tophat_threshold: int = 40
    tophat_kernel: int = 21
    blur_kernel: int = 5
    morphology_kernel: int = 3


@dataclass(frozen=True)
class BandMeasurement:
    y: int
    center_x: float
    left_x: float | None
    right_x: float | None
    quality: float


@dataclass(frozen=True)
class LaneEstimate:
    observation: LaneObservation
    mask: np.ndarray
    measurements: tuple[BandMeasurement, ...]


class PerspectiveLaneEstimator:
    """Find left/right lane boundaries in horizontal perspective-image bands."""

    def __init__(self, config: LaneEstimatorConfig) -> None:
        self.config = config
        self._previous_centers: dict[int, float] = {}
        self._previous_lane_widths: dict[int, float] = {}

    def estimate(
        self,
        image: np.ndarray,
        image_is_mask: bool = False,
    ) -> LaneEstimate:
        mask = self._prepare_mask(image, image_is_mask)
        height, width = mask.shape
        measurements: list[BandMeasurement] = []

        for level, y_ratio in enumerate(self.config.scan_y_ratios):
            y = int(round(y_ratio * (height - 1)))
            candidates = self._band_candidates(mask, y)
            center_hint = self._previous_centers.get(level, width * 0.5)
            measurement = self._select_boundaries(
                candidates,
                y,
                width,
                height,
                center_hint,
                self._previous_lane_widths.get(level),
            )
            if measurement is not None:
                measurements.append(measurement)
                self._previous_centers[level] = measurement.center_x
                if (
                    measurement.left_x is not None
                    and measurement.right_x is not None
                ):
                    measured_width = (
                        measurement.right_x - measurement.left_x
                    )
                    previous_width = self._previous_lane_widths.get(level)
                    if previous_width is None:
                        self._previous_lane_widths[level] = measured_width
                    else:
                        self._previous_lane_widths[level] = (
                            0.80 * previous_width + 0.20 * measured_width
                        )

        observation = self._make_observation(measurements, width, height)
        return LaneEstimate(observation, mask, tuple(measurements))

    def create_debug_image(
        self,
        image: np.ndarray,
        estimate: LaneEstimate,
    ) -> np.ndarray:
        if image.ndim == 2:
            debug = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            debug = image.copy()
        debug = self._resize_to_processing_width(debug)

        green = np.zeros_like(debug)
        green[:, :, 1] = estimate.mask
        debug = cv2.addWeighted(debug, 0.78, green, 0.35, 0.0)
        height, width = estimate.mask.shape
        for y_ratio in self.config.scan_y_ratios:
            y = int(round(y_ratio * (height - 1)))
            cv2.line(debug, (0, y), (width - 1, y), (0, 180, 255), 1)
        cv2.line(
            debug,
            (width // 2, 0),
            (width // 2, height - 1),
            (255, 0, 255),
            1,
        )

        for measurement in estimate.measurements:
            y = measurement.y
            if measurement.left_x is not None:
                cv2.circle(debug, (int(measurement.left_x), y), 5, (255, 0, 0), -1)
            if measurement.right_x is not None:
                cv2.circle(debug, (int(measurement.right_x), y), 5, (0, 0, 255), -1)
            cv2.circle(debug, (int(measurement.center_x), y), 6, (0, 255, 255), -1)

        observation = estimate.observation
        if observation.valid:
            far_y = int(min(self.config.scan_y_ratios) * height)
            near_y = int(max(self.config.scan_y_ratios) * height)
            cv2.line(
                debug,
                (int(observation.far_center_x), far_y),
                (int(observation.near_center_x), near_y),
                (0, 255, 255),
                3,
            )
        text = (
            f"confidence={observation.confidence:.2f} "
            f"bands={len(estimate.measurements)}"
        )
        cv2.putText(
            debug,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return debug

    def _prepare_mask(self, image: np.ndarray, image_is_mask: bool) -> np.ndarray:
        if image.size == 0:
            raise ValueError("input image is empty")
        resized = self._resize_to_processing_width(
            image,
            preserve_binary=image_is_mask or image.ndim == 2,
        )
        if image_is_mask or resized.ndim == 2:
            if resized.ndim == 3:
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            else:
                gray = resized
            _, candidate = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        else:
            gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
            blur_kernel = self._odd(self.config.blur_kernel)
            blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0.0)
            tophat_kernel = self._odd(self.config.tophat_kernel)
            element = cv2.getStructuringElement(
                cv2.MORPH_RECT,
                (tophat_kernel, tophat_kernel),
            )
            tophat = cv2.morphologyEx(blurred, cv2.MORPH_TOPHAT, element)
            _, bright = cv2.threshold(
                blurred,
                self.config.minimum_brightness,
                255,
                cv2.THRESH_BINARY,
            )
            _, narrow_bright = cv2.threshold(
                tophat,
                self.config.tophat_threshold,
                255,
                cv2.THRESH_BINARY,
            )
            candidate = cv2.bitwise_and(bright, narrow_bright)

        morphology_kernel = self._odd(self.config.morphology_kernel)
        element = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (morphology_kernel, morphology_kernel),
        )
        # lane_detect has already removed non-lane pixels. Keep the complete
        # extracted mask here. The old fixed trapezoid discarded a boundary
        # whenever the camera or vehicle was significantly off-centre.
        return cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, element)

    def _resize_to_processing_width(
        self,
        image: np.ndarray,
        preserve_binary: bool = False,
    ) -> np.ndarray:
        target_width = self.config.processing_width
        if target_width <= 0 or image.shape[1] == target_width:
            return image.copy()
        scale = target_width / float(image.shape[1])
        if preserve_binary:
            interpolation = cv2.INTER_NEAREST
        else:
            interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        return cv2.resize(image, None, fx=scale, fy=scale, interpolation=interpolation)

    def _band_candidates(
        self,
        mask: np.ndarray,
        y: int,
    ) -> list[tuple[float, float]]:
        height, width = mask.shape
        half_band = max(
            1,
            int(round(self.config.scan_band_height_ratio * height * 0.5)),
        )
        y0 = max(0, y - half_band)
        y1 = min(height, y + half_band + 1)
        profile = np.count_nonzero(mask[y0:y1, :], axis=0)
        minimum_hits = max(
            2,
            int(round((y1 - y0) * self.config.minimum_band_occupancy)),
        )
        active = profile >= minimum_hits
        maximum_width = max(
            3,
            int(round(self.config.maximum_segment_width_ratio * width)),
        )
        candidates: list[tuple[float, float]] = []
        start: int | None = None
        for x in range(width + 1):
            is_active = x < width and bool(active[x])
            if is_active and start is None:
                start = x
            elif not is_active and start is not None:
                end = x
                if end - start <= maximum_width:
                    weights = profile[start:end].astype(np.float64)
                    columns = np.arange(start, end, dtype=np.float64)
                    total = float(weights.sum())
                    if total > 0.0:
                        center = float(np.dot(columns, weights) / total)
                        strength = min(1.0, total / max(1.0, (end - start) * (y1 - y0)))
                        candidates.append((center, strength))
                start = None
        return candidates

    def _select_boundaries(
        self,
        candidates: Sequence[tuple[float, float]],
        y: int,
        width: int,
        height: int,
        center_hint: float,
        lane_width_hint: float | None = None,
    ) -> BandMeasurement | None:
        if not candidates:
            return None
        expected_width = (
            lane_width_hint
            if lane_width_hint is not None and lane_width_hint > 0.0
            else self._expected_lane_width(y, width, height)
        )
        minimum_width = expected_width * self.config.pair_minimum_width_scale
        maximum_width = expected_width * self.config.pair_maximum_width_scale
        best: tuple[float, float, float, float] | None = None
        best_cost = float("inf")

        ordered = sorted(candidates)
        for left_index, (left_x, left_strength) in enumerate(ordered):
            for right_x, right_strength in ordered[left_index + 1 :]:
                pair_width = right_x - left_x
                if pair_width < minimum_width or pair_width > maximum_width:
                    continue
                center = (left_x + right_x) * 0.5
                center_cost = abs(center - center_hint) / max(1.0, width * 0.20)
                width_cost = abs(pair_width - expected_width) / max(1.0, expected_width)
                strength_reward = 0.10 * (left_strength + right_strength)
                cost = 1.8 * center_cost + 0.8 * width_cost - strength_reward
                if cost < best_cost:
                    best_cost = cost
                    best = (left_x, right_x, center, min(left_strength, right_strength))

        if best is not None:
            left_x, right_x, center, strength = best
            quality = max(0.75, min(1.0, 0.85 + 0.15 * strength))
            return BandMeasurement(y, center, left_x, right_x, quality)

        expected_left = center_hint - expected_width * 0.5
        expected_right = center_hint + expected_width * 0.5
        maximum_error = (
            expected_width * self.config.single_boundary_maximum_error_scale
        )
        single_options: list[tuple[float, BandMeasurement]] = []
        for candidate_x, strength in ordered:
            left_error = abs(candidate_x - expected_left)
            if left_error <= maximum_error:
                single_options.append(
                    (
                        left_error,
                        BandMeasurement(
                            y,
                            candidate_x + expected_width * 0.5,
                            candidate_x,
                            None,
                            0.50 + 0.10 * strength,
                        ),
                    )
                )
            right_error = abs(candidate_x - expected_right)
            if right_error <= maximum_error:
                single_options.append(
                    (
                        right_error,
                        BandMeasurement(
                            y,
                            candidate_x - expected_width * 0.5,
                            None,
                            candidate_x,
                            0.50 + 0.10 * strength,
                        ),
                    )
                )
        if not single_options:
            return None
        return min(single_options, key=lambda item: item[0])[1]

    def _expected_lane_width(self, y: int, width: int, height: int) -> float:
        far_y = min(self.config.scan_y_ratios) * height
        near_y = max(self.config.scan_y_ratios) * height
        progress = (y - far_y) / max(1.0, near_y - far_y)
        progress = max(0.0, min(1.0, progress))
        width_ratio = (
            self.config.lane_width_far_ratio
            + (self.config.lane_width_near_ratio - self.config.lane_width_far_ratio)
            * progress
        )
        return max(4.0, width * width_ratio)

    def _make_observation(
        self,
        measurements: Sequence[BandMeasurement],
        width: int,
        height: int,
    ) -> LaneObservation:
        if len(measurements) < 2:
            return LaneObservation(image_width=width)
        y_values = np.array([item.y for item in measurements], dtype=np.float64)
        centers = np.array([item.center_x for item in measurements], dtype=np.float64)
        qualities = np.array([item.quality for item in measurements], dtype=np.float64)
        degree = 2 if len(measurements) >= 3 else 1
        coefficients = np.polyfit(y_values, centers, degree, w=qualities)
        far_y = min(self.config.scan_y_ratios) * height
        near_y = max(self.config.scan_y_ratios) * height
        far_center = float(np.polyval(coefficients, far_y))
        near_center = float(np.polyval(coefficients, near_y))
        far_center = max(0.0, min(float(width - 1), far_center))
        near_center = max(0.0, min(float(width - 1), near_center))
        coverage = min(1.0, len(measurements) / 3.0)
        confidence = float(np.mean(qualities)) * coverage
        return LaneObservation(
            image_width=width,
            near_center_x=near_center,
            far_center_x=far_center,
            confidence=max(0.0, min(1.0, confidence)),
            valid=True,
        )

    @staticmethod
    def _odd(value: int) -> int:
        value = max(1, int(value))
        return value if value % 2 == 1 else value + 1
