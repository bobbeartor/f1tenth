import unittest

import cv2
import numpy as np

from auto_control.perspective_lane_estimator import (
    LaneEstimatorConfig,
    PerspectiveLaneEstimator,
)


class PerspectiveLaneEstimatorTest(unittest.TestCase):
    def setUp(self):
        self.config = LaneEstimatorConfig(processing_width=640)

    def test_complete_input_mask_is_not_cut_by_a_trapezoid(self):
        estimator = PerspectiveLaneEstimator(self.config)
        mask = np.zeros((480, 640), dtype=np.uint8)
        mask[240, 5] = 255
        mask[240, 635] = 255

        prepared = estimator._prepare_mask(mask, image_is_mask=True)

        self.assertEqual(prepared[240, 5], 255)
        self.assertEqual(prepared[240, 635], 255)

    def test_left_boundary_alone_predicts_lane_center(self):
        estimator = PerspectiveLaneEstimator(self.config)
        mask = self._single_boundary_mask(side="left")

        estimate = estimator.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.observation.valid)
        self.assertGreaterEqual(estimate.observation.confidence, 0.45)
        self.assertAlmostEqual(
            estimate.observation.near_center_x,
            320.0,
            delta=5.0,
        )
        self.assertAlmostEqual(
            estimate.observation.far_center_x,
            320.0,
            delta=5.0,
        )
        self.assertTrue(
            all(item.left_x is not None for item in estimate.measurements)
        )
        self.assertTrue(
            all(item.right_x is None for item in estimate.measurements)
        )

    def test_right_boundary_alone_predicts_lane_center(self):
        estimator = PerspectiveLaneEstimator(self.config)
        mask = self._single_boundary_mask(side="right")

        estimate = estimator.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.observation.valid)
        self.assertGreaterEqual(estimate.observation.confidence, 0.45)
        self.assertAlmostEqual(
            estimate.observation.near_center_x,
            320.0,
            delta=5.0,
        )
        self.assertAlmostEqual(
            estimate.observation.far_center_x,
            320.0,
            delta=5.0,
        )
        self.assertTrue(
            all(item.left_x is None for item in estimate.measurements)
        )
        self.assertTrue(
            all(item.right_x is not None for item in estimate.measurements)
        )

    def test_measured_width_is_used_after_one_boundary_disappears(self):
        estimator = PerspectiveLaneEstimator(self.config)
        pair_mask = self._lane_mask(
            center_x=380.0,
            width_scale=0.80,
            sides=("left", "right"),
        )
        estimator.estimate(pair_mask, image_is_mask=True)
        left_only_mask = self._lane_mask(
            center_x=380.0,
            width_scale=0.80,
            sides=("left",),
        )

        estimate = estimator.estimate(left_only_mask, image_is_mask=True)

        self.assertTrue(estimate.observation.valid)
        self.assertAlmostEqual(
            estimate.observation.near_center_x,
            380.0,
            delta=6.0,
        )
        self.assertAlmostEqual(
            estimate.observation.far_center_x,
            380.0,
            delta=6.0,
        )

    def test_fixed_camera_geometry_selects_real_left_and_right_pairs(self):
        estimator = PerspectiveLaneEstimator(self.config)

        estimate = estimator.estimate(
            self._fixed_geometry_mask(),
            image_is_mask=True,
        )

        self.assertTrue(estimate.observation.valid)
        self.assertEqual(len(estimate.measurements), 4)
        self.assertTrue(
            all(
                item.left_x is not None and item.right_x is not None
                for item in estimate.measurements
            )
        )
        self.assertAlmostEqual(
            estimate.observation.near_center_x,
            350.0,
            delta=5.0,
        )
        self.assertAlmostEqual(
            estimate.observation.far_center_x,
            350.0,
            delta=5.0,
        )

    def test_half_lane_width_fragments_are_not_accepted_as_a_pair(self):
        estimator = PerspectiveLaneEstimator(self.config)
        mask = self._lane_mask(
            center_x=320.0,
            width_scale=1.0,
            sides=("right",),
        )
        cv2.line(mask, (320, 0), (320, 479), 255, 5)

        estimate = estimator.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.observation.valid)
        self.assertTrue(
            all(
                not (
                    item.left_x is not None and item.right_x is not None
                )
                for item in estimate.measurements
            )
        )
        self.assertTrue(
            all(item.right_x is not None for item in estimate.measurements)
        )

    def test_current_frame_pairs_anchor_a_missing_boundary_band(self):
        estimator = PerspectiveLaneEstimator(self.config)
        mask = self._fixed_geometry_mask()
        missing_y = int(round(0.58 * (480 - 1)))
        mask[missing_y - 12 : missing_y + 13, 350:] = 0

        estimate = estimator.estimate(mask, image_is_mask=True)

        measurement = next(
            item for item in estimate.measurements if item.y == missing_y
        )
        self.assertIsNotNone(measurement.left_x)
        self.assertIsNone(measurement.right_x)
        self.assertAlmostEqual(measurement.center_x, 350.0, delta=8.0)

    def test_stale_tracking_state_resets_after_missed_frames(self):
        estimator = PerspectiveLaneEstimator(self.config)
        estimator.estimate(self._fixed_geometry_mask(), image_is_mask=True)
        self.assertTrue(estimator._previous_centers)
        empty = np.zeros((480, 640), dtype=np.uint8)

        for _ in range(self.config.reset_after_missed_frames):
            estimator.estimate(empty, image_is_mask=True)

        self.assertFalse(estimator._previous_centers)
        self.assertFalse(estimator._previous_lane_widths)

    def _single_boundary_mask(self, side: str) -> np.ndarray:
        return self._lane_mask(
            center_x=320.0,
            width_scale=1.0,
            sides=(side,),
        )

    def _lane_mask(
        self,
        center_x: float,
        width_scale: float,
        sides: tuple[str, ...],
    ) -> np.ndarray:
        height = 480
        width = 640
        estimator = PerspectiveLaneEstimator(self.config)
        mask = np.zeros((height, width), dtype=np.uint8)
        for side in sides:
            points = []
            for y in range(height):
                expected_width = estimator._expected_lane_width(
                    y,
                    width,
                    height,
                )
                direction = -1.0 if side == "left" else 1.0
                x = int(
                    round(
                        center_x
                        + direction * expected_width * width_scale * 0.5
                    )
                )
                points.append((x, y))
            cv2.polylines(
                mask,
                [np.asarray(points, dtype=np.int32)],
                False,
                255,
                5,
            )
        return mask

    @staticmethod
    def _fixed_geometry_mask() -> np.ndarray:
        """Lane geometry measured from the current 640 px debug view."""
        mask = np.zeros((480, 640), dtype=np.uint8)
        cv2.line(mask, (254, 240), (84, 364), 255, 5)
        cv2.line(mask, (446, 240), (616, 364), 255, 5)
        return mask


if __name__ == "__main__":
    unittest.main()
