import unittest

import cv2
import numpy as np

from auto_control.lane_model import LaneModel, LaneModelConfig


class LaneModelTest(unittest.TestCase):
    def setUp(self):
        self.config = LaneModelConfig()

    def test_pixels_outside_y_60_through_83_are_removed(self):
        model = LaneModel(self.config)
        mask = np.full((100, 160), 255, dtype=np.uint8)

        prepared = model._prepare_mask(mask, image_is_mask=True)

        self.assertEqual(np.count_nonzero(prepared[:60]), 0)
        self.assertGreater(np.count_nonzero(prepared[60:84]), 0)
        self.assertEqual(np.count_nonzero(prepared[84:]), 0)

    def test_two_curved_boundaries_create_quadratic_centerline(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(sides=("left", "right"))

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "BOTH")
        self.assertIsNotNone(estimate.left)
        self.assertIsNotNone(estimate.right)
        self.assertGreater(abs(estimate.path.coefficients[0]), 0.005)
        for y in (60, 75, 83):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                self._center_x(y),
                delta=2.0,
            )

    def test_configured_width_matches_new_camera_calibration(self):
        model = LaneModel(self.config)

        for y_ratio, width_ratio in zip(
            self.config.expected_lane_width_y_ratios,
            self.config.expected_lane_width_ratios,
        ):
            self.assertAlmostEqual(
                model._configured_width_at(
                    y_ratio * self.config.processing_height
                ),
                width_ratio * self.config.processing_width,
                places=6,
            )

    def test_left_boundary_alone_reconstructs_centerline(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("left",)),
            image_is_mask=True,
        )

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        self.assertGreaterEqual(estimate.path.confidence, 0.45)
        for y in (60, 75, 83):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                self._center_x(y),
                delta=2.5,
            )

    def test_right_boundary_alone_reconstructs_centerline(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("right",)),
            image_is_mask=True,
        )

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "RIGHT_ONLY")
        self.assertGreaterEqual(estimate.path.confidence, 0.45)
        for y in (60, 75, 83):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                self._center_x(y),
                delta=2.5,
            )

    def test_measured_width_is_used_when_one_boundary_disappears(self):
        config = LaneModelConfig(lane_width_learning_alpha=1.0)
        model = LaneModel(config)
        model.estimate(
            self._lane_mask(sides=("left", "right"), width_scale=0.80),
            image_is_mask=True,
        )

        estimate = model.estimate(
            self._lane_mask(sides=("left",), width_scale=0.80),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        for y in (60, 75, 83):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                self._center_x(y),
                delta=2.5,
            )

    def test_only_roi_pixels_affect_the_curve(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(sides=("left", "right"))
        cv2.line(mask, (0, 0), (159, 59), 255, 5)
        mask[99, :] = 255

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertAlmostEqual(
            estimate.path.x_at(75),
            self._center_x(75),
            delta=2.0,
        )

    def test_lane_loss_is_invalid(self):
        model = LaneModel(self.config)
        empty = np.zeros((100, 160), dtype=np.uint8)

        estimate = model.estimate(empty, image_is_mask=True)

        self.assertFalse(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "NONE")

    def _lane_mask(
        self,
        sides: tuple[str, ...],
        width_scale: float = 1.0,
    ) -> np.ndarray:
        mask = np.zeros((100, 160), dtype=np.uint8)
        for side in sides:
            points = []
            for y in range(60, 84):
                direction = -1.0 if side == "left" else 1.0
                x = (
                    self._center_x(y)
                    + direction * self._lane_width(y) * width_scale * 0.5
                )
                points.append((int(round(x)), y))
            cv2.polylines(
                mask,
                [np.asarray(points, dtype=np.int32)],
                False,
                255,
                2,
            )
        return mask

    @staticmethod
    def _center_x(y: float) -> float:
        return 80.0 + 0.015 * (y - 83.0) ** 2

    def _lane_width(self, y: float) -> float:
        return LaneModel(self.config)._configured_width_at(y)


if __name__ == "__main__":
    unittest.main()
