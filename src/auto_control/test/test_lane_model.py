import unittest

import cv2
import numpy as np

from auto_control.lane_model import LaneModel, LaneModelConfig


class LaneModelTest(unittest.TestCase):
    def setUp(self):
        self.config = LaneModelConfig()

    def test_pixels_outside_y_60_through_90_are_removed(self):
        model = LaneModel(self.config)
        mask = np.full((100, 160), 255, dtype=np.uint8)

        prepared = model._prepare_mask(mask, image_is_mask=True)

        self.assertEqual(np.count_nonzero(prepared[:60]), 0)
        self.assertGreater(np.count_nonzero(prepared[60:91]), 0)
        self.assertEqual(np.count_nonzero(prepared[91:]), 0)

    def test_two_curved_boundaries_create_quadratic_centerline(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(sides=("left", "right"))

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "BOTH")
        self.assertIsNotNone(estimate.left)
        self.assertIsNotNone(estimate.right)
        self.assertGreater(abs(estimate.path.coefficients[0]), 0.005)
        for y in (60, 75, 90):
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
            self._lane_mask(sides=("left",), curved_center=False),
            image_is_mask=True,
        )

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        self.assertGreaterEqual(estimate.path.confidence, 0.45)
        for y in (60, 75, 90):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                80.0,
                delta=2.5,
            )

    def test_right_boundary_alone_reconstructs_centerline(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("right",), curved_center=False),
            image_is_mask=True,
        )

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "RIGHT_ONLY")
        self.assertGreaterEqual(estimate.path.confidence, 0.45)
        for y in (60, 75, 90):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                80.0,
                delta=2.5,
            )

    def test_measured_width_is_used_when_one_boundary_disappears(self):
        config = LaneModelConfig(lane_width_learning_alpha=1.0)
        model = LaneModel(config)
        model.estimate(
            self._lane_mask(
                sides=("left", "right"),
                width_scale=0.80,
                curved_center=False,
            ),
            image_is_mask=True,
        )

        estimate = model.estimate(
            self._lane_mask(
                sides=("left",),
                width_scale=0.80,
                curved_center=False,
            ),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        for y in (60, 75, 90):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                80.0,
                delta=2.5,
            )

    def test_single_curved_boundary_uses_larger_center_offset(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("left",)),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        self.assertGreater(estimate.single_lane_curvature, 0.0)
        self.assertGreater(estimate.single_lane_offset_scale, 1.0)
        y = 75.0
        simple_half_width_center = (
            estimate.left.x_at(y) + 0.5 * model._width_at(y)
        )
        self.assertGreater(
            estimate.path.x_at(y),
            simple_half_width_center + 2.0,
        )
        self.assertLessEqual(
            estimate.single_lane_offset_scale,
            self.config.single_lane_maximum_offset_scale,
        )

    def test_right_curved_boundary_offsets_center_inward(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("right",)),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "RIGHT_ONLY")
        y = 75.0
        simple_half_width_center = (
            estimate.right.x_at(y) - 0.5 * model._width_at(y)
        )
        self.assertLess(
            estimate.path.x_at(y),
            simple_half_width_center - 0.5,
        )

    def test_path_extrapolation_is_limited_to_five_rows(self):
        model = LaneModel(self.config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        points = []
        for y in range(70, 81):
            x = 80.0 - 0.5 * self._lane_width(y)
            points.append((int(round(x)), y))
        cv2.polylines(
            mask,
            [np.asarray(points, dtype=np.int32)],
            False,
            255,
            2,
        )

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertGreater(estimate.path.roi_y_min, self.config.roi_y_min)
        self.assertLess(estimate.path.roi_y_max, self.config.roi_y_max)
        self.assertLessEqual(
            estimate.observed_y_min - estimate.path.roi_y_min,
            self.config.maximum_extrapolation_rows,
        )
        self.assertLessEqual(
            estimate.path.roi_y_max - estimate.observed_y_max,
            self.config.maximum_extrapolation_rows,
        )

    def test_abrupt_white_tile_branch_is_not_joined_to_lane(self):
        model = LaneModel(self.config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        lane_points = []
        for y in range(75, 91):
            x = 80.0 - 0.5 * self._lane_width(y)
            lane_points.append((int(round(x)), y))
        cv2.polylines(
            mask,
            [np.asarray(lane_points, dtype=np.int32)],
            False,
            255,
            2,
        )
        cv2.line(mask, (50, 74), (50, 60), 255, 2)

        prepared = model._prepare_mask(mask, image_is_mask=True)
        left_points, _ = model._collect_boundary_points(prepared)

        self.assertTrue(left_points)
        # The 2 px lane stroke can occupy one row above its endpoint, but the
        # disconnected tile at x=50 must never become the tracked boundary.
        self.assertGreaterEqual(min(y for y, _ in left_points), 74.0)
        self.assertLess(max(x for _, x in left_points), 30.0)

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
        curved_center: bool = True,
    ) -> np.ndarray:
        mask = np.zeros((100, 160), dtype=np.uint8)
        for side in sides:
            points = []
            for y in range(60, 91):
                direction = -1.0 if side == "left" else 1.0
                center_x = self._center_x(y) if curved_center else 80.0
                x = (
                    center_x
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
        return 80.0 + 0.015 * (y - 90.0) ** 2

    def _lane_width(self, y: float) -> float:
        return LaneModel(self.config)._configured_width_at(y)


if __name__ == "__main__":
    unittest.main()
