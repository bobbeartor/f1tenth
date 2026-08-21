import unittest

import cv2
import numpy as np

from auto_control.lane_model import LaneModel, LaneModelConfig


class LaneModelTest(unittest.TestCase):
    def setUp(self):
        self.config = LaneModelConfig()

    def test_pixels_outside_y_50_through_74_are_removed(self):
        model = LaneModel(self.config)
        mask = np.full((100, 160), 255, dtype=np.uint8)

        prepared = model._prepare_mask(mask, image_is_mask=True)

        self.assertEqual(np.count_nonzero(prepared[:50]), 0)
        self.assertGreater(np.count_nonzero(prepared[50:75]), 0)
        self.assertEqual(np.count_nonzero(prepared[75:]), 0)

    def test_two_curved_boundaries_create_quadratic_centerline(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(sides=("left", "right"))

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "BOTH")
        self.assertIsNotNone(estimate.left)
        self.assertIsNotNone(estimate.right)
        self.assertGreater(abs(estimate.path.coefficients[0]), 0.005)
        for y in (50, 62, 74):
            self.assertAlmostEqual(
                estimate.path.x_at(y),
                self._center_x(y),
                delta=2.0,
            )

    def test_two_boundaries_with_narrow_apparent_width_are_kept(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(
            sides=("left", "right"),
            width_scale=0.48,
            curved_center=False,
        )

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "BOTH")
        self.assertIsNotNone(estimate.left)
        self.assertIsNotNone(estimate.right)

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
        for y in (50, 62, 74):
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
        for y in (50, 62, 74):
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
        for y in (50, 62, 74):
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
        y = 70.0
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
            self._lane_mask(sides=("right",), curve_direction=-1.0),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "RIGHT_ONLY")
        y = 70.0
        simple_half_width_center = (
            estimate.right.x_at(y) - 0.5 * model._width_at(y)
        )
        self.assertLess(
            estimate.path.x_at(y),
            simple_half_width_center - 0.5,
        )

    def test_left_only_rejects_left_bending_boundary(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("left",), curve_direction=-1.0),
            image_is_mask=True,
        )

        self.assertFalse(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "NONE")
        self.assertIsNone(estimate.left)

    def test_right_only_rejects_right_bending_boundary(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(sides=("right",), curve_direction=1.0),
            image_is_mask=True,
        )

        self.assertFalse(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "NONE")
        self.assertIsNone(estimate.right)

    def test_allowed_single_boundary_convexity_is_preserved(self):
        left_model = LaneModel(self.config)
        right_model = LaneModel(self.config)

        left = left_model.estimate(
            self._lane_mask(sides=("left",), curve_direction=1.0),
            image_is_mask=True,
        )
        right = right_model.estimate(
            self._lane_mask(sides=("right",), curve_direction=-1.0),
            image_is_mask=True,
        )

        self.assertEqual(left.path.mode, "LEFT_ONLY")
        self.assertGreater(left.left.coefficients[0], 0.0)
        self.assertEqual(right.path.mode, "RIGHT_ONLY")
        self.assertLess(right.right.coefficients[0], 0.0)

    def test_connected_trace_follows_near_horizontal_sharp_curve(self):
        model = LaneModel(self.config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        points = np.asarray(
            [
                (8, 74),
                (18, 70),
                (35, 66),
                (58, 62),
                (87, 58),
                (120, 54),
                (155, 52),
            ],
            dtype=np.int32,
        )
        cv2.polylines(mask, [points], False, 255, 3)

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertEqual(estimate.path.mode, "LEFT_ONLY")
        self.assertLessEqual(estimate.observed_y_min, 52)
        self.assertGreaterEqual(len(estimate.left.points), 20)

    def test_wide_irregular_background_component_is_rejected(self):
        model = LaneModel(self.config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        background_blob = np.asarray(
            [
                (4, 74),
                (30, 74),
                (42, 70),
                (28, 68),
                (48, 66),
                (20, 64),
                (38, 62),
                (10, 60),
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(mask, [background_blob], 255)

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertFalse(estimate.path.valid)
        self.assertIsNone(estimate.left)
        self.assertIsNone(estimate.right)

    def test_left_boundary_identity_survives_center_crossing_and_dropout(self):
        model = LaneModel(self.config)

        for x in (50, 70, 90):
            estimate = model.estimate(
                self._vertical_line_mask(x),
                image_is_mask=True,
            )
            self.assertEqual(estimate.path.mode, "LEFT_ONLY")

        dropout = model.estimate(
            np.zeros((100, 160), dtype=np.uint8),
            image_is_mask=True,
        )
        self.assertFalse(dropout.path.valid)

        reacquired = model.estimate(
            self._vertical_line_mask(100),
            image_is_mask=True,
        )
        self.assertTrue(reacquired.path.valid)
        self.assertEqual(reacquired.path.mode, "LEFT_ONLY")
        self.assertIsNotNone(reacquired.left)
        self.assertIsNone(reacquired.right)

    def test_both_boundaries_keep_measured_curve_direction(self):
        model = LaneModel(self.config)

        estimate = model.estimate(
            self._lane_mask(
                sides=("left", "right"),
                curve_direction=-1.0,
            ),
            image_is_mask=True,
        )

        self.assertEqual(estimate.path.mode, "BOTH")
        self.assertFalse(estimate.single_lane_direction_limited)
        self.assertLess(estimate.path.coefficients[0], -0.005)

    @staticmethod
    def _vertical_line_mask(x):
        mask = np.zeros((100, 160), dtype=np.uint8)
        cv2.line(mask, (x, 74), (x, 50), 255, 2)
        return mask

    def test_path_extrapolation_is_limited_to_configured_rows(self):
        config = LaneModelConfig(
            minimum_points_per_boundary=5,
            maximum_extrapolation_rows=2,
        )
        model = LaneModel(config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        points = []
        for y in range(65, 71):
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
        self.assertGreater(estimate.path.roi_y_min, config.roi_y_min)
        self.assertLess(estimate.path.roi_y_max, config.roi_y_max)
        self.assertLessEqual(
            estimate.observed_y_min - estimate.path.roi_y_min,
            config.maximum_extrapolation_rows,
        )
        self.assertLessEqual(
            estimate.path.roi_y_max - estimate.observed_y_max,
            config.maximum_extrapolation_rows,
        )

    def test_abrupt_white_tile_branch_is_not_joined_to_lane(self):
        model = LaneModel(self.config)
        mask = np.zeros((100, 160), dtype=np.uint8)
        lane_points = []
        for y in range(68, 75):
            x = 80.0 - 0.5 * self._lane_width(y)
            lane_points.append((int(round(x)), y))
        cv2.polylines(
            mask,
            [np.asarray(lane_points, dtype=np.int32)],
            False,
            255,
            2,
        )
        cv2.line(mask, (50, 67), (50, 60), 255, 2)

        prepared = model._prepare_mask(mask, image_is_mask=True)
        left_points, _ = model._collect_boundary_points(prepared)

        self.assertTrue(left_points)
        # The 2 px lane stroke can occupy one row above its endpoint, but the
        # disconnected tile at x=50 must never become the tracked boundary.
        self.assertGreaterEqual(min(y for y, _ in left_points), 67.0)
        self.assertLess(max(x for _, x in left_points), 40.0)

    def test_only_roi_pixels_affect_the_curve(self):
        model = LaneModel(self.config)
        mask = self._lane_mask(sides=("left", "right"))
        cv2.line(mask, (0, 0), (159, 49), 255, 5)
        mask[99, :] = 255

        estimate = model.estimate(mask, image_is_mask=True)

        self.assertTrue(estimate.path.valid)
        self.assertAlmostEqual(
            estimate.path.x_at(67),
            self._center_x(67),
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
        curve_direction: float = 1.0,
    ) -> np.ndarray:
        mask = np.zeros((100, 160), dtype=np.uint8)
        for side in sides:
            points = []
            for y in range(
                self.config.roi_y_min,
                self.config.roi_y_max + 1,
            ):
                direction = -1.0 if side == "left" else 1.0
                center_x = (
                    self._center_x(y, curve_direction)
                    if curved_center
                    else 80.0
                )
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
    def _center_x(y: float, direction: float = 1.0) -> float:
        return 80.0 + direction * 0.015 * (y - 90.0) ** 2

    def _lane_width(self, y: float) -> float:
        return LaneModel(self.config)._configured_width_at(y)


if __name__ == "__main__":
    unittest.main()
