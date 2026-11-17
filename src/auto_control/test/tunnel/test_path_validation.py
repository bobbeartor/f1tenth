from copy import deepcopy
import unittest
from auto_control_tunnel.path_validation import PathRules, metric_path
from lane_fixture import lane


class PathValidationTests(unittest.TestCase):
    def setUp(self):
        self.rules = PathRules()

    def test_invalid_controller_inputs_are_not_ready(self):
        for mutate in (
            lambda m: setattr(m, 'state', 0),
            lambda m: setattr(m, 'centerline_valid', False),
            lambda m: setattr(m, 'centerline_sample_limit_reached', True),
            lambda m: setattr(m.header, 'frame_id', 'wrong'),
            lambda m: setattr(m, 'centerline_bev_width_m', 2.),
            lambda m: setattr(m, 'source_width', 0),
            lambda m: m.centerline_support.pop(),
            lambda m: m.centerline_support.__setitem__(0, 99),
            lambda m: setattr(m.centerline_points[0], 'z', 1.),
            lambda m: setattr(m.centerline_points[0], 'x', float('nan')),
        ):
            m = deepcopy(lane(1))
            mutate(m)
            self.assertIsNone(metric_path(m, self.rules))
        self.assertIsNone(metric_path(lane(1, count=5), self.rules))
        self.assertIsNone(metric_path(lane(1, count=8, step=.01), self.rules))
        self.assertIsNone(metric_path(lane(1, count=10, step=.2), self.rules))

    def test_configured_controller_limits_are_used(self):
        self.assertIsNotNone(metric_path(lane(1), self.rules))
        self.assertIsNone(metric_path(lane(1), PathRules(minimum_points=60)))


if __name__ == '__main__':
    unittest.main()
