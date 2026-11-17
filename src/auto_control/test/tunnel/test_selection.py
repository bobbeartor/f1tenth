from copy import deepcopy
import unittest
from auto_control_tunnel.selection import LaneSelector
from auto_control_tunnel.path_validation import metric_path, stamp_ns
from lane_fixture import lane, ns


class HandoffTests(unittest.TestCase):
    def setUp(self):
        self.selector = LaneSelector(validation_mode=True)

    def send(self, source, t, **kwargs):
        return self.selector.lane(source, lane(t, **kwargs), ns(t + .005))

    def entry(self, t=1.):
        self.selector.event('entry', ns(t), ns(t))

    def test_outside_routes_original_only(self):
        message = lane(.95)
        self.assertIs(self.selector.lane('RGB', message, ns(.96)), message)
        self.assertEqual(message.lane_source, 'RGB')
        self.assertIsNone(self.send('IR', .98))
        self.assertEqual(self.selector.compute, 'RGB')

    def test_entry_prepares_ir_without_cutting_live_rgb(self):
        self.send('RGB', .98)
        self.entry()
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.camera,
                          self.selector.compute), ('ENTERING', 'RGB', 'IR', 'BOTH'))
        for t in (1.01, 1.04, 1.08, 1.12, 1.16, 1.20):
            self.assertIsNotNone(self.send('RGB', t))
        self.assertEqual(self.selector.selected, 'RGB')

    def test_first_valid_target_switches_on_same_callback_without_blending(self):
        self.entry()
        self.send('RGB', 1.02)
        target = lane(1.04, lateral=.1)
        before = deepcopy(target)
        result = self.selector.lane('IR', target, ns(1.045))
        self.assertIs(result, target)
        self.assertEqual(result.centerline_points, before.centerline_points)
        self.assertEqual(result.centerline_support, before.centerline_support)
        self.assertEqual(result.header, before.header)
        self.assertEqual(result.stop_line_present, before.stop_line_present)
        self.assertEqual(result.stop_line_distance_m, before.stop_line_distance_m)
        self.assertAlmostEqual(metric_path(result, self.selector.rules)[0][1], .1)
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute),
                         ('INSIDE', 'IR', 'IR'))
        self.assertEqual(self.selector.last_handover['reason'], 'first_valid_target')
        self.assertAlmostEqual(self.selector.last_handover['total_ms'], 45.)
        self.assertIsNone(self.send('RGB', 1.05))

    def test_exit_keeps_ir_profile_and_control_until_first_valid_rgb(self):
        self.entry()
        self.send('IR', 1.01)
        self.selector.event('exit', ns(2), ns(2))
        self.assertEqual((self.selector.state, self.selector.selected, self.selector.compute,
                          self.selector.camera), ('EXITING', 'IR', 'BOTH', 'IR'))
        self.assertIsNotNone(self.send('IR', 2.01))
        self.assertIsNone(self.send('RGB', 2.02, valid=False))
        self.assertEqual(self.selector.camera, 'IR')
        self.assertIsNotNone(self.send('RGB', 2.03, lateral=.05))
        self.assertEqual((self.selector.state, self.selector.camera, self.selector.compute),
                         ('OUTSIDE', 'RGB', 'RGB'))
        self.assertIsNone(self.send('IR', 2.04))

    def test_invalid_target_never_clears_origin_and_origin_invalid_is_delivered(self):
        self.entry()
        self.send('RGB', 1.02)
        self.assertIsNone(self.send('IR', 1.03, valid=False))
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertIsNotNone(self.send('RGB', 1.04))
        result = self.send('RGB', 1.05, valid=False)
        self.assertFalse(result.centerline_valid)
        self.assertIsNone(self.send('IR', 1.06, valid=False))
        self.assertEqual(self.selector.selected, 'RGB')

    def test_target_must_pass_controller_geometry_not_just_valid_flag(self):
        self.entry()
        self.send('RGB', 1.02)
        self.assertIsNone(self.send('IR', 1.03, count=5))
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertIsNotNone(self.send('IR', 1.04))
        self.assertEqual(self.selector.selected, 'IR')

    def test_no_wait_timeout_forces_unready_target_or_replays_origin(self):
        self.entry()
        self.assertIsNotNone(self.send('RGB', 2.0))
        self.assertIsNone(self.send('IR', 2.01, valid=False))
        self.assertEqual(self.selector.compute, 'BOTH')
        self.assertEqual(self.selector.selected, 'RGB')
        self.assertIsNone(self.send('RGB', 2.0))
        self.assertIsNotNone(self.send('IR', 2.02))
        self.assertEqual(self.selector.selected, 'IR')

    def test_duplicate_reports_do_not_restart_preparation(self):
        self.entry()
        self.assertFalse(self.selector.event('entry', ns(1.1), ns(1.1)))
        self.assertEqual(self.selector.started_ns, ns(1))

    def test_reverse_request_cancels_preparation_immediately(self):
        self.entry()
        self.send('RGB', 1.01)
        self.selector.event('exit', ns(1.1), ns(1.1))
        self.assertEqual((self.selector.state, self.selector.compute, self.selector.camera),
                         ('OUTSIDE', 'RGB', 'RGB'))
        self.assertIsNotNone(self.send('RGB', 1.12))

    def test_previous_visit_and_duplicate_results_cannot_trigger_handover(self):
        self.send('IR', .9)
        self.entry()
        self.assertIsNone(self.send('IR', .9))
        self.assertEqual(self.selector.state, 'ENTERING')
        self.assertIsNotNone(self.send('IR', 1.02))
        self.assertIsNone(self.send('IR', 1.02))

    def test_new_valid_target_can_be_older_than_last_other_camera_capture(self):
        self.send('RGB', .95)
        self.entry()
        self.send('IR', 1.05)
        self.selector.event('exit', ns(2), ns(2))
        self.selector.lane('IR', lane(2.01), ns(2.02))
        target = lane(1.95, lateral=.1)
        result = self.selector.lane('RGB', target, ns(2.03))
        self.assertIs(result, target)
        self.assertEqual(stamp_ns(result), ns(1.95))
        self.assertEqual(result.lane_source, 'RGB')
        self.assertEqual(self.selector.state, 'OUTSIDE')
        self.assertIsNone(self.selector.lane('RGB', lane(1.94), ns(2.04)))

    def test_stale_and_future_target_cannot_finish_preparation(self):
        self.entry()
        for stamp, now in ((.8, 1.1), (1.2, 1.1)):
            self.assertIsNone(self.selector.lane('IR', lane(stamp), ns(now)))
            self.assertEqual(self.selector.state, 'ENTERING')
        self.assertIsNotNone(self.send('IR', 1.15))

    def test_disagreement_does_not_create_a_third_path_or_add_confirmation(self):
        self.entry()
        self.send('RGB', 1.01)
        target = lane(1.03, lateral=.3)
        self.assertIs(self.selector.lane('IR', target, ns(1.04)), target)
        self.assertEqual(self.selector.selected, 'IR')

    def test_guarded_event_age_checks_remain(self):
        selector = LaneSelector()
        self.assertFalse(selector.event('entry', ns(1), ns(2)))
        self.assertFalse(selector.event('entry', ns(3), ns(2)))
        self.assertTrue(selector.event('entry', ns(2), ns(2)))
        self.assertFalse(selector.event('exit', ns(2), ns(2)))


if __name__ == '__main__':
    unittest.main()
