from types import SimpleNamespace as NS
import unittest
from auto_control_tunnel.ir_length_exit import IrLengthExit
from auto_control_tunnel.selection import LaneSelector
from lane_fixture import lane, ns


def measurement(t, length, **kwargs):
    message = lane(t, **kwargs)
    message.observed_lane_length_valid = True
    message.observed_lane_length_m = length
    return message


class LengthExitTests(unittest.TestCase):
    def create(self, **kwargs):
        self.selector = LaneSelector(validation_mode=True, ir_length_exit=IrLengthExit(**kwargs))
        self.selector.event('entry', ns(1), ns(1))
        self.selector.lane('IR', lane(1.01), ns(1.015))
        self.t = 1.01
        return self.selector

    def send(self, length, valid=True, **kwargs):
        self.t += .04
        m = measurement(self.t, length, valid=valid, **kwargs)
        return self.selector.lane('IR', m, ns(self.t + .005))

    def baseline(self):
        for _ in range(3):
            self.send(2.)

    def test_disabled_preserves_inside_and_tcn_policy(self):
        s = self.create()
        self.baseline()
        for _ in range(6): self.send(0.)
        self.assertEqual((s.state, s.selected, s.compute), ('INSIDE', 'IR', 'IR'))
        self.assertEqual(s.ir_length_exit.reference, 0.)
        self.assertTrue(s.event('exit', ns(2), ns(2)))

    def test_exact_threshold_starts_exit_but_only_valid_rgb_finishes(self):
        s = self.create(enabled=True)
        self.baseline()
        self.assertEqual(s.ir_length_exit.reference, 2.)
        self.send(.3, valid=False)
        self.send(.3, valid=False)
        self.assertEqual(s.state, 'INSIDE')
        self.send(.3, valid=False)
        self.assertEqual((s.state,s.selected,s.camera,s.compute), ('EXITING','IR','IR','BOTH'))
        self.assertEqual(s.reason, 'ir_length_drop')
        self.assertIsNone(s.lane('RGB', lane(self.t+.01,valid=False), ns(self.t+.015)))
        self.assertEqual(s.state, 'EXITING')
        rgb = lane(self.t+.02)
        self.assertIs(s.lane('RGB',rgb,ns(self.t+.025)),rgb)
        self.assertEqual((s.state,s.selected), ('OUTSIDE','RGB'))
        self.assertEqual(s.last_handover['trigger'], 'ir_length_drop')

    def test_ratio_configurable_and_low_sequence_resets_on_recovery(self):
        s = self.create(enabled=True, ratio=.5)
        self.baseline()
        self.send(.9); self.send(1.1); self.send(.9); self.send(.9)
        self.assertEqual(s.state,'INSIDE')
        self.send(.9)
        self.assertEqual(s.state,'EXITING')

    def test_reference_ignores_single_long_outlier_and_tiny_initial_length(self):
        s = self.create(enabled=True)
        for _ in range(3): self.send(.1)
        self.assertEqual(s.ir_length_exit.reference,0)
        self.baseline()
        self.send(10.)
        self.send(2.); self.send(2.)
        self.assertEqual(s.ir_length_exit.reference,2.)
        for _ in range(3): self.send(2.5)
        self.assertEqual(s.ir_length_exit.reference,2.5)

    def test_zero_is_evidence_but_unavailable_invalid_stale_future_are_not(self):
        for invalid in ('missing','nan','negative','flag','scale','stale','future'):
            with self.subTest(invalid=invalid):
                s=self.create(enabled=True); self.baseline(); self.send(0.); self.send(0.)
                self.t += .04
                m=measurement(self.t,0.)
                now=ns(self.t+.005)
                if invalid=='missing': del m.observed_lane_length_valid
                if invalid=='nan': m.observed_lane_length_m=float('nan')
                if invalid=='negative': m.observed_lane_length_m=-1.
                if invalid=='flag': m.observed_lane_length_valid=False
                if invalid=='scale': m.centerline_bev_width_m=2.
                if invalid=='stale': now=ns(self.t+.3)
                if invalid=='future': now=ns(self.t-.1)
                s.lane('IR',m,now)
                self.assertEqual(s.state,'INSIDE')
                self.send(0.); self.send(0.)
                self.assertEqual(s.state,'INSIDE')
                self.send(0.)
                self.assertEqual(s.state,'EXITING')

    def test_duplicates_and_long_gap_cannot_fill_confirmation(self):
        s=self.create(enabled=True); self.baseline(); self.send(0.)
        m=measurement(self.t,0.)
        for _ in range(10): s.lane('IR',m,ns(self.t+.01))
        self.assertEqual(s.ir_length_exit.below_count,1)
        self.t += .3
        self.send(0.); self.send(0.)
        self.assertEqual(s.state,'INSIDE')
        self.send(0.)
        self.assertEqual(s.state,'EXITING')

    def test_inside_reports_cannot_reverse_forced_exit_and_next_visit_resets_max(self):
        s=self.create(enabled=True); self.baseline()
        for _ in range(3): self.send(0.)
        trigger=s.ir_exit_latched_at
        self.assertFalse(s.observe_tcn_state('inside',trigger+1,trigger+2))
        self.assertFalse(s.event('entry',trigger+1,trigger+2))
        self.assertFalse(s.observe_tcn_state('outside',trigger-1,trigger+2))
        s.lane('RGB',lane(self.t+.02),ns(self.t+.025))
        self.assertFalse(s.observe_tcn_state('inside',trigger+1,trigger+2))
        self.assertFalse(s.observe_tcn_state('outside',trigger+1,trigger+ns(1)))
        self.assertTrue(s.observe_tcn_state('outside',trigger+ns(.04),trigger+ns(.05)))
        self.assertTrue(s.event('entry',trigger+ns(.1),trigger+ns(.11)))
        self.assertEqual(s.ir_length_exit.reference,0.)
        self.assertEqual(s.state,'ENTERING')

    def test_invalid_parameters(self):
        for values in (dict(ratio=0),dict(ratio=1),dict(ratio=float('nan')),
                       dict(confirm_frames=0),dict(confirm_frames=1.5),dict(confirm_frames=True),
                       dict(minimum_reference_m=-1),dict(enabled='false')):
            with self.assertRaises(ValueError): IrLengthExit(**values)


if __name__ == '__main__': unittest.main()
