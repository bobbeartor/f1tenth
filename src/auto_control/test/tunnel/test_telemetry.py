import unittest
from auto_control_tunnel.telemetry import TunnelPreviewTelemetry


class TelemetryTests(unittest.TestCase):
    def report(self, stamp, milliseconds, status='LIVE'):
        return dict(source_stamp_ns=stamp, status=status, inference_wall_ms=milliseconds,
                    scores=dict(entry=.75, exit=.05))

    def test_mean_time_reciprocal_without_receive_rate(self):
        telemetry = TunnelPreviewTelemetry()
        telemetry.update(self.report(1, 2.), 10.)
        telemetry.update(self.report(2, 4.), 10.1)
        lines = telemetry.lines('INSIDE', 10.1)
        self.assertEqual(lines[0], 'Tunnel: INSIDE')
        self.assertEqual(lines[1], 'TCN avg 333.3FPS')
        self.assertEqual(lines[2], 'TCN LIVE E:0.75 X:0.05')

    def test_waiting_warmup_and_stale_do_not_show_live_fps(self):
        telemetry = TunnelPreviewTelemetry()
        self.assertIn('WAITING', telemetry.lines('OUTSIDE', 0.)[2])
        telemetry.update(self.report(1, 2., 'WARMUP 1.5s'), 1.)
        self.assertIn('avg --FPS', telemetry.lines('OUTSIDE', 1.)[1])
        self.assertIn('WARMUP', telemetry.lines('OUTSIDE', 1.)[2])
        telemetry.update(self.report(2, 2.), 2.)
        self.assertIn('STALE', telemetry.lines('INSIDE', 2.6)[2])
        self.assertIn('avg --FPS', telemetry.lines('INSIDE', 2.6)[1])

    def test_duplicate_reports_do_not_inflate_measurements(self):
        telemetry = TunnelPreviewTelemetry()
        telemetry.update(self.report(1, 2.), 1.)
        telemetry.update(self.report(1, 100.), 1.1)
        self.assertEqual(telemetry.infer_count, 1)
        self.assertIn('500.0FPS', telemetry.lines('OUTSIDE', 1.1)[1])

    def test_invalid_time_does_not_poison_average_and_warmup_resets_it(self):
        telemetry = TunnelPreviewTelemetry()
        telemetry.update(self.report(1, 2.), 1.)
        telemetry.update(self.report(2, float('nan')), 1.1)
        telemetry.update(self.report(3, -1.), 1.2)
        self.assertIn('500.0FPS', telemetry.lines('OUTSIDE', 1.2)[1])
        telemetry.update(self.report(4, 100., 'WARMUP'), 2.)
        telemetry.update(self.report(5, 4.), 3.)
        self.assertIn('250.0FPS', telemetry.lines('OUTSIDE', 3.)[1])


if __name__ == '__main__':
    unittest.main()
