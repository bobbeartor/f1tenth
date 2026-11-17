"""Read-only preview telemetry; it never changes tunnel decisions or guards."""
import math


class TunnelPreviewTelemetry:
    def __init__(self):
        self.received_at = None
        self.last_stamp = None
        self.status = 'WAITING'
        self.entry = self.exit = None
        self.infer_sum_ms = 0.0
        self.infer_count = 0

    @staticmethod
    def number(value):
        return float(value) if type(value) in (int, float) and math.isfinite(value) else None

    def update(self, report, now):
        stamp = report.get('source_stamp_ns')
        if type(stamp) is not int:
            return
        status = str(report.get('status', 'WAITING'))
        reset = (self.last_stamp is not None and stamp < self.last_stamp) or (
            status.startswith('LIVE') and not self.status.startswith('LIVE'))
        if reset:
            self.infer_sum_ms = 0.0
            self.infer_count = 0
        duplicate = stamp == self.last_stamp
        self.last_stamp, self.received_at, self.status = stamp, now, status
        scores = report.get('scores', {})
        if isinstance(scores, dict):
            self.entry = self.number(scores.get('entry'))
            self.exit = self.number(scores.get('exit'))
        if not status.startswith('LIVE') or duplicate:
            return
        milliseconds = self.number(report.get('inference_wall_ms'))
        if milliseconds is not None and milliseconds > 0:
            self.infer_sum_ms += milliseconds
            self.infer_count += 1

    def lines(self, state, now):
        fresh = self.received_at is not None and 0 <= now - self.received_at <= .5
        status = self.status.split()[0] if fresh else ('WAITING' if self.received_at is None else 'STALE')
        fps = '--'
        if fresh and self.status.startswith('LIVE'):
            if self.infer_count:
                fps = f'{1000.0 * self.infer_count / self.infer_sum_ms:.1f}'
        entry = f'{self.entry:.2f}' if fresh and self.entry is not None else '--'
        exit_score = f'{self.exit:.2f}' if fresh and self.exit is not None else '--'
        return (f'Tunnel: {state}', f'TCN avg {fps}FPS',
                f'TCN {status} E:{entry} X:{exit_score}')
