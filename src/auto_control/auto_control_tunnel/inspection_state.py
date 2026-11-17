"""Read-only startup display state. The C++ controller authorizes movement."""
import time


class InspectionState:
    def __init__(self, expected):
        self.camera = 'WAITING | waiting for camera measurement'
        self.models = dict.fromkeys(expected, 'WAITING | not loaded')
        self.started = time.monotonic()
        self.model_received = {}
        self.model_changed = {}
        self.model_phase = {}
        self.controller = 'WAITING | waiting for controller'
        self.controller_at = None
        self.error = ''
        self.link_fault = ''
        self.link_recovered = False

    def update_model(self, name, text, now=None):
        if name not in self.models:
            return
        now = time.monotonic() if now is None else now
        phase = text.split(' | elapsed=', 1)[0]
        if self.model_phase.get(name) != phase:
            self.model_changed[name] = now
            self.model_phase[name] = phase
        self.model_received[name] = now
        self.models[name] = text

    def model_info(self, name, now):
        status = self.models[name]
        tag, _, detail = status.partition(' | ')
        phase = detail.split(' | elapsed=', 1)[0]
        age = max(0., now - self.model_received.get(name, self.started))
        reference = self.model_received.get(name, now) if tag in ('READY', 'FAILED') else now
        elapsed = max(0., reference - self.started)
        stage_age = max(0., reference - self.model_changed.get(name, self.started))
        for field in detail.split(' | '):
            key, _, value = field.partition('=')
            if key in ('elapsed', 'stage_age'):
                try:
                    seconds = max(0., float(value.rstrip('s'))) + (age if tag == 'LOADING' else 0.)
                except ValueError:
                    continue
                if key == 'elapsed':
                    elapsed = seconds
                else:
                    stage_age = seconds
        if tag == 'READY':
            health = 'Trial inference PASS'
        elif tag == 'FAILED':
            health = 'Load failed - see reason above'
        elif tag == 'WAITING':
            health = 'No loader report yet (component may be queued)'
        elif age > 5.:
            health = 'REPORT STALE - loader state unknown'
        elif 'monitor_seq=' not in detail:
            health = 'No loading heartbeat - progress unknown'
        elif stage_age >= 60.:
            health = 'No stage change for 60s+ - engine progress unknown'
        else:
            health = 'Monitor alive; engine progress not guaranteed'
        return dict(tag=tag, phase=phase, age=age, elapsed=elapsed,
                    stage_age=stage_age, health=health)

    def update_link(self, text):
        tag, _, reason = text.partition(' | ')
        if tag in ('FAULT', 'RECOVERED'):
            self.link_fault = reason
            self.link_recovered = tag == 'RECOVERED'

    def update_controller(self, text, now=None):
        self.controller = text
        self.controller_at = time.monotonic() if now is None else now

    def controller_fresh(self, now):
        return self.controller_at is not None and 0 <= now - self.controller_at <= 1.0

    def checks_ready(self):
        return self.camera.startswith('READY | ') and all(
            value.startswith('READY | ') for value in self.models.values())

    def can_start(self, now=None):
        now = time.monotonic() if now is None else now
        return self.controller_fresh(now) and self.checks_ready() and self.controller.startswith('READY_TO_START | ')

    def completed(self, now=None):
        now = time.monotonic() if now is None else now
        return self.controller_fresh(now) and (
            self.controller.startswith('STARTED | ') or
            (self.controller.startswith('MANUAL | ') and self.checks_ready()))
