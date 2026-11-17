"""GUI-owned speed requests; controller state confirms the applied ceiling."""
import math
import time


class InspectionSpeed:
    def __init__(self):
        self.bounds = None
        self.applied = self.selected = None
        self.revision = 0
        self.received = self.sent = -math.inf
        self.dragging = self.dirty = self.released = self.pending = False
        self.failed_until = -math.inf

    def observe(self, message):
        now = time.monotonic()
        minimum, ceiling, applied = (message.minimum_speed_mps,
                                    message.ceiling_speed_mps, message.applied_speed_mps)
        if not all(math.isfinite(v) for v in (minimum, ceiling, applied)) or not (
                0 < minimum <= applied <= ceiling):
            return
        reset = (self.bounds != (minimum, ceiling) or message.revision < self.revision
                 or now - self.received > .75)
        self.bounds = minimum, ceiling
        self.applied, self.revision, self.received = applied, message.revision, now
        if reset:
            self.selected = applied
            self.failed_until = -math.inf
            self.dragging = self.dirty = self.released = self.pending = False
        if not self.dragging and not self.dirty:
            if self.pending and abs(applied - self.selected) < 1e-6:
                self.pending = False
            if not self.pending:
                self.selected = applied

    def fresh(self, now):
        return self.bounds is not None and 0 <= now - self.received <= .75

    def select(self, x, left, right):
        if not self.fresh(time.monotonic()):
            return
        minimum, ceiling = self.bounds
        fraction = max(0., min(1., (x-left)/max(1, right-left)))
        raw = minimum + fraction*(ceiling-minimum)
        value = minimum if fraction == 0 else ceiling if fraction == 1 else max(
            minimum, min(ceiling, round(raw*20)/20))
        if self.selected != value:
            self.selected = value
            self.dirty = True
            self.failed_until = -math.inf

    def refresh(self, now):
        if not self.fresh(now):
            self.dragging = self.dirty = self.released = self.pending = False
        elif self.pending and not self.dirty and not self.dragging and now-self.sent > 1.5:
            self.selected = self.applied
            self.pending = False
            self.failed_until = now+2.

    def request(self, now):
        self.refresh(now)
        if not self.fresh(now) or not self.dirty or (not self.released and now-self.sent < .1):
            return None
        self.dirty = self.released = False
        self.pending = True
        self.sent = now
        return self.selected

    def view(self, now):
        self.refresh(now)
        status = ('CONTROLLER OFFLINE' if not self.fresh(now) else
                  'NOT APPLIED' if now < self.failed_until else
                  'APPLYING...' if self.pending or self.dirty else 'LIVE')
        return dict(bounds=self.bounds, selected=self.selected, applied=self.applied, status=status)
