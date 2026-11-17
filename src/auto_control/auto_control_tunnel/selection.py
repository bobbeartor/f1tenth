"""Prepare the target source while routing the current source, then switch once."""
import math
from .path_validation import PathRules, metric_path, stamp_ns
from .ir_length_exit import IrLengthExit


class LaneSelector:
    def __init__(self, max_age=.2, validation_mode=False, rules=None, ir_length_exit=None):
        if not math.isfinite(max_age) or max_age <= 0:
            raise ValueError('Invalid lane freshness setting')
        self.validation_mode, self.max_age = validation_mode, max_age
        self.rules = rules or PathRules()
        self.state = 'OUTSIDE'
        self.selected = self.requested = 'RGB'
        self.last_event_ns = 0
        self.last_input_ns = {'RGB': 0, 'IR': 0}
        self.started_ns = None
        self.reason = 'steady'
        self.last_handover = None
        self.ir_length_exit = ir_length_exit or IrLengthExit()
        self.ir_exit_latched_at = None
        self.transition_trigger = None

    @property
    def transitioning(self):
        return self.started_ns is not None

    @property
    def compute(self):
        return 'BOTH' if self.transitioning else self.selected

    @property
    def camera(self):
        # The separate RGB sensor can recover while IR exposure/lights stay
        # usable. Restore stereo settings only after a valid RGB path arrives.
        return 'IR' if self.transitioning else self.selected

    def observe_tcn_state(self, state, stamp, now_ns):
        # Retained TCN inside reports must not undo a length-triggered exit.
        # Require a fresh post-trigger outside acknowledgement before re-entry.
        if self.ir_exit_latched_at is None:
            return True
        if (state == 'outside' and stamp > self.ir_exit_latched_at and
                0 <= (now_ns - stamp) / 1e9 <= self.max_age):
            self.ir_exit_latched_at = None
            return True
        return False

    def event(self, kind, stamp, now_ns):
        if (kind not in ('entry', 'exit') or stamp <= self.last_event_ns or
                (not self.validation_mode and not 0 <= (now_ns - stamp) / 1e9 <= self.max_age)):
            return False
        if kind == 'entry' and self.ir_exit_latched_at is not None:
            return False
        self.last_event_ns = stamp
        target = 'IR' if kind == 'entry' else 'RGB'
        if target == self.requested:
            return False
        if target == 'IR':
            self.ir_length_exit.reset()
        self.transition_trigger = None
        self.requested = target
        if target == self.selected:
            self._finish(now_ns, 'reversed_request')
            return True
        self.started_ns = now_ns
        self.reason = 'waiting_target_path'
        self.state = 'ENTERING' if target == 'IR' else 'EXITING'
        return True

    def _finish(self, now_ns, reason):
        self.last_handover = dict(target=self.requested, reason=reason,
            total_ms=(now_ns - self.started_ns) / 1e6 if self.started_ns is not None else 0.)
        if self.transition_trigger:
            self.last_handover['trigger'] = self.transition_trigger
        self.selected = self.requested
        self.state = 'INSIDE' if self.selected == 'IR' else 'OUTSIDE'
        self.started_ns = None
        self.reason = reason

    def lane(self, source, message, now_ns):
        if source not in self.last_input_ns:
            return None
        stamp = stamp_ns(message)
        fresh = (self.validation_mode and source == 'IR' and not self.transitioning) or (
            0 <= (now_ns - stamp) / 1e9 <= self.max_age)
        if source == 'IR' and self.state == 'INSIDE':
            if self.ir_length_exit.observe(message, now_ns, self.max_age, self.rules):
                if self.event('exit', now_ns, now_ns):
                    self.ir_exit_latched_at = now_ns
                    self.transition_trigger = self.reason = 'ir_length_drop'
        if stamp <= self.last_input_ns[source] or not fresh:
            return None
        self.last_input_ns[source] = stamp
        if self.transitioning and source == self.requested:
            if metric_path(message, self.rules) is None:
                return None  # An unready target must not clear the current path.
            self._finish(now_ns, 'first_valid_target')
        if source != self.selected:
            return None
        # Preserve all geometry and capture/performance timestamps. Source
        # identity lets the controller check ordering within each camera stream.
        message.lane_source = source
        return message
