"""Optional observed-IR-length exit trigger, independent of driving-path validity."""
from collections import deque
import math
from .path_validation import stamp_ns


class IrLengthExit:
    def __init__(self, enabled=False, ratio=.15, confirm_frames=3, minimum_reference_m=.5):
        if (type(enabled) is not bool or not math.isfinite(ratio) or not 0 < ratio < 1 or
                type(confirm_frames) is not int or not 1 <= confirm_frames <= 300 or
                not math.isfinite(minimum_reference_m) or minimum_reference_m <= 0):
            raise ValueError('Invalid IR length exit settings')
        self.enabled, self.threshold = enabled, ratio
        self.confirm_frames, self.minimum_reference = confirm_frames, minimum_reference_m
        self.reset()

    def reset(self):
        self.window = deque(maxlen=self.confirm_frames)
        self.reference = 0.
        self.current = self.ratio = None
        self.below_count = 0
        self.last_stamp = None

    def break_sequence(self):
        self.window.clear()
        self.below_count = 0
        self.current = self.ratio = None

    def observe(self, message, now_ns, max_age, rules):
        if not self.enabled:
            return False
        try:
            stamp = stamp_ns(message)
            if self.last_stamp is not None and stamp <= self.last_stamp:
                return False
            if (not 0 <= (now_ns - stamp) / 1e9 <= max_age or
                    not message.observed_lane_length_valid or
                    message.header.frame_id != rules.frame_id or
                    message.source_width <= 0 or message.source_height <= 0 or
                    not math.isclose(message.centerline_bev_width_m / message.source_width,
                                     rules.mpp, rel_tol=1e-4) or
                    not math.isclose(message.centerline_bev_height_m / message.source_height,
                                     rules.mpp, rel_tol=1e-4) or
                    not math.isclose(message.centerline_bev_height_m, rules.x_max, rel_tol=1e-4) or
                    not math.isclose(message.centerline_bev_width_m, 2 * rules.y_max, rel_tol=1e-4)):
                self.break_sequence()
                return False
            length = float(message.observed_lane_length_m)
            if not math.isfinite(length) or length < 0:
                self.break_sequence()
                return False
            if self.last_stamp is not None and (stamp - self.last_stamp) / 1e9 > max_age:
                self.break_sequence()
            self.last_stamp = stamp
            self.current = length
            self.window.append(length)
            # A single long false detection cannot increase the reference.
            if len(self.window) == self.confirm_frames:
                stable = min(self.window)
                if stable >= self.minimum_reference:
                    self.reference = max(self.reference, stable)
            self.ratio = length / self.reference if self.reference else None
            low = self.ratio is not None and self.ratio <= self.threshold + 1e-8
            self.below_count = self.below_count + 1 if low else 0
            return self.below_count >= self.confirm_frames
        except (AttributeError, TypeError, ValueError, ZeroDivisionError, OverflowError):
            self.break_sequence()
            return False
