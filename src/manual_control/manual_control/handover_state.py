"""One-way AUTO -> STOPPING -> MANUAL handover, independent of ROS."""

from __future__ import annotations

import math


class HandoverState:
    def __init__(
        self, stopped_erpm: float = 100.0, stop_hold_sec: float = 0.5,
        telemetry_timeout_sec: float = 0.3,
    ) -> None:
        for value in (stopped_erpm, stop_hold_sec, telemetry_timeout_sec):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("Handover thresholds must be finite and positive")
        self.stopped_erpm = stopped_erpm
        self.stop_hold_sec = stop_hold_sec
        self.telemetry_timeout_sec = telemetry_timeout_sec
        self.mode = "AUTO"
        self._x_pressed = False
        self._last_erpm_time: float | None = None
        self._stopped_since: float | None = None

    def on_x(self, pressed: bool) -> bool:
        rising = pressed and not self._x_pressed
        self._x_pressed = pressed
        if self.mode != "AUTO" or not rising:
            return False
        self.mode = "STOPPING"
        # Measurements received before X cannot establish that braking stopped us.
        self.invalidate_telemetry()
        return True

    def invalidate_telemetry(self) -> None:
        self._last_erpm_time = None
        self._stopped_since = None

    def on_erpm(self, erpm: float, now: float) -> None:
        if self.mode != "STOPPING":
            return
        if not math.isfinite(erpm):
            self.invalidate_telemetry()
            return
        if self._last_erpm_time is None or not (
            0.0 <= now - self._last_erpm_time < self.telemetry_timeout_sec
        ):
            self._stopped_since = None
        self._last_erpm_time = now
        if abs(erpm) > self.stopped_erpm:
            self._stopped_since = None
        elif self._stopped_since is None:
            self._stopped_since = now

    def update(self, now: float, neutral: bool) -> bool:
        if self.mode != "STOPPING":
            return False
        if self._last_erpm_time is None or not (
            0.0 <= now - self._last_erpm_time < self.telemetry_timeout_sec
        ):
            self.invalidate_telemetry()
            return False
        if (self._stopped_since is not None
                and now - self._stopped_since >= self.stop_hold_sec
                and neutral and not self._x_pressed):
            self.mode = "MANUAL"
            return True
        return False
