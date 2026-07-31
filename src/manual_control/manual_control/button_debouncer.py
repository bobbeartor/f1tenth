from __future__ import annotations


class RisingEdgeDebouncer:
    """Accept the first press immediately and reject nearby bounce edges."""

    def __init__(self, debounce_sec: float) -> None:
        debounce_sec = float(debounce_sec)
        if debounce_sec < 0.0:
            raise ValueError("debounce_sec must be non-negative")

        self.debounce_sec = debounce_sec
        self._pressed = False
        self._last_accepted_time_sec: float | None = None

    def update(self, pressed: bool, now_sec: float) -> bool:
        pressed = bool(pressed)
        now_sec = float(now_sec)
        rising_edge = pressed and not self._pressed
        self._pressed = pressed

        if not rising_edge:
            return False

        if self._last_accepted_time_sec is not None:
            elapsed_sec = now_sec - self._last_accepted_time_sec
            if 0.0 <= elapsed_sec < self.debounce_sec:
                return False

        self._last_accepted_time_sec = now_sec
        return True
