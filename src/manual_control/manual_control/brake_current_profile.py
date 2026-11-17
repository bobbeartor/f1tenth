from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BrakeCurrentProfileConfig:
    maximum_brake_current_amps: float
    rise_amps_per_sec: float
    fall_amps_per_sec: float
    pedal_deadzone: float

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.maximum_brake_current_amps) or
            self.maximum_brake_current_amps <= 0.0
        ):
            raise ValueError("maximum_brake_current_amps must be positive")
        if (
            not math.isfinite(self.rise_amps_per_sec) or
            self.rise_amps_per_sec <= 0.0
        ):
            raise ValueError("rise_amps_per_sec must be positive")
        if (
            not math.isfinite(self.fall_amps_per_sec) or
            self.fall_amps_per_sec <= 0.0
        ):
            raise ValueError("fall_amps_per_sec must be positive")
        if (
            not math.isfinite(self.pedal_deadzone) or
            not 0.0 <= self.pedal_deadzone <= 1.0
        ):
            raise ValueError("pedal_deadzone must be between 0 and 1")


class BrakeCurrentProfile:
    """Map LT travel to a ramp-limited VESC brake-current magnitude."""

    def __init__(self, config: BrakeCurrentProfileConfig) -> None:
        self.config = config
        self.current_amps = 0.0

    @property
    def command_current_amps(self) -> float:
        return self.current_amps

    def update(self, brake: float, dt_sec: float) -> float:
        brake = self._pedal_value(brake)
        dt_sec = max(0.0, float(dt_sec))
        target_amps = brake * self.config.maximum_brake_current_amps
        rate = (
            self.config.rise_amps_per_sec
            if target_amps > self.current_amps
            else self.config.fall_amps_per_sec
        )
        self.current_amps = self._move_toward(
            self.current_amps,
            target_amps,
            rate * dt_sec,
        )
        return self.current_amps

    def reset(self) -> None:
        self.current_amps = 0.0

    def _pedal_value(self, value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        value = self._clamp(value, 0.0, 1.0)
        if value < self.config.pedal_deadzone:
            return 0.0
        return value

    @staticmethod
    def _move_toward(value: float, target: float, amount: float) -> float:
        if value < target:
            return min(value + amount, target)
        return max(value - amount, target)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
