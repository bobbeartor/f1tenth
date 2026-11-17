from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class AutomaticBrakeProfileConfig:
    entry_speed_error_mps: float
    exit_speed_error_mps: float
    minimum_vehicle_speed_mps: float
    minimum_brake_current_amps: float
    maximum_brake_current_amps: float
    current_gain_amps_per_mps: float
    rise_amps_per_sec: float
    fall_amps_per_sec: float

    def __post_init__(self) -> None:
        values = (
            self.entry_speed_error_mps,
            self.exit_speed_error_mps,
            self.minimum_vehicle_speed_mps,
            self.minimum_brake_current_amps,
            self.maximum_brake_current_amps,
            self.current_gain_amps_per_mps,
            self.rise_amps_per_sec,
            self.fall_amps_per_sec,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("automatic brake parameters must be finite")
        if self.entry_speed_error_mps <= 0.0:
            raise ValueError("entry_speed_error_mps must be positive")
        if not 0.0 <= self.exit_speed_error_mps < self.entry_speed_error_mps:
            raise ValueError(
                "exit_speed_error_mps must be in [0, entry_speed_error_mps)"
            )
        if self.minimum_vehicle_speed_mps < 0.0:
            raise ValueError("minimum_vehicle_speed_mps must not be negative")
        if not (
            0.0 <= self.minimum_brake_current_amps
            <= self.maximum_brake_current_amps
        ):
            raise ValueError(
                "brake current limits must satisfy 0 <= minimum <= maximum"
            )
        if self.maximum_brake_current_amps <= 0.0:
            raise ValueError("maximum_brake_current_amps must be positive")
        if self.current_gain_amps_per_mps <= 0.0:
            raise ValueError("current_gain_amps_per_mps must be positive")
        if self.rise_amps_per_sec <= 0.0 or self.fall_amps_per_sec <= 0.0:
            raise ValueError("brake current ramp rates must be positive")


class AutomaticBrakeProfile:
    """Generate a ramp-limited brake-current command from speed overshoot."""

    _SPEED_COMPARISON_EPSILON_MPS = 1.0e-9

    def __init__(self, config: AutomaticBrakeProfileConfig) -> None:
        self.config = config
        self.current_amps = 0.0
        self.braking_requested = False

    @property
    def command_current_amps(self) -> float:
        return self.current_amps

    def update(
        self,
        *,
        target_speed_mps: float,
        current_speed_mps: float,
        dt_sec: float,
    ) -> float:
        target_speed_mps = self._finite_nonnegative(target_speed_mps)
        current_speed_mps = self._finite_nonnegative(current_speed_mps)
        dt_sec = self._finite_nonnegative(dt_sec)
        speed_error_mps = current_speed_mps - target_speed_mps

        if current_speed_mps <= self.config.minimum_vehicle_speed_mps:
            self.braking_requested = False
        elif self.braking_requested:
            self.braking_requested = (
                speed_error_mps
                > self.config.exit_speed_error_mps
                + self._SPEED_COMPARISON_EPSILON_MPS
            )
        else:
            self.braking_requested = (
                speed_error_mps
                >= self.config.entry_speed_error_mps
                - self._SPEED_COMPARISON_EPSILON_MPS
            )

        target_current_amps = 0.0
        if self.braking_requested:
            target_current_amps = self._clamp(
                self.config.current_gain_amps_per_mps
                * max(0.0, speed_error_mps),
                self.config.minimum_brake_current_amps,
                self.config.maximum_brake_current_amps,
            )

        rate = (
            self.config.rise_amps_per_sec
            if target_current_amps > self.current_amps
            else self.config.fall_amps_per_sec
        )
        self.current_amps = self._move_toward(
            self.current_amps,
            target_current_amps,
            rate * dt_sec,
        )
        return self.current_amps

    def reset(self) -> None:
        self.current_amps = 0.0
        self.braking_requested = False

    @staticmethod
    def _finite_nonnegative(value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            return 0.0
        return max(0.0, value)

    @staticmethod
    def _move_toward(value: float, target: float, amount: float) -> float:
        if value < target:
            return min(value + amount, target)
        return max(value - amount, target)

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
