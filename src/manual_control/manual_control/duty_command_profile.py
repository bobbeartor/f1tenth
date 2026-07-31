from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Gear(Enum):
    FORWARD = 1
    REVERSE = -1


@dataclass(frozen=True)
class DutyProfileConfig:
    forward_max_duty: float
    reverse_max_duty: float
    start_duty: float
    reverse_start_duty: float
    acceleration_duty_per_sec: float
    coast_deceleration_duty_per_sec: float
    brake_duty_per_sec: float
    pedal_deadzone: float
    immediate_stop_on_accelerator_release: bool

    def __post_init__(self) -> None:
        if not 0.0 < self.forward_max_duty <= 1.0:
            raise ValueError("forward_max_duty must be in (0, 1]")
        if not 0.0 < self.reverse_max_duty <= 1.0:
            raise ValueError("reverse_max_duty must be in (0, 1]")
        if self.start_duty <= 0.0:
            raise ValueError("start_duty must be positive")
        if self.start_duty > self.forward_max_duty:
            raise ValueError("start_duty must not exceed forward_max_duty")
        if self.reverse_start_duty <= 0.0:
            raise ValueError("reverse_start_duty must be positive")
        if self.reverse_start_duty > self.reverse_max_duty:
            raise ValueError(
                "reverse_start_duty must not exceed reverse_max_duty"
            )
        if self.acceleration_duty_per_sec <= 0.0:
            raise ValueError("acceleration_duty_per_sec must be positive")
        if self.coast_deceleration_duty_per_sec <= 0.0:
            raise ValueError(
                "coast_deceleration_duty_per_sec must be positive"
            )
        if self.brake_duty_per_sec <= 0.0:
            raise ValueError("brake_duty_per_sec must be positive")
        if not 0.0 <= self.pedal_deadzone <= 1.0:
            raise ValueError("pedal_deadzone must be between 0 and 1")


class DutyCommandProfile:
    def __init__(self, config: DutyProfileConfig) -> None:
        self.config = config
        self.gear = Gear.FORWARD
        self.current_duty = 0.0

    @property
    def command_duty(self) -> float:
        return self.current_duty

    def update(self, accelerator: float, brake: float, dt_sec: float) -> float:
        accelerator = self._pedal_value(accelerator)
        brake = self._pedal_value(brake)
        dt_sec = max(0.0, float(dt_sec))

        if brake > 0.0:
            self.current_duty = self._move_toward(
                self.current_duty,
                0.0,
                brake * self.config.brake_duty_per_sec * dt_sec,
            )
        elif accelerator > 0.0:
            target_duty = self.gear.value * self._gear_limit()
            start_duty = self._gear_start_duty()
            if abs(self.current_duty) < start_duty:
                self.current_duty = self.gear.value * start_duty
            else:
                self.current_duty = self._move_toward(
                    self.current_duty,
                    target_duty,
                    accelerator * self.config.acceleration_duty_per_sec * dt_sec,
                )
        elif self.config.immediate_stop_on_accelerator_release:
            # Pedal messages describe the current operator state. Do not keep
            # replaying a previously accumulated duty after the pedal release.
            self.current_duty = 0.0
        else:
            self.current_duty = self._move_toward(
                self.current_duty,
                0.0,
                self.config.coast_deceleration_duty_per_sec * dt_sec,
            )

        return self.command_duty

    def toggle_gear(self) -> bool:
        if self.current_duty != 0.0:
            return False

        self.gear = (
            Gear.REVERSE if self.gear is Gear.FORWARD else Gear.FORWARD
        )
        return True

    def reset_speed(self) -> None:
        self.current_duty = 0.0

    def _gear_limit(self) -> float:
        if self.gear is Gear.FORWARD:
            return self.config.forward_max_duty
        return self.config.reverse_max_duty

    def _gear_start_duty(self) -> float:
        if self.gear is Gear.FORWARD:
            return self.config.start_duty
        return self.config.reverse_start_duty

    def _pedal_value(self, value: float) -> float:
        value = self._clamp(float(value), 0.0, 1.0)
        if value < self.config.pedal_deadzone:
            return 0.0
        return value

    @staticmethod
    def _move_toward(value: float, target: float, amount: float) -> float:
        if value < target:
            return min(target, value + amount)
        if value > target:
            return max(target, value - amount)
        return target

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))
