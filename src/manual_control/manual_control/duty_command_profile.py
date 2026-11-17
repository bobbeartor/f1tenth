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
        if not 0.0 <= self.pedal_deadzone <= 1.0:
            raise ValueError("pedal_deadzone must be between 0 and 1")


class GearRequestResult(Enum):
    CHANGED = "changed"      # 즉시 전환됨
    PENDING = "pending"      # duty가 0이 되면 전환하도록 예약됨
    CANCELLED = "cancelled"  # 기존 예약이 취소됨


class DutyCommandProfile:
    def __init__(self, config: DutyProfileConfig) -> None:
        self.config = config
        self.gear = Gear.FORWARD
        self.current_duty = 0.0
        self.pending_gear: Gear | None = None

    @property
    def command_duty(self) -> float:
        return self.current_duty

    def update(self, accelerator: float, dt_sec: float) -> float:
        accelerator = self._pedal_value(accelerator)
        dt_sec = max(0.0, float(dt_sec))

        # 예약된 기어는 duty가 0인 시점에만 적용한다. 전진 duty에서 곧바로
        # 후진 duty로 넘어가는 일은 없다.
        self.apply_pending_gear()

        if accelerator > 0.0:
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

        # 감속이 끝나 duty가 0이 된 같은 주기에 예약을 적용한다.
        self.apply_pending_gear()
        return self.command_duty

    def toggle_gear(self) -> bool:
        if self.current_duty != 0.0:
            return False

        self.gear = self._opposite(self.gear)
        self.pending_gear = None
        return True

    def request_gear_toggle(
        self, apply_now_allowed: bool = True
    ) -> GearRequestResult:
        """RB 입력 처리: 가능하면 즉시 전환하고, 아니면 예약/취소한다."""
        if self.pending_gear is not None:
            self.pending_gear = None
            return GearRequestResult.CANCELLED
        if apply_now_allowed and self.toggle_gear():
            return GearRequestResult.CHANGED
        self.pending_gear = self._opposite(self.gear)
        return GearRequestResult.PENDING

    def apply_pending_gear(self) -> bool:
        if self.pending_gear is None or self.current_duty != 0.0:
            return False
        self.gear = self.pending_gear
        self.pending_gear = None
        return True

    def clear_gear_request(self) -> None:
        self.pending_gear = None

    @staticmethod
    def _opposite(gear: Gear) -> Gear:
        return Gear.REVERSE if gear is Gear.FORWARD else Gear.FORWARD

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
