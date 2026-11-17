from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass
from typing import Literal

from vesc_bridge.vesc_driver import VescDriver, VescResponseError


Operation = Literal["duty", "brake", "erpm", "servo", "telemetry"]


@dataclass(frozen=True)
class VescIoResult:
    operation: Operation
    success: bool
    value: float | int | None
    started_at_sec: float
    finished_at_sec: float
    error: str | None = None
    recoverable_response_error: bool = False

    @property
    def midpoint_sec(self) -> float:
        return 0.5 * (self.started_at_sec + self.finished_at_sec)

    @property
    def round_trip_sec(self) -> float:
        return max(0.0, self.finished_at_sec - self.started_at_sec)


class VescIoWorker:
    """Owns the serial driver and schedules latest-only commands plus telemetry.

    ROS callbacks only replace pending desired states, so a slow UART response
    cannot block the executor or cause old actuator commands to be replayed.
    Drive and steering writes are handled before a due telemetry transaction.
    """

    def __init__(self, driver: VescDriver, telemetry_rate_hz: float) -> None:
        if telemetry_rate_hz < 0.0:
            raise ValueError("telemetry_rate_hz must not be negative")

        self._driver = driver
        self._telemetry_period_sec = (
            1.0 / telemetry_rate_hz if telemetry_rate_hz > 0.0 else None
        )
        self._condition = threading.Condition()
        self._pending_drive: tuple[Operation, float | int] | None = None
        self._pending_servo: float | None = None
        self._results: queue.SimpleQueue[VescIoResult] = queue.SimpleQueue()
        self._stop_requested = False
        self._started = False
        self._thread: threading.Thread | None = None
        self._next_telemetry_sec = 0.0

    def start(self) -> None:
        with self._condition:
            if self._started:
                return
            self._started = True
            self._next_telemetry_sec = time.monotonic()
            self._thread = threading.Thread(
                target=self._run,
                name="vesc-uart-worker",
                daemon=True,
            )
            self._thread.start()

    def submit_duty(self, duty: float) -> None:
        self._submit_drive("duty", float(duty))

    def submit_brake_current(self, brake_current_amps: float) -> None:
        self._submit_drive("brake", float(brake_current_amps))

    def submit_erpm(self, erpm: int) -> None:
        self._submit_drive("erpm", int(erpm))

    def submit_servo(self, position: float) -> None:
        with self._condition:
            if self._stop_requested:
                return
            self._pending_servo = float(position)
            self._condition.notify()

    def drain_results(self) -> list[VescIoResult]:
        results: list[VescIoResult] = []
        while True:
            try:
                results.append(self._results.get_nowait())
            except queue.Empty:
                return results

    def stop(self, send_duty_zero: bool = True, timeout_sec: float = 1.0) -> bool:
        with self._condition:
            if not self._started:
                return True
            self._pending_servo = None
            self._pending_drive = ("duty", 0.0) if send_duty_zero else None
            self._stop_requested = True
            self._condition.notify_all()
            thread = self._thread

        if thread is not None:
            thread.join(timeout=max(0.0, timeout_sec))
            return not thread.is_alive()
        return True

    def _submit_drive(self, operation: Operation, value: float | int) -> None:
        with self._condition:
            if self._stop_requested:
                return
            # Duty, brake current and ERPM are mutually exclusive motor modes.
            # Retain only the newest state across all three topics.
            self._pending_drive = (operation, value)
            self._condition.notify()

    def _run(self) -> None:
        while True:
            work = self._wait_for_work()
            if work is None:
                break
            operation, value = work
            started_at_sec = time.monotonic()
            try:
                result_value = self._execute(operation, value)
                finished_at_sec = time.monotonic()
                self._results.put(
                    VescIoResult(
                        operation=operation,
                        success=True,
                        value=result_value,
                        started_at_sec=started_at_sec,
                        finished_at_sec=finished_at_sec,
                    )
                )
            except Exception as exc:
                finished_at_sec = time.monotonic()
                # Only protocol/response failures can retain an open transport.
                # Serial read/open/write exceptions are actual I/O failures,
                # including a failed request write during telemetry.
                recoverable = operation == "telemetry" and isinstance(exc, VescResponseError)
                if not recoverable:
                    try:
                        self._driver.close()
                    except Exception:
                        pass
                self._results.put(
                    VescIoResult(
                        operation=operation,
                        success=False,
                        value=value,
                        started_at_sec=started_at_sec,
                        finished_at_sec=finished_at_sec,
                        error=str(exc),
                        recoverable_response_error=recoverable,
                    )
                )

        try:
            self._driver.close()
        except Exception:
            pass

    def _wait_for_work(self) -> tuple[Operation, float | int | None] | None:
        with self._condition:
            while True:
                if self._pending_drive is not None:
                    work = self._pending_drive
                    self._pending_drive = None
                    return work

                if self._stop_requested:
                    return None

                if self._pending_servo is not None:
                    position = self._pending_servo
                    self._pending_servo = None
                    return "servo", position

                now_sec = time.monotonic()
                if (
                    self._telemetry_period_sec is not None
                    and now_sec >= self._next_telemetry_sec
                ):
                    # Keep an absolute schedule and skip missed slots rather
                    # than issuing a burst of stale telemetry requests.
                    while self._next_telemetry_sec <= now_sec:
                        self._next_telemetry_sec += self._telemetry_period_sec
                    return "telemetry", None

                timeout_sec = None
                if self._telemetry_period_sec is not None:
                    timeout_sec = max(0.0, self._next_telemetry_sec - now_sec)
                self._condition.wait(timeout=timeout_sec)

    def _execute(
        self,
        operation: Operation,
        value: float | int | None,
    ) -> float | int | None:
        if operation == "duty":
            self._driver.set_duty(float(value))
            return value
        if operation == "brake":
            self._driver.set_brake_current(float(value))
            return value
        if operation == "erpm":
            self._driver.set_erpm(int(value))
            return value
        if operation == "servo":
            self._driver.set_servo_position(float(value))
            return value
        return self._driver.get_measured_erpm()
