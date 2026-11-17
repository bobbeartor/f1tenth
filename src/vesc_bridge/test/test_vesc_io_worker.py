from __future__ import annotations

import threading
import time
import unittest

from vesc_bridge.vesc_io_worker import VescIoResult, VescIoWorker


class RecordingDriver:
    def __init__(self) -> None:
        self.operations: list[tuple[str, float | int | None]] = []
        self.telemetry_value = 1000
        self.closed = False

    def set_duty(self, value: float) -> None:
        self.operations.append(("duty", value))

    def set_brake_current(self, value: float) -> None:
        self.operations.append(("brake", value))

    def set_erpm(self, value: int) -> None:
        self.operations.append(("erpm", value))

    def set_servo_position(self, value: float) -> None:
        self.operations.append(("servo", value))

    def get_measured_erpm(self) -> int:
        self.operations.append(("telemetry", None))
        self.telemetry_value += 1
        return self.telemetry_value

    def close(self) -> None:
        self.closed = True


class BlockingTelemetryDriver(RecordingDriver):
    def __init__(self) -> None:
        super().__init__()
        self.telemetry_entered = threading.Event()
        self.release_telemetry = threading.Event()

    def get_measured_erpm(self) -> int:
        self.operations.append(("telemetry", None))
        self.telemetry_entered.set()
        self.release_telemetry.wait(timeout=0.5)
        return 1234


def wait_for_results(
    worker: VescIoWorker,
    minimum_count: int,
    timeout_sec: float = 0.5,
) -> list[VescIoResult]:
    deadline = time.monotonic() + timeout_sec
    results: list[VescIoResult] = []
    while time.monotonic() < deadline:
        results.extend(worker.drain_results())
        if len(results) >= minimum_count:
            return results
        time.sleep(0.001)
    return results


class VescIoWorkerTest(unittest.TestCase):
    def test_rejects_negative_telemetry_rate(self) -> None:
        with self.assertRaisesRegex(ValueError, "must not be negative"):
            VescIoWorker(RecordingDriver(), -1.0)

    def test_latest_drive_command_replaces_superseded_value(self) -> None:
        driver = RecordingDriver()
        worker = VescIoWorker(driver, telemetry_rate_hz=0.0)
        worker.submit_duty(0.08)
        worker.submit_duty(0.0)
        worker.start()

        results = wait_for_results(worker, 1)
        self.assertEqual([(item.operation, item.value) for item in results], [
            ("duty", 0.0),
        ])
        self.assertTrue(worker.stop(send_duty_zero=False))
        self.assertEqual(driver.operations, [("duty", 0.0)])
        self.assertTrue(driver.closed)

    def test_brake_current_replaces_pending_propulsion_command(self) -> None:
        driver = RecordingDriver()
        worker = VescIoWorker(driver, telemetry_rate_hz=0.0)
        worker.submit_duty(0.08)
        worker.submit_brake_current(4.0)
        worker.start()

        results = wait_for_results(worker, 1)
        self.assertEqual(
            [(item.operation, item.value) for item in results],
            [("brake", 4.0)],
        )
        self.assertTrue(worker.stop(send_duty_zero=False))
        self.assertEqual(driver.operations, [("brake", 4.0)])

    def test_drive_and_steering_commands_precede_due_telemetry(self) -> None:
        driver = RecordingDriver()
        worker = VescIoWorker(driver, telemetry_rate_hz=80.0)
        worker.submit_servo(0.25)
        worker.submit_duty(0.05)
        worker.start()

        results = wait_for_results(worker, 3)
        self.assertGreaterEqual(len(results), 3)
        self.assertEqual(
            [result.operation for result in results[:3]],
            ["duty", "servo", "telemetry"],
        )
        self.assertTrue(worker.stop(send_duty_zero=False))

    def test_telemetry_runs_repeatedly_and_records_transaction_timing(self) -> None:
        driver = RecordingDriver()
        worker = VescIoWorker(driver, telemetry_rate_hz=80.0)
        worker.start()

        results = wait_for_results(worker, 4)
        self.assertGreaterEqual(len(results), 4)
        telemetry = [result for result in results if result.operation == "telemetry"]
        self.assertGreaterEqual(len(telemetry), 4)
        self.assertTrue(all(result.success for result in telemetry))
        self.assertTrue(all(result.round_trip_sec >= 0.0 for result in telemetry))
        self.assertTrue(all(
            result.started_at_sec <= result.midpoint_sec <= result.finished_at_sec
            for result in telemetry
        ))
        self.assertTrue(worker.stop(send_duty_zero=False))

    def test_blocked_telemetry_does_not_block_latest_control_submission(self) -> None:
        driver = BlockingTelemetryDriver()
        worker = VescIoWorker(driver, telemetry_rate_hz=80.0)
        worker.start()
        self.assertTrue(driver.telemetry_entered.wait(timeout=0.2))

        started_at = time.monotonic()
        worker.submit_duty(0.08)
        worker.submit_duty(0.0)
        submission_seconds = time.monotonic() - started_at

        driver.release_telemetry.set()
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            if any(operation == "duty" for operation, _ in driver.operations):
                break
            time.sleep(0.001)
        self.assertTrue(worker.stop(send_duty_zero=False))

        duty_values = [
            value for operation, value in driver.operations if operation == "duty"
        ]
        self.assertLess(submission_seconds, 0.1)
        self.assertEqual(duty_values, [0.0])


if __name__ == "__main__":
    unittest.main()
