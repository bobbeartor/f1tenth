from __future__ import annotations

import struct
import unittest

from vesc_bridge.vesc_driver import VescDriver, VescDriverError


class FakeSerial:
    def __init__(self, response: bytes = b"") -> None:
        self.is_open = True
        self.response = bytearray(response)
        self.writes: list[bytes] = []

    def write(self, packet: bytes) -> int:
        self.writes.append(packet)
        return len(packet)

    def read(self, size: int) -> bytes:
        chunk = bytes(self.response[:size])
        del self.response[:size]
        return chunk

    def reset_input_buffer(self) -> None:
        pass

    def close(self) -> None:
        self.is_open = False


def make_driver(fake_serial: FakeSerial) -> VescDriver:
    return VescDriver(
        startup_delay=0.0,
        serial_factory=lambda **_kwargs: fake_serial,
    )


class VescDriverTest(unittest.TestCase):
    def test_set_duty_encodes_scaled_comm_set_duty_payload(self) -> None:
        fake_serial = FakeSerial()
        driver = make_driver(fake_serial)

        driver.set_duty(0.07)

        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([5]) + struct.pack(">i", 7000))],
        )

    def test_set_duty_clamps_to_protocol_range(self) -> None:
        fake_serial = FakeSerial()
        driver = make_driver(fake_serial)

        driver.set_duty(-2.0)

        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([5]) + struct.pack(">i", -100000))],
        )

    def test_set_brake_current_encodes_scaled_brake_payload(self) -> None:
        fake_serial = FakeSerial()
        driver = make_driver(fake_serial)

        driver.set_brake_current(8.0)

        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([7]) + struct.pack(">i", 8000))],
        )

    def test_set_brake_current_clamps_negative_magnitude_to_zero(self) -> None:
        fake_serial = FakeSerial()
        driver = make_driver(fake_serial)

        driver.set_brake_current(-1.0)

        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([7]) + struct.pack(">i", 0))],
        )

    def test_get_firmware_version_requires_a_valid_vesc_response(self) -> None:
        response = VescDriver.make_packet(bytes([0, 6, 5]))
        fake_serial = FakeSerial(response=response)
        driver = make_driver(fake_serial)

        self.assertEqual(driver.get_firmware_version(), (6, 5))
        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([0]))],
        )

    def test_get_firmware_version_rejects_invalid_crc(self) -> None:
        response = bytearray(VescDriver.make_packet(bytes([0, 6, 5])))
        response[-2] ^= 0xFF
        driver = make_driver(FakeSerial(response=bytes(response)))

        with self.assertRaisesRegex(VescDriverError, "CRC"):
            driver.get_firmware_version()

    def test_get_firmware_version_times_out_without_a_response(self) -> None:
        driver = make_driver(FakeSerial())

        with self.assertRaisesRegex(VescDriverError, "Timed out"):
            driver.get_firmware_version()

    def test_get_measured_erpm_requests_and_parses_selected_value(self) -> None:
        response_payload = bytes([50]) + struct.pack(">Ii", 1 << 7, -4321)
        fake_serial = FakeSerial(VescDriver.make_packet(response_payload))
        driver = make_driver(fake_serial)

        self.assertEqual(driver.get_measured_erpm(), -4321)
        self.assertEqual(
            fake_serial.writes,
            [VescDriver.make_packet(bytes([50]) + struct.pack(">I", 1 << 7))],
        )

    def test_get_measured_erpm_handles_fields_before_erpm(self) -> None:
        returned_mask = (1 << 0) | (1 << 7)
        response_payload = (
            bytes([50])
            + struct.pack(">I", returned_mask)
            + struct.pack(">h", 275)
            + struct.pack(">i", 6789)
        )
        driver = make_driver(
            FakeSerial(VescDriver.make_packet(response_payload))
        )

        self.assertEqual(driver.get_measured_erpm(), 6789)

    def test_get_measured_erpm_rejects_response_without_erpm(self) -> None:
        response_payload = bytes([50]) + struct.pack(">Ih", 1 << 0, 275)
        driver = make_driver(
            FakeSerial(VescDriver.make_packet(response_payload))
        )

        with self.assertRaisesRegex(VescDriverError, "does not contain ERPM"):
            driver.get_measured_erpm()


if __name__ == "__main__":
    unittest.main()
