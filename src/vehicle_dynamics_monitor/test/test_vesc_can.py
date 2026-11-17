import struct
import unittest

from vehicle_dynamics_monitor.vesc_can import (
    CAN_EFF_FLAG,
    CAN_PACKET_STATUS,
    CAN_PACKET_STATUS_4,
    CAN_PACKET_STATUS_5,
    CAN_PACKET_STATUS_6,
    decode_vesc_can_frame,
)


class VescCanDecoderTest(unittest.TestCase):
    @staticmethod
    def _can_id(packet_id: int, controller_id: int = 7) -> int:
        return CAN_EFF_FLAG | (packet_id << 8) | controller_id

    def test_status_decodes_erpm_current_and_duty(self) -> None:
        update = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS),
            struct.pack(">ihh", 20620, 123, 65),
            expected_controller_id=7,
        )

        self.assertIsNotNone(update)
        assert update is not None
        self.assertEqual(update.values["measured_erpm"], 20620)
        self.assertAlmostEqual(update.values["motor_current_a"], 12.3)
        self.assertAlmostEqual(update.values["duty_cycle"], 0.065)

    def test_status_4_and_5_decode_environment_values(self) -> None:
        status_4 = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS_4),
            struct.pack(">hhhh", 451, 372, 87, 250),
            expected_controller_id=7,
        )
        status_5 = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS_5),
            struct.pack(">ih", 123456, 164),
            expected_controller_id=7,
        )

        assert status_4 is not None
        assert status_5 is not None
        self.assertAlmostEqual(status_4.values["fet_temperature_c"], 45.1)
        self.assertAlmostEqual(status_4.values["motor_temperature_c"], 37.2)
        self.assertAlmostEqual(status_4.values["input_current_a"], 8.7)
        self.assertAlmostEqual(status_5.values["input_voltage_v"], 16.4)
        self.assertAlmostEqual(
            status_5.values["electrical_revolutions"], 20576.0
        )

    def test_status_6_decodes_adc_and_ppm(self) -> None:
        status_6 = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS_6),
            struct.pack(">hhhh", 3300, 1200, 5000, -250),
            expected_controller_id=7,
        )

        assert status_6 is not None
        self.assertAlmostEqual(status_6.values["adc_1_v"], 3.3)
        self.assertAlmostEqual(status_6.values["adc_2_v"], 1.2)
        self.assertAlmostEqual(status_6.values["adc_3_v"], 5.0)
        self.assertAlmostEqual(status_6.values["ppm"], -0.25)

    def test_wrong_controller_and_short_frame_are_ignored(self) -> None:
        wrong_controller = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS, controller_id=8),
            bytes(8),
            expected_controller_id=7,
        )
        short_frame = decode_vesc_can_frame(
            self._can_id(CAN_PACKET_STATUS),
            bytes(7),
            expected_controller_id=7,
        )
        standard_frame = decode_vesc_can_frame(
            (CAN_PACKET_STATUS << 8) | 7,
            bytes(8),
            expected_controller_id=7,
        )

        self.assertIsNone(wrong_controller)
        self.assertIsNone(short_frame)
        self.assertIsNone(standard_frame)


if __name__ == "__main__":
    unittest.main()
