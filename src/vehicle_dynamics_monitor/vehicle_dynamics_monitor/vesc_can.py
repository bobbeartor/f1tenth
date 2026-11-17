from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from typing import Any


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_STATUS_5 = 27
CAN_PACKET_STATUS_6 = 58


@dataclass(frozen=True)
class VescCanUpdate:
    packet_id: int
    controller_id: int
    values: dict[str, int | float]


def decode_vesc_can_frame(
    can_id: int, data: bytes, expected_controller_id: int
) -> VescCanUpdate | None:
    # VESC status traffic uses 29-bit extended identifiers. Reject standard
    # frames that happen to share the same low identifier bits.
    if not can_id & CAN_EFF_FLAG:
        return None
    arbitration_id = can_id & CAN_EFF_MASK
    controller_id = arbitration_id & 0xFF
    packet_id = (arbitration_id >> 8) & 0xFF
    if controller_id != expected_controller_id:
        return None

    if packet_id == CAN_PACKET_STATUS and len(data) >= 8:
        erpm, motor_current_tenths, duty_thousandths = struct.unpack(
            ">ihh", data[:8]
        )
        values: dict[str, int | float] = {
            "measured_erpm": erpm,
            "motor_current_a": motor_current_tenths / 10.0,
            "duty_cycle": duty_thousandths / 1000.0,
        }
    elif packet_id == CAN_PACKET_STATUS_2 and len(data) >= 8:
        amp_hours, amp_hours_charged = struct.unpack(">ii", data[:8])
        values = {
            "amp_hours": amp_hours / 10000.0,
            "amp_hours_charged": amp_hours_charged / 10000.0,
        }
    elif packet_id == CAN_PACKET_STATUS_3 and len(data) >= 8:
        watt_hours, watt_hours_charged = struct.unpack(">ii", data[:8])
        values = {
            "watt_hours": watt_hours / 10000.0,
            "watt_hours_charged": watt_hours_charged / 10000.0,
        }
    elif packet_id == CAN_PACKET_STATUS_4 and len(data) >= 8:
        temp_fet_tenths, temp_motor_tenths, input_current_tenths, pid_pos = (
            struct.unpack(">hhhh", data[:8])
        )
        values = {
            "fet_temperature_c": temp_fet_tenths / 10.0,
            "motor_temperature_c": temp_motor_tenths / 10.0,
            "input_current_a": input_current_tenths / 10.0,
            "pid_position_deg": pid_pos / 50.0,
        }
    elif packet_id == CAN_PACKET_STATUS_5 and len(data) >= 6:
        tachometer, input_voltage_tenths = struct.unpack(">ih", data[:6])
        values = {
            "tachometer": tachometer,
            "electrical_revolutions": tachometer / 6.0,
            "input_voltage_v": input_voltage_tenths / 10.0,
        }
    elif packet_id == CAN_PACKET_STATUS_6 and len(data) >= 8:
        adc_1_thousandths, adc_2_thousandths, adc_3_thousandths, ppm = (
            struct.unpack(">hhhh", data[:8])
        )
        values = {
            "adc_1_v": adc_1_thousandths / 1000.0,
            "adc_2_v": adc_2_thousandths / 1000.0,
            "adc_3_v": adc_3_thousandths / 1000.0,
            "ppm": ppm / 1000.0,
        }
    else:
        return None

    return VescCanUpdate(
        packet_id=packet_id,
        controller_id=controller_id,
        values=values,
    )


class SocketCanReceiver:
    """Small non-blocking, receive-only Linux SocketCAN adapter."""

    _CAN_FRAME = struct.Struct("=IB3x8s")

    def __init__(self, interface: str) -> None:
        self.interface = interface
        self._socket: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    def open(self) -> None:
        if self.is_open:
            return
        if not hasattr(socket, "AF_CAN") or not hasattr(socket, "CAN_RAW"):
            raise RuntimeError("SocketCAN is available only on Linux")
        can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            can_socket.setblocking(False)
            can_socket.bind((self.interface,))
        except Exception:
            can_socket.close()
            raise
        self._socket = can_socket

    def close(self) -> None:
        if self._socket is None:
            return
        self._socket.close()
        self._socket = None

    def drain(self, maximum_frames: int = 256) -> list[tuple[int, bytes]]:
        if self._socket is None:
            return []
        frames: list[tuple[int, bytes]] = []
        for _ in range(maximum_frames):
            try:
                raw_frame = self._socket.recv(self._CAN_FRAME.size)
            except BlockingIOError:
                break
            except OSError:
                self.close()
                raise
            if len(raw_frame) != self._CAN_FRAME.size:
                continue
            can_id, data_length, data = self._CAN_FRAME.unpack(raw_frame)
            frames.append((can_id, data[: min(int(data_length), 8)]))
        return frames
