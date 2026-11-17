#!/usr/bin/env python3

from __future__ import annotations

import struct
from dataclasses import dataclass


START_BYTE = 2
END_BYTE = 3
MAX_SHORT_PAYLOAD_SIZE = 255


@dataclass(frozen=True)
class VescCommandIds:
    set_duty: int = 5
    set_current: int = 6
    set_current_brake: int = 7

    # 현재 활성화된 명령.
    set_erpm: int = 8
    set_servo_pos: int = 12


@dataclass(frozen=True)
class VescScales:
    duty: int = 100000
    current: int = 1000
    brake_current: int = 1000

    # 현재 활성화된 명령 스케일.
    servo: int = 1000


class VescPacketError(ValueError):
    pass


def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_packet(payload: bytes) -> bytes:
    if len(payload) > MAX_SHORT_PAYLOAD_SIZE:
        raise VescPacketError(
            f"Payload too large for short VESC packet: {len(payload)} bytes"
        )

    crc = crc16_xmodem(payload)
    return (
        bytes([START_BYTE, len(payload)])
        + payload
        + struct.pack(">H", crc)
        + bytes([END_BYTE])
    )


def make_duty_packet(
    duty: float,
    command_ids: VescCommandIds | None = None,
    scales: VescScales | None = None,
) -> bytes:
    command_ids = command_ids or VescCommandIds()
    scales = scales or VescScales()
    duty = clamp(duty, -1.0, 1.0)
    value = int(duty * scales.duty)
    payload = bytes([command_ids.set_duty]) + struct.pack(">i", value)
    return make_packet(payload)


def make_current_packet(
    current_amps: float,
    command_ids: VescCommandIds | None = None,
    scales: VescScales | None = None,
) -> bytes:
    # 현재 current 패킷 생성은 비활성화한다. 안전 정책이 정해진 뒤 다시 켠다.
    # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
    # command_ids = command_ids or VescCommandIds()
    # scales = scales or VescScales()
    # value = int(current_amps * scales.current)
    # payload = bytes([command_ids.set_current]) + struct.pack(">i", value)
    # return make_packet(payload)
    raise VescPacketError(
        "Current 패킷 생성은 비활성화되어 있습니다. ERPM 명령을 사용하세요."
    )


def make_brake_current_packet(
    brake_current_amps: float,
    command_ids: VescCommandIds | None = None,
    scales: VescScales | None = None,
) -> bytes:
    command_ids = command_ids or VescCommandIds()
    scales = scales or VescScales()
    brake_current_amps = max(0.0, float(brake_current_amps))
    value = int(brake_current_amps * scales.brake_current)
    payload = bytes([command_ids.set_current_brake]) + struct.pack(">i", value)
    return make_packet(payload)


def make_erpm_packet(
    erpm: int | float,
    command_ids: VescCommandIds | None = None,
) -> bytes:
    # VESC set_rpm expects ERPM, not wheel RPM. ERPM is electrical RPM.
    command_ids = command_ids or VescCommandIds()
    payload = bytes([command_ids.set_erpm]) + struct.pack(">i", int(erpm))
    return make_packet(payload)


def make_servo_packet(
    position: float,
    command_ids: VescCommandIds | None = None,
    scales: VescScales | None = None,
) -> bytes:
    command_ids = command_ids or VescCommandIds()
    scales = scales or VescScales()
    position = clamp(position, 0.0, 1.0)
    value = int(position * scales.servo)
    payload = bytes([command_ids.set_servo_pos]) + struct.pack(">h", value)
    return make_packet(payload)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))
