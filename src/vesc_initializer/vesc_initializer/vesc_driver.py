#!/usr/bin/env python3

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class VescCommandIds:
    set_duty: int = 5
    # 현재는 비활성화. 안전 정책이 정해진 뒤 사용한다.
    set_current: int = 6
    set_current_brake: int = 7

    # ERPM은 텔레메트리와 별도 시험 경로에서 사용할 수 있다.
    set_erpm: int = 8
    set_servo_pos: int = 12
    get_firmware_version: int = 0
    get_values_selective: int = 50


@dataclass(frozen=True)
class VescScales:
    duty: int = 100000
    # 현재는 비활성화. 나중에 다시 켤 때 설정 구조를 유지하려고 남겨둔다.
    current: int = 1000
    brake_current: int = 1000

    # 현재 활성화된 명령 스케일.
    servo: int = 1000


class VescDriverError(RuntimeError):
    pass


class VescDriver:
    """VESC 시리얼 포트의 패킷 송수신을 담당하는 하위 드라이버.

    이 클래스는 ROS2 의존성을 갖지 않는다. ROS 노드는 토픽을 구독한 뒤
    이 메서드들을 호출하고, 이 파일은 패킷 인코딩과 시리얼 쓰기만 맡는다.
    """

    START_BYTE = 2
    LONG_START_BYTE = 3
    END_BYTE = 3
    MAX_SHORT_PAYLOAD_SIZE = 255
    MAX_READ_PAYLOAD_SIZE = 4096
    ERPM_VALUE_MASK = 1 << 7
    VALUE_FIELD_SIZES_BEFORE_ERPM = (2, 2, 4, 4, 4, 4, 2)

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.1,
        write_timeout: float = 2.0,
        startup_delay: float = 0.2,
        command_ids: VescCommandIds | None = None,
        scales: VescScales | None = None,
        serial_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.startup_delay = startup_delay
        self.command_ids = command_ids or VescCommandIds()
        self.scales = scales or VescScales()
        self._serial_factory = serial_factory
        self._serial: Any | None = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def open(self) -> None:
        if self.is_open:
            return

        factory = self._serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise VescDriverError(
                    "pyserial is required to open a VESC serial port."
                ) from exc

            factory = serial.Serial

        try:
            self._serial = factory(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
        except Exception as exc:
            raise VescDriverError(f"Failed to open VESC port {self.port}: {exc}") from exc

        if self.startup_delay > 0.0:
            time.sleep(self.startup_delay)

    def close(self) -> None:
        if self._serial is None:
            return

        try:
            self._serial.close()
        finally:
            self._serial = None

    def __enter__(self) -> "VescDriver":
        self.open()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def set_duty(self, duty: float) -> None:
        duty = self._clamp(duty, -1.0, 1.0)
        value = int(duty * self.scales.duty)
        payload = bytes([self.command_ids.set_duty]) + struct.pack(">i", value)
        self.write_payload(payload)

    def set_current(self, current_amps: float) -> None:
        # 현재 current 명령은 비활성화한다. 안전 정책이 정해진 뒤 다시 켠다.
        # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
        # value = int(current_amps * self.scales.current)
        # payload = bytes([self.command_ids.set_current]) + struct.pack(">i", value)
        # self.write_payload(payload)
        raise VescDriverError("Current 명령은 비활성화되어 있습니다. ERPM 명령을 사용하세요.")

    def set_brake_current(self, brake_current_amps: float) -> None:
        # 현재 brake current 명령은 비활성화한다.
        # 정지는 우선 ERPM 0 또는 상위 안전 정책에서 처리한다.
        # 기존 구현은 나중에 다시 켤 수 있도록 주석으로 남겨둔다.
        # value = int(brake_current_amps * self.scales.brake_current)
        # payload = bytes([self.command_ids.set_current_brake]) + struct.pack(">i", value)
        # self.write_payload(payload)
        raise VescDriverError(
            "Brake current 명령은 비활성화되어 있습니다. ERPM 명령을 사용하세요."
        )

    def set_erpm(self, erpm: int | float) -> None:
        # VESC set_rpm expects ERPM, not wheel RPM. ERPM is electrical RPM.
        payload = bytes([self.command_ids.set_erpm]) + struct.pack(">i", int(erpm))
        self.write_payload(payload)

    def set_servo_position(self, position: float) -> None:
        position = self._clamp(position, 0.0, 1.0)
        value = int(position * self.scales.servo)
        payload = bytes([self.command_ids.set_servo_pos]) + struct.pack(">h", value)
        self.write_payload(payload)

    def get_firmware_version(self) -> tuple[int, int]:
        """Verify that the serial device answers as a VESC."""
        self.open()
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        reset_input_buffer = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset_input_buffer):
            reset_input_buffer()

        request_id = self.command_ids.get_firmware_version
        self.write_payload(bytes([request_id]))
        payload = self.read_payload()

        if len(payload) < 3 or payload[0] != request_id:
            raise VescDriverError(
                "Invalid firmware-version response from the serial device."
            )

        return int(payload[1]), int(payload[2])

    def get_measured_erpm(self) -> int:
        """Read the ERPM estimated by the VESC motor controller."""
        self.open()
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        reset_input_buffer = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset_input_buffer):
            reset_input_buffer()

        request_id = self.command_ids.get_values_selective
        request = bytes([request_id]) + struct.pack(">I", self.ERPM_VALUE_MASK)
        self.write_payload(request)
        payload = self.read_payload()

        if len(payload) < 5 or payload[0] != request_id:
            raise VescDriverError("Invalid VESC values response.")

        returned_mask = struct.unpack(">I", payload[1:5])[0]
        if not returned_mask & self.ERPM_VALUE_MASK:
            raise VescDriverError("VESC values response does not contain ERPM.")

        erpm_offset = 5 + sum(
            field_size
            for bit, field_size in enumerate(self.VALUE_FIELD_SIZES_BEFORE_ERPM)
            if returned_mask & (1 << bit)
        )
        if len(payload) < erpm_offset + 4:
            raise VescDriverError("VESC values response contains incomplete ERPM data.")

        return struct.unpack(">i", payload[erpm_offset : erpm_offset + 4])[0]

    def write_payload(self, payload: bytes) -> None:
        self.write_packet(self.make_packet(payload))

    def write_packet(self, packet: bytes) -> None:
        self.open()
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        try:
            bytes_written = self._serial.write(packet)
            if bytes_written is not None and int(bytes_written) != len(packet):
                raise VescDriverError(
                    f"Incomplete VESC serial write: {bytes_written}/{len(packet)} bytes"
                )
        except VescDriverError:
            raise
        except Exception as exc:
            raise VescDriverError(f"Failed to write VESC packet: {exc}") from exc

    def read_payload(self) -> bytes:
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        start_byte = self._read_exact(1)[0]
        if start_byte == self.START_BYTE:
            payload_length = self._read_exact(1)[0]
        elif start_byte == self.LONG_START_BYTE:
            payload_length = struct.unpack(">H", self._read_exact(2))[0]
        else:
            raise VescDriverError(
                f"Invalid VESC response start byte: {start_byte}"
            )

        if payload_length < 1 or payload_length > self.MAX_READ_PAYLOAD_SIZE:
            raise VescDriverError(
                f"Invalid VESC response payload length: {payload_length}"
            )

        payload = self._read_exact(payload_length)
        received_crc = struct.unpack(">H", self._read_exact(2))[0]
        end_byte = self._read_exact(1)[0]

        if end_byte != self.END_BYTE:
            raise VescDriverError(f"Invalid VESC response end byte: {end_byte}")

        expected_crc = self.crc16_xmodem(payload)
        if received_crc != expected_crc:
            raise VescDriverError(
                "Invalid VESC response CRC: "
                f"received={received_crc}, expected={expected_crc}"
            )

        return payload

    def _read_exact(self, size: int) -> bytes:
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        received = bytearray()
        while len(received) < size:
            chunk = self._serial.read(size - len(received))
            if not chunk:
                raise VescDriverError(
                    f"Timed out waiting for VESC response ({len(received)}/{size} bytes)"
                )
            received.extend(chunk)

        return bytes(received)

    @classmethod
    def make_packet(cls, payload: bytes) -> bytes:
        if len(payload) > cls.MAX_SHORT_PAYLOAD_SIZE:
            raise VescDriverError(
                f"Payload too large for short VESC packet: {len(payload)} bytes"
            )

        crc = cls.crc16_xmodem(payload)
        return (
            bytes([cls.START_BYTE, len(payload)])
            + payload
            + struct.pack(">H", crc)
            + bytes([cls.END_BYTE])
        )

    @staticmethod
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

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, float(value)))
