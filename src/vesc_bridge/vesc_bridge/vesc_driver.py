#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path
import sys
import threading
import struct
import time
from dataclasses import dataclass, replace
from typing import Any, Callable


@dataclass
class SerialWriteStats:
    attempts: int = 0
    failures: int = 0
    timeouts: int = 0
    bytes_sent: int = 0
    duration_sum_sec: float = 0.0
    duration_max_sec: float = 0.0
    last_duration_sec: float = 0.0


@dataclass(frozen=True)
class VescCommandIds:
    set_duty: int = 5
    # Propulsion current mode remains disabled; brake-current mode is enabled.
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
    # Propulsion current mode remains disabled.
    current: int = 1000
    brake_current: int = 1000

    # 현재 활성화된 명령 스케일.
    servo: int = 1000


class VescDriverError(RuntimeError):
    pass


class VescResponseError(VescDriverError):
    """A missing or malformed response; the transport may still be usable."""


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
        port: str = "/dev/ttyTHS1",
        baudrate: int = 115200,
        timeout: float = 0.02,
        write_timeout: float = 0.05,
        startup_delay: float = 0.2,
        command_ids: VescCommandIds | None = None,
        scales: VescScales | None = None,
        serial_factory: Callable[..., Any] | None = None,
        exclusive: bool = True,
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
        self.exclusive = exclusive
        self._stats_lock = threading.Lock()
        self._write_stats = SerialWriteStats()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def write_stats(self) -> SerialWriteStats:
        with self._stats_lock:
            return replace(self._write_stats)

    def _check_port_owners(self) -> None:
        # Inspect only PID/comm/fd paths, never command lines or process contents.
        # This is diagnostic; kernel exclusivity below closes the subsequent-open race.
        target = os.path.realpath(self.port)
        owners = []
        try:
            processes = list(Path('/proc').iterdir())
        except OSError:
            return
        for process in processes:
            if not process.name.isdigit() or int(process.name) == os.getpid():
                continue
            try:
                descriptors = list((process / 'fd').iterdir())
            except OSError:
                continue
            for descriptor in descriptors:
                try:
                    if os.path.realpath(descriptor) != target:
                        continue
                    name = (process / 'comm').read_text().strip()
                    owners.append(f'{process.name}({name})')
                    break
                except OSError:
                    continue
        if owners:
            raise VescDriverError(f'UART port {self.port} is already open by PID ' + ', '.join(owners))

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
            options = dict(port=self.port, baudrate=self.baudrate,
                           timeout=self.timeout, write_timeout=self.write_timeout)
            linux_exclusive = self.exclusive and sys.platform.startswith('linux') and self._serial_factory is None
            if linux_exclusive:
                self._check_port_owners()
                options['exclusive'] = True
            self._serial = factory(**options)
            if linux_exclusive:
                import fcntl
                import termios
                fcntl.ioctl(self._serial.fileno(), termios.TIOCEXCL)
                self._check_port_owners()
        except Exception as exc:
            try:
                self.close()
            except Exception:
                pass
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
        brake_current_amps = max(0.0, float(brake_current_amps))
        value = int(brake_current_amps * self.scales.brake_current)
        payload = bytes([self.command_ids.set_current_brake]) + struct.pack(
            ">i", value
        )
        self.write_payload(payload)

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
            raise VescResponseError("Invalid VESC values response.")

        returned_mask = struct.unpack(">I", payload[1:5])[0]
        if not returned_mask & self.ERPM_VALUE_MASK:
            raise VescResponseError("VESC values response does not contain ERPM.")

        erpm_offset = 5 + sum(
            field_size
            for bit, field_size in enumerate(self.VALUE_FIELD_SIZES_BEFORE_ERPM)
            if returned_mask & (1 << bit)
        )
        if len(payload) < erpm_offset + 4:
            raise VescResponseError("VESC values response contains incomplete ERPM data.")

        return struct.unpack(">i", payload[erpm_offset : erpm_offset + 4])[0]

    def write_payload(self, payload: bytes) -> None:
        self.write_packet(self.make_packet(payload))

    def write_packet(self, packet: bytes) -> None:
        self.open()
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        started = time.monotonic()
        failure = None
        bytes_written = 0
        try:
            bytes_written = self._serial.write(packet)
            if bytes_written is not None and int(bytes_written) != len(packet):
                raise VescDriverError(
                    f"Incomplete VESC serial write: {bytes_written}/{len(packet)} bytes"
                )
        except Exception as exc:
            failure = exc
            if isinstance(exc, VescDriverError):
                raise
            raise VescDriverError(f"Failed to write VESC packet: {exc}") from exc
        finally:
            elapsed = max(0., time.monotonic()-started)
            with self._stats_lock:
                stats = self._write_stats
                stats.attempts += 1
                stats.failures += int(failure is not None)
                stats.timeouts += int(failure is not None and 'timeout' in
                                      (type(failure).__name__ + str(failure)).lower())
                if failure is None:
                    stats.bytes_sent += len(packet) if bytes_written is None else int(bytes_written)
                stats.duration_sum_sec += elapsed
                stats.duration_max_sec = max(stats.duration_max_sec, elapsed)
                stats.last_duration_sec = elapsed

    def read_payload(self) -> bytes:
        if self._serial is None:
            raise VescDriverError("VESC serial port is not open.")

        start_byte = self._read_exact(1)[0]
        if start_byte == self.START_BYTE:
            payload_length = self._read_exact(1)[0]
        elif start_byte == self.LONG_START_BYTE:
            payload_length = struct.unpack(">H", self._read_exact(2))[0]
        else:
            raise VescResponseError(
                f"Invalid VESC response start byte: {start_byte}"
            )

        if payload_length < 1 or payload_length > self.MAX_READ_PAYLOAD_SIZE:
            raise VescResponseError(
                f"Invalid VESC response payload length: {payload_length}"
            )

        payload = self._read_exact(payload_length)
        received_crc = struct.unpack(">H", self._read_exact(2))[0]
        end_byte = self._read_exact(1)[0]

        if end_byte != self.END_BYTE:
            raise VescResponseError(f"Invalid VESC response end byte: {end_byte}")

        expected_crc = self.crc16_xmodem(payload)
        if received_crc != expected_crc:
            raise VescResponseError(
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
                raise VescResponseError(
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
