from __future__ import annotations

import importlib
from typing import Any

from vehicle_dynamics_monitor.vesc_can import CAN_EFF_FLAG


class VescSlcanInterface:
    """Receive-only python-can transport for a CANable SLCAN adapter."""

    def __init__(
        self,
        channel: str,
        bitrate: int = 500_000,
        receive_timeout_sec: float = 0.01,
    ) -> None:
        if not channel:
            raise ValueError("SLCAN channel must not be empty")
        if bitrate <= 0:
            raise ValueError("SLCAN bitrate must be greater than zero")
        if receive_timeout_sec < 0.0:
            raise ValueError("SLCAN receive timeout must be non-negative")
        self.channel = channel
        self.bitrate = bitrate
        self.receive_timeout_sec = receive_timeout_sec
        self._bus: Any | None = None

    @property
    def endpoint(self) -> str:
        return self.channel

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(self) -> None:
        if self.is_open:
            return
        try:
            can_module = importlib.import_module("can")
        except ImportError as exc:
            raise RuntimeError(
                "python-can with serial support is required; install "
                "it with: python3 -m pip install 'python-can[serial]'"
            ) from exc

        # Keep this identical to the standalone receive test known to work on
        # the Jetson/CANable 2 combination. python-can selects the serial-line
        # defaults required by the adapter.
        self._bus = can_module.Bus(
            interface="slcan",
            channel=self.channel,
            bitrate=self.bitrate,
        )

    def close(self) -> None:
        bus = self._bus
        self._bus = None
        if bus is not None:
            bus.shutdown()

    def drain(self, maximum_frames: int = 256) -> list[tuple[int, bytes]]:
        bus = self._bus
        if bus is None:
            return []
        frames: list[tuple[int, bytes]] = []
        try:
            for frame_index in range(maximum_frames):
                # A zero-only timeout can repeatedly miss SLCAN serial data.
                # Wait briefly for the first frame, then drain queued frames
                # without blocking the ROS executor.
                timeout_sec = (
                    self.receive_timeout_sec if frame_index == 0 else 0.0
                )
                message = bus.recv(timeout=timeout_sec)
                if message is None:
                    break
                if message.is_error_frame or message.is_remote_frame:
                    continue
                can_id = int(message.arbitration_id)
                if message.is_extended_id:
                    can_id |= CAN_EFF_FLAG
                frames.append((can_id, bytes(message.data)))
        except Exception:
            self.close()
            raise
        return frames
