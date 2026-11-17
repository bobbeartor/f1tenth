import unittest
from unittest.mock import patch

from vehicle_dynamics_monitor.vesc_can import CAN_EFF_FLAG
from vehicle_dynamics_monitor.vesc_slcan import VescSlcanInterface


class FakeMessage:
    def __init__(
        self,
        arbitration_id: int,
        data: bytes,
        is_extended_id: bool,
        is_error_frame: bool = False,
        is_remote_frame: bool = False,
    ) -> None:
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id
        self.is_error_frame = is_error_frame
        self.is_remote_frame = is_remote_frame


class FakeBus:
    def __init__(self) -> None:
        self.received: list[FakeMessage] = []
        self.receive_timeouts: list[float] = []
        self.shutdown_called = False

    def recv(self, timeout: float) -> FakeMessage | None:
        self.receive_timeouts.append(timeout)
        if not self.received:
            return None
        return self.received.pop(0)

    def shutdown(self) -> None:
        self.shutdown_called = True


class FakeCanModule:
    def __init__(self) -> None:
        self.bus = FakeBus()
        self.bus_arguments: dict[str, object] = {}

    def Bus(self, **kwargs: object) -> FakeBus:
        self.bus_arguments = kwargs
        return self.bus


class VescSlcanInterfaceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fake_can = FakeCanModule()
        patcher = patch(
            "vehicle_dynamics_monitor.vesc_slcan.importlib.import_module",
            return_value=self.fake_can,
        )
        self.addCleanup(patcher.stop)
        patcher.start()

    def test_open_and_drain_normalize_python_can_frames(self) -> None:
        interface = VescSlcanInterface("/dev/ttyACM0", bitrate=500_000)
        interface.open()
        self.fake_can.bus.received.extend(
            [
                FakeMessage(0x970, bytes(range(8)), True),
                FakeMessage(0x123, b"\x01\x02", False),
            ]
        )

        frames = interface.drain()

        self.assertEqual(
            self.fake_can.bus_arguments,
            {
                "interface": "slcan",
                "channel": "/dev/ttyACM0",
                "bitrate": 500_000,
            },
        )
        self.assertEqual(
            frames,
            [
                (CAN_EFF_FLAG | 0x970, bytes(range(8))),
                (0x123, b"\x01\x02"),
            ],
        )
        self.assertEqual(
            self.fake_can.bus.receive_timeouts, [0.01, 0.0, 0.0]
        )

    def test_close_shuts_down_bus(self) -> None:
        interface = VescSlcanInterface("/dev/ttyACM0")
        interface.open()

        interface.close()

        self.assertTrue(self.fake_can.bus.shutdown_called)
        self.assertFalse(interface.is_open)


if __name__ == "__main__":
    unittest.main()
