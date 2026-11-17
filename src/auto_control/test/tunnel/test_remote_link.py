import time
import unittest

from auto_control_tunnel.remote_link import LatestSlot, LinkWatchdog


class WatchdogTests(unittest.TestCase):
    def test_watchdog_fires_once_per_loss(self):
        dog = LinkWatchdog(1.0)
        dog.heartbeat(0.0)
        self.assertTrue(dog.should_stop(1.2, True))
        self.assertFalse(dog.should_stop(1.5, True))
        dog.heartbeat(2.0)
        self.assertTrue(dog.should_stop(3.1, True))

    def test_watchdog_ignores_when_not_started_or_disabled(self):
        dog = LinkWatchdog(1.0)
        dog.heartbeat(0.0)
        self.assertFalse(dog.should_stop(5.0, False))
        disabled = LinkWatchdog(0.0)
        disabled.heartbeat(0.0)
        self.assertFalse(disabled.should_stop(99.0, True))

    def test_watchdog_no_heartbeat_yet_does_not_fire(self):
        self.assertFalse(LinkWatchdog(1.0).should_stop(10.0, True))

    def test_watchdog_any_client_heartbeat_counts(self):
        dog = LinkWatchdog(1.0)
        t = 0.0
        while t < 3.0:
            dog.heartbeat(t)
            self.assertFalse(dog.should_stop(t + 0.3, True))
            t += 0.6


class SlotTests(unittest.TestCase):
    def test_latest_slot_drops_old_items(self):
        slot = LatestSlot()
        for item in (1, 2, 3):
            slot.put(item)
        self.assertEqual(slot.get(0.01), 3)
        self.assertIsNone(slot.get(0.01))
        slot.close()
        started = time.monotonic()
        self.assertIsNone(slot.get(5.0))
        self.assertLess(time.monotonic() - started, 0.5)


if __name__ == '__main__':
    unittest.main()
