from __future__ import annotations

import unittest

from manual_control.button_debouncer import RisingEdgeDebouncer


class RisingEdgeDebouncerTest(unittest.TestCase):
    def test_accepts_first_press_immediately_and_ignores_hold(self) -> None:
        debouncer = RisingEdgeDebouncer(0.20)

        self.assertFalse(debouncer.update(False, 0.00))
        self.assertTrue(debouncer.update(True, 0.01))
        self.assertFalse(debouncer.update(True, 0.10))

    def test_rejects_bounce_but_accepts_a_later_press(self) -> None:
        debouncer = RisingEdgeDebouncer(0.20)

        self.assertTrue(debouncer.update(True, 1.00))
        self.assertFalse(debouncer.update(False, 1.02))
        self.assertFalse(debouncer.update(True, 1.04))
        self.assertFalse(debouncer.update(False, 1.10))
        self.assertTrue(debouncer.update(True, 1.21))

    def test_rejects_negative_debounce_interval(self) -> None:
        with self.assertRaisesRegex(ValueError, "non-negative"):
            RisingEdgeDebouncer(-0.01)


if __name__ == "__main__":
    unittest.main()
