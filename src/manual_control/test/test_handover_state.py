import unittest

from manual_control.handover_state import HandoverState


class HandoverStateTest(unittest.TestCase):
    def test_requires_x_and_new_stationary_measurements(self):
        state = HandoverState()
        state.on_erpm(0, 0.0)
        self.assertFalse(state.update(1.0, True))
        self.assertTrue(state.on_x(True))
        state.on_x(False)
        self.assertFalse(state.update(1.0, True))
        for now in (1.0, 1.2, 1.4):
            state.on_erpm(0, now)
            self.assertFalse(state.update(now, True))
        state.on_erpm(0, 1.6)
        self.assertTrue(state.update(1.6, True))
        self.assertEqual(state.mode, "MANUAL")

    def test_motion_in_either_direction_resets_stop_confirmation(self):
        for erpm in (101, -101):
            with self.subTest(erpm=erpm):
                state = HandoverState()
                state.on_x(True)
                state.on_x(False)
                state.on_erpm(0, 0.0)
                state.on_erpm(erpm, 0.2)
                state.on_erpm(0, 0.4)
                state.on_erpm(0, 0.6)
                self.assertFalse(state.update(0.6, True))

    def test_stale_telemetry_and_sample_gaps_require_full_new_confirmation(self):
        state = HandoverState()
        state.on_x(True)
        state.on_x(False)
        state.on_erpm(0, 0.0)
        state.on_erpm(0, 0.2)
        self.assertFalse(state.update(0.6, True))
        state.on_erpm(0, 0.7)
        self.assertFalse(state.update(0.7, True))
        # Even without a watchdog callback, a gap between measurements resets it.
        state.on_erpm(0, 1.3)
        self.assertFalse(state.update(1.3, True))

    def test_held_x_or_non_neutral_controls_block_manual_and_x_never_rearms(self):
        state = HandoverState()
        state.on_x(True)
        for now in (0.0, 0.2, 0.4, 0.6):
            state.on_erpm(0, now)
        self.assertFalse(state.update(0.6, True))
        state.on_x(False)
        self.assertFalse(state.update(0.6, False))
        self.assertTrue(state.update(0.6, True))
        for pressed in (True, True, False, True):
            self.assertFalse(state.on_x(pressed))
        self.assertEqual(state.mode, "MANUAL")

    def test_invalid_measurement_or_connection_loss_clears_confirmation(self):
        for invalid in ("connection", "nan"):
            state = HandoverState()
            state.on_x(True)
            state.on_x(False)
            for now in (0.0, 0.2, 0.4):
                state.on_erpm(0, now)
            if invalid == "connection":
                state.invalidate_telemetry()
            else:
                state.on_erpm(float("nan"), 0.5)
            state.on_erpm(0, 0.6)
            self.assertFalse(state.update(0.6, True))

    def test_rejects_invalid_thresholds(self):
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                HandoverState(stopped_erpm=value)


if __name__ == "__main__":
    unittest.main()
