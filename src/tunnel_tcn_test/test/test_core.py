import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
from tunnel_tcn_test.features import FeatureStream, FeatureExtractor, EventDecoder, preprocess, sigmoid
from tunnel_tcn_test.display import mono_image, footer


class FeatureTests(unittest.TestCase):
    def test_original_training_temporal_features(self):
        path = Path(__file__).resolve().parents[1] / 'models/golden_replay.npz'
        with np.load(path) as data:
            extractor = FeatureExtractor()
            for i, image in enumerate(data['synthetic_images']):
                actual = extractor.push(preprocess(image), i/30.)
                np.testing.assert_allclose(actual, data['synthetic_features'][i], atol=1e-6)

    def test_checkpoint_normalization_mask_and_causal_context(self):
        stream = FeatureStream(mean=np.ones(16), std=np.full(16, 2))
        for frame in range(110):
            x = stream.push_features(np.full(16, frame), frame/30.)
            targets = frame/30.-np.arange(31, -1, -1)/10.
            expected = (np.maximum(0, np.floor(targets*30.+1e-6))-1)/2
            np.testing.assert_allclose(x[0, 0], expected, atol=1e-5)
            np.testing.assert_array_equal(x[0, 16], targets >= -1e-8)
        self.assertEqual(x.shape, (1,17,32))

    def test_missing_samples_cannot_use_future_and_gap_clears_mask(self):
        stream = FeatureStream(warmup_seconds=0)
        stream.push_features(np.zeros(16), 0.)
        x = stream.push_features(np.ones(16), .2)
        self.assertEqual(x[0,0,-2], 0.)
        self.assertEqual(x[0,0,-1], 1.)
        x = stream.push_features(np.ones(16), 1.)
        self.assertTrue(stream.discontinuity)
        np.testing.assert_array_equal(x[0,16,:-1], 0.)
        self.assertEqual(x[0,16,-1], 1.)

    def test_independent_scores(self):
        np.testing.assert_allclose(sigmoid([[0, 0]]), [.5, .5])
        self.assertTrue((sigmoid([[10,10]]) > .99).all())


class DecoderTests(unittest.TestCase):
    def test_alternation_crossings_and_simultaneous_scores(self):
        d = EventDecoder()
        self.assertEqual(d.step([.6,.9], 0)['type'], 'exit')
        self.assertIsNone(d.step([.9,.9], .1)) # entry was already active, no new crossing
        d.step([0,0], .2)
        self.assertEqual(d.step([.9,.1], .3)['type'], 'entry')
        d.pause()
        self.assertEqual(d.state, 'inside')
        self.assertIsNone(d.step([1,0], 2))
        d.reset()
        self.assertEqual(d.state, 'unknown')

    def test_cooldown_rejected_crossing_is_not_queued(self):
        d = EventDecoder()
        d.step([1,0], 0)
        d.step([0,1], .1)
        self.assertIsNone(d.step([1,0], .2))
        self.assertIsNone(d.step([1,0], 2))
        d.step([0,0], 2.1)
        self.assertEqual(d.step([1,0], 2.2)['type'], 'entry')

    def test_explicit_initial_state(self):
        d = EventDecoder(initial_state='outside')
        self.assertIsNone(d.step([0,1], 0))
        self.assertEqual(d.step([1,0], .1)['type'], 'entry')


class DisplayTests(unittest.TestCase):
    def test_padded_mono_and_bgr(self):
        m = SimpleNamespace(encoding='mono8', width=2, height=2, step=3, data=bytes([1,2,99,3,4,99]))
        np.testing.assert_array_equal(mono_image(m), [[1,2],[3,4]])
        m = SimpleNamespace(encoding='bgr8', width=1, height=1, step=4, data=bytes([1,2,3,99]))
        np.testing.assert_array_equal(mono_image(m), [[[3,2,1]]])
        m.data = b''
        with self.assertRaises(ValueError): mono_image(m)

    def test_nv12_full_range_conversion(self):
        m = SimpleNamespace(encoding='nv12', width=2, height=2, step=2, data=bytes([235]*4+[128]*2))
        np.testing.assert_array_equal(mono_image(m), np.full((400,640),255,np.uint8))

    def test_nv12_luma_ignores_chroma_and_padding_without_color_conversion(self):
        # Two constant luma halves, padded rows, deliberately extreme chroma.
        y = bytes([16, 16, 235, 235, 99, 99]) * 2
        m = SimpleNamespace(encoding='nv12', width=4, height=2, step=6,
                            data=y + bytes([0, 255, 0, 255, 99, 99]))
        with patch('tunnel_tcn_test.display.cv2.cvtColor', side_effect=AssertionError('RGB conversion')):
            gray = mono_image(m)
        np.testing.assert_array_equal(gray[:, :320], np.zeros((400,320), np.uint8))
        np.testing.assert_array_equal(gray[:, 320:], np.full((400,320),255,np.uint8))
        m.data = y + bytes([128] * 6)
        np.testing.assert_array_equal(mono_image(m), gray)
        m.data = m.data[:-1]
        with self.assertRaises(ValueError): mono_image(m)

    def test_nv12_neutral_luma_matches_previous_rgb_features(self):
        import cv2
        y = np.repeat(np.arange(16, 236, dtype=np.uint8)[None, :], 40, axis=0)
        uv = np.full((20, 220), 128, np.uint8)
        raw = np.concatenate((y, uv))
        m = SimpleNamespace(encoding='nv12', width=220, height=40, step=220, data=raw.tobytes())
        previous = preprocess(cv2.cvtColor(raw, cv2.COLOR_YUV2RGB_NV12))
        actual = preprocess(mono_image(m))
        self.assertLessEqual(int(np.abs(actual.astype(int) - previous.astype(int)).max()), 1)
        old_stream, new_stream = FeatureStream(), FeatureStream()
        np.testing.assert_allclose(new_stream.push(actual, 0), old_stream.push(previous, 0), atol=.02)

    def test_footer_preserves_camera_pixels(self):
        frame = np.full((600,960),83,np.uint8)
        rendered = footer(frame, [.98,.01], 'inside', 'ENTRY .980', 'LIVE',30.,.3,5.,0,'RGB auto')
        self.assertEqual(rendered.shape, (760,960,3))
        np.testing.assert_array_equal(rendered[:600], np.full((600,960,3),83,np.uint8))


if __name__ == '__main__': unittest.main()
