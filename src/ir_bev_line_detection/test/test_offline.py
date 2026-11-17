"""Offline image/config verification. Does not import or run ROS."""
from pathlib import Path
import hashlib
import json
import tempfile
import unittest
import numpy as np
import yaml

from ir_bev_line_detection.image import mono_to_bgr
from ir_bev_line_detection.settings import parameters, build_parameters


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent
SHARES = {p: SOURCE / p for p in ('ir_camera_driver', 'line_detactor', 'auto_control')}


class ImageTests(unittest.TestCase):
    def test_preserves_all_brightness_values_and_strips_padding(self):
        gray = np.arange(300 * 120, dtype=np.uint32).astype(np.uint8).reshape(300, 120)
        padded = np.pad(gray, ((0, 0), (0, 8)), constant_values=99)
        actual = np.frombuffer(mono_to_bgr(padded.tobytes(), 120, 300, 128), np.uint8).reshape(300,120,3)
        for channel in range(3):
            np.testing.assert_array_equal(actual[:, :, channel], gray)

    def test_rejects_incompatible_geometry_and_truncated_buffers(self):
        for width, height, step, data in (
            (300, 120, 300, bytes(36000)),
            (120, 300, 119, bytes(36000)),
            (120, 300, 120, bytes(35999)),
        ):
            with self.assertRaises(ValueError):
                mono_to_bgr(data, width, height, step)


class ConfigurationTests(unittest.TestCase):
    def test_reuses_production_steering_centerline_and_stale_stops(self):
        configs = build_parameters(SHARES, PACKAGE / 'config/ir_bev_test.yaml', .3)
        for node, filename, prefixes in (
            ('auto_control', 'auto_control.yaml', ('stanley_', 'steering_', 'path_', 'erpm_timeout', 'longitudinal_')),
            ('line_detactor', 'line_detactor.yaml', ('centerline_', 'connection_')),
        ):
            base = parameters(SOURCE / node / 'config' / filename, node)
            for name, value in base.items():
                if name.startswith(prefixes):
                    self.assertEqual(configs[node][name], value, name)
        controller = configs['auto_control']
        self.assertEqual(controller['minimum_speed_mps'], .3)
        self.assertEqual(controller['maximum_speed_mps'], .3)
        self.assertEqual(controller['bev_meter_per_pixel'], .01)
        self.assertEqual(controller['lane_result_topic'], configs['line_detactor']['result_topic'])
        self.assertFalse(controller['traffic_stop_enabled'])
        self.assertFalse(controller['avoidance_control_enabled'])
        camera = configs['ir_camera_driver']
        self.assertFalse(camera['reprojection_enabled'])
        self.assertEqual(camera['selected_camera'], 'LEFT')
        self.assertEqual(camera['manual_exposure_us'], 30000)
        self.assertEqual(camera['manual_sensitivity_iso'], 800)
        self.assertEqual(camera['ir_flood_light_intensity'], .5)
        self.assertEqual(camera['ir_dot_projector_intensity'], 0.)
        self.assertFalse(camera['rgb_enabled'])

    def test_applies_requested_speed_without_point_three_limit(self):
        for speed in (.2, .3, .31, 1.0, 2.0):
            with self.subTest(speed=speed):
                controller = build_parameters(
                    SHARES, PACKAGE / 'config/ir_bev_test.yaml', str(speed))['auto_control']
                self.assertEqual(controller['minimum_speed_mps'], speed)
                self.assertEqual(controller['maximum_speed_mps'], speed)

    def test_rejects_nonpositive_or_nonfinite_speed(self):
        for speed in (0, -.1, float('nan'), float('inf'), float('-inf')):
            with self.assertRaises(ValueError):
                build_parameters(SHARES, PACKAGE / 'config/ir_bev_test.yaml', speed)

    def test_rejects_camera_geometry_that_would_mislead_controller(self):
        config = yaml.safe_load((PACKAGE / 'config/ir_bev_test.yaml').read_text(encoding='utf-8'))
        config['ir_camera_driver']['ros__parameters']['bev.meter_per_pixel'] = .02
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.yaml'
            path.write_text(yaml.safe_dump(config), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'geometry'):
                build_parameters(SHARES, path, .3)


class ModelTests(unittest.TestCase):
    def test_bundled_model_matches_verified_ir_export(self):
        path = SOURCE / 'line_detactor/models/ir_stereo_bev_120x300.onnx'
        report = json.loads(path.with_suffix('.json').read_text(encoding='utf-8'))
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), report['onnx_sha256'])
        self.assertIn('stereo_only_v1', report['checkpoint'])
        self.assertEqual(report['input'], [1, 3, 300, 120])
        self.assertEqual(report['output'], [1, 3, 300, 120])
        self.assertEqual(report['channels'], ['left', 'right', 'stop_line'])
        self.assertEqual(len(report['validation']), 10)
        for record in report['validation']:
            self.assertEqual(record['mask_difference_pixels'], [0, 0, 0])


if __name__ == '__main__':
    unittest.main()
