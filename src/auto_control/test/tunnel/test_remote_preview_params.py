import tempfile
import unittest

try:
    from auto_control_tunnel.launch_support import remote_preview_parameters
except ImportError:  # launch_ros only exists on the ROS machine
    remote_preview_parameters = None


@unittest.skipIf(remote_preview_parameters is None, 'launch_ros unavailable')
class RemotePreviewParameterTests(unittest.TestCase):
    def load(self, text, width=0.7):
        with tempfile.NamedTemporaryFile('w', suffix='.yaml') as stream:
            stream.write(text)
            stream.flush()
            return remote_preview_parameters(stream.name, width)

    def test_yaml_values_are_used_and_zero_width_falls_back_to_detector(self):
        params = self.load('remote_preview:\n  ros__parameters:\n    port: 9000\n    lane_width_m: 0.0\n')
        self.assertEqual(params['port'], 9000)
        self.assertEqual(params['lane_width_m'], 0.7)

    def test_explicit_width_and_wildcard_section(self):
        params = self.load('/**:\n  ros__parameters:\n    lane_width_m: 0.8\n    link_loss_stop_sec: 0.0\n')
        self.assertEqual(params['lane_width_m'], 0.8)
        self.assertEqual(params['link_loss_stop_sec'], 0.0)


if __name__ == '__main__':
    unittest.main()
