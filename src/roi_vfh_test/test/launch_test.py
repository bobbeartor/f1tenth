"""Exercise the real launch assembly with lightweight ROS launch API doubles."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]


def record(kind):
    return lambda *args, **kwargs: dict(kind=kind, args=args, **kwargs)


class Configuration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class LaunchTest(unittest.TestCase):
    def setUp(self):
        modules = {}
        for name in ("ament_index_python", "ament_index_python.packages", "launch", "launch.actions",
                     "launch.launch_description_sources", "launch.substitutions", "launch_ros", "launch_ros.actions"):
            modules[name] = types.ModuleType(name)
        modules["ament_index_python.packages"].get_package_share_directory = lambda name: str(ROOT / name)
        modules["launch"].LaunchDescription = list
        for name in ("DeclareLaunchArgument", "IncludeLaunchDescription", "OpaqueFunction"):
            setattr(modules["launch.actions"], name, record(name))
        modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = record("PythonSource")
        modules["launch.substitutions"].LaunchConfiguration = Configuration
        modules["launch_ros.actions"].Node = record("Node")
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        spec = importlib.util.spec_from_file_location("roi_test_launch", ROOT / "roi_vfh_test/launch/preview.launch.py")
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.context = {a["args"][0]: a["default_value"] for a in self.module.generate_launch_description()
                        if a["kind"] == "DeclareLaunchArgument"}

    def test_preview_defaults_and_shared_camera(self):
        actions = self.module.nodes(self.context)
        roi = next(a for a in actions if a.get("package") == "roi_lidar")["parameters"][0]
        self.assertEqual((roi["camera.fps"], roi["preview.width_m"], roi["preview.forward_m"]), (40, 1.5, 4))
        self.assertFalse(roi["preview.show_bev_window"])
        self.assertEqual((roi["preview.rgb_fps"], roi["preview.fps"]), (80, 30))
        self.assertEqual(roi["terrain.cell_m"], .025)
        self.assertTrue(roi["ground.use_startup_plane"])
        self.assertFalse(roi["cluster.enabled"])
        self.assertTrue(roi["preview.gui"] and roi["preview.publish_lane_bev"] and roi["motion.publish_imu"])
        test = next(a for a in actions if a.get("package") == "roi_vfh_test")["parameters"][0]
        self.assertEqual((test["nominal_speed_mps"], test["vehicle_width_m"], test["wheelbase_m"]), (1.0, .3, .33))
        self.assertEqual((test["front_overhang_m"], test["rear_extent_m"]), (.05, .45))
        self.assertEqual((test["speed_ceiling_mps"], test["lane_lookahead_m"]), (1.8, .05))
        self.assertEqual((test["horizon_m"], test["safety_margin_m"], test["avoidance_clearance_weight"]), (1.2, .02, 1.0))
        self.assertEqual((test["avoidance_curve_samples"], test["avoidance_length_slack_m"]), (17.0, .4))
        self.assertEqual(test["lane_memory_enabled"], 1.0)
        self.assertEqual(test["avoidance_lane_weight"], 1.5)
        self.assertNotIn("lane_trust_duration_sec", test)
        self.assertNotIn("max_sync_sec", test)
        self.assertNotIn("lane_departure_m", test)
        self.assertEqual((test["planning_rate_hz"], test["control_rate_hz"]), (80, 100))
        self.assertEqual((test["max_age_sec"], test["drive.max_command_age_sec"], test["input_resume_stable_sec"]), (.30, .30, .50))
        self.assertEqual(test["lane_receipt_timeout_sec"], .30)
        self.assertEqual(test["preview_fps"], 30)
        self.assertEqual((roi["bev.camera_x_m"], roi["preview.rgb_width_px"], roi["preview.rgb_height_px"]), (-.09, 1280, 800))
        self.assertEqual(roi["motion.imu_rate_hz"], 120)
        self.assertFalse(any(a.get("package") in ("camera_driver", "vehicle_bringup") for a in actions))
        bridge = next(a for a in actions if a.get("package") == "vesc_bridge")
        self.assertEqual(bridge["parameters"][1]["port"], "/dev/ttyTHS1")
        self.assertEqual(bridge["parameters"][1]["duty_topic"], test["duty_topic"])
        lane = next(a for a in actions if a["kind"] == "IncludeLaunchDescription")
        self.assertEqual(dict(lane["launch_arguments"])["input_topic"], "/roi_lidar/lane_bev")
        dynamics = next(a for a in actions if a.get("package") == "vehicle_dynamics_monitor")["parameters"][0]
        self.assertEqual((dynamics["input_mode"], dynamics["imu_topic"], dynamics["imu_yaw_axis"]),
                         ("ros_topic", "/roi_lidar/imu", "z"))

    def test_controller_telemetry_defaults(self):
        control = self.module.load(self.context["vehicle_params_file"], "auto_control")
        actions = self.module.nodes(self.context)
        dynamics = next(a for a in actions if a.get("package") == "vehicle_dynamics_monitor")["parameters"][0]
        self.assertEqual(dynamics["measured_erpm_topic"], control["measured_erpm_topic"])
        self.assertEqual(dynamics["connection_status_topic"], control["connection_status_topic"])
        test = next(a for a in actions if a.get("package") == "roi_vfh_test")["parameters"][0]
        self.assertEqual(dynamics["speed_topic"], test["measured_speed_topic"])
        original = self.module.load
        def customized(path, node):
            result = original(path, node)
            if node == "auto_control":
                result.update(wheel_diameter_m=.12, speed_scale_correction=1.2,
                              speed_filter_time_constant_sec=.08)
            return result
        with patch.object(self.module, "load", side_effect=customized):
            dynamics = next(a for a in self.module.nodes(self.context)
                            if a.get("package") == "vehicle_dynamics_monitor")["parameters"][0]
        self.assertEqual((dynamics["wheel_diameter_m"], dynamics["speed_scale_correction"],
                          dynamics["speed_filter_time_constant_sec"]), (.12, 1.2, .08))

    def test_namespaced_and_explicit_telemetry(self):
        self.context["vehicle_namespace"] = "/autopilot03/"
        dynamics = next(a for a in self.module.nodes(self.context)
                        if a.get("package") == "vehicle_dynamics_monitor")["parameters"][0]
        self.assertEqual(dynamics["measured_erpm_topic"], "/autopilot03/vesc/measured_erpm")
        self.assertEqual(dynamics["connection_status_topic"], "/autopilot03/vesc/connected")
        self.context.update(measured_erpm_topic="/custom/erpm", vesc_connection_topic="/custom/connected")
        dynamics = next(a for a in self.module.nodes(self.context)
                        if a.get("package") == "vehicle_dynamics_monitor")["parameters"][0]
        self.assertEqual(dynamics["measured_erpm_topic"], "/custom/erpm")
        self.assertEqual(dynamics["connection_status_topic"], "/custom/connected")

    def test_external_telemetry(self):
        self.context.update(dynamics_enabled="false", measured_speed_topic="/existing/speed")
        actions = self.module.nodes(self.context)
        self.assertFalse(any(a.get("package") == "vehicle_dynamics_monitor" for a in actions))
        test = next(a for a in actions if a.get("package") == "roi_vfh_test")["parameters"][0]
        self.assertEqual(test["measured_speed_topic"], "/existing/speed")

    def test_invalid_mode(self):
        self.context["dynamics_enabled"] = "invalid"
        with self.assertRaises(ValueError):
            self.module.nodes(self.context)


if __name__ == "__main__":
    unittest.main()
