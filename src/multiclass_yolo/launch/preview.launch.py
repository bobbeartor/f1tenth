from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_profile(path, node):
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)[node]["ros__parameters"]


def generate_launch_description():
    share = get_package_share_directory("multiclass_yolo")
    reference = Path(share) / "config" / "vfh_reference"
    roi_defaults = Path(get_package_share_directory("roi_lidar")) / "config" / "roi_lidar.yaml"
    profile = load_profile(roi_defaults, "roi_lidar")
    profile.update(load_profile(reference / "roi_lidar_test.yaml", "roi_lidar"))
    ground = load_profile(reference / "bev_config_test.yaml", "bev_processor")
    profile.update({key: value for key, value in ground.items()
                    if key.startswith(("measurement_", "manual_camera_height_"))})
    for key in ("camera_x_m", "camera_y_m", "camera_yaw_deg"):
        profile["bev." + key] = ground[key]
    profile["measurement_roi_preview_enabled"] = False

    defaults = {
        "engine_cache_path": ("", str),
        "input_format": ("nv12", str),
        "inference_backend": ("TENSORRT", str),
        "model_variant": ("holdout10", str),
        "engine_precision": ("fp16", str),
        "model_path": (share + "/models/multiclass_yolox_tiny_512x320_holdout10.onnx", str),
        "int8_model_path": (share + "/models/multiclass_yolox_tiny_512x320_holdout10.int8.qdq.onnx", str),
        "measurement_seconds": ("15", int),
        "measurement_directory": ("", str),
        "width": ("1280", int), "height": ("800", int),
        "depth_fusion_enabled": ("true", bool),
        "ground_estimation_enabled": ("true", bool),
        "fixed_camera_height_m": ("0.20", float),
        "fixed_camera_roll_deg": ("0.0", float),
        "fixed_camera_pitch_down_deg": ("14.0", float),
        "fusion_sync_ms": ("20.0", float),
        "sensor_fps": ("40.0", float), "undistort_enabled": ("true", bool),
        "roi_height_percent": ("65", int),
        "frame_timeout_sec": ("5.0", float),
        "tensorrt_workspace_size_mb": ("1024", int),
        "nms_threshold": ("0.65", float),
        **{name + "_threshold": ("0.70", float)
           for name in ("con", "truck", "sedan", "traffic_light")},
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, (value, _) in defaults.items()]
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=kind)
                  for name, (_, kind) in defaults.items()}
    return LaunchDescription(arguments + [Node(
        package="multiclass_yolo", executable="multiclass_yolo_node",
        name="multiclass_yolo", output="screen", parameters=[profile, parameters],
    )])
