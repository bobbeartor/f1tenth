from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    config_path = Path(value("config_file")).expanduser().resolve(strict=True)
    with config_path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{config_path}: expected roi_lidar: ros__parameters: mapping")
    node_config = document.get("roi_lidar", document.get("/roi_lidar"))
    if not isinstance(node_config, dict) or not isinstance(node_config.get("ros__parameters"), dict):
        raise ValueError(f"{config_path}: expected roi_lidar: ros__parameters: mapping")
    with Path(value("bev_config_file")).expanduser().open(encoding="utf-8") as stream:
        reference = yaml.safe_load(stream)["bev_processor"]["ros__parameters"]
    # Same vehicle origin/mount and startup procedure as BEV; no 3m XY crop.
    mount = {"bev." + key: reference[key] for key in
             ("camera_x_m", "camera_y_m", "camera_yaw_deg")}
    mount["frame_id"] = reference["output_frame_id"]
    mount.update({key: item for key, item in reference.items()
                  if key.startswith("measurement_") or key.startswith("manual_camera_height_")})
    profiles = {
        "yaml": {},  # Apply the supplied YAML without any profile overrides.
        "balanced": {},  # Backward-compatible alias for yaml.
        "fast": {"camera.resolution": "400p", "camera.fps": 110.0,
                 "depth.subpixel": False},
        "far": {"camera.resolution": "800p", "camera.fps": 30.0,
                "depth.subpixel": True, "depth.subpixel_fractional_bits": 3,
                "points.max_depth_m": 25.0, "range.max_m": 20.0,
                "points.pixel_stride": 1},
    }
    profile = value("profile")
    if profile not in profiles:
        raise ValueError("profile: yaml / balanced / fast / far")
    overrides = dict(profiles[profile])
    for arg, name, convert in (("fps", "camera.fps", float),
                                ("range", "range.max_m", float),
                                ("resolution", "camera.resolution", str)):
        if value(arg):
            overrides[name] = convert(value(arg))
    if value("gui"):
        if value("gui").lower() not in ("true", "false"):
            raise ValueError("gui: true / false")
        overrides["preview.gui"] = value("gui").lower() == "true"
    actions = [LogInfo(msg=f"roi_lidar YAML: {config_path}")]
    if overrides:
        actions.append(LogInfo(msg=f"Explicit launch overrides: {overrides}"))
    actions.append(Node(package="roi_lidar", executable="roi_lidar_node", name="roi_lidar",
                        output="screen", parameters=[mount, str(config_path), overrides]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("roi_lidar"))
    point_cloud = Path(get_package_share_directory("point_cloud"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config/roi_lidar.yaml")),
        DeclareLaunchArgument("bev_config_file", default_value=str(point_cloud / "config/bev_reference.yaml")),
        DeclareLaunchArgument("profile", default_value="yaml"),
        DeclareLaunchArgument("fps", default_value=""),
        DeclareLaunchArgument("range", default_value=""),
        DeclareLaunchArgument("resolution", default_value=""),
        DeclareLaunchArgument("gui", default_value=""),
        OpaqueFunction(function=_nodes),
    ])
