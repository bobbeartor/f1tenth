from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(value):
    if value.lower() not in ("true", "false"):
        raise ValueError("Boolean launch argument must be true or false")
    return value.lower() == "true"


def _nodes(context):
    # Empty shortcuts preserve the YAML values, including custom config files.
    with open(LaunchConfiguration("bev_config_file").perform(context), encoding="utf-8") as stream:
        reference = yaml.safe_load(stream)["bev_processor"]["ros__parameters"]
    bev_parameters = {"bev." + key: reference[key] for key in (
        "x_min_m", "x_max_m", "y_min_m", "y_max_m", "camera_x_m", "camera_y_m", "camera_yaw_deg")}
    bev_parameters["bev.frame_id"] = reference["output_frame_id"]
    bev_parameters.update({key: value for key, value in reference.items()
                           if key.startswith("measurement_") or key.startswith("manual_camera_height_")})
    # The standalone RViz viewer needs no measurement image window.
    bev_parameters["measurement_roi_preview_enabled"] = False
    overrides = {}
    for argument, parameter, convert in (
        ("bev", "bev.enabled", _boolean),
        ("ground", "ground.enabled", _boolean),
        ("resolution", "camera.resolution", str),
        ("fps", "camera.fps", float),
        ("dot", "depth.ir_dot_projector_intensity", float),
        ("flood", "depth.ir_flood_light_intensity", float),
    ):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides[parameter] = convert(value)
    return [
        Node(
            package="point_cloud",
            executable="point_cloud_node",
            name="point_cloud",
            output="screen",
            parameters=[bev_parameters, LaunchConfiguration("config_file"), overrides],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="point_cloud_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rviz_config")],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("point_cloud"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=str(share / "config" / "point_cloud.yaml")),
            DeclareLaunchArgument("bev_config_file", default_value=str(share / "config" / "bev_reference.yaml"),
                                  description="bev_processor YAML: geometry and startup measurement settings"),
            DeclareLaunchArgument("bev", default_value="", description="Crop to BEV footprint: true / false; empty uses YAML"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=str(share / "rviz" / "point_cloud.rviz")),
            DeclareLaunchArgument("resolution", default_value="", description="400p / 480p / 720p / 800p; empty uses YAML"),
            DeclareLaunchArgument("fps", default_value="", description="Requested FPS; empty uses YAML"),
            DeclareLaunchArgument("dot", default_value="", description="Dot projector 0..1; empty uses YAML"),
            DeclareLaunchArgument("flood", default_value="", description="IR flood light 0..1; empty uses YAML"),
            DeclareLaunchArgument("ground", default_value="", description="Remove ground: true / false; empty uses YAML"),
            OpaqueFunction(function=_nodes),
        ]
    )
