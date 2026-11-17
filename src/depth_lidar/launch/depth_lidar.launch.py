from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("depth_lidar"))
        / "config"
        / "depth_lidar.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the depth_lidar parameter file",
            ),
            Node(
                package="depth_lidar",
                executable="depth_lidar_node",
                name="depth_lidar",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
