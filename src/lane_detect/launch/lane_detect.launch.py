from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    preview_enabled = LaunchConfiguration("preview_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lane_detect"),
                        "config",
                        "lane_mask.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("preview_enabled", default_value="false"),
            Node(
                package="lane_detect",
                executable="lane_detect_node",
                name="lane_mask",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "preview_enabled": ParameterValue(
                            preview_enabled, value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
