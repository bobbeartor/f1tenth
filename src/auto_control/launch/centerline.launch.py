from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    drive_enabled = LaunchConfiguration("drive_enabled")
    publish_debug = LaunchConfiguration("publish_debug")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("auto_control"),
                        "config",
                        "centerline.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("drive_enabled", default_value="false"),
            DeclareLaunchArgument("publish_debug", default_value="false"),
            DeclareLaunchArgument("image_topic", default_value="/lane_mask"),
            Node(
                package="auto_control",
                executable="centerline_node",
                name="centerline_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "drive_enabled": ParameterValue(
                            drive_enabled,
                            value_type=bool,
                        ),
                        "publish_debug": ParameterValue(
                            publish_debug,
                            value_type=bool,
                        ),
                        "image_topic": image_topic,
                    },
                ],
            ),
        ]
    )
