from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("jetson_resource_monitor"), "config", "monitor.yaml"]
    )
    config_argument = DeclareLaunchArgument(
        "config", default_value=default_config, description="Monitor YAML file"
    )
    monitor_node = Node(
        package="jetson_resource_monitor",
        executable="resource_monitor_node",
        name="jetson_resource_monitor",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_argument, monitor_node])

