from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vesc_port = LaunchConfiguration("vesc_port")
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_bringup"), "config", "vesc_config.yaml"]
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vesc_port",
                default_value="/dev/ttyTHS1",
                description="VESC serial device path",
            ),
            vesc_bridge_node,
        ]
    )
