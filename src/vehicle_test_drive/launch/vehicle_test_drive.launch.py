from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
        parameters=[
            vesc_config,
            {
                "port": vesc_port,
                "log_commands": True,
            },
        ],
    )

    test_drive_node = Node(
        package="vehicle_test_drive",
        executable="vehicle_test_drive",
        name="vehicle_test_drive",
        output="screen",
    )

    shutdown_when_test_finishes = RegisterEventHandler(
        OnProcessExit(
            target_action=test_drive_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyUSB0"),
            vesc_bridge_node,
            test_drive_node,
            shutdown_when_test_finishes,
        ]
    )
