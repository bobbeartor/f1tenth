from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Test only:
    #   8BitDo Bluetooth Controller -> haptic-free joy_input_node -> /joy
    #
    # Run:
    #   ros2 launch vehicle_bringup joy_test.launch.py
    #
    # Check:
    #   ros2 topic list
    #   ros2 topic echo /joy
    #
    # A name substring accepts the minor naming differences used by BlueZ/SDL.
    controller_name_contains = LaunchConfiguration("controller_name_contains")
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "device_id": "0",
            "device_name_contains": controller_name_contains,
            "deadzone": "0.05",
            "autorepeat_rate": "50.0",
            "sticky_buttons": "false",
            "coalesce_interval_ms": "1",
            "reconnect_interval_sec": "0.2",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            joy_launch,
        ]
    )
