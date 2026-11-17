from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    vesc_port = LaunchConfiguration("vesc_port")
    controller_name_contains = LaunchConfiguration("controller_name_contains")
    input_mode = LaunchConfiguration("input_mode")
    can_interface = LaunchConfiguration("can_interface")
    can_controller_id = LaunchConfiguration("can_controller_id")
    slcan_channel = LaunchConfiguration("slcan_channel")
    slcan_bitrate = LaunchConfiguration("slcan_bitrate")

    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_bringup"),
                    "launch",
                    "manual_drive.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "vesc_port": vesc_port,
            "controller_name_contains": controller_name_contains,
        }.items(),
    )
    dynamics_monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("vehicle_dynamics_monitor"),
                    "launch",
                    "vehicle_dynamics_monitor.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "input_mode": input_mode,
            "can_interface": can_interface,
            "can_controller_id": can_controller_id,
            "slcan_channel": slcan_channel,
            "slcan_bitrate": slcan_bitrate,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_namespace",
                default_value="autopilot03",
                description="Vehicle-unique ROS namespace.",
            ),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            DeclareLaunchArgument(
                "input_mode",
                default_value="ros_topic",
                description="ros_topic, socketcan, or CANable 2 slcan.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            DeclareLaunchArgument(
                "slcan_channel", default_value="/dev/ttyACM0"
            ),
            DeclareLaunchArgument("slcan_bitrate", default_value="500000"),
            manual_drive,
            dynamics_monitor,
        ]
    )
