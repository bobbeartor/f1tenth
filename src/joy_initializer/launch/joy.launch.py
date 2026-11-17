from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    device_id = LaunchConfiguration("device_id")
    device_name_contains = LaunchConfiguration("device_name_contains")
    deadzone = LaunchConfiguration("deadzone")
    autorepeat_rate = LaunchConfiguration("autorepeat_rate")
    sticky_buttons = LaunchConfiguration("sticky_buttons")
    coalesce_interval_ms = LaunchConfiguration("coalesce_interval_ms")
    reconnect_interval_sec = LaunchConfiguration("reconnect_interval_sec")

    joy_node = Node(
        package="joy_initializer",
        executable="joy_input_node",
        name="joy_node",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            {
                "device_id": ParameterValue(device_id, value_type=int),
                "device_name_contains": ParameterValue(
                    device_name_contains, value_type=str
                ),
                "deadzone": ParameterValue(deadzone, value_type=float),
                "autorepeat_rate": ParameterValue(
                    autorepeat_rate, value_type=float
                ),
                "sticky_buttons": ParameterValue(sticky_buttons, value_type=bool),
                "coalesce_interval_ms": ParameterValue(
                    coalesce_interval_ms, value_type=int
                ),
                "reconnect_interval_sec": ParameterValue(
                    reconnect_interval_sec, value_type=float
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_namespace", default_value=""),
            DeclareLaunchArgument("device_id", default_value="0"),
            DeclareLaunchArgument(
                "device_name_contains",
                default_value="8BitDo",
            ),
            DeclareLaunchArgument("deadzone", default_value="0.05"),
            DeclareLaunchArgument("autorepeat_rate", default_value="50.0"),
            DeclareLaunchArgument("sticky_buttons", default_value="false"),
            DeclareLaunchArgument("coalesce_interval_ms", default_value="1"),
            DeclareLaunchArgument("reconnect_interval_sec", default_value="1.0"),
            joy_node,
        ]
    )
