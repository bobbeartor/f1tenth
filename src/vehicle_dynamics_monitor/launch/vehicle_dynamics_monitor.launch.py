from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    params_file = LaunchConfiguration("params_file")
    input_mode = LaunchConfiguration("input_mode")
    can_interface = LaunchConfiguration("can_interface")
    can_controller_id = LaunchConfiguration("can_controller_id")
    slcan_channel = LaunchConfiguration("slcan_channel")
    slcan_bitrate = LaunchConfiguration("slcan_bitrate")

    default_params = PathJoinSubstitution(
        [
            FindPackageShare("vehicle_dynamics_monitor"),
            "config",
            "vehicle_dynamics.yaml",
        ]
    )

    dynamics_node = Node(
        package="vehicle_dynamics_monitor",
        executable="vehicle_dynamics_node",
        name="vehicle_dynamics_node",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            params_file,
            {
                "input_mode": input_mode,
                "can_interface": can_interface,
                "can_controller_id": ParameterValue(
                    can_controller_id, value_type=int
                ),
                "slcan_channel": slcan_channel,
                "slcan_bitrate": ParameterValue(
                    slcan_bitrate, value_type=int
                ),
                "measured_erpm_topic": "vesc/measured_erpm",
                "connection_status_topic": "vesc/connected",
                "commanded_duty_topic": "manual/current_duty",
                "servo_position_topic": "vesc/servo_position",
                "acceleration_topic": "vehicle/dynamics/acceleration",
                "speed_topic": "vehicle/dynamics/speed_mps",
                "longitudinal_acceleration_topic": (
                    "vehicle/dynamics/longitudinal_acceleration_mps2"
                ),
                "lateral_acceleration_topic": (
                    "vehicle/dynamics/lateral_acceleration_mps2"
                ),
                "yaw_rate_topic": "vehicle/dynamics/yaw_rate_radps",
                "motor_rpm_topic": "vehicle/dynamics/motor_rpm",
                "wheel_rpm_topic": "vehicle/dynamics/wheel_rpm",
                "motor_current_topic": "vehicle/dynamics/motor_current_a",
                "input_current_topic": "vehicle/dynamics/input_current_a",
                "input_voltage_topic": "vehicle/dynamics/input_voltage_v",
                "duty_cycle_topic": "vehicle/dynamics/duty_cycle",
                "fet_temperature_topic": (
                    "vehicle/dynamics/fet_temperature_c"
                ),
                "motor_temperature_topic": (
                    "vehicle/dynamics/motor_temperature_c"
                ),
                "diagnostics_topic": "vehicle/dynamics/diagnostics",
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_namespace", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Vehicle dynamics monitor parameter YAML.",
            ),
            DeclareLaunchArgument(
                "input_mode",
                default_value="ros_topic",
                description="ros_topic, receive-only socketcan, or USB slcan.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            DeclareLaunchArgument(
                "slcan_channel", default_value="/dev/ttyACM0"
            ),
            DeclareLaunchArgument("slcan_bitrate", default_value="500000"),
            dynamics_node,
        ]
    )
