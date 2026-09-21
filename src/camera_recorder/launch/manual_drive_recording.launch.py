from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    recorder_share = get_package_share_directory("camera_recorder")
    vehicle_namespace = LaunchConfiguration("vehicle_namespace")
    vesc_port = LaunchConfiguration("vesc_port")
    controller_name_contains = LaunchConfiguration("controller_name_contains")

    manual_control_config = PathJoinSubstitution(
        [FindPackageShare("camera_recorder"), "config", "manual_control.yaml"]
    )
    vesc_config = PathJoinSubstitution(
        [FindPackageShare("camera_recorder"), "config", "vesc.yaml"]
    )
    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare("joy_initializer"), "launch", "joy.launch.py"]
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            "vehicle_namespace": vehicle_namespace,
            "device_id": "0",
            "device_name_contains": controller_name_contains,
            "deadzone": "0.05",
            "autorepeat_rate": "50.0",
            "sticky_buttons": "false",
            "coalesce_interval_ms": "1",
            "reconnect_interval_sec": "0.2",
        }.items(),
    )

    actuator_commander_node = Node(
        package="manual_control",
        executable="actuator_commander_node",
        name="actuator_commander_node",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            manual_control_config,
            {
                "joy_topic": "joy",
                "current_duty_topic": "manual/current_duty",
                "current_brake_current_topic": (
                    "manual/current_brake_current"
                ),
                "gear_state_topic": "manual/gear",
                "duty_topic": "vesc/duty",
                "brake_current_topic": "vesc/brake_current",
                "servo_position_topic": "vesc/servo_position",
            },
        ],
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        namespace=vehicle_namespace,
        output="screen",
        parameters=[
            vesc_config,
            {
                "port": vesc_port,
                "duty_topic": "vesc/duty",
                "brake_current_topic": "vesc/brake_current",
                "erpm_topic": "vesc/erpm",
                "measured_erpm_topic": "vesc/measured_erpm",
                "servo_position_topic": "vesc/servo_position",
                "connection_status_topic": "vesc/connected",
            },
        ],
    )

    recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{recorder_share}/launch/camera_recording.launch.py"
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "output_directory": LaunchConfiguration("output_directory"),
            "device_id": LaunchConfiguration("camera_device_id"),
            "fps": LaunchConfiguration("fps"),
            "ir_dot_projector_intensity": LaunchConfiguration(
                "ir_dot_projector_intensity"
            ),
            "ir_flood_light_intensity": LaunchConfiguration(
                "ir_flood_light_intensity"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_namespace", default_value="autopilot03"
            ),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=(
                    f"{recorder_share}/config/camera_recorder.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "output_directory", default_value="recordings"
            ),
            DeclareLaunchArgument(
                "camera_device_id",
                default_value="",
                description=(
                    "DepthAI camera device ID; empty selects the first device."
                ),
            ),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity", default_value="0.5"
            ),
            joy_launch,
            actuator_commander_node,
            vesc_bridge_node,
            recording,
        ]
    )
