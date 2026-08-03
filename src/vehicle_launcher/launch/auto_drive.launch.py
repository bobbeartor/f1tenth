from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_enabled = LaunchConfiguration("drive_enabled")
    publish_debug = LaunchConfiguration("publish_debug")
    camera_preview_enabled = LaunchConfiguration("camera_preview_enabled")
    camera_publish_fps = LaunchConfiguration("camera_publish_fps")
    vesc_port = LaunchConfiguration("vesc_port")

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("camera_driver"),
                    "launch",
                    "camera_driver.launch.py",
                ]
            )
        ),
        launch_arguments={
            "preview_enabled": camera_preview_enabled,
            "publish_enabled": "true",
            "publish_fps": camera_publish_fps,
            "imu_stabilization_enabled": "true",
        }.items(),
    )

    vesc_config = PathJoinSubstitution(
        [FindPackageShare("vehicle_config"), "config", "vesc_config.yaml"]
    )
    vesc_node = Node(
        package="vesc_initializer",
        executable="vesc_initialize_node",
        name="vesc_initialize_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    auto_config = PathJoinSubstitution(
        [FindPackageShare("auto_control"), "config", "perspective_lane.yaml"]
    )
    lane_node = Node(
        package="auto_control",
        executable="perspective_lane_node",
        name="perspective_lane_node",
        output="screen",
        parameters=[
            auto_config,
            {
                "drive_enabled": ParameterValue(
                    drive_enabled, value_type=bool
                ),
                "publish_debug": ParameterValue(
                    publish_debug, value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drive_enabled",
                default_value="false",
                description=(
                    "Allow autonomous VESC commands. Keep false for dry-run."
                ),
            ),
            DeclareLaunchArgument("publish_debug", default_value="false"),
            DeclareLaunchArgument(
                "camera_preview_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "camera_publish_fps", default_value="30.0"
            ),
            DeclareLaunchArgument(
                "vesc_port", default_value="/dev/ttyACM0"
            ),
            camera_launch,
            vesc_node,
            lane_node,
        ]
    )
