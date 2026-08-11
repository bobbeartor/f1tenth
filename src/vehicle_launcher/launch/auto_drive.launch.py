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

    lane_detect_config = PathJoinSubstitution(
        [
            FindPackageShare("lane_detect"),
            "config",
            "lane_mask.yaml",
        ]
    )
    lane_detect_node = Node(
        package="lane_detect",
        executable="lane_detect_node",
        name="lane_mask",
        output="screen",
        parameters=[
            lane_detect_config,
            {
                # The centerline controller consumes mono8 directly. Process
                # every 60 Hz camera frame (0 disables the second rate gate)
                # and retain source rows through normalized y=98 before the
                # centerline model applies its exact ROI.
                "process_max_fps": 0.0,
                "bottom_cut_ratio": 0.0,
            },
        ],
    )

    vesc_node = Node(
        package="vesc_initializer",
        executable="vesc_initialize_node",
        name="vesc_initialize_node",
        output="screen",
        parameters=[vesc_config, {"port": vesc_port}],
    )

    auto_config = PathJoinSubstitution(
        [FindPackageShare("auto_control"), "config", "centerline.yaml"]
    )
    lane_node = Node(
        package="auto_control",
        executable="centerline_node",
        name="centerline_node",
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
                "camera_publish_fps", default_value="60.0"
            ),
            DeclareLaunchArgument(
                "vesc_port", default_value="/dev/ttyTHS1"
            ),
            camera_launch,
            lane_detect_node,
            vesc_node,
            lane_node,
        ]
    )
