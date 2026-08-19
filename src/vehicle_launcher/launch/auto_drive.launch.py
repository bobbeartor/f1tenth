from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_enabled = LaunchConfiguration("drive_enabled")
    publish_debug = LaunchConfiguration("publish_debug")
    camera_preview_enabled = LaunchConfiguration("camera_preview_enabled")
    camera_performance_measurement_enabled = LaunchConfiguration(
        "camera_performance_measurement_enabled"
    )
    camera_imu_stabilization_enabled = LaunchConfiguration(
        "camera_imu_stabilization_enabled"
    )
    camera_publish_fps = LaunchConfiguration("camera_publish_fps")
    lane_process_width = LaunchConfiguration("lane_process_width")
    control_rate_hz = LaunchConfiguration("control_rate_hz")
    vesc_port = LaunchConfiguration("vesc_port")

    camera_config = PathJoinSubstitution(
        [FindPackageShare("camera_driver"), "config", "camera_config.yaml"]
    )
    lane_detect_config = PathJoinSubstitution(
        [
            FindPackageShare("lane_detect"),
            "config",
            "lane_mask.yaml",
        ]
    )
    camera_lane_container = ComposableNodeContainer(
        name="camera_lane_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="camera_driver",
                plugin="camera_driver::CameraDriverNode",
                name="camera_driver",
                parameters=[
                    camera_config,
                    {
                        "preview_enabled": ParameterValue(
                            camera_preview_enabled, value_type=bool
                        ),
                        "performance_measurement_enabled": ParameterValue(
                            camera_performance_measurement_enabled,
                            value_type=bool,
                        ),
                        "publish_enabled": True,
                        "publish_fps": ParameterValue(
                            camera_publish_fps, value_type=float
                        ),
                        "imu_stabilization_enabled": ParameterValue(
                            camera_imu_stabilization_enabled,
                            value_type=bool,
                        ),
                        "imu_bridge_enabled": False,
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="lane_detect",
                plugin="lane_mask::LaneMaskNode",
                name="lane_mask",
                parameters=[
                    lane_detect_config,
                    {
                        # Process every camera frame and publish the working
                        # resolution instead of a full-size upscaled mask.
                        "process_width": ParameterValue(
                            lane_process_width, value_type=int
                        ),
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
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
                "control_rate_hz": ParameterValue(
                    control_rate_hz, value_type=float
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
                "camera_performance_measurement_enabled",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "camera_imu_stabilization_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "camera_publish_fps", default_value="100.0"
            ),
            DeclareLaunchArgument(
                "lane_process_width", default_value="640"
            ),
            DeclareLaunchArgument(
                "control_rate_hz", default_value="100.0"
            ),
            DeclareLaunchArgument(
                "vesc_port", default_value="/dev/ttyTHS1"
            ),
            camera_lane_container,
            vesc_node,
            lane_node,
        ]
    )
