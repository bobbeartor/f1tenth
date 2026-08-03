from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_share = get_package_share_directory("camera_driver")
    default_params = f"{package_share}/config/camera_config.yaml"

    params_file = LaunchConfiguration("params_file")
    preview_enabled = LaunchConfiguration("preview_enabled")
    preview_grid_enabled = LaunchConfiguration("preview_grid_enabled")
    imu_stabilization_enabled = LaunchConfiguration(
        "imu_stabilization_enabled"
    )
    publish_enabled = LaunchConfiguration("publish_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Camera driver parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="false",
                description="Show the independent latest-frame preview.",
            ),
            DeclareLaunchArgument(
                "preview_grid_enabled",
                default_value="true",
                description="Draw a light-gray 20-pixel grid on the preview.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_enabled",
                default_value="true",
                description=(
                    "Hold preview/published NV12 at the startup pitch/roll "
                    "reference with the OAK IMU."
                ),
            ),
            DeclareLaunchArgument(
                "publish_enabled",
                default_value="false",
                description="Publish sensor_msgs/Image frames.",
            ),
            ComposableNodeContainer(
                name="camera_container",
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
                            params_file,
                            {
                                "preview_enabled": preview_enabled,
                                "preview_grid_enabled": preview_grid_enabled,
                                "imu_stabilization_enabled": (
                                    imu_stabilization_enabled
                                ),
                                "publish_enabled": publish_enabled,
                                # Standalone launch does not publish IMU;
                                # stabilization can still use it internally.
                                "imu_bridge_enabled": False,
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    )
                ],
            ),
        ]
    )
