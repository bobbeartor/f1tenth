import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_share = get_package_share_directory("camera_driver")
    bev_share = get_package_share_directory("bev_processor")

    camera_params = os.path.join(
        camera_share, "config", "camera_config.yaml"
    )
    bev_params = os.path.join(bev_share, "config", "bev_config.yaml")
    performance_measurement_enabled = LaunchConfiguration(
        "performance_measurement_enabled"
    )
    performance_measurement_parameter = ParameterValue(
        performance_measurement_enabled,
        value_type=bool,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Camera driver parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_params,
                description=(
                    "BEV parameter YAML; its root must be bev_processor"
                ),
            ),
            DeclareLaunchArgument(
                "performance_measurement_enabled",
                default_value="false",
                description=(
                    "Disable GUI previews and print stabilized/BEV pipeline "
                    "performance measurements."
                ),
            ),
            ComposableNodeContainer(
                name="bev_processor_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    # BEV가 먼저 OAK를 단독으로 열어 높이·roll·pitch를
                    # 측정하고 장치를 닫은 뒤 camera_driver가 시작된다.
                    ComposableNode(
                        package="bev_processor",
                        plugin="bev_processor::BevProcessorNode",
                        name="bev_processor",
                        parameters=[
                            LaunchConfiguration("bev_params_file"),
                            {
                                "performance_measurement_enabled": (
                                    performance_measurement_parameter
                                ),
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    # 안정화된 전체 NV12 영상을 BEV에 전달한다. BEV는
                    # 시작 측정 LUT를 유지하므로 IMU 토픽 발행은 필요 없다.
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            LaunchConfiguration("camera_params_file"),
                            {
                                "preview_enabled": False,
                                "publish_enabled": True,
                                "imu_bridge_enabled": False,
                                "imu_stabilization_enabled": True,
                                "output_crop_top_px": 0,
                                "performance_measurement_enabled": (
                                    performance_measurement_parameter
                                ),
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                ],
            ),
        ]
    )
