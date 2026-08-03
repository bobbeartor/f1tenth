"""Camera, mask, manual BEV, and BEV lane extraction in one container.

The components share a multi-threaded container so images move by
intra-process pointer hand-off instead of DDS serialisation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    camera_share = get_package_share_directory("camera_driver")
    lane_share = get_package_share_directory("lane_detect")
    bev_share = get_package_share_directory("bev_processor")

    camera_params = os.path.join(camera_share, "config", "camera_config.yaml")
    lane_params = os.path.join(lane_share, "config", "lane_mask.yaml")
    bev_params = os.path.join(bev_share, "config", "bev_config_manual.yaml")
    bev_override = os.path.join(
        lane_share, "config", "bev_override.yaml"
    )
    bev_lane_params = os.path.join(
        lane_share, "config", "bev_lane_extractor.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_params_file",
                default_value=camera_params,
                description="Camera driver parameter YAML",
            ),
            DeclareLaunchArgument(
                "lane_params_file",
                default_value=lane_params,
                description="Lane mask parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_params,
                description="Manual BEV parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_lane_params_file",
                default_value=bev_lane_params,
                description="BEV-space lane extractor parameter YAML",
            ),
            DeclareLaunchArgument(
                "bev_input_topic",
                default_value="/camera/image_lane",
                description=(
                    "Set to /camera/image_rect to bypass the mask and warp "
                    "the raw camera frame instead."
                ),
            ),
            DeclareLaunchArgument(
                "lane_preview",
                default_value="false",
                description="Show the mask preview window.",
            ),
            DeclareLaunchArgument(
                "bev_lane_preview",
                default_value="false",
                description="Show the BEV lane extraction preview window.",
            ),
            DeclareLaunchArgument(
                "bev_lane_enabled",
                default_value="true",
                description=(
                    "Run BEV lane fitting. Set false for raw-image BEV "
                    "bypass diagnostics."
                ),
            ),
            ComposableNodeContainer(
                name="lane_bev_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="bev_processor",
                        plugin="bev_processor::BevProcessorNode",
                        name="bev_processor_manual",
                        parameters=[
                            LaunchConfiguration("bev_params_file"),
                            bev_override,
                        ],
                        remappings=[
                            (
                                "/camera/image_lane",
                                LaunchConfiguration("bev_input_topic"),
                            ),
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    ComposableNode(
                        package="lane_detect",
                        plugin="lane_mask::BevLaneExtractorNode",
                        name="bev_lane_extractor",
                        parameters=[
                            LaunchConfiguration("bev_lane_params_file"),
                            {
                                "enabled": LaunchConfiguration(
                                    "bev_lane_enabled"
                                ),
                                "preview_enabled": LaunchConfiguration(
                                    "bev_lane_preview"
                                ),
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    ComposableNode(
                        package="lane_detect",
                        plugin="lane_mask::LaneMaskNode",
                        name="lane_mask",
                        parameters=[
                            LaunchConfiguration("lane_params_file"),
                            {
                                "preview_enabled": LaunchConfiguration(
                                    "lane_preview"
                                ),
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
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
