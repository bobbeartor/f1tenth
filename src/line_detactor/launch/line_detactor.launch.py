from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


OVERRIDES = {
    "connection_ccl_serial_enabled": bool,
    "stop_line_mask_threshold": float,
    "stop_line_min_present_pixels": int,
    "stop_line_distance_enabled": bool,
    "stop_line_min_pixels": int,
    "stop_line_max_fit_samples": int,
    "stop_line_support_trim_quantile": float,
    "stop_line_support_margin_px": float,
    "stop_line_near_edge_quantile": float,
    "stop_line_min_crossing_alignment": float,
    "stop_line_fit_distance_tolerance_px": float,
    "stop_line_fit_angle_tolerance_rad": float,
    "profiling_enabled": bool,
    "profiling_directory": str,
    "centerline_enabled": bool,
    "centerline_lane_width_m": float,
    "centerline_bev_width_m": float,
    "centerline_bev_height_m": float,
    "centerline_sample_spacing_m": float,
    "centerline_output_spacing_m": float,
    "centerline_clearance_check_spacing_m": float,
    "centerline_min_fragment_length_m": float,
    "centerline_tangent_window_m": float,
    "centerline_width_tolerance_m": float,
    "centerline_pair_along_tolerance_m": float,
    "centerline_pair_heading_tolerance_deg": float,
    "centerline_max_gap_m": float,
    "centerline_max_start_distance_m": float,
    "centerline_min_clearance_m": float,
    "centerline_outside_margin_m": float,
    "centerline_max_samples": int,
    "centerline_line_width_px": int,
    "centerline_corner_outer_enabled": bool,
    "centerline_corner_outer_weight": float,
    "centerline_corner_outward_offset_m": float,
    "centerline_corner_entry_distance_m": float,
    "centerline_corner_entry_full_offset_distance_m": float,
    "centerline_corner_outer_window_m": float,
    "centerline_corner_outer_tangent_window_m": float,
    "centerline_corner_outer_min_length_m": float,
    "centerline_corner_outer_min_turn_deg": float,
    "centerline_corner_outer_full_turn_deg": float,
    "centerline_smoothing_enabled": bool,
    "centerline_smoothing_sigma_m": float,
    "centerline_smoothing_window_m": float,
    "centerline_smoothing_max_shift_m": float,
    "centerline_smoothing_strength": float,
    "centerline_straight_turn_deg": float,
    "centerline_corner_turn_deg": float,
    "centerline_turn_window_m": float,

    "model_path": str,
    "input_topic": str,
    "direct_bev_input_enabled": bool,
    "engine_cache_path": str,
    "engine_precision": str,
    "tensorrt_workspace_size_mb": int,
    "model_input_width": int,
    "model_input_height": int,
    "mask_threshold": float,
    "overlay_alpha": float,
    "preview_enabled": bool,
    "bev_theme_enable": bool,
    "bev_theme_preview_fps": float,
    "bev_theme_speed_topic": str,
    "preview_target_speed_topic": str,
    "preview_corner_state_topic": str,
    "preview_result_only_enabled": bool,
    "preview_fps": float,
    "connection_enabled": bool,
    "result_padding_px": int,
    "connection_min_component_area_px": int,
    "connection_skeleton_downsample_factor": int,
    "connection_min_fragment_length_px": float,
    "connection_max_fragments": int,
    "connection_tangent_window_px": float,
    "connection_max_gap_px": float,
    "connection_corridor_half_width_px": float,
    "connection_direction_tolerance_deg": float,
    "connection_max_turn_deg": float,
    "connection_max_curvature_per_px": float,
    "connection_max_arc_ratio": float,
    "connection_border_endpoint_distance_px": float,
    "result_line_width_px": int,
    "result_publish_enabled": bool,
    "result_topic": str,
    "result_image_topic": str,
    "control_latency_topic": str,
}


def launch_container(context):
    overrides = {}
    for name, value_type in OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value == "":
            continue
        if value_type is bool:
            if value.lower() not in ("true", "false"):
                raise ValueError(f"{name} must be true or false")
            overrides[name] = value.lower() == "true"
        else:
            overrides[name] = value_type(value)

    return [
        ComposableNodeContainer(
            name="line_detactor_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="line_detactor",
                    plugin="line_detactor::LineDetactorNode",
                    name="line_detactor",
                    parameters=[
                        LaunchConfiguration("params_file").perform(context),
                        overrides,
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                )
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("line_detactor")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/line_detactor.yaml",
                description="ROS parameter YAML; explicit launch arguments override it",
            ),
        ]
        + [
            DeclareLaunchArgument(
                name, default_value="", description="Empty uses the YAML/node default"
            )
            for name in OVERRIDES
        ]
        + [OpaqueFunction(function=launch_container)]
    )
