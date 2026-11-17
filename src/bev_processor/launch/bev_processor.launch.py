import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_FILE_DEFAULT = "__PARAMETER_FILE_DEFAULT__"


def _ros_parameters(config_path, node_name):
    with open(config_path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    return config.get(node_name, {}).get("ros__parameters", {})


def _launch_default(parameters, name, fallback):
    value = parameters.get(name, fallback)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _merged_parameters(base_path, selected_path, node_name):
    parameters = _ros_parameters(base_path, node_name)
    selected_path = os.path.abspath(selected_path)
    if not os.path.isfile(selected_path):
        raise RuntimeError(f"parameter file does not exist: {selected_path}")
    if os.path.realpath(selected_path) != os.path.realpath(base_path):
        overrides = _ros_parameters(selected_path, node_name)
        if not overrides:
            raise RuntimeError(
                f"{selected_path} has no {node_name}.ros__parameters section"
            )
        parameters.update(overrides)
    return parameters


def _apply_parameter_file_defaults(
    context,
    *,
    bev_params,
    launch_parameter_defaults,
):
    parameters = _merged_parameters(
        bev_params,
        LaunchConfiguration("bev_params_file").perform(context),
        "bev_processor",
    )
    for launch_name, parameter_name, fallback in launch_parameter_defaults:
        if (
            context.launch_configurations[launch_name]
            == _PARAMETER_FILE_DEFAULT
        ):
            context.launch_configurations[launch_name] = _launch_default(
                parameters, parameter_name, fallback
            )
    camera = _ros_parameters(
        LaunchConfiguration("camera_params_file").perform(context), "camera_driver"
    )
    for argument, parameter, fallback in (
        ("measured_erpm_topic", "imu_stabilization_measured_erpm_topic", "/vesc/measured_erpm"),
        ("vehicle_acceleration_topic", "imu_stabilization_can_acceleration_topic", "/vehicle/dynamics/acceleration"),
    ):
        if context.launch_configurations[argument] == _PARAMETER_FILE_DEFAULT:
            context.launch_configurations[argument] = str(camera.get(parameter, fallback))
    return []


def _configure_depthai_runtime(context, package_share):
    library_dir = LaunchConfiguration("depthai_library_dir").perform(context).strip()
    if not library_dir:
        runtime_file = Path(package_share) / "depthai_runtime_dir.txt"
        try:
            library_dir = runtime_file.read_text(encoding="utf-8").strip()
        except OSError as error:
            raise RuntimeError(
                f"Cannot read {runtime_file}; rebuild bev_processor to record its DepthAI SDK, "
                "or set depthai_library_dir to the matching SDK library directory."
            ) from error
    if not library_dir:
        raise RuntimeError("The recorded DepthAI runtime library directory is empty")
    library_dir = os.path.abspath(os.path.expanduser(library_dir))
    if not os.path.isdir(library_dir):
        raise RuntimeError(f"DepthAI runtime library directory does not exist: {library_dir}")
    previous = context.environment.get("LD_LIBRARY_PATH", "")
    runtime_path = os.pathsep.join(item for item in (library_dir, previous) if item)
    return [
        SetLaunchConfiguration("depthai_runtime_library_path", runtime_path),
        LogInfo(msg=f"[DepthAI runtime] shared BEV/camera container library dir={library_dir}"),
    ]


def generate_launch_description():
    camera_share = get_package_share_directory("camera_driver")
    bev_share = get_package_share_directory("bev_processor")

    camera_params = os.path.join(
        camera_share, "config", "camera_config.yaml"
    )
    bev_params = os.path.join(bev_share, "config", "bev_config.yaml")
    imu_stabilization_enabled = LaunchConfiguration(
        "imu_stabilization_enabled"
    )
    high_frequency_only = LaunchConfiguration(
        "imu_stabilization_high_frequency_only"
    )
    high_frequency_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_cutoff_hz"
    )
    gyroscope_correction_gain = LaunchConfiguration(
        "imu_stabilization_gyroscope_correction_gain"
    )
    can_longitudinal_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_longitudinal_compensation_gain"
    )
    can_lateral_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_lateral_compensation_gain"
    )
    moving_accelerometer_nudge_strength = LaunchConfiguration(
        "imu_stabilization_moving_accelerometer_nudge_strength"
    )
    moving_gravity_anchor_maximum_correction_rate_degps = LaunchConfiguration(
        "imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps"
    )
    invalid_correction_hold_frames = LaunchConfiguration(
        "imu_stabilization_invalid_correction_hold_frames"
    )
    bev_input_bottom_fraction = LaunchConfiguration(
        "bev_input_bottom_fraction"
    )
    preview_enabled = LaunchConfiguration("preview_enabled")
    camera_preview_enabled = LaunchConfiguration("camera_preview_enabled")
    camera_publish_enabled = LaunchConfiguration("camera_publish_enabled")
    camera_publish_fps = LaunchConfiguration("camera_publish_fps")
    bev_interpolation = LaunchConfiguration("bev_interpolation")
    imu_stabilization_parameter = ParameterValue(
        imu_stabilization_enabled,
        value_type=bool,
    )
    high_frequency_only_parameter = ParameterValue(
        high_frequency_only,
        value_type=bool,
    )
    high_frequency_cutoff_parameter = ParameterValue(
        high_frequency_cutoff_hz,
        value_type=float,
    )
    gyroscope_correction_gain_parameter = ParameterValue(
        gyroscope_correction_gain,
        value_type=float,
    )
    can_longitudinal_compensation_parameter = ParameterValue(
        can_longitudinal_compensation_gain,
        value_type=float,
    )
    can_lateral_compensation_parameter = ParameterValue(
        can_lateral_compensation_gain,
        value_type=float,
    )
    moving_accelerometer_nudge_strength_parameter = ParameterValue(
        moving_accelerometer_nudge_strength,
        value_type=float,
    )
    moving_gravity_anchor_rate_parameter = ParameterValue(
        moving_gravity_anchor_maximum_correction_rate_degps,
        value_type=float,
    )
    invalid_correction_hold_frames_parameter = ParameterValue(
        invalid_correction_hold_frames,
        value_type=int,
    )

    # Keep capture/publication settings overridable from `ros2 launch ... name:=x`.
    # When omitted, each value is resolved from bev_params_file at launch time.
    runtime_parameters = [
        ("capture_directory", ".", str),
        ("capture_joy_topic", "/joy", str),
        ("capture_joy_button", "1", int),
        ("publish_enabled", "true", bool),
        ("direct_output_enabled", "false", bool),
        ("direct_host_copy_enabled", "false", bool),
        ("dataset_collection_enabled", "false", bool),
        ("dataset_collection_manual_capture_mode", "false", bool),
        ("dataset_collection_manual_capture_button", "0", int),
        ("dataset_collection_root_directory", "datasets", str),
        ("dataset_collection_fps", "10.0", float),
        ("dataset_collection_target_count", "1000", int),
        ("dataset_collection_stop_auto_on_complete", "true", bool),
    ]
    launch_parameter_defaults = [
        ("bev_input_bottom_fraction", "input_bottom_fraction", "0.70"),
        ("preview_enabled", "preview_enabled", "true"),
        ("bev_interpolation", "bev_interpolation", "bilinear"),
        *[(name, name, fallback) for name, fallback, _ in runtime_parameters],
    ]
    runtime_launch_arguments = [
        DeclareLaunchArgument(
            name,
            default_value=_PARAMETER_FILE_DEFAULT,
            description=f"Override bev_processor parameter '{name}'.",
        )
        for name, _, _ in runtime_parameters
    ]
    runtime_parameter_overrides = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type in runtime_parameters
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("startup_inspection_enabled",default_value="false"),
            DeclareLaunchArgument(
                "depthai_library_dir", default_value="",
                description="DepthAI shared library directory; empty uses the SDK selected at build time.",
            ),
            OpaqueFunction(function=_configure_depthai_runtime, kwargs={"package_share": bev_share}),
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
            DeclareLaunchArgument("obstacles_enabled", default_value="false"),
            DeclareLaunchArgument("tunnel_enabled", default_value="false"),
            DeclareLaunchArgument("tunnel_validation_mode", default_value="true"),
            DeclareLaunchArgument("tunnel_ir_imu_enabled", default_value="true"),
            DeclareLaunchArgument("measured_erpm_topic", default_value=_PARAMETER_FILE_DEFAULT),
            DeclareLaunchArgument("vehicle_acceleration_topic", default_value=_PARAMETER_FILE_DEFAULT),
            DeclareLaunchArgument("perception_params_file", default_value=LaunchConfiguration("bev_params_file"),
                                  description="Optional shared obstacle parameters; default reuses the selected BEV YAML"),
            DeclareLaunchArgument(
                "startup_preview_image_enabled", default_value="true",
                description="false: publish startup status text only (remote_preview mode)"),
            DeclareLaunchArgument(
                "imu_stabilization_enabled",
                default_value="true",
                description=(
                    "Apply OAK IMU pitch/roll stabilization. The camera's "
                    "fixed_view_zoom remains active when this is disabled."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_only",
                default_value="false",
                description=(
                    "Disable CAN and moving accelerometer corrections, "
                    "leaving gyro high-frequency stabilization only."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_cutoff_hz",
                default_value="3.0",
                description="High-frequency-only gyro cutoff in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_gyroscope_correction_gain",
                default_value="1.0",
                description=(
                    "Applied gyro image correction fraction from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_longitudinal_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN longitudinal acceleration removed "
                    "from the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_lateral_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN lateral acceleration removed from "
                    "the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_moving_accelerometer_nudge_strength",
                default_value="0.15",
                description=(
                    "Strength of the bounded residual-accelerometer image "
                    "correction, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                (
                    "imu_stabilization_moving_gravity_anchor_maximum_"
                    "correction_rate_degps"
                ),
                default_value="0.50",
                description=(
                    "Maximum persistent CAN-compensated gravity anchor "
                    "correction rate in degrees per second."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_invalid_correction_hold_frames",
                default_value="2",
                description=(
                    "Reuse the last valid stabilization homography for this "
                    "many consecutive camera/IMU matching misses."
                ),
            ),
            DeclareLaunchArgument(
                "bev_input_bottom_fraction",
                default_value=_PARAMETER_FILE_DEFAULT,
                description=(
                    "Bottom fraction of the rectified camera frame sent to "
                    "the fused CUDA stabilization/BEV path."
                ),
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value=_PARAMETER_FILE_DEFAULT,
                description="Show or completely disable the BEV GUI preview.",
            ),
            DeclareLaunchArgument(
                "camera_preview_enabled",
                default_value="false",
                description="Show the stabilized rectified RGB camera preview.",
            ),
            DeclareLaunchArgument(
                "camera_publish_enabled",
                default_value="false",
                description="Publish the rectified RGB image topic.",
            ),
            DeclareLaunchArgument(
                "camera_publish_fps",
                default_value="80.0",
                description="Maximum rectified RGB publication rate.",
            ),
            DeclareLaunchArgument(
                "bev_interpolation",
                default_value=_PARAMETER_FILE_DEFAULT,
                description="Interpolation used by the CUDA NV12-to-BEV warp.",
            ),
            *runtime_launch_arguments,
            OpaqueFunction(
                function=_apply_parameter_file_defaults,
                kwargs={
                    "bev_params": bev_params,
                    "launch_parameter_defaults": launch_parameter_defaults,
                },
            ),
            ComposableNodeContainer(
                additional_env={"LD_LIBRARY_PATH": LaunchConfiguration("depthai_runtime_library_path")},
                name="bev_processor_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    # BEV가 먼저 OAK를 단독으로 열어 roll·pitch와 자동 모드의
                    # 높이를 측정하고 장치를 닫은 뒤 camera_driver가 시작된다.
                    ComposableNode(
                        package="bev_processor",
                        plugin="bev_processor::BevProcessorNode",
                        name="bev_processor",
                        parameters=[
                            bev_params,
                            LaunchConfiguration("bev_params_file"),
                            LaunchConfiguration("perception_params_file"),
                            {
                                "startup_inspection_enabled": ParameterValue(
                                    LaunchConfiguration("startup_inspection_enabled"),value_type=bool),
                                "obstacles.share_reference": ParameterValue(PythonExpression([
                                    "'", LaunchConfiguration("obstacles_enabled"), "' == 'true' or '",
                                    LaunchConfiguration("tunnel_enabled"), "' == 'true'"]), value_type=bool),
                                "tunnel.enabled": ParameterValue(LaunchConfiguration("tunnel_enabled"), value_type=bool),
                                "startup_preview_image_enabled": ParameterValue(
                                    LaunchConfiguration("startup_preview_image_enabled"), value_type=bool),
                                "preview_enabled": ParameterValue(
                                    preview_enabled,
                                    value_type=bool,
                                ),
                                "input_bottom_fraction": ParameterValue(
                                    bev_input_bottom_fraction,
                                    value_type=float,
                                ),
                                "bev_interpolation": ParameterValue(
                                    bev_interpolation,
                                    value_type=str,
                                ),
                            },
                            runtime_parameter_overrides,
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    ),
                    # 하단 raw NV12와 보정 행렬을 전달한다. CUDA가 안정화와
                    # BEV를 한 번에 수행하므로 전체 CPU warp는 실행하지 않는다.
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            LaunchConfiguration("camera_params_file"),
                            LaunchConfiguration("perception_params_file"),
                            {
                                "obstacles.depth.enabled": ParameterValue(LaunchConfiguration("obstacles_enabled"), value_type=bool),
                                "tunnel.enabled": ParameterValue(LaunchConfiguration("tunnel_enabled"), value_type=bool),
                                "tunnel.validation_mode": ParameterValue(LaunchConfiguration("tunnel_validation_mode"), value_type=bool),
                                "tunnel.ir_imu_enabled": ParameterValue(LaunchConfiguration("tunnel_ir_imu_enabled"), value_type=bool),
                                "imu_stabilization_measured_erpm_topic": LaunchConfiguration("measured_erpm_topic"),
                                "imu_stabilization_can_acceleration_topic": LaunchConfiguration("vehicle_acceleration_topic"),
                                "preview_enabled": ParameterValue(
                                    camera_preview_enabled,
                                    value_type=bool,
                                ),
                                "publish_enabled": ParameterValue(
                                    camera_publish_enabled,
                                    value_type=bool,
                                ),
                                "publish_fps": ParameterValue(
                                    camera_publish_fps,
                                    value_type=float,
                                ),
                                "fused_bev_output_enabled": True,
                                "bev_input_bottom_fraction": ParameterValue(
                                    bev_input_bottom_fraction,
                                    value_type=float,
                                ),
                                # CAN dynamics uses this yaw rate for ay.
                                "imu_bridge_enabled": True,
                                "imu_stabilization_enabled": (
                                    imu_stabilization_parameter
                                ),
                                "imu_stabilization_high_frequency_only": (
                                    high_frequency_only_parameter
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_cutoff_hz"
                                ): high_frequency_cutoff_parameter,
                                (
                                    "imu_stabilization_gyroscope_"
                                    "correction_gain"
                                ): gyroscope_correction_gain_parameter,
                                (
                                    "imu_stabilization_can_longitudinal_"
                                    "compensation_gain"
                                ): can_longitudinal_compensation_parameter,
                                (
                                    "imu_stabilization_can_lateral_"
                                    "compensation_gain"
                                ): can_lateral_compensation_parameter,
                                (
                                    "imu_stabilization_moving_"
                                    "accelerometer_nudge_strength"
                                ): (
                                    moving_accelerometer_nudge_strength_parameter
                                ),
                                (
                                    "imu_stabilization_moving_gravity_anchor_"
                                    "maximum_correction_rate_degps"
                                ): moving_gravity_anchor_rate_parameter,
                                (
                                    "imu_stabilization_invalid_correction_"
                                    "hold_frames"
                                ): invalid_correction_hold_frames_parameter,
                                (
                                    "imu_stabilization_external_reference_required"
                                ): True,
                                "output_crop_top_px": 0,
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
