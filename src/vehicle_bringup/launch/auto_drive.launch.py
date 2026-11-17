import os
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


_PARAMETER_FILE_DEFAULT = "__PARAMETER_FILE_DEFAULT__"

# Omitted arguments preserve external test YAML values.
_DETECTOR_POSTPROCESS_OVERRIDES = {
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
}


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


def _apply_speed_override(context, speed):
    """Keep the integrated target-speed shorthand on the normal auto launch."""
    if speed == "inherit":
        return
    value = float(speed)
    if not math.isfinite(value) or value <= 0:
        raise ValueError("speed_mps must be a finite positive number")
    for name in ("minimum_speed_mps", "maximum_speed_mps"):
        context.launch_configurations[name] = str(value)


def _configure_handover(context, controller_defaults, namespace):
    """Route AUTO through the manual controller's one-way handover gate."""
    config = context.launch_configurations
    channels = ("duty", "brake_current", "servo_position")
    for name in (*[f"{channel}_topic" for channel in channels],
                 "measured_erpm_topic", "connection_status_topic"):
        config["bridge_" + name] = config["effective_" + name]
    config["dynamics_commanded_duty_topic"] = str(
        controller_defaults.get("command_duty_topic", "/auto/current_duty"))
    enabled = LaunchConfiguration("manual_handover_enabled").perform(context).lower()
    if enabled not in ("true", "false"):
        raise RuntimeError("manual_handover_enabled must be true or false")
    if (enabled != "true" or config["manual_test"].lower() == "true"
            or config["auto_control_mode"] != "drive"):
        return []
    prefix = f"/{namespace}" if namespace else ""
    # AUTO and the bridge run at the root. Preserve their relative topic
    # resolution when the handover node itself is in the vehicle namespace.
    def root_topic(value):
        return "/" + str(value).lstrip("/")

    parameters = {
        "joy_topic": prefix + "/joy",
        "auto_enable_topic": root_topic(controller_defaults.get("enable_topic", "/auto/enabled")),
        "measured_erpm_topic": root_topic(config["bridge_measured_erpm_topic"]),
        "connection_status_topic": root_topic(config["bridge_connection_status_topic"]),
        "current_duty_topic": prefix + "/drive/current_duty",
        "current_brake_current_topic": prefix + "/drive/current_brake_current",
        "gear_state_topic": prefix + "/manual/gear",
        "drive_mode_topic": prefix + "/drive/mode",
    }
    for channel in channels:
        input_topic = prefix + f"/handover/auto/{channel}"
        parameters[f"auto_{channel}_topic"] = input_topic
        parameters[f"{channel}_topic"] = root_topic(config[f"bridge_{channel}_topic"])
        config[f"effective_{channel}_topic"] = input_topic
    config["dynamics_commanded_duty_topic"] = parameters["current_duty_topic"]
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("joy_initializer"), "launch", "joy.launch.py")),
            launch_arguments={
                "vehicle_namespace": namespace,
                "device_name_contains": LaunchConfiguration("controller_name_contains").perform(context),
                "reconnect_interval_sec": "0.2",
            }.items(),
        ),
        Node(
            package="manual_control", executable="auto_handover_node",
            name="auto_handover", namespace=namespace, output="screen",
            parameters=[os.path.join(get_package_share_directory("vehicle_bringup"),
                                     "config", "manual_vesc_config.yaml"), parameters],
        ),
    ]


def _apply_parameter_file_defaults(
    context,
    *,
    bev_config,
    auto_control_config,
    bev_arguments,
    controller_arguments,
    line_detactor_config,
    bev_launch,
):
    bev_defaults = _merged_parameters(
        bev_config,
        LaunchConfiguration("bev_params_file").perform(context),
        "bev_processor",
    )
    controller_defaults = _merged_parameters(
        auto_control_config,
        LaunchConfiguration("auto_control_params_file").perform(context),
        "auto_control",
    )
    detector = _merged_parameters(
        line_detactor_config,
        LaunchConfiguration("line_detactor_params_file").perform(context),
        "line_detactor",
    )
    _apply_speed_override(context, LaunchConfiguration("speed_mps").perform(context))
    manual_test = LaunchConfiguration("manual_test").perform(context).lower()
    obstacles_enabled = LaunchConfiguration("obstacles_enabled").perform(context).lower()
    if manual_test not in ("true", "false") or obstacles_enabled not in ("true", "false"):
        raise RuntimeError("manual_test / obstacles_enabled must be true or false")
    obstacle_file = os.path.abspath(os.path.expanduser(LaunchConfiguration("obstacle_params_file").perform(context)))
    context.launch_configurations["obstacle_params_file"] = obstacle_file
    context.launch_configurations["effective_perception_params_file"] = (
        obstacle_file if obstacles_enabled == "true"
        else LaunchConfiguration("bev_params_file").perform(context)
    )
    if obstacles_enabled == "true":
        for section in ("camera_driver", "bev_processor", "auto_obstacles", "line_detactor"):
            values = _ros_parameters(obstacle_file, section)
            if not values or any(not key.startswith("obstacles.") for key in values):
                raise RuntimeError(f"{obstacle_file}: {section} must contain only obstacles.* parameters")
        detector.update(_ros_parameters(obstacle_file, "line_detactor"))
        detector["bev_theme_enable"] = False
    detector["obstacles.overlay_enabled"] = obstacles_enabled == "true"
    avoidance = LaunchConfiguration("avoidance_test_enabled").perform(context)
    if avoidance == _PARAMETER_FILE_DEFAULT:
        avoidance = _launch_default(
            _ros_parameters(obstacle_file, "auto_obstacles") if obstacles_enabled == "true" else {},
            "obstacles.avoidance.enabled", False)
    if avoidance.lower() not in ("true", "false"):
        raise RuntimeError("avoidance_test_enabled must be true or false")
    avoidance = avoidance.lower() == "true"
    if avoidance and obstacles_enabled != "true":
        raise RuntimeError("avoidance_test_enabled requires obstacles_enabled:=true")
    apply_avoidance = LaunchConfiguration("avoidance_control_enabled").perform(context)
    if apply_avoidance == _PARAMETER_FILE_DEFAULT:
        apply_avoidance = _launch_default(controller_defaults, "avoidance_control_enabled", False)
    if apply_avoidance.lower() not in ("true", "false"):
        raise RuntimeError("avoidance_control_enabled must be true or false")
    apply_avoidance = apply_avoidance.lower() == "true"
    if apply_avoidance and obstacles_enabled != "true":
        raise RuntimeError("avoidance_control_enabled requires obstacles_enabled:=true")
    slowdown_enabled = controller_defaults.get("obstacle_slowdown_enabled", True)
    if not isinstance(slowdown_enabled, bool):
        raise RuntimeError("obstacle_slowdown_enabled must be a YAML boolean")
    if apply_avoidance or (slowdown_enabled and obstacles_enabled == "true"):
        avoidance = True
    context.launch_configurations["effective_avoidance_control_enabled"] = "true" if apply_avoidance else "false"
    deformation_only = controller_defaults.get("avoidance_deformation_only", True)
    if not isinstance(deformation_only, bool):
        raise RuntimeError("avoidance_deformation_only must be a YAML boolean")
    context.launch_configurations["effective_avoidance_deformation_only"] = str(deformation_only).lower()
    planner_overrides = {"obstacles.avoidance.enabled": avoidance,
                         "obstacles.avoidance.control_requested": apply_avoidance,
                         "obstacles.avoidance.deformation_only": deformation_only}
    reference_speed = LaunchConfiguration("maximum_speed_mps").perform(context)
    if reference_speed == _PARAMETER_FILE_DEFAULT:
        reference_speed = controller_defaults.get("maximum_speed_mps", 1.8)
    reference_speed = float(reference_speed)
    if not math.isfinite(reference_speed) or reference_speed <= 0:
        raise RuntimeError("maximum_speed_mps must be finite and positive")
    planner_overrides["obstacles.avoidance.reference_speed_mps"] = reference_speed
    # Share the normal controller's rules; do not invent separate departure gates.
    for key, default in {
        "path_minimum_points": 8, "path_minimum_span_m": .12,
        "path_minimum_x_m": .01, "path_maximum_x_m": 2.98,
        "path_maximum_gap_m": .15, "path_geometry_window_m": .16,
    }.items():
        planner_overrides["obstacles.avoidance." + key] = controller_defaults.get(key, default)
    if apply_avoidance:
        planner_config = _ros_parameters(obstacle_file, "auto_obstacles")
        wheelbase = float(controller_defaults.get("avoidance_wheelbase_m", .33))
        steering = float(controller_defaults.get("maximum_steering_angle_deg", 30.0))
        age = float(controller_defaults.get("avoidance_max_age_sec", .20))
        if not all(math.isfinite(v) for v in (wheelbase, steering, age)) or not (
            .1 <= wheelbase <= 1 and 0 < steering < 80 and .05 <= age <= .20):
            raise RuntimeError("Invalid avoidance control settings: wheelbase 0.1..1m, steering (0,80)deg, age 0.05..0.20s")
        planner_overrides.update({
            "obstacles.avoidance.max_curvature_per_m": min(.98*math.tan(math.radians(steering))/wheelbase,
                float(planner_config.get("obstacles.avoidance.max_curvature_per_m", 2.0))),
            "obstacles.avoidance.max_age_sec": min(age, float(planner_config.get("obstacles.avoidance.max_age_sec", .25))),
        })
    if manual_test == "true":
        # Keep inference/diagnostics active, suppress all automatic actuators.
        context.launch_configurations["auto_control_mode"] = "monitor_only"
        context.launch_configurations["auto_enabled"] = "true"
        context.launch_configurations["traffic_stop_enabled"] = "false"
        context.launch_configurations["dataset_collection_enabled"] = "false"
        context.launch_configurations["preview_result_only_enabled"] = "true"
    ns = LaunchConfiguration("vehicle_namespace").perform(context).strip("/")
    prefix = f"/{ns}" if ns else ""
    for name, fallback in (("measured_erpm_topic", "/vesc/measured_erpm"),
                           ("connection_status_topic", "/vesc/connected"),
                           ("duty_topic", "/vesc/duty"),
                           ("brake_current_topic", "/vesc/brake_current"),
                           ("servo_position_topic", "/vesc/servo_position")):
        context.launch_configurations["effective_" + name] = (
            prefix + fallback if manual_test == "true" else str(controller_defaults.get(name, fallback)))
    context.launch_configurations["perception_erpm_topic"] = (
        prefix + "/vesc/measured_erpm" if manual_test == "true" else _PARAMETER_FILE_DEFAULT)
    context.launch_configurations["perception_acceleration_topic"] = (
        prefix + "/vehicle/dynamics/acceleration" if manual_test == "true" else _PARAMETER_FILE_DEFAULT)
    context.launch_configurations["ml_engine_precision"] = str(
        detector.get("engine_precision", "fp32")
    )
    context.launch_configurations["ml_model_path"] = str(
        detector.get("model_path", os.path.join(
            get_package_share_directory("line_detactor"), "models",
            "fast_scnn_stop_line_120x300_batch_1.onnx",
        ))
    )
    # One physical geometry and result topic for producer and consumer. Old BEV
    # lane parameters in external YAML cannot reactivate the removed detector.
    x_min, x_max = float(bev_defaults["x_min_m"]), float(bev_defaults["x_max_m"])
    y_min, y_max = float(bev_defaults["y_min_m"]), float(bev_defaults["y_max_m"])
    mpp = float(bev_defaults["meter_per_pixel"])
    if not all(math.isfinite(v) for v in (x_min, x_max, y_min, y_max, mpp)) or mpp <= 0:
        raise RuntimeError("BEV geometry must be finite with positive meter_per_pixel")
    if abs(x_min) > 1e-6 or not math.isclose(y_min, -y_max, abs_tol=1e-6) or min(x_max, y_max) <= 0:
        raise RuntimeError("ML auto drive requires front-axle BEV: x_min=0, symmetric +/-Y")
    width, height = int(bev_defaults["output_width"]), int(bev_defaults["output_height"])
    if width != round((y_max-y_min)/mpp) or height != round((x_max-x_min)/mpp):
        raise RuntimeError("BEV output size and metric extent disagree")
    if width != int(detector["model_input_width"]) or height != int(detector["model_input_height"]):
        raise RuntimeError("BEV output size must match the static line_detactor model input")
    if obstacles_enabled == "true" and not (
        math.isclose(x_max, 3.0) and math.isclose(y_max-y_min, 1.2) and width == 120 and height == 300):
        raise RuntimeError("Obstacle integration retains the existing 120x300cm / 120x300px lane BEV")
    for required in ("connection_enabled", "centerline_enabled", "result_publish_enabled"):
        if detector.get(required) is not True:
            raise RuntimeError(f"ML auto drive requires line_detactor {required}: true")
    detector.update({
        "input_topic": str(bev_defaults["output_topic"]),
        "direct_bev_input_enabled": True,
        "centerline_bev_width_m": y_max-y_min,
        "centerline_bev_height_m": x_max-x_min,
    })
    context.launch_configurations["ml_lane_result_topic"] = str(detector["result_topic"])
    context.launch_configurations["ml_lane_frame_id"] = str(bev_defaults["output_frame_id"])
    context.launch_configurations["ml_bev_x_max_m"] = str(x_max)
    context.launch_configurations["ml_bev_y_max_m"] = str(y_max)
    context.launch_configurations["ml_bev_meter_per_pixel"] = str(mpp)
    if (
        context.launch_configurations["preview_enabled"]
        == _PARAMETER_FILE_DEFAULT
    ):
        context.launch_configurations["preview_enabled"] = _launch_default(
            detector, "preview_enabled", "true"
        )
    for name, fallback in bev_arguments:
        if context.launch_configurations[name] == _PARAMETER_FILE_DEFAULT:
            context.launch_configurations[name] = _launch_default(
                bev_defaults, name, fallback
            )
    for argument_name, fallback, parameter_name, _ in controller_arguments:
        if (
            context.launch_configurations[argument_name]
            == _PARAMETER_FILE_DEFAULT
        ):
            context.launch_configurations[argument_name] = _launch_default(
                controller_defaults, parameter_name, fallback
            )
    detector["control_latency_topic"] = context.launch_configurations[
        "input_to_control_decision_topic"
    ]
    detector["bev_theme_speed_topic"] = str(
        controller_defaults.get("current_speed_topic", "/auto/current_speed")
    )
    detector["preview_target_speed_topic"] = str(
        controller_defaults.get("target_speed_topic", "/auto/target_speed")
    )
    detector["preview_corner_state_topic"] = str(
        controller_defaults.get("corner_state_topic", "/auto/corner_state")
    )
    theme = LaunchConfiguration("bev_theme_enable").perform(context)
    if theme != _PARAMETER_FILE_DEFAULT:
        if theme.lower() not in ("true", "false"):
            raise RuntimeError("bev_theme_enable must be true or false")
        detector["bev_theme_enable"] = theme.lower() == "true"
    if obstacles_enabled == "true" and detector.get("bev_theme_enable", False):
        raise RuntimeError("Obstacle overlay uses the existing lane result view; set bev_theme_enable:=false")
    preview = context.launch_configurations["preview_enabled"].lower()
    if preview not in ("true", "false"):
        raise RuntimeError("preview_enabled must be true or false")
    detector["preview_enabled"] = preview == "true"
    remote = LaunchConfiguration("remote_preview_enabled").perform(context).lower()
    if remote not in ("true", "false"):
        raise RuntimeError("remote_preview_enabled must be true or false")
    # START approval stays mandatory in both local and remote modes.
    context.launch_configurations["startup_inspection_active"] = "true"
    context.launch_configurations["startup_drive_mode_topic"] = prefix + "/drive/mode"
    context.launch_configurations["startup_preview_image_enabled"] = "false" if remote == "true" else "true"
    if remote == "true":
        # SSH/headless: no Jetson-side windows or image topics; the browser draws the preview.
        detector["preview_enabled"] = False
        detector["preview_result_only_enabled"] = True
        detector["bev_theme_enable"] = False
        preview = "false"
    for name, value_type in _DETECTOR_POSTPROCESS_OVERRIDES.items():
        value = LaunchConfiguration(name).perform(context)
        if value == _PARAMETER_FILE_DEFAULT:
            continue
        if value_type is bool:
            if value.lower() not in ("true", "false"):
                raise RuntimeError(f"{name} must be true or false")
            detector[name] = value.lower() == "true"
        else:
            detector[name] = value_type(value)
    profiling = LaunchConfiguration("profiling_enabled").perform(context)
    if profiling != _PARAMETER_FILE_DEFAULT:
        if profiling.lower() not in ("true", "false"):
            raise RuntimeError("profiling_enabled must be true or false")
        detector["profiling_enabled"] = profiling.lower() == "true"
    profiling_directory = LaunchConfiguration("profiling_directory").perform(context)
    if profiling_directory != _PARAMETER_FILE_DEFAULT:
        detector["profiling_directory"] = profiling_directory
    result_only = LaunchConfiguration("preview_result_only_enabled").perform(context)
    if remote == "true":
        result_only = _PARAMETER_FILE_DEFAULT
    if result_only != _PARAMETER_FILE_DEFAULT:
        if result_only.lower() not in ("true", "false"):
            raise RuntimeError("preview_result_only_enabled must be true or false")
        detector["preview_result_only_enabled"] = result_only.lower() == "true"
    for parameter_name in (
        "centerline_corner_outward_offset_m",
        "centerline_corner_entry_distance_m",
        "centerline_corner_entry_full_offset_distance_m",
    ):
        value = LaunchConfiguration(parameter_name).perform(context)
        if value != _PARAMETER_FILE_DEFAULT:
            detector[parameter_name] = float(value)
    context.launch_configurations["direct_host_copy_enabled"] = (
        "true" if detector["preview_enabled"] and not detector.get(
            "preview_result_only_enabled", True
        ) else "false"
    )
    nodes = [ComposableNode(
        package="line_detactor", plugin="line_detactor::LineDetactorNode",
        name="line_detactor", parameters=[detector],
        extra_arguments=[{"use_intra_process_comms": True}],
    )]
    tunnel_actions = []
    tunnel = LaunchConfiguration("tunnel_enabled").perform(context).lower()
    if tunnel not in ("true", "false"):
        raise RuntimeError("tunnel_enabled must be true or false")
    if tunnel == "true":
        if not (width == 120 and height == 300 and math.isclose(x_max, 3.0) and
                math.isclose(y_max, .6) and math.isclose(mpp, .01) and
                bev_defaults["output_frame_id"] == "front_axle_bev"):
            raise RuntimeError("Tunnel IR model requires 120x300 front-axle BEV at 1cm/pixel")
        from auto_control_tunnel.launch_support import integrated_nodes
        components, tunnel_actions = integrated_nodes(detector, controller_defaults, context)
        # Recreate RGB description: ComposableNode normalizes/copies parameters.
        nodes[0] = ComposableNode(
            package="line_detactor", plugin="line_detactor::LineDetactorNode",
            name="line_detactor", parameters=[detector],
            extra_arguments=[{"use_intra_process_comms": True}])
        nodes.extend(components)
        context.launch_configurations["ml_lane_result_topic"] = "/tunnel/lane_result"
    traffic_enabled = LaunchConfiguration("traffic_light_enabled").perform(context).lower()
    if traffic_enabled not in ("true", "false"):
        raise RuntimeError("traffic_light_enabled must be true or false")
    if traffic_enabled == "true":
        traffic_parameters = [LaunchConfiguration("traffic_light_params_file").perform(context)]
        traffic_fps = LaunchConfiguration("traffic_light_inference_fps").perform(context)
        if traffic_fps != _PARAMETER_FILE_DEFAULT:
            traffic_parameters.append({"inference_fps": float(traffic_fps)})
        nodes.append(ComposableNode(
            package="traffic_detection_test", plugin="traffic_detection_test::TrafficLightNode",
            name="traffic_light_detector", parameters=traffic_parameters,
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
    if obstacles_enabled == "true":
        nodes.append(ComposableNode(
            package="auto_control", plugin="auto_control::ObstacleDetectorNode", name="auto_obstacles",
            parameters=[obstacle_file, planner_overrides],
            extra_arguments=[{"use_intra_process_comms": True}],
        ))
    manual_actions = _configure_handover(context, controller_defaults, ns)
    if manual_test == "true":
        for flag in ("manual_enabled", "dynamics_enabled"):
            if LaunchConfiguration(flag).perform(context).lower() not in ("true", "false"):
                raise RuntimeError(f"{flag} must be true or false")
        if LaunchConfiguration("manual_enabled").perform(context).lower() == "true":
            manual_actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("vehicle_bringup"), "launch", "manual_drive.launch.py")),
                launch_arguments={"vehicle_namespace": ns,
                                  "vesc_port": LaunchConfiguration("vesc_port").perform(context),
                                  "controller_name_contains": LaunchConfiguration("controller_name_contains").perform(context)}.items()))
        if LaunchConfiguration("dynamics_enabled").perform(context).lower() == "true":
            manual_actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("vehicle_dynamics_monitor"), "launch", "vehicle_dynamics_monitor.launch.py")),
                launch_arguments={"vehicle_namespace": ns, **{name: LaunchConfiguration(name).perform(context)
                    for name in ("input_mode", "can_interface", "slcan_channel", "slcan_bitrate", "can_controller_id")}}.items()))
    from auto_control_tunnel.launch_support import model_check_node
    startup_check = model_check_node(tunnel == "true", traffic_enabled == "true")
    inspection_actions = []
    connection_topic = '/' + context.launch_configurations['bridge_connection_status_topic'].strip('/')
    if remote == "true":
        from auto_control_tunnel.launch_support import remote_preview_node
        inspection_actions.append(remote_preview_node(
            tunnel == "true", traffic_enabled == "true",
            LaunchConfiguration("remote_preview_params_file").perform(context),
            float(detector.get("centerline_lane_width_m", 0.65)),
            connection_topic, connection_topic + '/diagnostics'))
    else:
        from auto_control_tunnel.launch_support import startup_inspection_node
        inspection_actions.append(startup_inspection_node(tunnel == "true", traffic_enabled == "true",
            '/' + context.launch_configurations['bridge_connection_status_topic'].strip('/') + '/diagnostics'))
        # Tools belong to the inspection controls, not the raw BEV preview.
        # Start them before including BEV's launch, which sets preview_enabled=false.
        inspection_actions.append(Node(
            package="auto_control", executable="runtime_tools", name="auto_runtime_tools",
            output="screen"))
    return [LogInfo(msg=(
        "[ML auto drive] launch=" + os.path.realpath(__file__) +
        " | BEV=" + LaunchConfiguration("bev_params_file").perform(context) +
        " | detector_yaml=" + LaunchConfiguration("line_detactor_params_file").perform(context) +
        " | model=" + str(detector.get("model_path", os.path.join(
            get_package_share_directory("line_detactor"), "models",
            "fast_scnn_stop_line_120x300_batch_1.onnx"))) +
        " | pipeline=direct CUDA BEV -> " + str(detector["result_topic"]) +
        " -> auto_control | ML preview=" + preview + " | raw BEV preview=false"
        " | remote_preview=" + remote +
        " | traffic observation=" + traffic_enabled + " | traffic stop=" + context.launch_configurations["traffic_stop_enabled"] +
        " | obstacles=" + obstacles_enabled + " | obstacle_yaml=" + obstacle_file +
        " | avoidance_preview=" + str(avoidance) + " | avoidance_control=" + str(apply_avoidance) +
        " | deformation_only=" + str(deformation_only) +
        " | tunnel_ir_imu=" + LaunchConfiguration("tunnel_ir_imu_enabled").perform(context) +
        " | tunnel_validation=" + LaunchConfiguration("tunnel_validation_mode").perform(context) +
        " | manual_test=" + manual_test + " | control_mode=" + context.launch_configurations["auto_control_mode"]
    )), startup_check, *inspection_actions, *manual_actions, *tunnel_actions, bev_launch, LoadComposableNodes(
        target_container="/bev_processor_container",
        composable_node_descriptions=nodes,
    )]


def generate_launch_description():
    bev_share = get_package_share_directory("bev_processor")
    auto_control_share = get_package_share_directory("auto_control")
    vehicle_bringup_share = get_package_share_directory("vehicle_bringup")
    bev_config = os.path.join(bev_share, "config", "bev_config.yaml")
    line_detactor_config = os.path.join(get_package_share_directory("line_detactor"), "config", "line_detactor.yaml")
    camera_config = os.path.join(get_package_share_directory("camera_driver"), "config", "camera_config.yaml")
    dynamics_config = os.path.join(get_package_share_directory("vehicle_dynamics_monitor"), "config", "vehicle_dynamics.yaml")
    auto_control_config = os.path.join(
        auto_control_share, "config", "auto_control.yaml"
    )
    vesc_config = os.path.join(
        vehicle_bringup_share, "config", "vesc_config.yaml"
    )
    bev_defaults = _ros_parameters(bev_config, "bev_processor")
    controller_defaults = _ros_parameters(auto_control_config, "auto_control")

    vesc_port = LaunchConfiguration("vesc_port")
    bev_params_file = LaunchConfiguration("bev_params_file")
    auto_control_params_file = LaunchConfiguration("auto_control_params_file")
    bev_argument_fallbacks = [
        ("dataset_collection_enabled", "false"),
        ("dataset_collection_root_directory", "datasets"),
        ("dataset_collection_fps", "10.0"),
        ("dataset_collection_target_count", "1000"),
        ("dataset_collection_stop_auto_on_complete", "true"),
    ]
    # YAML is the single source of default values. A value supplied through
    # `ros2 launch ... name:=value` still replaces the declared default.
    bev_arguments = [
        (name, _launch_default(bev_defaults, name, fallback))
        for name, fallback in bev_argument_fallbacks
    ]
    bev_overrides = {
        name: LaunchConfiguration(name) for name, _ in bev_arguments
    }
    controller_argument_fallbacks = [
        ("auto_enabled", "true", "enabled", bool),
        ("auto_control_mode", "drive", "control_mode", str),
        (
            "input_to_control_decision_topic",
            "/auto/detector_input_to_control_decision_ms",
            "input_to_control_decision_topic",
            str,
        ),
        ("minimum_duty", "0.070", "minimum_duty", float),
        ("maximum_duty", "0.090", "maximum_duty", float),
        ("longitudinal_start_duty", "0.05", "longitudinal_start_duty", float),
        ("traffic_stop_range_max_m", "0.30", "traffic_stop_range_max_m", float),
        ("longitudinal_recovery_start_ramp_sec", "0.20", "longitudinal_recovery_start_ramp_sec", float),
        ("traffic_brake_urgent_rise_amps_per_sec", "6.0", "traffic_brake_urgent_rise_amps_per_sec", float),
        (
            "duty_rise_rate_per_sec",
            "0.04",
            "duty_rise_rate_per_sec",
            float,
        ),
        (
            "duty_fall_rate_per_sec",
            "0.08",
            "duty_fall_rate_per_sec",
            float,
        ),
        ("minimum_speed_mps", "0.80", "minimum_speed_mps", float),
        ("maximum_speed_mps", "1.8", "maximum_speed_mps", float),
        ("stanley_gain", "1.40", "stanley_gain", float),
        (
            "stanley_softening_speed_mps",
            "0.40",
            "stanley_softening_speed_mps",
            float,
        ),
        (
            "stanley_heading_lookahead_m",
            "0.15",
            "stanley_heading_lookahead_m",
            float,
        ),
        (
            "stanley_corner_heading_threshold_deg",
            "4.0",
            "stanley_corner_heading_threshold_deg",
            float,
        ),
        (
            "stanley_corner_opposing_correction_ratio",
            "0.45",
            "stanley_corner_opposing_correction_ratio",
            float,
        ),
        (
            "steering_current_weight",
            "0.47",
            "steering_current_weight",
            float,
        ),
        (
            "steering_servo_inverted",
            "true",
            "steering_servo_inverted",
            bool,
        ),
        (
            "maximum_lateral_acceleration_mps2",
            "0.6",
            "maximum_lateral_acceleration_mps2",
            float,
        ),
        ("curvature_percentile", "90.0", "curvature_percentile", float),
        ("speed_pid_kp", "0.012", "speed_pid_kp", float),
        ("speed_pid_ki", "0.004", "speed_pid_ki", float),
        ("speed_pid_kd", "0.0", "speed_pid_kd", float),
        (
            "speed_pid_integral_limit",
            "1.0",
            "speed_pid_integral_limit",
            float,
        ),
        (
            "speed_filter_time_constant_sec",
            "0.05",
            "speed_filter_time_constant_sec",
            float,
        ),
        (
            "electrical_brake_enabled",
            "true",
            "electrical_brake_enabled",
            bool,
        ),
        (
            "brake_entry_speed_error_mps",
            "0.10",
            "brake_entry_speed_error_mps",
            float,
        ),
        (
            "brake_exit_speed_error_mps",
            "0.03",
            "brake_exit_speed_error_mps",
            float,
        ),
        (
            "brake_minimum_vehicle_speed_mps",
            "0.20",
            "brake_minimum_vehicle_speed_mps",
            float,
        ),
        (
            "brake_minimum_current_amps",
            "0.5",
            "brake_minimum_current_amps",
            float,
        ),
        (
            "brake_maximum_current_amps",
            "2.5",
            "brake_maximum_current_amps",
            float,
        ),
        (
            "brake_current_gain_amps_per_mps",
            "6.0",
            "brake_current_gain_amps_per_mps",
            float,
        ),
        (
            "brake_current_rise_amps_per_sec",
            "6.0",
            "brake_current_rise_amps_per_sec",
            float,
        ),
        (
            "brake_current_fall_amps_per_sec",
            "16.0",
            "brake_current_fall_amps_per_sec",
            float,
        ),
        ("path_maximum_gap_m", "0.15", "path_maximum_gap_m", float),
        (
            "path_geometry_window_m",
            "0.14",
            "path_geometry_window_m",
            float,
        ),
        (
            "path_minimum_x_m",
            "0.05",
            "path_minimum_x_m",
            float,
        ),
        (
            "path_minimum_points",
            "8",
            "path_minimum_points",
            int,
        ),
        (
            "path_minimum_span_m",
            "0.12",
            "path_minimum_span_m",
            float,
        ),
    ]
    controller_argument_fallbacks.extend([
        ("traffic_stop_enabled", "false", "traffic_stop_enabled", bool),
        ("curvature_speed_control_enabled", "false", "curvature_speed_control_enabled", bool),
        ("longitudinal_pid_enabled", "true", "longitudinal_pid_enabled", bool),
    ])
    controller_arguments = [
        (
            argument_name,
            _launch_default(controller_defaults, parameter_name, fallback),
            parameter_name,
            value_type,
        )
        for (
            argument_name,
            fallback,
            parameter_name,
            value_type,
        ) in controller_argument_fallbacks
    ]
    controller_overrides = {
        parameter_name: ParameterValue(
            LaunchConfiguration(argument_name), value_type=value_type
        )
        for argument_name, _, parameter_name, value_type in controller_arguments
    }

    bev_launch_path = os.path.join(
        bev_share, "launch", "bev_processor.launch.py"
    )

    bev_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bev_launch_path),
        launch_arguments={
            "startup_inspection_enabled": LaunchConfiguration("startup_inspection_active"),
            "depthai_library_dir": LaunchConfiguration("depthai_library_dir"),
            "bev_params_file": bev_params_file,
            "camera_params_file": LaunchConfiguration("camera_params_file"),
            "obstacles_enabled": LaunchConfiguration("obstacles_enabled"),
            "tunnel_enabled": LaunchConfiguration("tunnel_enabled"),
            "tunnel_validation_mode": LaunchConfiguration("tunnel_validation_mode"),
            "tunnel_ir_imu_enabled": LaunchConfiguration("tunnel_ir_imu_enabled"),
            "perception_params_file": LaunchConfiguration("effective_perception_params_file"),
            "measured_erpm_topic": LaunchConfiguration("perception_erpm_topic"),
            "vehicle_acceleration_topic": LaunchConfiguration("perception_acceleration_topic"),
            "preview_enabled": "false",
            "publish_enabled": "false",
            "direct_output_enabled": "true",
            "direct_host_copy_enabled": LaunchConfiguration(
                "direct_host_copy_enabled"
            ),
            "startup_preview_image_enabled": LaunchConfiguration("startup_preview_image_enabled"),
            **bev_overrides,
        }.items(),
    )

    controller_overrides.update({
        "model_startup_check_required": True,
        "startup_safety_enabled": ParameterValue(LaunchConfiguration("startup_inspection_active"),value_type=bool),
        "startup_drive_mode_topic": LaunchConfiguration("startup_drive_mode_topic"),
        "tunnel.enabled": ParameterValue(LaunchConfiguration("tunnel_enabled"), value_type=bool),
        "avoidance_control_enabled": ParameterValue(LaunchConfiguration("effective_avoidance_control_enabled"), value_type=bool),
        "avoidance_deformation_only": ParameterValue(LaunchConfiguration("effective_avoidance_deformation_only"), value_type=bool),
        **{name: LaunchConfiguration("effective_" + name) for name in (
            "measured_erpm_topic", "connection_status_topic", "duty_topic", "brake_current_topic", "servo_position_topic")},
        "lane_result_topic": LaunchConfiguration("ml_lane_result_topic"),
        "lane_result_frame_id": LaunchConfiguration("ml_lane_frame_id"),
        "bev_x_max_m": ParameterValue(LaunchConfiguration("ml_bev_x_max_m"), value_type=float),
        "bev_y_max_m": ParameterValue(LaunchConfiguration("ml_bev_y_max_m"), value_type=float),
        "bev_meter_per_pixel": ParameterValue(LaunchConfiguration("ml_bev_meter_per_pixel"), value_type=float),
    })
    auto_control_node = Node(
        package="auto_control",
        executable="auto_control_node",
        name="auto_control",
        output="screen",
        parameters=[
            auto_control_config,
            auto_control_params_file,
            controller_overrides,
        ],
    )

    vesc_bridge_node = Node(
        package="vesc_bridge",
        executable="vesc_bridge_node",
        name="vesc_bridge_node",
        condition=UnlessCondition(LaunchConfiguration("manual_test")),
        output="screen",
        parameters=[vesc_config, {"port": vesc_port, **{
            name: LaunchConfiguration("bridge_" + name) for name in (
                "duty_topic", "brake_current_topic", "servo_position_topic",
                "measured_erpm_topic", "connection_status_topic")
        }}],
    )

    dynamics_node = Node(
        package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node",
        name="vehicle_dynamics_node", output="screen",
        condition=UnlessCondition(LaunchConfiguration("manual_test")),
        parameters=[dynamics_config, {
            "input_mode": LaunchConfiguration("input_mode"),
            "can_interface": LaunchConfiguration("can_interface"),
            "slcan_channel": LaunchConfiguration("slcan_channel"),
            "slcan_bitrate": ParameterValue(LaunchConfiguration("slcan_bitrate"), value_type=int),
            "can_controller_id": ParameterValue(LaunchConfiguration("can_controller_id"), value_type=int),
            "commanded_duty_topic": LaunchConfiguration("dynamics_commanded_duty_topic"),
            **{name: LaunchConfiguration("bridge_" + name) for name in (
                "measured_erpm_topic", "connection_status_topic", "servo_position_topic")},
        }],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("tunnel_enabled", default_value="false"),
            DeclareLaunchArgument("tunnel_ir_length_exit_enabled", default_value="false",
                description="Experimental IR observed-length exit trigger."),
            DeclareLaunchArgument("tunnel_ir_length_exit_ratio", default_value="0.15",
                description="Trigger when current/reference observed IR length is at or below this fraction."),
            DeclareLaunchArgument("tunnel_ir_length_exit_confirm_frames", default_value="3"),
            DeclareLaunchArgument("tunnel_ir_length_exit_minimum_reference_m", default_value="0.5"),
            DeclareLaunchArgument("tunnel_ir_imu_enabled", default_value="true",
                                  description="Apply capture-time IMU tilt to tunnel IR BEV; false uses fixed measured ground pose."),
            DeclareLaunchArgument("tunnel_validation_mode", default_value="true",
                                  description="Use standalone IR publication timing and bypass integration-only input gates."),
            DeclareLaunchArgument("depthai_library_dir", default_value="",
                                  description="Optional DepthAI runtime library directory; default uses build SDK."),
            DeclareLaunchArgument("speed_mps", default_value="inherit",
                                  description="Positive target m/s; inherit keeps selected controller YAML speed settings"),
            DeclareLaunchArgument("manual_test", default_value="false",
                                  description="Keep all perception and auto_control diagnostics; force monitor_only and use manual driving"),
            DeclareLaunchArgument("manual_enabled", default_value="true",
                                  description="In manual_test, start joystick/VESC; false when manual_drive already runs"),
            DeclareLaunchArgument("manual_handover_enabled", default_value="true",
                                  description="In drive mode, X brakes then switches to manual without closing perception/preview"),
            DeclareLaunchArgument("dynamics_enabled", default_value="true"),
            DeclareLaunchArgument("vehicle_namespace", default_value="autopilot03"),
            DeclareLaunchArgument("controller_name_contains", default_value="8BitDo"),
            DeclareLaunchArgument("obstacles_enabled", default_value="true"),
            DeclareLaunchArgument("avoidance_control_enabled", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Use obstacle-deformed centerline for steering. Default false; behavior follows avoidance_deformation_only in controller YAML"),
            DeclareLaunchArgument("avoidance_test_enabled", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Preview-only local avoidance candidates; defaults to obstacle YAML (false). Never commands actuators"),
            DeclareLaunchArgument("obstacle_params_file", default_value=os.path.join(auto_control_share, "config", "obstacles.yaml")),
            DeclareLaunchArgument("camera_params_file", default_value=camera_config),
            DeclareLaunchArgument("traffic_light_enabled", default_value="true",
                                  description="Observe traffic signal state only; does not command brakes or drive"),
            DeclareLaunchArgument("traffic_light_params_file", default_value=os.path.join(
                get_package_share_directory("traffic_detection_test"), "config", "traffic_light.yaml")),
            DeclareLaunchArgument("traffic_light_inference_fps", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Traffic inference cap; omitted uses YAML (20Hz)"),
            DeclareLaunchArgument("line_detactor_params_file", default_value=line_detactor_config,
                                  description="ML lane/centerline YAML; source geometry follows BEV YAML"),
            *[
                DeclareLaunchArgument(name, default_value=_PARAMETER_FILE_DEFAULT,
                                      description="Override lane postprocessing YAML parameter")
                for name in _DETECTOR_POSTPROCESS_OVERRIDES
            ],
            DeclareLaunchArgument("profiling_enabled", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Save detailed lane processing CSV without changing preview/control; omitted uses YAML"),
            DeclareLaunchArgument("profiling_directory", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Lane profile output directory; omitted uses YAML/node default"),
            DeclareLaunchArgument("preview_result_only_enabled", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Show lanes/path on black background; omitted uses ML YAML"),
            DeclareLaunchArgument("bev_theme_enable", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Lightweight white BEV theme; omitted uses ML YAML (false)"),
            DeclareLaunchArgument("centerline_corner_outward_offset_m", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Corner path shift toward observed outer lane in metres; 0 disables"),
            DeclareLaunchArgument("centerline_corner_entry_distance_m", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Extend outward shift before observed corners in metres; 0 uses local evidence only"),
            DeclareLaunchArgument("centerline_corner_entry_full_offset_distance_m", default_value=_PARAMETER_FILE_DEFAULT,
                                  description="Hold full observed corner offset confidence within this boundary distance; 0 keeps old ramp"),
            DeclareLaunchArgument("input_mode", default_value="ros_topic"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("slcan_channel", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("slcan_bitrate", default_value="500000"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            DeclareLaunchArgument(
                "remote_preview_enabled", default_value="false",
                description="SSH/headless mode: no Jetson previews; browser preview at http://<jetson-ip>:8765"),
            DeclareLaunchArgument(
                "remote_preview_params_file",
                default_value=os.path.join(get_package_share_directory("auto_control"), "config", "remote_preview.yaml"),
                description="remote_preview node YAML (port, frame rate, lane width, link-loss STOP)"),
            DeclareLaunchArgument(
                "bev_params_file",
                default_value=bev_config,
                description="BEV parameter override YAML",
            ),
            DeclareLaunchArgument(
                "auto_control_params_file",
                default_value=auto_control_config,
                description="Auto-control parameter override YAML",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value=_PARAMETER_FILE_DEFAULT,
                description=(
                    "Show ML lanes and yellow centerline preview. Set false for no GUI."
                ),
            ),
            *[
                DeclareLaunchArgument(
                    name, default_value=_PARAMETER_FILE_DEFAULT
                )
                for name, _ in bev_arguments
            ],
            *[
                DeclareLaunchArgument(
                    argument_name, default_value=_PARAMETER_FILE_DEFAULT
                )
                for argument_name, _, _, _ in controller_arguments
            ],
            OpaqueFunction(
                function=_apply_parameter_file_defaults,
                kwargs={
                    "bev_config": bev_config,
                    "auto_control_config": auto_control_config,
                    "bev_arguments": bev_arguments,
                    "controller_arguments": controller_arguments,
                    "line_detactor_config": line_detactor_config,
                    "bev_launch": bev_launch,
                },
            ),
            vesc_bridge_node,
            dynamics_node,
            auto_control_node,
        ]
    )
