from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load(path, node):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)[node]["ros__parameters"]


def telemetry_topic(override, configured, namespace):
    if override.strip():
        return override.strip()
    prefix = namespace.strip("/")
    return "/" + "/".join(part for part in (prefix, configured.strip("/")) if part)


def nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    # The attached ROI file predates the RANSAC/terrain options; retain defaults
    # for absent keys, then apply every explicitly supplied ROI value.
    roi = load(Path(get_package_share_directory("roi_lidar")) / "config/roi_lidar.yaml", "roi_lidar")
    roi.update(load(arg("roi_params_file"), "roi_lidar"))
    bev = load(arg("bev_params_file"), "bev_processor")
    camera = load(arg("camera_params_file"), "camera_driver")
    if (bev["x_min_m"], bev["x_max_m"], bev["y_min_m"], bev["y_max_m"],
        bev["meter_per_pixel"], bev["output_width"], bev["output_height"]) != (0, 3, -.6, .6, .01, 120, 300):
        raise ValueError("Shared lane BEV currently requires 0..3m, +/-0.6m, 120x300 at 1cm/pixel")
    if camera["camera_socket"] != "CAM_A" or camera["resize_mode"] != "CROP" or not camera["undistort_enabled"]:
        raise ValueError("ROI shared RGB requires undistorted CAM_A with CROP resize")
    imu_rate = int(arg("imu_rate_hz"))
    if not 1 <= imu_rate <= 1000:
        raise ValueError("imu_rate_hz must be in 1..1000")
    # RGB/depth rates follow the reference profiles; VFH requests a lower IMU rate.
    # Keep preview rendering separate from the lane input producer.
    rgb_rates = [float(camera["sensor_fps"]), float(camera["publish_fps"])]
    if float(bev["publish_max_fps"]) > 0:
        rgb_rates.append(float(bev["publish_max_fps"]))
    roi.update({"camera.fps": load(arg("obstacle_params_file"), "camera_driver")["obstacles.camera.fps"],
                "preview.rgb_fps": min(rgb_rates), "preview.fps": bev["preview_max_fps"],
                "preview.show_bev_window": False, "preview.publish_lane_bev": True, "motion.publish_imu": True,
                "preview.rgb_width_px": camera["width"], "preview.rgb_height_px": camera["height"],
                "motion.imu_rate_hz": imu_rate,
                "motion.imu_queue_size": camera["imu_queue_size"],
                "motion.imu_max_batch_reports": camera["imu_max_batch_reports"]})
    for key in ("camera_x_m", "camera_y_m", "camera_yaw_deg"):
        roi["bev." + key] = bev[key]
    roi.update({k: v for k, v in bev.items()
                if k.startswith("measurement_") or k.startswith("manual_camera_height_")})
    roi["frame_id"] = bev["output_frame_id"]
    lane = load(arg("lane_params_file"), "line_detactor")
    control = load(arg("vehicle_params_file"), "auto_control")
    obstacles = load(arg("obstacle_params_file"), "auto_obstacles")
    erpm_topic = telemetry_topic(arg("measured_erpm_topic"), control["measured_erpm_topic"], arg("vehicle_namespace"))
    connection_topic = telemetry_topic(arg("vesc_connection_topic"), control["connection_status_topic"], arg("vehicle_namespace"))
    test = load(arg("params_file"), "roi_vfh_test")
    drive_mode = arg("drive_enabled").strip().lower() or str(test.get("drive_enabled", False)).lower()
    if drive_mode not in ("true", "false"):
        raise ValueError("drive_enabled must be true or false")
    if drive_mode == "true" and (not control["enabled"] or control["control_mode"] != "drive"):
        raise ValueError("Vehicle profile must enable control_mode=drive, or launch with drive_enabled:=false")
    test.update({"drive_enabled": drive_mode == "true", "vesc_connection_topic": connection_topic,
                 "speed_ceiling_mps": control["maximum_speed_mps"],
                 "nominal_speed_mps": min(test["nominal_speed_mps"], control["maximum_speed_mps"]),
                 "wheelbase_m": test.get("wheelbase_m", control["avoidance_wheelbase_m"]),
                 "max_steering_deg": test.get("max_steering_deg", control["maximum_steering_angle_deg"]),
                 "steering_rate_deg_per_sec": test.get("steering_rate_deg_per_sec", control["steering_rate_limit_deg_per_sec"]),
                 "vehicle_width_m": test.get("vehicle_width_m", 0.30),
                 "rear_extent_m": test.get("rear_extent_m", 0.45),
                 "front_overhang_m": test.get("front_overhang_m", 0.05),
                 "safety_margin_m": test["safety_margin_m"],
                 "obstacle_uncertainty_m": test.get("obstacle_uncertainty_m", obstacles["obstacles.avoidance.unknown_extent_m"]),
                 "sample_step_m": test["sample_step_m"],
                 "max_curvature_per_m": test.get("max_curvature_per_m", obstacles["obstacles.avoidance.max_curvature_per_m"]),
                 "lane_lateral_acceleration_mps2": control["maximum_lateral_acceleration_mps2"],
                 "avoidance_lateral_acceleration_mps2": obstacles["obstacles.avoidance.lateral_acceleration_mps2"],
                 "curvature_speed_control_enabled": control["curvature_speed_control_enabled"],
                 "lane_width_m": lane["centerline_lane_width_m"],
                 "lane_lookahead_m": test.get("lane_lookahead_m", control["stanley_heading_lookahead_m"]),
                 "stanley_gain": test.get("stanley_gain", control["stanley_gain"]),
                 "stanley_softening_speed_mps": test.get("stanley_softening_speed_mps", control["stanley_softening_speed_mps"]),
                 "stanley_corner_heading_threshold_deg": test.get("stanley_corner_heading_threshold_deg", control["stanley_corner_heading_threshold_deg"]),
                 "stanley_corner_opposing_correction_ratio": test.get("stanley_corner_opposing_correction_ratio", control["stanley_corner_opposing_correction_ratio"]),
                 "steering_current_weight": test.get("steering_current_weight", control["steering_current_weight"]),
                 "control_rate_hz": control["control_rate_hz"],
                 "planning_rate_hz": obstacles["obstacles.avoidance.max_fps"],
                 "preview_fps": bev["preview_max_fps"],
                 "status_log_rate_hz": control["status_log_rate_hz"],
                 "minimum_path_length_m": control["path_minimum_span_m"],
                 "path_geometry_window_m": control["path_geometry_window_m"],
                 "lane_receipt_timeout_sec": max(control["path_timeout_sec"], test["max_age_sec"]),
                 "erpm_timeout_sec": control["erpm_timeout_sec"],
                 "max_lane_points": lane["centerline_max_samples"],
                 "obstacle_slowdown_enabled": control["obstacle_slowdown_enabled"],
                 "obstacle_slowdown_speed_mps": control["obstacle_slowdown_speed_mps"],
                 "obstacle_slowdown_clear_sec": control["obstacle_slowdown_clear_sec"],
                 "target_recovery_acceleration_mps2": test.get("target_recovery_acceleration_mps2", control["corner_planning_acceleration_mps2"]),
                 "driving_log_enabled": control["driving_log_enabled"],
                 "driving_log_rate_hz": control["driving_log_rate_hz"],
                 "driving_log_directory": control["driving_log_directory"],
                 "reference_profile": " | ".join(arg(name) for name in ("vehicle_params_file", "obstacle_params_file", "bev_params_file", "camera_params_file", "lane_params_file", "roi_params_file")),
                 "lane_result_topic": lane["result_topic"], "frame_id": bev["output_frame_id"],
                 "measured_speed_topic": arg("measured_speed_topic")})
    for key in ("duty_topic", "brake_current_topic", "servo_position_topic"):
        test[key] = telemetry_topic(arg(key), control[key], arg("vehicle_namespace"))
    # A single gate owns the real actuator topics. VFH always sends only to its
    # private inputs, including braking on perception/plan failures.
    actuator_topics = {key: test[key] for key in ("duty_topic", "brake_current_topic", "servo_position_topic")}
    handover = arg("manual_handover_enabled").strip().lower()
    if handover not in ("true", "false"):
        raise ValueError("manual_handover_enabled must be true or false")
    handover_enabled = handover == "true" and drive_mode == "true"
    test["manual_handover_enabled"] = handover_enabled
    handover_actions = []
    namespace = arg("vehicle_namespace").strip("/")
    prefix = "/" + namespace if namespace else ""
    actual_duty_log = "/roi_vfh_test/command_duty"
    if handover_enabled:
        test["manual_mode_request_topic"] = "/roi_vfh_test/drive_mode_request"
        test["manual_mode_ack_topic"] = "/roi_vfh_test/drive_mode_ack"
        test["handover_stop_brake_current_amps"] = control["brake_maximum_current_amps"]
        gate = {**actuator_topics, "joy_topic": prefix + "/joy",
                "measured_erpm_topic": erpm_topic, "connection_status_topic": connection_topic,
                "mode_request_topic": test["manual_mode_request_topic"],
                "mode_ack_topic": test["manual_mode_ack_topic"],
                "drive_mode_topic": prefix + "/drive/mode",
                "current_duty_topic": prefix + "/manual/current_duty",
                "current_brake_current_topic": prefix + "/manual/current_brake_current",
                "gear_state_topic": prefix + "/manual/gear",
                "auto_command_timeout_sec": test["max_age_sec"],
                "handover_stop_brake_current_amps": test["handover_stop_brake_current_amps"]}
        actual_duty_log = gate["current_duty_topic"]
        for channel in ("duty", "brake_current", "servo_position"):
            test[channel + "_topic"] = prefix + "/handover/vfh/" + channel
            gate["auto_" + channel + "_topic"] = test[channel + "_topic"]
        handover_actions = [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("joy_initializer")) / "launch/joy.launch.py")),
                launch_arguments={"vehicle_namespace": namespace,
                                  "device_name_contains": arg("controller_name_contains"),
                                  "reconnect_interval_sec": "0.2"}.items()),
            Node(package="manual_control", executable="vfh_handover_node", name="roi_vfh_handover",
                 namespace=namespace, output="screen", parameters=[str(
                     Path(get_package_share_directory("vehicle_bringup")) / "config/manual_vesc_config.yaml"), gate])]
    test["steering_ceiling_deg"] = control["maximum_steering_angle_deg"]
    test["steering_rate_ceiling_deg_per_sec"] = control["steering_rate_limit_deg_per_sec"]
    editable_drive = {"longitudinal_start_duty", "duty_rise_rate_per_sec", "duty_fall_rate_per_sec",
                      "longitudinal_recovery_duty_rise_per_sec", "longitudinal_recovery_start_ramp_sec",
                      "brake_maximum_current_amps", "brake_current_rise_amps_per_sec",
                      "brake_current_fall_amps_per_sec", "longitudinal_brake_speed_gain"}
    for key in ("maximum_duty", "longitudinal_start_duty", "duty_rise_rate_per_sec", "duty_fall_rate_per_sec",
                "longitudinal_pid_kp", "longitudinal_pid_ki", "longitudinal_pid_kd", "longitudinal_pid_integral_limit",
                "longitudinal_feedforward_offset_duty", "longitudinal_feedforward_duty_per_mps",
                "longitudinal_feedforward_fade_speed_mps", "brake_maximum_current_amps",
                "longitudinal_brake_speed_gain",
                "brake_current_rise_amps_per_sec", "brake_current_fall_amps_per_sec",
                "servo_left", "servo_center", "servo_right", "steering_servo_inverted",
                "longitudinal_recovery_duty_rise_per_sec", "longitudinal_recovery_start_ramp_sec",
                "electrical_brake_enabled", "longitudinal_deceleration_filter_sec",
                "longitudinal_brake_deceleration_gain", "brake_entry_speed_error_mps", "brake_exit_speed_error_mps",
                "obstacle_brake_entry_margin_mps", "obstacle_brake_exit_margin_mps",
                "obstacle_coast_probe_sec", "corner_coast_probe_sec",
                "obstacle_brake_deceleration_margin_mps2", "corner_brake_deceleration_margin_mps2"):
        if key in editable_drive:
            test.setdefault("drive." + key, control[key])
        else:
            test["drive." + key] = control[key]
    test["drive.max_command_age_sec"] = test["max_age_sec"]
    test["deceleration_ceiling_mps2"] = min(control["longitudinal_planning_deceleration_mps2"],
                                           obstacles["obstacles.avoidance.deceleration_mps2"])
    test["deceleration_mps2"] = min(test["deceleration_mps2"], control["longitudinal_planning_deceleration_mps2"],
                                    obstacles["obstacles.avoidance.deceleration_mps2"])
    # VFH response delay is user-selected; the node still applies measured input age + planning-period floor.
    lane_overrides = {"input_topic": "/roi_lidar/lane_bev", "direct_bev_input_enabled": "false",
                      "preview_enabled": "false", "bev_theme_enable": "false",
                      "centerline_enabled": "true", "connection_enabled": "true",
                      "centerline_bev_width_m": str(bev["y_max_m"]-bev["y_min_m"]),
                      "centerline_bev_height_m": str(bev["x_max_m"]-bev["x_min_m"]),
                      "result_publish_enabled": "true", "result_topic": lane["result_topic"]}
    actions = [*handover_actions, Node(package="roi_lidar", executable="roi_lidar_node", name="roi_lidar",
                    output="screen", parameters=[roi]),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory("line_detactor")) / "launch/line_detactor.launch.py")),
                                        launch_arguments={"params_file": arg("lane_params_file"), **lane_overrides}.items()),
               Node(package="roi_vfh_test", executable="roi_vfh_preview", name="roi_vfh_test",
                    output="screen", parameters=[test])]
    bridge_mode = arg("vesc_bridge_enabled").strip().lower()
    if bridge_mode not in ("true", "false"):
        raise ValueError("vesc_bridge_enabled must be true or false")
    if bridge_mode == "true":
        # Same bridge executable/config as vehicle_bringup/vesc_only.launch.py.
        # Resolve all five topics exactly as the VFH controller and speed monitor
        # do, including explicit overrides and vehicle_namespace.
        bridge_topics = dict(actuator_topics)
        bridge_topics.update({"measured_erpm_topic": erpm_topic,
                              "connection_status_topic": connection_topic,
                              "port": arg("vesc_port")})
        actions.insert(0, Node(package="vesc_bridge", executable="vesc_bridge_node",
                               name="vesc_bridge_node", namespace=arg("vehicle_namespace").strip("/"),
                               output="screen", parameters=[arg("vesc_params_file"), bridge_topics]))
    if arg("dynamics_enabled").lower() == "true":
        dynamics = load(Path(get_package_share_directory("vehicle_dynamics_monitor")) / "config/vehicle_dynamics.yaml", "/**")
        input_mode = arg("input_mode").lower()
        if input_mode not in ("ros_topic", "socketcan", "slcan"):
            raise ValueError("input_mode must be ros_topic, socketcan or slcan")
        # VFH and handover stop confirmation both use the bridge's UART ERPM.
        # CANable is an independent dynamics instrument: losing CAN frames must
        # not hide otherwise healthy UART speed from the VFH watchdog.
        dynamics.update({"input_mode": "ros_topic", "measured_erpm_topic": erpm_topic,
                         "can_interface": arg("can_interface"), "can_controller_id": int(arg("can_controller_id")),
                         "slcan_channel": arg("slcan_channel"), "slcan_bitrate": int(arg("slcan_bitrate")),
                         "commanded_duty_topic": actual_duty_log,
                         "servo_position_topic": actuator_topics["servo_position_topic"],
                         "connection_status_topic": connection_topic,
                         "speed_topic": arg("measured_speed_topic"), "imu_topic": "/roi_lidar/imu",
                         "imu_yaw_axis": "z", "imu_yaw_rate_sign": 1.0,
                         "sample_timeout_sec": control["erpm_timeout_sec"],
                         "wheelbase_m": test["wheelbase_m"],
                         "maximum_left_steering_angle_deg": control["maximum_steering_angle_deg"],
                         "maximum_right_steering_angle_deg": control["maximum_steering_angle_deg"],
                         "servo_left": control["servo_left"], "servo_center": control["servo_center"],
                         "servo_right": control["servo_right"]})
        for key in ("wheel_diameter_m", "motor_pole_pairs", "motor_pinion_teeth", "spur_gear_teeth",
                    "differential_pinion_teeth", "differential_ring_teeth", "erpm_direction_sign",
                    "speed_scale_correction", "speed_filter_time_constant_sec"):
            dynamics[key] = control[key]
        actions.append(Node(package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node",
                            name="roi_vfh_dynamics", output="screen", parameters=[dynamics]))
        if input_mode != "ros_topic":
            can_dynamics = dict(dynamics)
            can_dynamics["input_mode"] = input_mode
            # Distinct outputs keep stale/valid CAN samples from competing with
            # the one UART producer of /roi_vfh_test/measured_speed.
            monitor_prefix = (prefix or "/manual") + "/vehicle/dynamics"
            base_dynamics = load(Path(get_package_share_directory("vehicle_dynamics_monitor")) /
                                 "config/vehicle_dynamics.yaml", "/**")
            for key, value in base_dynamics.items():
                if key.endswith("_topic") and str(value).startswith("/vehicle/dynamics/"):
                    can_dynamics[key] = monitor_prefix + "/" + str(value).rsplit("/", 1)[-1]
            if can_dynamics["speed_topic"] == dynamics["speed_topic"]:
                raise ValueError("CAN diagnostics speed topic must differ from VFH measured_speed_topic")
            actions.append(Node(package="vehicle_dynamics_monitor", executable="vehicle_dynamics_node",
                                name="roi_vfh_can_dynamics", output="screen", parameters=[can_dynamics]))
    elif arg("dynamics_enabled").lower() != "false":
        raise ValueError("dynamics_enabled must be true or false")
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("roi_vfh_test"))
    reference = share / "config/reference"
    defaults = {"params_file": share / "config/preview.yaml",
                "roi_params_file": reference / "roi_lidar_test.yaml",
                "lane_params_file": reference / "line_detactor_test.yaml",
                "bev_params_file": reference / "bev_config_test.yaml",
                "camera_params_file": reference / "camera_config_test.yaml",
                "vehicle_params_file": reference / "auto_control_test.yaml",
                "obstacle_params_file": reference / "obstacles_test.yaml",
                "drive_enabled": "", "duty_topic": "", "brake_current_topic": "", "servo_position_topic": "",
                "vesc_bridge_enabled": "true", "vesc_port": "/dev/ttyTHS1",
                "vesc_params_file": Path(get_package_share_directory("vehicle_bringup")) / "config/vesc_config.yaml",
                "dynamics_enabled": "true", "vehicle_namespace": "autopilot03", "measured_erpm_topic": "",
                "manual_handover_enabled": "true", "controller_name_contains": "8BitDo",
                "input_mode": "slcan", "slcan_channel": "/dev/ttyACM0", "slcan_bitrate": "500000",
                "can_interface": "can0", "can_controller_id": "112",
                "imu_rate_hz": "120",
                "vesc_connection_topic": "", "measured_speed_topic": "/roi_vfh_test/measured_speed"}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=str(v)) for k, v in defaults.items()] + [OpaqueFunction(function=nodes)])
