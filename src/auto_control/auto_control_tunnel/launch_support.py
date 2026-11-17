"""Optional integration used only by vehicle_bringup tunnel_enabled:=true."""
import hashlib
import json
from .ir_length_exit import IrLengthExit
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def integrated_nodes(rgb_detector, controller, context):
    validation_text = str(context.launch_configurations.get('tunnel_validation_mode', 'true')).lower()
    if validation_text not in ('true', 'false'):
        raise ValueError('tunnel_validation_mode must be true or false')
    validation = validation_text == 'true'
    cfg = context.launch_configurations
    length_text = str(cfg.get('tunnel_ir_length_exit_enabled', 'false')).lower()
    if length_text not in ('true', 'false'):
        raise ValueError('tunnel_ir_length_exit_enabled must be true or false')
    length = IrLengthExit(length_text == 'true',
        float(cfg.get('tunnel_ir_length_exit_ratio', '.15')),
        int(cfg.get('tunnel_ir_length_exit_confirm_frames', '3')),
        float(cfg.get('tunnel_ir_length_exit_minimum_reference_m', '.5')))
    shares = {p: Path(get_package_share_directory(p)) for p in (
        'ir_camera_driver', 'line_detactor', 'auto_control', 'tunnel_tcn_test')}
    # Perception assets are shared by production and standalone tests. The
    # controller must not depend on a test package that depends on bringup.
    def parameters(path):
        return yaml.safe_load(path.read_text())['line_detactor']['ros__parameters']
    ir = parameters(shares['line_detactor'] / 'config/line_detactor.yaml')
    ir.update(parameters(shares['auto_control'] / 'config/tunnel_ir.yaml'))
    model = shares['line_detactor'] / 'models/ir_stereo_bev_120x300.onnx'
    report = json.loads(model.with_suffix('.json').read_text())
    if hashlib.sha256(model.read_bytes()).hexdigest() != report['onnx_sha256']:
        raise ValueError('Bundled IR model checksum mismatch')
    # SharedIrBev is the sole 30Hz gate. A second host-arrival gate loses
    # valid frames when camera/ROS delivery jitters around a 30Hz boundary.
    ir.update(model_path=str(model), inference_max_fps=0.0,
              preview_enabled=False, bev_theme_enable=False,
              shared_ir_preview_output_enabled=rgb_detector['preview_enabled'],
              preview_result_only_enabled=rgb_detector.get('preview_result_only_enabled', True))
    rgb_detector.update({'tunnel.enabled': True, 'inference_max_fps': 70.0})
    age = controller.get('path_capture_maximum_age_sec', .2)
    rule_age = context.launch_configurations.get('path_capture_maximum_age_sec', age)
    selector_parameters = dict(validation_mode=validation, max_age_sec=min(.25, float(rule_age)),
                               rgb_result_topic=rgb_detector['result_topic'],
                               ir_length_exit_enabled=length.enabled,
                               ir_length_exit_ratio=length.threshold,
                               ir_length_exit_confirm_frames=length.confirm_frames,
                               ir_length_exit_minimum_reference_m=length.minimum_reference)
    # Match the actual controller YAML and resolved CLI geometry overrides.
    for name, fallback, cast in (
        ('path_minimum_points', 8, int), ('path_minimum_span_m', .12, float),
        ('path_minimum_x_m', .05, float), ('path_maximum_x_m', 2.20, float),
        ('path_maximum_gap_m', .15, float)):
        selector_parameters[name] = cast(context.launch_configurations.get(name, controller.get(name, fallback)))
    selector_parameters.update(
        lane_result_frame_id=context.launch_configurations.get('ml_lane_frame_id', 'front_axle_bev'),
        bev_x_max_m=float(context.launch_configurations.get('ml_bev_x_max_m', 3.)),
        bev_y_max_m=float(context.launch_configurations.get('ml_bev_y_max_m', .6)),
        bev_meter_per_pixel=float(context.launch_configurations.get('ml_bev_meter_per_pixel', .01)))
    components = [
        ComposableNode(package='ir_camera_driver', plugin='ir_camera_driver::SharedIrBevNode',
                       name='shared_ir_bev', parameters=[{'validation_mode': validation, 'standby_fps': 0.0, 'active_fps': 30.0,
                                                          'model_input_topic': ir['input_topic']}]),
        ComposableNode(package='line_detactor', plugin='line_detactor::LineDetactorNode',
                       name='ir_line_detactor', parameters=[ir],
                       extra_arguments=[{'use_intra_process_comms': True}]),
        # C++ selector in the detector container: RGB/IR results arrive intra-process.
        ComposableNode(package='auto_control', plugin='auto_control::TunnelLaneSelector',
                       name='tunnel_lane_selector', parameters=[selector_parameters],
                       extra_arguments=[{'use_intra_process_comms': True}])]
    processes = [
        Node(package='tunnel_tcn_test', executable='tunnel_tcn_node', name='tunnel_tcn_test',
             output='screen', parameters=[str(shares['tunnel_tcn_test'] / 'config/tunnel_tcn.yaml'),
                                          {'image_topic': '/tunnel/rgb_raw', 'initial_state': 'outside',
                                           'preview_enabled': False}])]
    return components, processes


def model_check_node(tunnel_enabled, traffic_enabled):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    return Node(package='auto_control', executable='model_startup_check',
                name='model_startup_check', output='screen',
                parameters=[{'expected_nodes': expected}])


def startup_inspection_node(tunnel_enabled, traffic_enabled,
                            connection_diagnostics_topic='/vesc/connected/diagnostics'):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    return Node(package='auto_control', executable='startup_inspection',
                name='startup_inspection', output='screen', parameters=[{'expected_nodes': expected,
                    'connection_diagnostics_topic': connection_diagnostics_topic}])


def remote_preview_parameters(params_file, detector_lane_width_m=0.65):
    """Load remote_preview ros__parameters from a YAML file (node key `remote_preview` or `/**`)."""
    import yaml
    with open(params_file, encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    section = {}
    for key in ('remote_preview', '/remote_preview', '/**'):
        if isinstance(data.get(key), dict):
            section = dict(data[key].get('ros__parameters') or {})
            break
    if float(section.get('lane_width_m', 0.0) or 0.0) <= 0.0:
        section['lane_width_m'] = float(detector_lane_width_m)
    return section


def remote_preview_node(tunnel_enabled, traffic_enabled, params_file, detector_lane_width_m=0.65,
                        connection_status_topic='/vesc/connected',
                        connection_diagnostics_topic='/vesc/connected/diagnostics'):
    expected = ['line_detactor']
    if tunnel_enabled:
        expected += ['ir_line_detactor', 'tunnel_tcn_test']
    if traffic_enabled:
        expected += ['traffic_light_detector']
    parameters = remote_preview_parameters(params_file, detector_lane_width_m)
    parameters.update({'expected_nodes': expected,
                       'connection_diagnostics_topic': connection_diagnostics_topic})
    return Node(package='auto_control', executable='remote_preview', name='remote_preview',
                output='screen', parameters=[parameters])
