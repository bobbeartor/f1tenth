"""IR lane following with the unmodified production detector and controller."""
import hashlib
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ir_bev_line_detection.settings import build_parameters


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false')
    return value == 'true'


def _nodes(context):
    shares = {p: Path(get_package_share_directory(p)) for p in (
        'ir_bev_line_detection', 'ir_camera_driver', 'line_detactor',
        'auto_control', 'vehicle_bringup', 'joy_initializer',
    )}
    config = LaunchConfiguration('params_file').perform(context)
    settings = build_parameters(shares, config, LaunchConfiguration('speed_mps').perform(context))
    camera, detector, controller = (settings[n] for n in ('ir_camera_driver', 'line_detactor', 'auto_control'))
    model = Path(LaunchConfiguration('model_path').perform(context))
    if not model.is_file():
        raise ValueError(f'IR ONNX model not found: {model}')
    bundled_model = shares['line_detactor'] / 'models/ir_stereo_bev_120x300.onnx'
    if model.resolve() == bundled_model.resolve():
        report = json.loads(model.with_suffix('.json').read_text(encoding='utf-8'))
        if hashlib.sha256(model.read_bytes()).hexdigest() != report['onnx_sha256']:
            raise ValueError('Bundled IR ONNX checksum differs from its export report')
    detector.update(model_path=str(model), preview_enabled=_boolean(context, 'preview_enabled'))
    controller['enabled'] = _boolean(context, 'auto_enabled')
    mode = LaunchConfiguration('control_mode').perform(context)
    if mode not in ('drive', 'monitor_only'):
        raise ValueError('control_mode must be drive or monitor_only')
    controller['control_mode'] = mode
    namespace = LaunchConfiguration('vehicle_namespace').perform(context).strip('/')
    prefix = '/' + namespace if namespace else ''

    # Same routing as vehicle_bringup auto_drive: AUTO -> handover -> one VESC bridge.
    # The existing handover owns actuator outputs and X -> STOPPING -> MANUAL logic.
    handover = {
        'joy_topic': prefix + '/joy', 'auto_enable_topic': controller['enable_topic'],
        'measured_erpm_topic': controller['measured_erpm_topic'],
        'connection_status_topic': controller['connection_status_topic'],
        'current_duty_topic': prefix + '/drive/current_duty',
        'current_brake_current_topic': prefix + '/drive/current_brake_current',
        'gear_state_topic': prefix + '/manual/gear', 'drive_mode_topic': prefix + '/drive/mode',
    }
    for channel in ('duty', 'brake_current', 'servo_position'):
        topic = channel + '_topic'
        handover[topic] = controller[topic]
        handover['auto_' + topic] = prefix + '/handover/auto/' + channel
        controller[topic] = handover['auto_' + topic]

    # Match the DepthAI library selected by ir_camera_driver at build time.
    runtime_file = shares['ir_camera_driver'] / 'depthai_runtime_dir.txt'
    library = LaunchConfiguration('depthai_library_dir').perform(context).strip()
    if not library and runtime_file.is_file():
        library = runtime_file.read_text(encoding='utf-8').strip()
    environment = {}
    if library:
        environment['LD_LIBRARY_PATH'] = os.pathsep.join(filter(None, (
            library, context.environment.get('LD_LIBRARY_PATH', ''),
        )))
    actions = [
        LogInfo(msg=f'IR-only lane test | {mode} | {controller["maximum_speed_mps"]} m/s | {model}'),
        Node(package='ir_camera_driver', executable='ir_camera_driver_node',
             name='ir_camera_driver', output='screen', parameters=[camera], additional_env=environment),
        Node(package='ir_bev_line_detection', executable='mono_bev_adapter',
             name='ir_bev_adapter', output='screen', parameters=[{
                 'input_topic': camera['bev_topic'], 'output_topic': detector['input_topic'],
             }]),
        Node(package='line_detactor', executable='line_detactor_node',
             name='line_detactor', output='screen', parameters=[detector]),
        Node(package='auto_control', executable='auto_control_node',
             name='auto_control', output='screen', parameters=[controller]),
    ]
    # monitor_only performs perception/control calculations with no actuator processes.
    if mode == 'drive':
        actions.extend([
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
                shares['joy_initializer'] / 'launch/joy.launch.py')), launch_arguments={
                    'vehicle_namespace': namespace,
                    'device_name_contains': LaunchConfiguration('controller_name_contains').perform(context),
                    'reconnect_interval_sec': '0.2',
                }.items()),
            Node(package='manual_control', executable='auto_handover_node',
                 name='auto_handover', namespace=namespace, output='screen', parameters=[
                     str(shares['vehicle_bringup'] / 'config/manual_vesc_config.yaml'), handover,
                 ]),
            Node(package='vesc_bridge', executable='vesc_bridge_node', name='vesc_bridge_node',
                 output='screen', parameters=[
                     str(shares['vehicle_bringup'] / 'config/vesc_config.yaml'),
                     {'port': LaunchConfiguration('vesc_port').perform(context), **{
                         key: handover[key] for key in (
                             'duty_topic', 'brake_current_topic', 'servo_position_topic',
                             'measured_erpm_topic', 'connection_status_topic',
                         )
                     }},
                 ]),
        ])
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('ir_bev_line_detection'))
    defaults = {
        'params_file': str(share / 'config/ir_bev_test.yaml'),
        'model_path': str(Path(get_package_share_directory('line_detactor')) / 'models/ir_stereo_bev_120x300.onnx'),
        'speed_mps': '0.3', 'auto_enabled': 'true', 'control_mode': 'drive',
        'preview_enabled': 'true', 'vesc_port': '/dev/ttyTHS1',
        'vehicle_namespace': 'autopilot03', 'controller_name_contains': '8BitDo',
        'depthai_library_dir': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=_nodes),
    ])
