"""Manual joystick driving plus the single-owner OAK RGB/stereo collector."""
from datetime import datetime
from pathlib import Path
import os
import re

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    RegisterEventHandler, EmitEvent,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_collection(context):
    sequence = LaunchConfiguration('sequence').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', sequence):
        raise RuntimeError('sequence must contain only letters, digits, _ or -')
    root = Path(LaunchConfiguration('output_directory').perform(context)).expanduser().resolve()
    if (root / sequence).exists():
        raise RuntimeError(f'Collection sequence already exists: {root / sequence}; use a new sequence')
    root.mkdir(parents=True, exist_ok=True)
    if not os.access(root, os.W_OK):
        raise RuntimeError(f'Collection directory is not writable: {root}')

    collector = Node(
        package='multiclass_yolo', executable='yolo_distance_node',
        name='yolo_distance', output='screen',
        parameters=[{
            'mode': 'collect', 'sequence': sequence, 'output_directory': str(root),
            'backend': LaunchConfiguration('backend'),
            'preview': ParameterValue(LaunchConfiguration('preview'), value_type=bool),
            'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=float),
            'sync_ms': ParameterValue(LaunchConfiguration('sync_ms'), value_type=float),
        }],
    )
    manual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('vehicle_bringup'), 'launch', 'manual_drive.launch.py'])),
        launch_arguments={name: LaunchConfiguration(name) for name in [
            'vehicle_namespace', 'vesc_port', 'controller_name_contains']}.items(),
    )
    # Register before starting the collector: failures/normal Q exits end this session.
    return [RegisterEventHandler(OnProcessExit(
        target_action=collector,
        on_exit=[EmitEvent(event=Shutdown(reason='Distance collector exited'))],
    )), collector, manual]


def generate_launch_description():
    defaults = {
        'vehicle_namespace': 'autopilot03',
        'vesc_port': '/dev/ttyTHS1',
        'controller_name_contains': '8BitDo',
        'sequence': datetime.now().strftime('manual_%Y%m%d_%H%M%S_%f'),
        'output_directory': str(Path.home() / 'distance_dataset'),
        'backend': 'TENSORRT', 'preview': 'true', 'camera_fps': '30.0', 'sync_ms': '20.0',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=_launch_collection),
    ])
