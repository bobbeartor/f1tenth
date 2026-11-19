"""RGB-only OAK + YOLO + fitted portable distance model; no stereo sensors."""
from datetime import datetime
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _start(context):
    share=Path(get_package_share_directory('yolo_distance_runtime'))
    algorithm=LaunchConfiguration('model').perform(context)
    if algorithm not in ['mlp','lightgbm']:
        raise RuntimeError('model must be mlp or lightgbm')
    requested=LaunchConfiguration('distance_model').perform(context)
    model=Path(requested).expanduser().resolve() if requested else share/'models'/(algorithm+'_rgb.json')
    if not model.is_file():
        raise RuntimeError(f'Portable trained distance model missing: {model}')
    contract=json.loads(model.read_text())
    if contract.get('version')!=3 or contract.get('algorithm')!=algorithm:
        raise RuntimeError('Model file must be portable version 3 and match model:=mlp/lightgbm')
    parameters={
        'mode':'rgb_infer', 'distance_model':str(model),
        'backend':LaunchConfiguration('backend'),
        'preview':ParameterValue(LaunchConfiguration('preview'),value_type=bool),
        'fps':ParameterValue(LaunchConfiguration('fps'),value_type=float),
        'confidence_threshold':ParameterValue(LaunchConfiguration('confidence_threshold'),value_type=float),
        'nms_threshold':ParameterValue(LaunchConfiguration('nms_threshold'),value_type=float),
        'output_directory':LaunchConfiguration('output_directory'),
        'engine_cache':LaunchConfiguration('engine_cache'),
    }
    return [Node(package='yolo_distance_runtime',executable='yolo_distance_node',name='yolo_distance',
                 output='screen',parameters=[str(share/'config'/'rgb_distance.yaml'),parameters])]


def generate_launch_description():
    defaults={
        'model':'lightgbm','distance_model':'','backend':'TENSORRT','preview':'true','fps':'30.0',
        'confidence_threshold':'0.4','nms_threshold':'0.65',
        'engine_cache':str(Path.home()/'.cache/yolo_distance_runtime/holdout10.fp16.engine'),
        'output_directory':str(Path.home()/'distance_live_logs'/datetime.now().strftime('rgb_%Y%m%d_%H%M%S_%f')),
    }
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
                              OpaqueFunction(function=_start)])
