from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'mode': 'collect', 'sequence': '', 'output_directory': 'distance_data',
                'distance_model': '', 'backend': 'TENSORRT', 'preview': 'true'}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()] +
        [Node(package='multiclass_yolo', executable='yolo_distance_node',
              output='screen', parameters=[{k: ParameterValue(LaunchConfiguration(k),
                  value_type=bool if k == 'preview' else str) for k in defaults}])])
