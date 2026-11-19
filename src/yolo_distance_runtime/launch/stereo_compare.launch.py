"""Live portable model versus synchronized Stereo reference, without point clouds."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch=Path(get_package_share_directory('yolo_distance_runtime'))/'launch'/'rgb_distance.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch)),
        launch_arguments={'compare_stereo':'true'}.items())])
