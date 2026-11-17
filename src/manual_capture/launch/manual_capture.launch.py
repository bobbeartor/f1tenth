"""Capture GUI + manual driving in one launch.

ros2 launch manual_capture manual_capture.launch.py vehicle_namespace:=autopilot03 ...
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_MANUAL_DRIVE_ARGUMENTS = (
    ("vehicle_namespace", "autopilot03"),
    ("vesc_port", "/dev/ttyTHS1"),
    ("controller_name_contains", "8BitDo"),
    ("input_mode", "ros_topic"),
    ("can_interface", "can0"),
    ("can_controller_id", "0"),
    ("slcan_channel", "/dev/ttyACM0"),
    ("slcan_bitrate", "500000"),
)


def _workspace_root(package_share):
    # <ws>/install/manual_capture/share/manual_capture -> <ws>
    return Path(package_share).resolve().parents[3]


def _capture_node(context):
    package_share = get_package_share_directory("manual_capture")
    capture_root = LaunchConfiguration("capture_root").perform(context).strip()
    if not capture_root:
        capture_root = str(_workspace_root(package_share) / "captures")
    joy_topic = LaunchConfiguration("capture_joy_topic").perform(context).strip()
    if not joy_topic:
        namespace = LaunchConfiguration("vehicle_namespace").perform(context).strip("/")
        joy_topic = f"/{namespace}/joy" if namespace else "/joy"

    environment = {}
    library_dir = LaunchConfiguration("depthai_library_dir").perform(context).strip()
    if library_dir:
        previous = context.environment.get("LD_LIBRARY_PATH", "")
        environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            item for item in (library_dir, previous) if item
        )
    return [
        Node(
            package="manual_capture",
            executable="manual_capture_node",
            name="manual_capture",
            output="screen",
            additional_env=environment,
            parameters=[
                LaunchConfiguration("params_file"),
                {"capture_root": os.path.abspath(capture_root), "capture_joy_topic": joy_topic},
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("manual_capture")
    runtime_file = Path(package_share) / "depthai_runtime_dir.txt"
    default_library_dir = (
        runtime_file.read_text(encoding="utf-8").strip() if runtime_file.exists() else ""
    )
    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("vehicle_bringup"), "launch",
                 "manual_drive_with_dynamics.launch.py"]
            )
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name, _ in _MANUAL_DRIVE_ARGUMENTS
        }.items(),
        condition=IfCondition(LaunchConfiguration("manual_drive_enabled")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/manual_capture.yaml",
            ),
            DeclareLaunchArgument(
                "capture_root",
                default_value="",
                description="Empty: <workspace root>/captures.",
            ),
            DeclareLaunchArgument(
                "capture_joy_topic",
                default_value="",
                description="Empty: /<vehicle_namespace>/joy.",
            ),
            DeclareLaunchArgument("manual_drive_enabled", default_value="true"),
            DeclareLaunchArgument("depthai_library_dir", default_value=default_library_dir),
            *[DeclareLaunchArgument(name, default_value=value)
              for name, value in _MANUAL_DRIVE_ARGUMENTS],
            manual_drive,
            OpaqueFunction(function=_capture_node),
        ]
    )
