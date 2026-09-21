from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("camera_recorder")
    default_params = f"{package_share}/config/camera_recorder.yaml"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Camera recorder parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "output_directory",
                default_value="recordings",
                description="Root directory for recording sessions.",
            ),
            DeclareLaunchArgument(
                "device_id",
                default_value="",
                description="DepthAI device ID; empty selects the first device.",
            ),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity", default_value="0.5"
            ),
            Node(
                package="camera_recorder",
                executable="camera_recorder_node",
                name="camera_recorder",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "output_directory": LaunchConfiguration(
                            "output_directory"
                        ),
                        "device_id": ParameterValue(
                            LaunchConfiguration("device_id"), value_type=str
                        ),
                        "fps": ParameterValue(
                            LaunchConfiguration("fps"), value_type=float
                        ),
                        "ir_dot_projector_intensity": ParameterValue(
                            LaunchConfiguration("ir_dot_projector_intensity"),
                            value_type=float,
                        ),
                        "ir_flood_light_intensity": ParameterValue(
                            LaunchConfiguration("ir_flood_light_intensity"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
