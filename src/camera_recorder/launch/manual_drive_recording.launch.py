from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    recorder_share = get_package_share_directory("camera_recorder")
    bringup_share = get_package_share_directory("vehicle_bringup")

    manual_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{bringup_share}/launch/manual_drive.launch.py"
        ),
        launch_arguments={
            "vehicle_namespace": LaunchConfiguration("vehicle_namespace"),
            "vesc_port": LaunchConfiguration("vesc_port"),
            "controller_name_contains": LaunchConfiguration(
                "controller_name_contains"
            ),
        }.items(),
    )

    recording = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{recorder_share}/launch/camera_recording.launch.py"
        ),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "output_directory": LaunchConfiguration("output_directory"),
            "device_id": LaunchConfiguration("device_id"),
            "fps": LaunchConfiguration("fps"),
            "ir_dot_projector_intensity": LaunchConfiguration(
                "ir_dot_projector_intensity"
            ),
            "ir_flood_light_intensity": LaunchConfiguration(
                "ir_flood_light_intensity"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_namespace", default_value="autopilot03"
            ),
            DeclareLaunchArgument("vesc_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument(
                "controller_name_contains", default_value="8BitDo"
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=(
                    f"{recorder_share}/config/camera_recorder.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "output_directory", default_value="recordings"
            ),
            DeclareLaunchArgument("device_id", default_value=""),
            DeclareLaunchArgument("fps", default_value="30.0"),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity", default_value="0.5"
            ),
            manual_drive,
            recording,
        ]
    )
