from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("camera_driver")
    default_params = f"{package_share}/config/camera_config.yaml"

    params_file = LaunchConfiguration("params_file")
    preview_enabled = LaunchConfiguration("preview_enabled")
    preview_grid_enabled = LaunchConfiguration("preview_grid_enabled")
    capture_directory = LaunchConfiguration("capture_directory")
    capture_joy_topic = LaunchConfiguration("capture_joy_topic")
    capture_joy_button = LaunchConfiguration("capture_joy_button")
    capture_debounce_sec = LaunchConfiguration("capture_debounce_sec")
    imu_stabilization_enabled = LaunchConfiguration(
        "imu_stabilization_enabled"
    )
    high_frequency_only = LaunchConfiguration(
        "imu_stabilization_high_frequency_only"
    )
    high_frequency_cutoff_hz = LaunchConfiguration(
        "imu_stabilization_high_frequency_vibration_cutoff_hz"
    )
    gyroscope_correction_gain = LaunchConfiguration(
        "imu_stabilization_gyroscope_correction_gain"
    )
    can_longitudinal_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_longitudinal_compensation_gain"
    )
    can_lateral_compensation_gain = LaunchConfiguration(
        "imu_stabilization_can_lateral_compensation_gain"
    )
    moving_accelerometer_nudge_strength = LaunchConfiguration(
        "imu_stabilization_moving_accelerometer_nudge_strength"
    )
    moving_gravity_anchor_maximum_correction_rate_degps = LaunchConfiguration(
        "imu_stabilization_moving_gravity_anchor_maximum_correction_rate_degps"
    )
    invalid_correction_hold_frames = LaunchConfiguration(
        "imu_stabilization_invalid_correction_hold_frames"
    )
    publish_enabled = LaunchConfiguration("publish_enabled")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Camera driver parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "preview_enabled",
                default_value="false",
                description="Show the independent latest-frame preview.",
            ),
            DeclareLaunchArgument(
                "preview_grid_enabled",
                default_value="false",
                description="Draw a light-gray 20-pixel grid on the preview.",
            ),
            DeclareLaunchArgument(
                "capture_directory",
                default_value=".",
                description=(
                    "Directory for A-button camera captures; default is the "
                    "launch working directory."
                ),
            ),
            DeclareLaunchArgument(
                "capture_joy_topic",
                default_value="/joy",
                description="Joy topic used by the camera A-button capture.",
            ),
            DeclareLaunchArgument(
                "capture_joy_button",
                default_value="0",
                description="Joy button index used for camera capture (A=0).",
            ),
            DeclareLaunchArgument(
                "capture_debounce_sec",
                default_value="0.30",
                description="Minimum interval between camera captures.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_enabled",
                default_value="true",
                description=(
                    "Apply OAK IMU roll/pitch correction to preview and "
                    "published NV12 output."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_only",
                default_value="false",
                description=(
                    "Disable CAN and moving accelerometer corrections, "
                    "leaving gyro high-frequency stabilization only."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_high_frequency_vibration_cutoff_hz",
                default_value="3.0",
                description="High-frequency-only gyro cutoff in Hz.",
            ),
            DeclareLaunchArgument(
                "imu_stabilization_gyroscope_correction_gain",
                default_value="1.0",
                description=(
                    "Applied gyro image correction fraction from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_longitudinal_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN longitudinal acceleration removed "
                    "from the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_can_lateral_compensation_gain",
                default_value="1.0",
                description=(
                    "Fraction of CAN lateral acceleration removed from "
                    "the camera accelerometer, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_moving_accelerometer_nudge_strength",
                default_value="0.15",
                description=(
                    "Strength of the bounded residual-accelerometer image "
                    "correction, from 0.0 to 1.0."
                ),
            ),
            DeclareLaunchArgument(
                (
                    "imu_stabilization_moving_gravity_anchor_maximum_"
                    "correction_rate_degps"
                ),
                default_value="0.50",
                description=(
                    "Maximum persistent CAN-compensated gravity anchor "
                    "correction rate in degrees per second."
                ),
            ),
            DeclareLaunchArgument(
                "imu_stabilization_invalid_correction_hold_frames",
                default_value="2",
                description=(
                    "Reuse the last valid stabilization homography for this "
                    "many consecutive camera/IMU matching misses."
                ),
            ),
            DeclareLaunchArgument(
                "publish_enabled",
                default_value="false",
                description="Publish sensor_msgs/Image frames.",
            ),
            ComposableNodeContainer(
                name="camera_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="camera_driver",
                        plugin="camera_driver::CameraDriverNode",
                        name="camera_driver",
                        parameters=[
                            params_file,
                            {
                                "preview_enabled": preview_enabled,
                                "preview_grid_enabled": preview_grid_enabled,
                                "capture_directory": capture_directory,
                                "capture_joy_topic": capture_joy_topic,
                                "capture_joy_button": ParameterValue(
                                    capture_joy_button, value_type=int
                                ),
                                "capture_debounce_sec": ParameterValue(
                                    capture_debounce_sec, value_type=float
                                ),
                                "imu_stabilization_enabled": (
                                    imu_stabilization_enabled
                                ),
                                "imu_stabilization_high_frequency_only": (
                                    ParameterValue(
                                        high_frequency_only,
                                        value_type=bool,
                                    )
                                ),
                                (
                                    "imu_stabilization_high_frequency_"
                                    "vibration_cutoff_hz"
                                ): ParameterValue(
                                    high_frequency_cutoff_hz,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_gyroscope_"
                                    "correction_gain"
                                ): ParameterValue(
                                    gyroscope_correction_gain,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_can_longitudinal_"
                                    "compensation_gain"
                                ): ParameterValue(
                                    can_longitudinal_compensation_gain,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_can_lateral_"
                                    "compensation_gain"
                                ): ParameterValue(
                                    can_lateral_compensation_gain,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_moving_"
                                    "accelerometer_nudge_strength"
                                ): ParameterValue(
                                    moving_accelerometer_nudge_strength,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_moving_gravity_anchor_"
                                    "maximum_correction_rate_degps"
                                ): ParameterValue(
                                    moving_gravity_anchor_maximum_correction_rate_degps,
                                    value_type=float,
                                ),
                                (
                                    "imu_stabilization_invalid_correction_"
                                    "hold_frames"
                                ): ParameterValue(
                                    invalid_correction_hold_frames,
                                    value_type=int,
                                ),
                                "publish_enabled": publish_enabled,
                                # The CAN dynamics monitor needs camera yaw.
                                "imu_bridge_enabled": True,
                            },
                        ],
                        extra_arguments=[
                            {"use_intra_process_comms": True},
                        ],
                    )
                ],
            ),
        ]
    )
