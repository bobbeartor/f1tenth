import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_node(context):
    library_dir = LaunchConfiguration("depthai_library_dir").perform(context).strip()
    runtime_environment = {}
    if library_dir:
        previous = context.environment.get("LD_LIBRARY_PATH", "")
        runtime_environment["LD_LIBRARY_PATH"] = os.pathsep.join(
            item for item in (library_dir, previous) if item
        )
    # Empty optional arguments preserve values supplied by params_file.
    optional_overrides = {}
    for name, value_type in (
        ("measurement_ir_dot_projector_intensity", float),
        ("measurement_manual_camera_height_enabled", bool),
        ("measurement_manual_camera_height_m", float),
    ):
        value = LaunchConfiguration(name).perform(context).strip()
        if not value:
            continue
        if value_type is bool:
            if value.lower() not in ("true", "false"):
                raise ValueError(f"{name} must be true or false")
            optional_overrides[name] = value.lower() == "true"
        else:
            optional_overrides[name] = value_type(value)
    return [
        Node(
            package="ir_camera_driver",
            executable="ir_camera_driver_node",
            name="ir_camera_driver",
            output="screen",
            additional_env=runtime_environment,
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "reprojection_enabled": ParameterValue(
                        LaunchConfiguration("reprojection_enabled"),
                        value_type=bool,
                    ),
                    "selected_camera": LaunchConfiguration(
                        "selected_camera"
                    ),
                    "virtual_camera_position_ratio": ParameterValue(
                        LaunchConfiguration(
                            "virtual_camera_position_ratio"
                        ),
                        value_type=float,
                    ),
                    "width": ParameterValue(
                        LaunchConfiguration("stereo_width"), value_type=int
                    ),
                    "height": ParameterValue(
                        LaunchConfiguration("stereo_height"), value_type=int
                    ),
                    "reprojection_fps": ParameterValue(
                        LaunchConfiguration("stereo_fps"), value_type=float
                    ),
                    "single_camera_fps": ParameterValue(
                        LaunchConfiguration("stereo_fps"), value_type=float
                    ),
                    "rgb_width": ParameterValue(
                        LaunchConfiguration("rgb_width"), value_type=int
                    ),
                    "rgb_height": ParameterValue(
                        LaunchConfiguration("rgb_height"), value_type=int
                    ),
                    "rgb_fps": ParameterValue(
                        LaunchConfiguration("rgb_fps"), value_type=float
                    ),
                    "preview_max_fps": ParameterValue(
                        LaunchConfiguration("processing_fps"), value_type=float
                    ),
                    "rgb_preview_enabled": ParameterValue(
                        LaunchConfiguration("rgb_preview_enabled"), value_type=bool
                    ),
                    "stereo_preview_enabled": ParameterValue(
                        LaunchConfiguration("stereo_preview_enabled"), value_type=bool
                    ),
                    "bev_preview_enabled": ParameterValue(
                        LaunchConfiguration("bev_preview_enabled"), value_type=bool
                    ),
                    "controls_preview_enabled": ParameterValue(
                        LaunchConfiguration("controls_preview_enabled"), value_type=bool
                    ),
                    "ir_enabled": ParameterValue(
                        LaunchConfiguration("ir_enabled"),
                        value_type=bool,
                    ),
                    "ir_dot_projector_intensity": ParameterValue(
                        LaunchConfiguration(
                            "ir_dot_projector_intensity"
                        ),
                        value_type=float,
                    ),
                    "ir_flood_light_intensity": ParameterValue(
                        LaunchConfiguration("ir_flood_light_intensity"),
                        value_type=float,
                    ),
                    "capture_directory": LaunchConfiguration(
                        "capture_directory"
                    ),
                },
                optional_overrides,
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("ir_camera_driver")
    default_params = f"{package_share}/config/ir_camera_config.yaml"
    runtime_file = Path(package_share) / "depthai_runtime_dir.txt"
    default_library_dir = runtime_file.read_text(encoding="utf-8").strip()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depthai_library_dir",
                default_value=default_library_dir,
                description="Prefer the DepthAI library selected during build for this camera process.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="IR camera preview parameter file.",
            ),
            DeclareLaunchArgument(
                "reprojection_enabled",
                default_value="true",
                description=(
                    "Reproject synchronized stereo IR images to one virtual "
                    "camera view."
                ),
            ),
            DeclareLaunchArgument(
                "selected_camera",
                default_value="LEFT",
                description=(
                    "Lens used when reprojection is off and as the invalid-"
                    "disparity fallback: LEFT/CAM_B or RIGHT/CAM_C."
                ),
            ),
            DeclareLaunchArgument(
                "virtual_camera_position_ratio",
                default_value="0.5",
                description=(
                    "Virtual position from LEFT/CAM_B (0.0) to RIGHT/CAM_C "
                    "(1.0)."
                ),
            ),
            DeclareLaunchArgument("rgb_width", default_value="1280"),
            DeclareLaunchArgument("rgb_height", default_value="800"),
            DeclareLaunchArgument("rgb_fps", default_value="30.0"),
            DeclareLaunchArgument("stereo_width", default_value="1280"),
            DeclareLaunchArgument("stereo_height", default_value="800"),
            DeclareLaunchArgument("stereo_fps", default_value="30.0"),
            DeclareLaunchArgument(
                "processing_fps",
                default_value="30.0",
                description="Maximum stereo-center/BEV processing and publish rate.",
            ),
            DeclareLaunchArgument("rgb_preview_enabled", default_value="true"),
            DeclareLaunchArgument("stereo_preview_enabled", default_value="true"),
            DeclareLaunchArgument("bev_preview_enabled", default_value="true"),
            DeclareLaunchArgument("controls_preview_enabled", default_value="true"),
            DeclareLaunchArgument(
                "ir_enabled",
                default_value="true",
                description="Enable configured OAK IR emitters at startup.",
            ),
            DeclareLaunchArgument(
                "ir_dot_projector_intensity",
                default_value="1.0",
                description="IR laser dot-projector intensity from 0.0 to 1.0.",
            ),
            DeclareLaunchArgument(
                "ir_flood_light_intensity",
                default_value="0.0",
                description="IR flood-light intensity from 0.0 to 1.0.",
            ),
            DeclareLaunchArgument(
                "measurement_ir_dot_projector_intensity",
                default_value="",
                description="Dot intensity during startup ground measurement only.",
            ),
            DeclareLaunchArgument(
                "measurement_manual_camera_height_enabled",
                default_value="",
                description=(
                    "Use manual CAM_A height and IMU startup attitude; "
                    "skip startup stereo. Empty uses params_file."
                ),
            ),
            DeclareLaunchArgument(
                "measurement_manual_camera_height_m",
                default_value="",
                description="Manual CAM_A optical-center height above ground in meters.",
            ),
            DeclareLaunchArgument(
                "capture_directory",
                default_value=".",
                description="Directory used by the B-key PNG capture.",
            ),
            OpaqueFunction(function=_launch_node),
        ]
    )
