from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("traffic_detection_test")
    default_params = f"{package_share}/config/traffic_detection_test.yaml"
    default_model = (
        f"{package_share}/models/"
        "traffic_light_yolox_s_640x160_batch_1.int8.qdq.onnx"
    )

    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")
    inference_backend = LaunchConfiguration("inference_backend")
    engine_precision = LaunchConfiguration("engine_precision")
    engine_cache_path = LaunchConfiguration("engine_cache_path")
    tensorrt_workspace_size_mb = LaunchConfiguration(
        "tensorrt_workspace_size_mb"
    )
    model_input_width = LaunchConfiguration("model_input_width")
    model_input_height = LaunchConfiguration("model_input_height")
    roi_center_x = LaunchConfiguration("roi_center_x")
    roi_center_y = LaunchConfiguration("roi_center_y")
    roi_width = LaunchConfiguration("roi_width")
    roi_height = LaunchConfiguration("roi_height")
    score_threshold = LaunchConfiguration("score_threshold")
    nms_threshold = LaunchConfiguration("nms_threshold")
    color_min_saturation = LaunchConfiguration("color_min_saturation")
    color_min_value = LaunchConfiguration("color_min_value")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Traffic detection preview parameter YAML.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model,
                description="Decoded YOLOX ONNX model; defaults to explicit Q/DQ INT8.",
            ),
            DeclareLaunchArgument(
                "inference_backend",
                default_value="TENSORRT",
                description="Inference backend: TENSORRT or CPU.",
            ),
            DeclareLaunchArgument(
                "engine_precision",
                default_value="int8",
                description="TensorRT precision: fp32, fp16, or int8.",
            ),
            DeclareLaunchArgument(
                "engine_cache_path",
                default_value="",
                description=(
                    "TensorRT engine cache path. Empty stores it beside ONNX."
                ),
            ),
            DeclareLaunchArgument(
                "tensorrt_workspace_size_mb",
                default_value="1024",
                description="TensorRT engine-build workspace in MiB.",
            ),
            DeclareLaunchArgument(
                "model_input_width",
                default_value="640",
                description="Fixed ONNX input tensor width; multiple of 32.",
            ),
            DeclareLaunchArgument(
                "model_input_height",
                default_value="160",
                description="Fixed ONNX input tensor height; multiple of 32.",
            ),
            DeclareLaunchArgument(
                "roi_center_x",
                default_value="320",
                description="Inference ROI horizontal center in source pixels.",
            ),
            DeclareLaunchArgument(
                "roi_center_y",
                default_value="145",
                description="Inference ROI vertical center in source pixels.",
            ),
            DeclareLaunchArgument(
                "roi_width",
                default_value="640",
                description="Inference ROI width in source pixels.",
            ),
            DeclareLaunchArgument(
                "roi_height",
                default_value="160",
                description="Inference ROI height in source pixels.",
            ),
            DeclareLaunchArgument(
                "score_threshold",
                default_value="0.25",
                description="Objectness times class-score threshold.",
            ),
            DeclareLaunchArgument(
                "nms_threshold",
                default_value="0.65",
                description="Intersection-over-union threshold for NMS.",
            ),
            DeclareLaunchArgument(
                "color_min_saturation",
                default_value="80",
                description="Minimum HSV saturation for signal color pixels.",
            ),
            DeclareLaunchArgument(
                "color_min_value",
                default_value="60",
                description="Minimum HSV brightness for signal color pixels.",
            ),
            ComposableNodeContainer(
                name="traffic_detection_test_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="traffic_detection_test",
                        plugin=(
                            "traffic_detection_test::"
                            "TrafficDetectionTestNode"
                        ),
                        name="traffic_detection_test",
                        parameters=[
                            params_file,
                            {
                                "model_path": model_path,
                                "inference_backend": inference_backend,
                                "engine_precision": engine_precision,
                                "engine_cache_path": engine_cache_path,
                                "tensorrt_workspace_size_mb": ParameterValue(
                                    tensorrt_workspace_size_mb,
                                    value_type=int,
                                ),
                                "model_input_width": ParameterValue(
                                    model_input_width, value_type=int
                                ),
                                "model_input_height": ParameterValue(
                                    model_input_height, value_type=int
                                ),
                                "roi_center_x": ParameterValue(
                                    roi_center_x, value_type=int
                                ),
                                "roi_center_y": ParameterValue(
                                    roi_center_y, value_type=int
                                ),
                                "roi_width": ParameterValue(
                                    roi_width, value_type=int
                                ),
                                "roi_height": ParameterValue(
                                    roi_height, value_type=int
                                ),
                                "score_threshold": ParameterValue(
                                    score_threshold, value_type=float
                                ),
                                "nms_threshold": ParameterValue(
                                    nms_threshold, value_type=float
                                ),
                                "color_min_saturation": ParameterValue(
                                    color_min_saturation, value_type=int
                                ),
                                "color_min_value": ParameterValue(
                                    color_min_value, value_type=int
                                ),
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
