cmake_minimum_required(VERSION 3.18)
project(traffic_detection_test LANGUAGES C CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(bev_handoff REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn highgui imgproc)
find_package(CUDAToolkit REQUIRED)
find_path(TENSORRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/TensorRT/include
    /usr/local/include
)
find_library(TENSORRT_NVINFER_LIBRARY
  NAMES nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
find_library(TENSORRT_ONNX_PARSER_LIBRARY
  NAMES nvonnxparser
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
find_library(TENSORRT_PLUGIN_LIBRARY
  NAMES nvinfer_plugin
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
if(
  NOT TENSORRT_INCLUDE_DIR OR
  NOT TENSORRT_NVINFER_LIBRARY OR
  NOT TENSORRT_ONNX_PARSER_LIBRARY OR
  NOT TENSORRT_PLUGIN_LIBRARY
)
  message(FATAL_ERROR
    "TensorRT headers or libraries were not found. Install the JetPack "
    "TensorRT development packages (libnvinfer-dev, "
    "libnvonnxparsers-dev, and libnvinfer-plugin-dev)."
  )
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrafficLightState.msg"
  DEPENDENCIES std_msgs
)

add_library(traffic_yolox_detector STATIC
  src/cuda_yolox_preprocessor.cu
  src/tensorrt_yolox_backend.cpp
  src/yolox_detector.cpp
)
set_target_properties(
  traffic_yolox_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(traffic_yolox_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
  PRIVATE
    ${TENSORRT_INCLUDE_DIR}
)
target_link_libraries(traffic_yolox_detector
  PUBLIC
    ${OpenCV_LIBS}
  PRIVATE
    CUDA::cudart
    ${TENSORRT_NVINFER_LIBRARY}
    ${TENSORRT_ONNX_PARSER_LIBRARY}
    ${TENSORRT_PLUGIN_LIBRARY}
)

add_library(traffic_detection_test_component SHARED
  src/traffic_detection_test_node.cpp
)
target_compile_definitions(traffic_detection_test_component
  PRIVATE TRAFFIC_DETECTION_TEST_BUILDING_DLL
)
target_include_directories(traffic_detection_test_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(traffic_detection_test_component
  ament_index_cpp
  rclcpp
  rclcpp_components
)
target_link_libraries(traffic_detection_test_component
  traffic_yolox_detector
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

rclcpp_components_register_nodes(
  traffic_detection_test_component
  "traffic_detection_test::TrafficDetectionTestNode"
)

add_library(traffic_light_component SHARED src/traffic_light_node.cpp)
ament_target_dependencies(traffic_light_component
  ament_index_cpp bev_handoff rclcpp rclcpp_components std_msgs)
target_link_libraries(traffic_light_component traffic_yolox_detector Threads::Threads)
rosidl_get_typesupport_target(traffic_light_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(traffic_light_component "${traffic_light_typesupport}")
rclcpp_components_register_nodes(traffic_light_component
  "traffic_detection_test::TrafficLightNode")

add_executable(traffic_detection_test_node
  src/main.cpp
)
ament_target_dependencies(traffic_detection_test_node rclcpp)
target_link_libraries(
  traffic_detection_test_node traffic_detection_test_component)

install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    traffic_yolox_detector
    traffic_detection_test_component
    traffic_light_component
  EXPORT export_traffic_detection_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  TARGETS traffic_detection_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_traffic_detection_test HAS_LIBRARY_TARGET)
ament_export_dependencies(
  depthai
  Threads
  bev_handoff
  std_msgs
  rosidl_default_runtime
  ament_index_cpp
  CUDAToolkit
  OpenCV
  rclcpp
  rclcpp_components
)

ament_package()
