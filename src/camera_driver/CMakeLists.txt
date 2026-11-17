cmake_minimum_required(VERSION 3.16)
project(camera_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bev_handoff REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs imgproc)
# Load the complete DepthAI export set in this directory before ament loads
# dependencies that may expose only depthai::core from a nested scope.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)
find_package(point_cloud REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BevInput.msg"
  DEPENDENCIES std_msgs
)

add_library(camera_imu_stabilizer STATIC
  src/imu_image_stabilizer.cpp
)
set_target_properties(
  camera_imu_stabilizer PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(camera_imu_stabilizer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_imu_stabilizer PUBLIC ${OpenCV_LIBS})

add_library(camera_vehicle_motion STATIC
  src/vehicle_motion_estimator.cpp
)
set_target_properties(
  camera_vehicle_motion PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(camera_vehicle_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(camera_driver_component SHARED
  src/camera_driver_node.cpp
)
target_compile_definitions(camera_driver_component
  PRIVATE CAMERA_DRIVER_BUILDING_DLL
)
target_include_directories(camera_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_driver_component
  bev_handoff
  rclcpp
  rcl_interfaces
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(camera_driver_component
  camera_imu_stabilizer
  camera_vehicle_motion
  point_cloud::point_cloud_depth_source
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)
rosidl_get_typesupport_target(
  camera_driver_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(
  camera_driver_component "${camera_driver_typesupport_target}")

rclcpp_components_register_nodes(
  camera_driver_component
  "camera_driver::CameraDriverNode"
)

add_executable(camera_driver_node
  src/main.cpp
)
ament_target_dependencies(camera_driver_node
  rclcpp
)
target_link_libraries(camera_driver_node
  camera_driver_component
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    imu_image_stabilizer_test test/imu_image_stabilizer_test.cpp)
  target_link_libraries(
    imu_image_stabilizer_test camera_imu_stabilizer ${OpenCV_LIBS})
  add_test(
    NAME imu_image_stabilizer_test COMMAND imu_image_stabilizer_test)
  add_executable(ir_imu_synchronizer_test test/ir_imu_synchronizer_test.cpp)
  target_link_libraries(ir_imu_synchronizer_test camera_imu_stabilizer ${OpenCV_LIBS})
  add_test(NAME ir_imu_synchronizer_test COMMAND ir_imu_synchronizer_test)
  add_executable(rgb_exposure_recovery_test test/rgb_exposure_recovery_test.cpp)
  target_include_directories(rgb_exposure_recovery_test PRIVATE include)
  add_test(NAME rgb_exposure_recovery_test COMMAND rgb_exposure_recovery_test)
  add_executable(
    vehicle_motion_estimator_test test/vehicle_motion_estimator_test.cpp)
  target_link_libraries(
    vehicle_motion_estimator_test camera_vehicle_motion)
  add_test(
    NAME vehicle_motion_estimator_test COMMAND vehicle_motion_estimator_test)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    camera_imu_stabilizer
    camera_vehicle_motion
    camera_driver_component
  EXPORT export_camera_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
# Executables must not be part of the exported library target set: packages
# that depend on camera_driver for BevInput would otherwise try to link it.
install(
  TARGETS
    camera_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_camera_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  bev_handoff point_cloud
  OpenCV rclcpp rcl_interfaces rclcpp_components geometry_msgs rosidl_default_runtime
  sensor_msgs std_msgs)

ament_package()
