cmake_minimum_required(VERSION 3.8)
project(auto_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Depth projection and planning run per frame; never fall back to an
# unoptimized build when colcon is invoked without an explicit build type.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Keep the helper module distinct from auto_control's generated ROS messages.
ament_python_install_package(auto_control_tunnel)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(line_detactor REQUIRED)
find_package(traffic_detection_test REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(bev_handoff REQUIRED)
find_package(bev_processor REQUIRED)
find_package(camera_driver REQUIRED)
find_package(point_cloud REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidancePlan.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)
rosidl_get_typesupport_target(avoidance_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

# Export the existing pure control helpers for the VFH actuator adapter.
add_library(auto_control_core STATIC src/control_core.cpp src/driving_log.cpp)
target_link_libraries(auto_control_core PUBLIC Threads::Threads)
set_target_properties(auto_control_core PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME control_core)
target_include_directories(auto_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS auto_control_core EXPORT auto_control_core_targets ARCHIVE DESTINATION lib)
ament_export_targets(auto_control_core_targets HAS_LIBRARY_TARGET)

add_library(auto_obstacle_component SHARED
  src/obstacle_detector_node.cpp src/obstacle_worker.cpp src/control_core.cpp)
target_include_directories(auto_obstacle_component PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(auto_obstacle_component
  rclcpp rclcpp_components rcl_interfaces bev_handoff bev_processor camera_driver geometry_msgs std_msgs)
target_link_libraries(auto_obstacle_component point_cloud::point_cloud_depth_source ${OpenCV_LIBS} Threads::Threads ${avoidance_typesupport})
rclcpp_components_register_nodes(auto_obstacle_component "auto_control::ObstacleDetectorNode")
install(TARGETS auto_obstacle_component LIBRARY DESTINATION lib)

# C++ tunnel lane selector, loaded into the detector container (intra-process LaneResult).
add_library(auto_tunnel_selector_component SHARED src/tunnel_lane_selector.cpp)
target_include_directories(auto_tunnel_selector_component PRIVATE include)
ament_target_dependencies(auto_tunnel_selector_component rclcpp rclcpp_components std_msgs line_detactor)
rclcpp_components_register_nodes(auto_tunnel_selector_component "auto_control::TunnelLaneSelector")
install(TARGETS auto_tunnel_selector_component LIBRARY DESTINATION lib)


add_executable(auto_control_node
  src/auto_control_node.cpp
  src/control_core.cpp
  src/driving_log.cpp
)
target_include_directories(auto_control_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(auto_control_node
  builtin_interfaces
  bev_handoff
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
)
# These packages also export camera/inference components. Control consumes
# only their ROS messages; do not link every exported implementation target.
target_link_libraries(auto_control_node
  Threads::Threads
  ${avoidance_typesupport}
  line_detactor::line_detactor__rosidl_typesupport_cpp
  traffic_detection_test::traffic_detection_test__rosidl_typesupport_cpp
)

install(TARGETS auto_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/tunnel_lane_selector scripts/model_startup_check scripts/startup_inspection scripts/runtime_tools scripts/remote_preview DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY auto_control_tunnel/web/ DESTINATION share/${PROJECT_NAME}/web)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES
  README.md
  AUTO_CONTROL_PARAMETER_TUNING_KO.txt
  AUTO_CONTROL_OBSTACLES.md
  AUTO_CONTROL_TUNNEL.md
  STARTUP_INSPECTION.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(runtime_speed_limit_test test/runtime_speed_limit_test.cpp)
  target_include_directories(runtime_speed_limit_test PRIVATE include)
  add_test(NAME runtime_speed_limit_test COMMAND runtime_speed_limit_test)
  add_executable(staged_brake_test test/staged_brake_test.cpp src/control_core.cpp)
  target_include_directories(staged_brake_test PRIVATE include)
  add_test(NAME staged_brake_test COMMAND staged_brake_test)
endif()

ament_export_dependencies(rosidl_default_runtime Threads)
ament_package()
