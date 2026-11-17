cmake_minimum_required(VERSION 3.16)
project(point_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
# Keep DepthAI's complete export set visible before loading oak_startup.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(oak_startup REQUIRED)
find_package(Threads REQUIRED)

add_library(point_cloud_depth_source STATIC src/depth_source.cpp)
set_target_properties(point_cloud_depth_source PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(point_cloud_depth_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(point_cloud_depth_source PUBLIC depthai::core)

add_executable(point_cloud_node src/point_cloud_node.cpp)
ament_target_dependencies(point_cloud_node
  rclcpp rcl_interfaces sensor_msgs std_msgs geometry_msgs tf2_ros tf2 visualization_msgs)
target_link_libraries(point_cloud_node point_cloud_depth_source oak_startup::measurement Threads::Threads)

install(TARGETS point_cloud_depth_source EXPORT point_cloud_depth_targets ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(point_cloud_depth_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(depthai)

install(TARGETS point_cloud_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES ../bev_processor/config/bev_config.yaml
  DESTINATION share/${PROJECT_NAME}/config RENAME bev_reference.yaml)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
  add_executable(point_cloud_projection_test test/projection_test.cpp)
  target_include_directories(point_cloud_projection_test PRIVATE include)
  add_test(NAME point_cloud_projection_test COMMAND point_cloud_projection_test)
  add_executable(point_cloud_bev_test test/bev_crop_test.cpp)
  target_include_directories(point_cloud_bev_test PRIVATE include)
  add_test(NAME point_cloud_bev_test COMMAND point_cloud_bev_test)
  add_executable(point_cloud_bev_reference_test
    test/bev_reference_test.cpp ../bev_processor/src/bev_geometry.cpp)
  target_include_directories(point_cloud_bev_reference_test PRIVATE
    include ../bev_processor/include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(point_cloud_bev_reference_test ${OpenCV_LIBS})
  add_test(NAME point_cloud_bev_reference_test COMMAND point_cloud_bev_reference_test)
  add_executable(point_cloud_ground_test test/ground_filter_test.cpp)
  target_include_directories(point_cloud_ground_test PRIVATE include)
  add_test(NAME point_cloud_ground_test COMMAND point_cloud_ground_test)
  add_executable(point_cloud_config_test test/config_test.cpp)
  target_link_libraries(point_cloud_config_test point_cloud_depth_source)
  add_test(NAME point_cloud_config_test COMMAND point_cloud_config_test)
endif()

ament_package()
