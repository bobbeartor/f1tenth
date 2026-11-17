cmake_minimum_required(VERSION 3.16)
project(depth_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(oak_startup REQUIRED)

add_library(depth_lidar_geometry src/depth_lidar_geometry.cpp src/depth_lidar_stabilization.cpp src/depth_lidar_grid.cpp)
target_include_directories(depth_lidar_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(depth_lidar_geometry PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_lidar_geometry PRIVATE opencv_core)

add_executable(depth_lidar_node src/depth_lidar_node.cpp src/depth_lidar_preview.cpp)
target_include_directories(depth_lidar_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(depth_lidar_node builtin_interfaces rclcpp rcl_interfaces sensor_msgs std_msgs nav_msgs geometry_msgs visualization_msgs)
target_link_libraries(depth_lidar_node depth_lidar_geometry oak_startup::measurement depthai::core ${OpenCV_LIBS})

install(
  TARGETS depth_lidar_geometry depth_lidar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(depth_lidar_geometry_test test/depth_lidar_geometry_test.cpp)
  target_link_libraries(depth_lidar_geometry_test depth_lidar_geometry)
  target_include_directories(depth_lidar_geometry_test PRIVATE include)
  ament_add_gtest(depth_lidar_preview_test
    test/depth_lidar_preview_test.cpp src/depth_lidar_preview.cpp)
  target_link_libraries(depth_lidar_preview_test depth_lidar_geometry ${OpenCV_LIBS})
  target_include_directories(depth_lidar_preview_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
endif()

ament_export_include_directories(include)
ament_export_libraries(depth_lidar_geometry)
ament_export_dependencies(OpenCV)
ament_package()
