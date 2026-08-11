cmake_minimum_required(VERSION 3.16)
project(lane_detect)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(lane_detect_components SHARED
  src/lane_mask_node.cpp
)
target_compile_definitions(lane_detect_components
  PRIVATE LANE_MASK_BUILDING_DLL
)
target_include_directories(lane_detect_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(lane_detect_components
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(lane_detect_components
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(lane_detect_components
  "lane_mask::LaneMaskNode"
)

add_executable(lane_detect_node src/main.cpp)
ament_target_dependencies(lane_detect_node rclcpp)
target_link_libraries(lane_detect_node lane_detect_components)

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
    lane_detect_components
    lane_detect_node
  EXPORT export_lane_detect
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_lane_detect HAS_LIBRARY_TARGET)
ament_export_dependencies(
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
