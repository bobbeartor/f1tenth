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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(bev_lane_detector STATIC
  src/bev_lane_detector.cpp
)
set_target_properties(
  bev_lane_detector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_lane_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_lane_detector PUBLIC ${OpenCV_LIBS})

add_library(lane_detect_components SHARED
  src/lane_mask_node.cpp
  src/bev_lane_extractor_node.cpp
)
target_compile_definitions(lane_detect_components
  PRIVATE LANE_MASK_BUILDING_DLL
)
target_include_directories(lane_detect_components
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lane_detect_components
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(lane_detect_components
  bev_lane_detector
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(lane_detect_components
  "lane_mask::LaneMaskNode"
  "lane_mask::BevLaneExtractorNode"
)

add_executable(lane_detect_node src/main.cpp)
ament_target_dependencies(lane_detect_node rclcpp)
target_link_libraries(lane_detect_node lane_detect_components)

add_executable(bev_lane_extractor_node src/bev_lane_extractor_main.cpp)
ament_target_dependencies(bev_lane_extractor_node rclcpp)
target_link_libraries(bev_lane_extractor_node lane_detect_components)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    bev_lane_detector_test test/bev_lane_detector_test.cpp)
  target_link_libraries(
    bev_lane_detector_test bev_lane_detector ${OpenCV_LIBS})
  add_test(
    NAME bev_lane_detector_test COMMAND bev_lane_detector_test)
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
    bev_lane_detector
    lane_detect_components
    lane_detect_node
    bev_lane_extractor_node
  EXPORT export_lane_detect
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_lane_detect HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
)

ament_package()
