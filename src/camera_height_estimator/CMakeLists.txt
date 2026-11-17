cmake_minimum_required(VERSION 3.16)
project(camera_height_estimator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")

add_library(camera_height_geometry STATIC
  src/camera_height_geometry.cpp
)
target_include_directories(camera_height_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(camera_height_estimator_node
  src/camera_height_estimator_node.cpp
)
ament_target_dependencies(camera_height_estimator_node
  rclcpp
)
target_link_libraries(camera_height_estimator_node
  camera_height_geometry
  depthai::core
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(
    camera_height_geometry_test
    test/camera_height_geometry_test.cpp
  )
  target_link_libraries(
    camera_height_geometry_test
    camera_height_geometry
  )
  add_test(
    NAME camera_height_geometry_test
    COMMAND camera_height_geometry_test
  )
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
    camera_height_geometry
    camera_height_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
