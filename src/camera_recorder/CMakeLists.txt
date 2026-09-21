cmake_minimum_required(VERSION 3.16)
project(camera_recorder)

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
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc videoio)
find_package(depthai 3.6 CONFIG QUIET)
if(NOT depthai_FOUND)
  message(FATAL_ERROR
    "DepthAI C++ 3.6+ was not found. Install depthai-core and add its "
    "installation prefix to CMAKE_PREFIX_PATH or set depthai_DIR.")
endif()
find_package(Threads REQUIRED)

add_executable(camera_recorder_node src/camera_recorder_node.cpp)
target_include_directories(camera_recorder_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_recorder_node rclcpp)
target_link_libraries(
  camera_recorder_node
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

install(
  TARGETS camera_recorder_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
