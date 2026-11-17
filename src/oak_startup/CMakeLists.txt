cmake_minimum_required(VERSION 3.18)
project(oak_startup LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)
add_library(measurement SHARED
  src/ground_plane_estimator.cpp src/startup_attitude.cpp src/oak_startup_measurement.cpp)
set_target_properties(measurement PROPERTIES OUTPUT_NAME oak_startup_measurement)
target_include_directories(measurement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(measurement PUBLIC depthai::core ${OpenCV_LIBS} Threads::Threads)
install(TARGETS measurement EXPORT export_oak_startup
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_oak_startup HAS_LIBRARY_TARGET NAMESPACE oak_startup::)
ament_export_dependencies(OpenCV depthai Threads)
ament_package()
