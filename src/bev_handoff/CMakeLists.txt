cmake_minimum_required(VERSION 3.18)
project(bev_handoff LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core)
find_package(std_msgs REQUIRED)

add_library(bev_handoff SHARED
  src/direct_bev_handoff.cpp
  src/direct_camera_handoff.cpp
  src/direct_obstacle_handoff.cpp
  src/avoidance_handoff.cpp
)
target_include_directories(bev_handoff
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(bev_handoff std_msgs)
target_link_libraries(bev_handoff ${OpenCV_LIBS})

install(
  TARGETS bev_handoff
  EXPORT export_bev_handoff
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_bev_handoff HAS_LIBRARY_TARGET)
ament_export_libraries(bev_handoff)
ament_export_include_directories(include)
ament_export_dependencies(OpenCV std_msgs)
ament_package()
