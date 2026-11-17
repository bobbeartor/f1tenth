cmake_minimum_required(VERSION 3.18)
project(bev_processor LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(bev_handoff REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Import all DepthAI targets before the ament dependency exports.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)
find_package(oak_startup REQUIRED)
find_package(camera_driver REQUIRED)

add_library(bev_geometry STATIC
  src/bev_geometry.cpp
)
set_target_properties(bev_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_geometry
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_geometry PUBLIC ${OpenCV_LIBS})

add_library(bev_cuda STATIC
  src/cuda_bev_processor.cu
)
set_target_properties(bev_cuda PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_cuda
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(bev_cuda PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
)
target_link_libraries(bev_cuda PUBLIC CUDA::cudart ${OpenCV_LIBS})

add_library(bev_processor_component SHARED
  src/bev_processor_node.cpp
)
target_include_directories(
  bev_processor_component PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(bev_processor_component
  bev_handoff
  camera_driver
  rcl_interfaces
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(bev_processor_component
  bev_geometry
  bev_cuda
  oak_startup::measurement
  ${OpenCV_LIBS}
  Threads::Threads
)
rclcpp_components_register_nodes(
  bev_processor_component
  "bev_processor::BevProcessorNode"
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME depthai_runtime_launch_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_depthai_runtime_launch.py)
  add_executable(bev_geometry_test test/bev_geometry_test.cpp)
  target_link_libraries(bev_geometry_test bev_geometry ${OpenCV_LIBS})
  add_test(NAME bev_geometry_test COMMAND bev_geometry_test)

  add_executable(
    ground_plane_estimator_test test/ground_plane_estimator_test.cpp)
  target_link_libraries(
    ground_plane_estimator_test
    oak_startup::measurement
    bev_geometry
    ${OpenCV_LIBS}
  )
  add_test(
    NAME ground_plane_estimator_test COMMAND ground_plane_estimator_test)

  add_executable(startup_attitude_test test/startup_attitude_test.cpp)
  target_include_directories(startup_attitude_test PRIVATE include)
  target_link_libraries(
    startup_attitude_test oak_startup::measurement ${OpenCV_LIBS})
  add_test(NAME startup_attitude_test COMMAND startup_attitude_test)
endif()

install(TARGETS bev_geometry EXPORT export_bev_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_export_targets(export_bev_geometry HAS_LIBRARY_TARGET)

install(TARGETS
  bev_processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_export_include_directories(include)
ament_export_dependencies(
  bev_handoff camera_driver oak_startup OpenCV rclcpp_components rcl_interfaces geometry_msgs)
ament_package()
