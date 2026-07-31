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
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(depthai 3 CONFIG QUIET)
if(NOT depthai_FOUND)
  message(FATAL_ERROR
    "DepthAI C++ 3.x was not found. bev_processor performs an OAK stereo "
    "ground-plane and IMU startup measurement and therefore "
    "links directly against depthai::core."
  )
endif()
find_package(Threads REQUIRED)

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

add_library(bev_ground_plane STATIC
  src/ground_plane_estimator.cpp
)
set_target_properties(bev_ground_plane PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_ground_plane
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_ground_plane PUBLIC ${OpenCV_LIBS})

add_library(bev_startup_attitude STATIC
  src/startup_attitude.cpp
)
set_target_properties(
  bev_startup_attitude PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_startup_attitude
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bev_startup_attitude PUBLIC ${OpenCV_LIBS})

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

add_library(bev_startup_measurement STATIC
  src/oak_startup_measurement.cpp
)
set_target_properties(
  bev_startup_measurement PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(bev_startup_measurement
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  bev_startup_measurement
  PUBLIC depthai::core bev_ground_plane bev_startup_attitude)

add_library(bev_processor_component SHARED
  src/bev_processor_node.cpp
)
target_include_directories(
  bev_processor_component PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(bev_processor_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)
target_link_libraries(bev_processor_component
  bev_geometry
  bev_cuda
  bev_startup_measurement
  ${OpenCV_LIBS}
  Threads::Threads
)
rclcpp_components_register_nodes(
  bev_processor_component
  "bev_processor::BevProcessorNode"
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(bev_geometry_test test/bev_geometry_test.cpp)
  target_link_libraries(bev_geometry_test bev_geometry ${OpenCV_LIBS})
  add_test(NAME bev_geometry_test COMMAND bev_geometry_test)

  add_executable(
    ground_plane_estimator_test test/ground_plane_estimator_test.cpp)
  target_link_libraries(
    ground_plane_estimator_test
    bev_ground_plane
    bev_geometry
    ${OpenCV_LIBS}
  )
  add_test(
    NAME ground_plane_estimator_test COMMAND ground_plane_estimator_test)

  add_executable(startup_attitude_test test/startup_attitude_test.cpp)
  target_link_libraries(
    startup_attitude_test bev_startup_attitude ${OpenCV_LIBS})
  add_test(NAME startup_attitude_test COMMAND startup_attitude_test)
endif()

install(TARGETS
  bev_geometry
  bev_ground_plane
  bev_startup_attitude
  bev_startup_measurement
  bev_processor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(OpenCV rclcpp_components)
ament_package()
