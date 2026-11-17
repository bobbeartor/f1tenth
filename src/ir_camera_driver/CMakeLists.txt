cmake_minimum_required(VERSION 3.18)
project(ir_camera_driver LANGUAGES CXX CUDA)

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
find_package(OpenCV 4 REQUIRED COMPONENTS core highgui imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bev_handoff REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Import all DepthAI targets before the ament dependency exports.
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/DepthaiWorkspace.cmake")
find_package(Threads REQUIRED)
find_package(bev_processor REQUIRED)
find_package(camera_driver REQUIRED)
find_package(oak_startup REQUIRED)

add_library(ir_center_reprojector STATIC
  src/cuda_center_reprojector.cu
)
set_target_properties(
  ir_center_reprojector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ir_center_reprojector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(ir_center_reprojector PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:--use_fast_math>
)
target_link_libraries(
  ir_center_reprojector PUBLIC CUDA::cudart ${OpenCV_LIBS})

add_library(ir_bev_projector STATIC
  src/ir_bev_projector.cpp
)
set_target_properties(ir_bev_projector PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ir_bev_projector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ir_bev_projector PUBLIC bev_processor::bev_geometry ${OpenCV_LIBS})

add_executable(ir_camera_driver_node
  src/ir_camera_driver_node.cpp
)
target_include_directories(ir_camera_driver_node
  PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(ir_camera_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(ir_camera_driver_node
  ir_center_reprojector
  ir_bev_projector
  camera_driver::camera_imu_stabilizer
  oak_startup::measurement
  depthai::core
  ${OpenCV_LIBS}
  Threads::Threads
)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(shared_ir_bev_component SHARED src/shared_ir_bev_node.cpp)
ament_target_dependencies(shared_ir_bev_component rclcpp rclcpp_components
  sensor_msgs std_msgs bev_handoff)
target_link_libraries(shared_ir_bev_component ir_bev_projector ${OpenCV_LIBS})
rclcpp_components_register_nodes(shared_ir_bev_component "ir_camera_driver::SharedIrBevNode")
install(TARGETS shared_ir_bev_component LIBRARY DESTINATION lib)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(
  TARGETS
    ir_center_reprojector
    ir_bev_projector
    ir_camera_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
